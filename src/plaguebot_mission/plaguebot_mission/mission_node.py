"""Mission executor (SPEC §6.2).

Drives one inspection cycle as an 8-state machine:

    IDLE -> NAVIGATE -> DEPLOY -> SCAN -> DETECT -> IK_POSITION -> RETURN -> IDLE

The folded/deploy/scan arm motions are fixed joint configurations, so they are
commanded straight to the arm_controller via its FollowJointTrajectory action —
no move_group planning needed, which keeps the pipeline runnable in simulation.
MoveIt's /compute_ik service is used only to turn a 3D Detection point into joint
values (best-effort: if move_group isn't up or IK fails, IK_POSITION is skipped,
which SPEC allows for the no-detection case).

This node also subsumes the Phase 4 nav-arm coordinator (SPEC §5.1): the arm is
folded before NAVIGATE and deployed on arrival. See ADR-0003 for the perception
backend; the mission is backend-agnostic — it just calls /perception/detect.
"""

import threading
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from nav2_msgs.action import NavigateToPose
from moveit_msgs.srv import GetPositionIK

from plaguebot_msgs.srv import Detect


class State(Enum):
    IDLE = auto()
    NAVIGATE = auto()
    DEPLOY = auto()
    SCAN = auto()
    DETECT = auto()
    IK_POSITION = auto()
    RETURN = auto()


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')
        cb = ReentrantCallbackGroup()

        self.declare_parameter('joint_names',
                               ['joint_1', 'joint_2', 'joint_3',
                                'joint_4', 'joint_5', 'joint_6'])
        self.declare_parameter('folded', [0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0])
        self.declare_parameter('deploy', [0.0, 0.0, 0.0, 0.0, -0.5, 0.0])
        self.declare_parameter('scan_min', -0.5)
        self.declare_parameter('scan_max', 0.5)
        self.declare_parameter('scan_duration', 4.0)
        self.declare_parameter('arm_move_duration', 3.0)
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('ik_link', 'wrist_2_link')
        self.declare_parameter('return_x', 0.0)
        self.declare_parameter('return_y', 0.0)
        self.declare_parameter('use_ik', True)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.folded = list(self.get_parameter('folded').value)
        self.deploy = list(self.get_parameter('deploy').value)

        self._state = State.IDLE
        self._busy = threading.Lock()

        self._nav = ActionClient(self, NavigateToPose, '/navigate_to_pose',
                                 callback_group=cb)
        self._arm = ActionClient(self, FollowJointTrajectory,
                                 '/arm_controller/follow_joint_trajectory',
                                 callback_group=cb)
        self._detect = self.create_client(Detect, '/perception/detect',
                                          callback_group=cb)
        self._ik = self.create_client(GetPositionIK, '/compute_ik',
                                      callback_group=cb)

        self.create_subscription(PoseStamped, '/mission/start',
                                 self._on_start, 10, callback_group=cb)

        self.get_logger().info('mission_node up; waiting on /mission/start')

    # ----- mission entry -------------------------------------------------

    def _on_start(self, waypoint):
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn('mission already running; ignoring waypoint')
            return
        threading.Thread(target=self._run, args=(waypoint,), daemon=True).start()

    def _run(self, waypoint):
        try:
            self._set_state(State.NAVIGATE)
            if not self._fold_arm():
                return
            if not self._navigate(waypoint):
                self.get_logger().error('navigation failed; aborting mission')
                return

            self._set_state(State.DEPLOY)
            if not self._move_arm(self.deploy):
                return

            self._set_state(State.SCAN)
            self._scan()

            self._set_state(State.DETECT)
            detections = self._run_detect()

            self._set_state(State.IK_POSITION)
            if detections and self.get_parameter('use_ik').value:
                self._ik_position(detections[0])
            else:
                self.get_logger().info('no detection / IK disabled; skipping IK')

            self._set_state(State.RETURN)
            self._move_arm(self.folded)
            self._navigate(self._return_pose())

            self.get_logger().info('mission complete')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'mission aborted: {exc}')
        finally:
            self._set_state(State.IDLE)
            self._busy.release()

    # ----- steps ---------------------------------------------------------

    def _fold_arm(self):
        self.get_logger().info('folding arm for navigation')
        return self._move_arm(self.folded)

    def _navigate(self, pose):
        if not self._nav.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose server unavailable')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(
            f'navigating to ({pose.pose.position.x:.2f}, '
            f'{pose.pose.position.y:.2f})')
        return self._send_action(self._nav, goal)

    def _move_arm(self, positions, duration=None):
        if duration is None:
            duration = float(self.get_parameter('arm_move_duration').value)
        if not self._arm.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('arm_controller action unavailable')
            return False
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = self._dur(duration)
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        return self._send_action(self._arm, goal)

    def _scan(self):
        """Scanning Routine: sweep joint_1 from scan_min to scan_max (SPEC)."""
        smin = float(self.get_parameter('scan_min').value)
        smax = float(self.get_parameter('scan_max').value)
        dur = float(self.get_parameter('scan_duration').value)
        if not self._arm.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('arm_controller action unavailable for scan')
            return False
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        base = list(self.deploy)
        points = []
        for frac, j1 in ((0.25, smin), (1.0, smax)):
            pt = JointTrajectoryPoint()
            pos = list(base)
            pos[0] = j1
            pt.positions = [float(p) for p in pos]
            pt.time_from_start = self._dur(dur * frac)
            points.append(pt)
        traj.points = points
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.get_logger().info('executing Scanning Routine')
        return self._send_action(self._arm, goal)

    def _run_detect(self):
        if not self._detect.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/perception/detect unavailable')
            return []
        resp = self._detect.call(Detect.Request())
        if resp is None or not resp.success:
            self.get_logger().warn('detect returned no success')
            return []
        self.get_logger().info(f'detect: {resp.message}')
        return list(resp.detections)

    def _ik_position(self, detection):
        if not self._ik.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/compute_ik unavailable; skipping IK_POSITION')
            return False
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.get_parameter('planning_group').value
        req.ik_request.ik_link_name = self.get_parameter('ik_link').value
        req.ik_request.pose_stamped.header.frame_id = \
            detection.position.header.frame_id
        req.ik_request.pose_stamped.pose.position = detection.position.point
        req.ik_request.pose_stamped.pose.orientation.w = 1.0
        req.ik_request.timeout.sec = 1
        resp = self._ik.call(req)
        if resp is None or resp.error_code.val != 1:
            code = None if resp is None else resp.error_code.val
            self.get_logger().warn(f'IK failed (error_code={code}); skipping')
            return False
        sol = resp.solution.joint_state
        order = {n: p for n, p in zip(sol.name, sol.position)}
        target = [order.get(n, 0.0) for n in self.joint_names]
        self.get_logger().info('IK solved; moving gripper to detection')
        return self._move_arm(target)

    # ----- helpers -------------------------------------------------------

    def _return_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.get_parameter('return_x').value)
        pose.pose.position.y = float(self.get_parameter('return_y').value)
        pose.pose.orientation.w = 1.0
        return pose

    @staticmethod
    def _dur(seconds):
        from builtin_interfaces.msg import Duration
        d = Duration()
        d.sec = int(seconds)
        d.nanosec = int((seconds - int(seconds)) * 1e9)
        return d

    def _send_action(self, client, goal):
        """Send an action goal and block until the result; True on success."""
        send_future = client.send_goal_async(goal)
        self._wait(send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('action goal rejected')
            return False
        result_future = handle.get_result_async()
        self._wait(result_future)
        result = result_future.result()
        # status 4 == SUCCEEDED (action_msgs/GoalStatus)
        ok = result is not None and result.status == 4
        if not ok:
            self.get_logger().warn(f'action ended with status '
                                   f'{getattr(result, "status", None)}')
        return ok

    @staticmethod
    def _wait(future, timeout=120.0):
        """Block until a future completes (executor spins it in another thread)."""
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                raise TimeoutError('timed out waiting on action/future')
            time.sleep(0.05)

    def _set_state(self, state):
        self._state = state
        self.get_logger().info(f'state -> {state.name}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
