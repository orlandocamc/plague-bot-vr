"""Mission executor (SPEC §6.2, ADR-0004).

Drives one inspection cycle as a state machine:

    IDLE -> NAVIGATE -> DEPLOY -> SCAN -> DETECT -> AIM -> RETURN -> IDLE

All arm motions are commanded straight to the arm_controller via its
FollowJointTrajectory action — no MoveIt/move_group needed, which keeps the
pipeline runnable in simulation and on the real robot.

The robot is detection-only (no sprayer — ADR-0004), so the arm never reaches the
plant. The AIM step is a camera *look-at*: it rotates the base yaw (joint_1) and
wrist pitch (joint_5) by the detection's angular offset in the camera frame,
centering the pest in the D435 view. This replaces SPEC §6.2's IK_POSITION, which
was geometrically infeasible (arm reach ~0.5 m < row standoff ~0.7 m).

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

from geometry_msgs.msg import PoseStamped, PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from nav2_msgs.action import NavigateToPose

import math
import tf2_ros
from rclpy.time import Time
from rclpy.duration import Duration as RclDuration
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transforms)

from plaguebot_msgs.srv import Detect


class State(Enum):
    IDLE = auto()
    NAVIGATE = auto()
    DEPLOY = auto()
    SCAN = auto()
    DETECT = auto()
    AIM = auto()
    RETURN = auto()


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')
        cb = ReentrantCallbackGroup()

        self.declare_parameter('joint_names',
                               ['joint_1', 'joint_2', 'joint_3',
                                'joint_4', 'joint_5', 'joint_6'])
        self.declare_parameter('folded', [0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0])
        # deploy: shoulder pitched forward + wrist tilted down so the D435 looks
        # at the plant-row foliage to the robot's side. Tuned empirically against
        # the sim depth cloud in a corridor (~0.6 m median return, ~49% valid).
        self.declare_parameter('deploy', [0.0, -0.4, 0.0, 0.0, -0.6, 0.0])
        self.declare_parameter('scan_min', -0.5)
        self.declare_parameter('scan_max', 0.5)
        self.declare_parameter('scan_duration', 4.0)
        self.declare_parameter('arm_move_duration', 3.0)
        # AIM (camera look-at, ADR-0004): the detection point is expressed in the
        # camera frame; we rotate joint_1 (yaw) and joint_5 (wrist pitch) by the
        # target's angular offset off the optical axis to center it in the view.
        self.declare_parameter('camera_frame', 'd435_link')
        self.declare_parameter('aim_enabled', True)
        # Per-axis sign: maps the camera-frame angular offset onto joint motion
        # (depends on how joint_1/joint_5 axes are oriented vs the optical axis).
        self.declare_parameter('aim_yaw_sign', -1.0)
        self.declare_parameter('aim_pitch_sign', 1.0)
        self.declare_parameter('aim_max_step', 1.0)  # clamp per-step (rad)
        # Home/dock to return to after a mission — the robot spawn (open floor
        # south of the plant rows). (0,0) is a map corner and not reachable.
        self.declare_parameter('return_x', 0.5)
        self.declare_parameter('return_y', -5.0)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.folded = list(self.get_parameter('folded').value)
        self.deploy = list(self.get_parameter('deploy').value)

        self._state = State.IDLE
        self._busy = threading.Lock()

        # TF, to express the detection point in the camera frame for AIM (the
        # perception node may tag it in the cloud frame, which is d435_link).
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._nav = ActionClient(self, NavigateToPose, '/navigate_to_pose',
                                 callback_group=cb)
        self._arm = ActionClient(self, FollowJointTrajectory,
                                 '/arm_controller/follow_joint_trajectory',
                                 callback_group=cb)
        self._detect = self.create_client(Detect, '/perception/detect',
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

            self._set_state(State.AIM)
            if detections and self.get_parameter('aim_enabled').value:
                self._aim_camera(detections[0])
            else:
                self.get_logger().info('no detection / aim disabled; skipping AIM')

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
        # Sweep joint_1 min -> max, then recenter to the deploy pose so DETECT
        # and IK_POSITION run from a known, stable configuration (the camera
        # yaw at scan end would otherwise move the detection point and make the
        # local KDL IK solver fail).
        for frac, j1 in ((0.25, smin), (0.75, smax), (1.0, base[0])):
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

    def _aim_camera(self, detection):
        """Camera look-at (ADR-0004): center the detection in the D435 view.

        The robot is detection-only, so the arm never reaches the plant. Instead
        we rotate the base yaw (joint_1) and wrist pitch (joint_5) by the target's
        angular offset off the optical axis, in the camera frame (+Z forward,
        +X image-right, +Y image-down). This is always feasible — no IK, no reach.
        """
        cam = self.get_parameter('camera_frame').value
        point = self._to_frame(detection.position, cam)
        if point is None:
            self.get_logger().warn(
                f'could not transform detection into {cam}; skipping AIM')
            return False
        tx, ty, tz = point.x, point.y, point.z
        if tz <= 0.05:
            self.get_logger().warn('detection behind/at camera; skipping AIM')
            return False
        # Angular offsets off the optical axis (+Z).
        yaw_off = math.atan2(tx, tz)    # horizontal
        pitch_off = math.atan2(ty, tz)  # vertical
        step = float(self.get_parameter('aim_max_step').value)
        yaw = self.get_parameter('aim_yaw_sign').value * yaw_off
        pitch = self.get_parameter('aim_pitch_sign').value * pitch_off
        yaw = max(-step, min(step, yaw))
        pitch = max(-step, min(step, pitch))
        target = list(self.deploy)
        target[0] = self.deploy[0] + yaw      # joint_1 base yaw
        target[4] = self.deploy[4] + pitch    # joint_5 wrist pitch
        self.get_logger().info(
            f'aiming camera at detection (off yaw={math.degrees(yaw_off):.1f}°, '
            f'pitch={math.degrees(pitch_off):.1f}°)')
        return self._move_arm(target)

    def _to_frame(self, point_stamped, target_frame):
        """Transform a detection PointStamped into target_frame via tf2.

        Returns the geometry_msgs/Point in target_frame, or None on failure.
        If the point is already in target_frame, returns it unchanged.
        """
        if point_stamped.header.frame_id == target_frame:
            return point_stamped.point
        src = PointStamped()
        src.header.frame_id = point_stamped.header.frame_id
        src.header.stamp = Time().to_msg()  # latest available transform
        src.point = point_stamped.point
        try:
            out = self._tf_buffer.transform(
                src, target_frame, timeout=RclDuration(seconds=2.0))
            return out.point
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'tf transform to {target_frame} failed: {exc}')
            return None

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
