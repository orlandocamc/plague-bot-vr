# Plague-Bot VR — TODO

## Phase 2: Full Simulation

### Pre-requisites (modify existing files)

- [ ] **`plaguebot_description/urdf/plaguebot.urdf.xacro`**: add `<xacro:arg name="is_standalone" default="true"/>` and wrap `<link name="world"/>` + `<joint name="virtual_joint">` in `<xacro:if value="$(arg is_standalone)">`. Touch nothing else. (See ADR-0001)
- [ ] **`plaguebot_moveit/config/plaguebot.srdf`**: add `folded` group state (joint_2=-1.5708, joint_3=1.5708, others=0)
- [ ] **`plaguebot_moveit/config/plaguebot.srdf`**: add `deploy` group state (joint_5=-0.5, others=0)
- [ ] **`plaguebot_base_description/config/base_controllers.yaml`**: add comment block explaining the wheel naming inversion (see ADR-0002)

### New sensors in `plaguebot_base_description`

- [ ] Add `kiyo_link` + `kiyo_joint` to `plaguebot_base.urdf.xacro` (parent: `base_track_link`, xyz="-0.1 0.0 0.16", visual: box 0.03×0.07×0.03)
- [ ] Add `imu_link` + `imu_joint` to `plaguebot_base.urdf.xacro` (parent: `base_track_link`, xyz="0 0 0", no visual)
- [ ] Add RPLIDAR C1 Gazebo GPU LiDAR plugin to `laser` link (topic: `/scan`, 360°, 10 Hz, range 0.15–12 m) inside `<xacro:if is_sim>`
- [ ] Add Kiyo Gazebo camera plugin (topic: `/kiyo/image_raw`, 1280×720, 30 Hz) inside `<xacro:if is_sim>`
- [ ] Add IMU Gazebo plugin (topic: `/imu/data`, 100 Hz) inside `<xacro:if is_sim>`

### New package: `plaguebot_robot`

- [ ] Create package skeleton: `CMakeLists.txt`, `package.xml` (ament_cmake, depends on plaguebot_base_description, plaguebot_description, plaguebot_moveit)
- [ ] Create `urdf/plaguebot_robot.urdf.xacro`:
  - [ ] `xacro:include` `plaguebot_base.urdf.xacro` (pass `is_sim`)
  - [ ] `xacro:include` `plaguebot.urdf.xacro` (pass `is_sim`, `is_standalone:=false`)
  - [ ] Add `arm_mount_joint` (type=fixed, parent=`top_plate_link`, child=`base_link`, xyz="0 0 0", rpy="0 0 0")
  - [ ] Add `d435_link` (box 0.025×0.09×0.025) + `d435_joint` (fixed, parent=`wrist_2_link`, xyz="0 0 0.04")
  - [ ] Add D435 Gazebo RGBD plugin (topics: `/d435/image_raw`, `/d435/depth/image_raw`, `/d435/depth/points`, 640×480, 30 Hz, depth 0.1–10 m) inside `<xacro:if is_sim>`
- [ ] Create `worlds/greenhouse.sdf` (Harmonic-native plugin filenames):
  - [ ] Physics + UserCommands + SceneBroadcaster + Sensors system plugins
  - [ ] Sun light
  - [ ] Ground plane (100×100)
  - [ ] 4 perimeter walls (0.1 thick, 2.5 tall): N wall y=10, S wall y=-10, E wall x=10, W wall x=0
  - [ ] 3 roof beams (cylinder r=0.05, l=10): x=3, x=6, x=9, z=2.5
  - [ ] 5 plant rows (y ∈ {-3, -1.5, 0, 1.5, 3}), each with 8 stakes at x ∈ {1..8}:
    - [ ] Each stake: cylinder r=0.03, h=1.5, z=0.75
    - [ ] Each foliage: box 0.3×0.3×0.5, z=1.25
- [ ] Create `launch/sim.launch.py`:
  - [ ] `robot_state_publisher` node (`plaguebot_robot.urdf.xacro`, `use_sim_time:=true`, `is_sim:=true`)
  - [ ] `gz_sim` launch (`greenhouse.sdf`)
  - [ ] `ros_gz_sim/create` spawn at x=0.5, y=-5.0, z=0.30
  - [ ] `ros_gz_bridge/parameter_bridge` — all 9 topic pairs (see SPEC §3.2.4)
  - [ ] `TimerAction(3s)` — load `joint_state_broadcaster`
  - [ ] `TimerAction(5s)` — load `diff_drive_controller`
- [ ] Install meshes and launch files in `CMakeLists.txt`

### Verify Phase 2

- [ ] `colcon build --packages-select plaguebot_base_description plaguebot_description plaguebot_robot`
- [ ] `ros2 launch plaguebot_robot sim.launch.py` — Gazebo opens greenhouse, robot spawns
- [ ] `ros2 control list_controllers` — both controllers `active`
- [ ] `ros2 topic list` — confirm `/scan`, `/imu/data`, `/kiyo/image_raw`, `/d435/image_raw`, `/d435/depth/image_raw`, `/d435/depth/points`, `/odom` present
- [ ] Teleop test: robot drives through greenhouse corridors
- [ ] RViz: LaserScan visible on `/scan`, plant rows appear as obstacles
- [ ] Visualize unified URDF in RViz; validate arm attachment on top plate

---

## Phase 3: SLAM

- [ ] `sudo apt install ros-jazzy-slam-toolbox`
- [ ] Create package `plaguebot_slam` (ament_cmake)
- [ ] Write `config/slam_params.yaml` (online_async, `/scan`, `use_sim_time: true`, odom/map/base frames)
- [ ] Write `launch/slam.launch.py` (includes `sim.launch.py` + slam_toolbox node + RViz with map display)
- [ ] Drive robot through all 4 corridors with teleop until map is complete
- [ ] Save map: `ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'greenhouse'}}"` → `maps/greenhouse.yaml` + `maps/greenhouse.pgm`
- [ ] Verify map: all walls closed, plant row obstacles visible, corridors open and traversable

---

## Phase 4: Nav2

- [ ] `sudo apt install ros-jazzy-nav2-bringup ros-jazzy-robot-localization`
- [ ] Create package `plaguebot_nav` (ament_cmake)
- [ ] Write `config/nav2_params.yaml` (AMCL, BT navigator, planner, controller — standard nav2_bringup defaults as starting point)
- [ ] Write `config/ekf.yaml` (fuse `/odom` + `/imu/data` → `/odometry/filtered`, `use_sim_time: true`)
- [ ] Write `launch/nav2.launch.py` (includes `sim.launch.py` + `nav2_bringup` + `robot_localization` EKF node)
- [ ] Test: publish 2D Nav Goal in RViz → robot navigates to goal in greenhouse without hitting plant rows
- [ ] Write `plaguebot_nav/nav_arm_coordinator.py`:
  - [ ] On goal received → MoveIt2 `folded` state (action client to `/move_group`)
  - [ ] On goal succeeded → MoveIt2 `deploy` state
- [ ] Test: arm folds before navigation, deploys on arrival
- [ ] Refine `folded` and `deploy` joint values in MoveIt2 Setup Assistant using the unified URDF in RViz

---

## Phase 5: Mission Execution

### Setup

- [ ] `sudo apt install ros-jazzy-rosbridge-suite ros-jazzy-image-transport ros-jazzy-cv-bridge`
- [ ] `pip install ultralytics` (for NCNN model export)
- [ ] Export YOLOv8n to NCNN: `yolo export model=yolov8n.pt format=ncnn`

### `plaguebot_perception` package

- [ ] Create package (ament_python)
- [ ] Write `perception_node.py`:
  - [ ] Subscribes to `/d435/image_raw` + `/d435/depth/points`
  - [ ] Loads YOLOv8n NCNN model
  - [ ] Implements `/perception/detect` service (`std_srvs/srv/Trigger` or custom msg in `plaguebot_msgs`)
  - [ ] On service call: run inference on latest frame, reproject bbox centroid into PointCloud2 for 3D position
  - [ ] Returns detection list; publishes `/perception/detections` MarkerArray

### `plaguebot_mission` package

- [ ] Create package (ament_python)
- [ ] Write `mission_node.py` implementing the 8-state machine (IDLE → NAVIGATE → DEPLOY → SCAN → DETECT → IK_POSITION → RETURN → IDLE)
  - [ ] Subscribe to `/mission/start` (PoseStamped)
  - [ ] Nav2 `NavigateToPose` action client
  - [ ] MoveIt2 `MoveGroup` action client (for `folded` / `deploy` transitions)
  - [ ] JointTrajectory publisher for Scanning Routine (joint_1: -0.5 → +0.5 rad, 4s)
  - [ ] Service client for `/perception/detect`
  - [ ] MoveIt2 IK call (`ComputeIK`) on detection
- [ ] Write `launch/mission.launch.py` (includes `nav2.launch.py` + `rosbridge_websocket` + `mission_node` + `perception_node`)

### VR WebXR page

- [ ] Write `plaguebot_mission/web/index.html`:
  - [ ] Waypoint x/y number inputs
  - [ ] "Start Mission" button
  - [ ] roslib.js WebSocket connection to `ws://<robot-ip>:9090`
  - [ ] On click: publish `/mission/start` PoseStamped (map frame, stamp=now)
- [ ] Serve from robot: `python3 -m http.server 8080` in the web directory (or include in launch)
- [ ] Test: open page in Quest 3 browser, verify `/mission/start` appears in `ros2 topic echo`

### End-to-end test

- [ ] Full mission: Quest button press → robot navigates to plant row → arm deploys → Scanning Routine executes → detection logged → arm folds → robot returns
