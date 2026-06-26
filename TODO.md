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

- [x] `sudo apt install ros-jazzy-nav2-bringup ros-jazzy-robot-localization`
- [x] Create package `plaguebot_nav` (ament_cmake)
- [x] Write `config/nav2_params.yaml` (AMCL, BT navigator, planner, controller — switched MPPI → Regulated Pure Pursuit)
- [x] Write `config/ekf.yaml` — lives in **`plaguebot_localization`** package (deviation from SPEC §5, which placed it in plaguebot_nav)
- [x] Write `launch/nav.launch.py` (includes `sim.launch.py` + `nav2_bringup` localization+navigation + EKF + RViz; `headless` arg)
- [ ] Test: publish 2D Nav Goal in RViz → robot navigates to goal in greenhouse without hitting plant rows  ← runtime, not yet confirmed
- [~] nav-arm coordinator (§5.1) → **subsumed into `plaguebot_mission/mission_node.py`** (folds before NAVIGATE, deploys on arrival). No standalone coordinator node. See ADR decision.
- [x] Test: arm folds before navigation, deploys on arrival  ← verified in sim
- [x] Refine `deploy` joint values — tuned empirically vs sim depth cloud to
  `[0,-0.4,0,0,-0.6,0]` (D435 looks down at foliage row, ~49% valid depth,
  median ~0.6 m). Updated in mission_node default + SRDF `deploy` group_state.
- [ ] Refine `folded` joint values (current Z-fold is functional)

---

## Phase 5: Mission Execution

> **Perception backend change (ADR-0003):** SPEC's "YOLOv8n NCNN, no Hailo" is
> superseded. perception_node has a pluggable `backend`: `mock` (sim default),
> `torch`/`ncnn` on `best.pt` (dev), `hailo` on `best.hef` (real robot). SPEC §6.3
> and Constraint #5 updated.

### Setup

- [x] `image-transport` + `cv-bridge` already installed
- [x] `sudo apt install ros-jazzy-rosbridge-suite`
- [~] `ultralytics` + `ncnn` installing in workspace `.venv` (--system-site-packages); slow (torch). Background job.
- [~] Export model to NCNN — generic yolov8n export running to validate toolchain; **real export will use `best.pt`** once copied to `models/`

### `plaguebot_perception` package

- [x] Create package (ament_python)
- [x] `backends.py`: mock / torch / ncnn / hailo(stub) with lazy imports
- [x] `perception_node.py`: subscribes `/d435/image_raw` + `/d435/depth/points`; `/perception/detect` service (`plaguebot_msgs/srv/Detect`); reprojects bbox centroid into organized PointCloud2; publishes `/perception/detections` MarkerArray
- [x] `plaguebot_msgs`: added `Detection.msg` + `Detect.srv` (geometry_msgs dep)
- [x] `launch/perception.launch.py` (backend arg)
- [ ] Drop `best.pt` in `models/`, set `backend:=torch` (or `ncnn`), verify real inference
- [ ] Implement `HailoBackend.infer` on the Raspberry Pi against `best.hef`

### `plaguebot_mission` package

- [x] Create package (ament_python)
- [x] `mission_node.py`: 8-state machine (IDLE→NAVIGATE→DEPLOY→SCAN→DETECT→IK_POSITION→RETURN→IDLE)
  - [x] Subscribe to `/mission/start` (PoseStamped)
  - [x] Nav2 `NavigateToPose` action client
  - [x] Arm motions (folded/deploy/return) via `arm_controller` FollowJointTrajectory — **deviation from SPEC** (which used MoveGroup); fixed joint configs don't need planning, far more robust in sim
  - [x] Scanning Routine: joint_1 sweep -0.5 → +0.5 over 4s
  - [x] Service client for `/perception/detect`
  - [x] IK via MoveIt `/compute_ik` (best-effort) — VERIFIED with `use_moveit:=true`.
    move_group loads the unified robot model (via the `/robot_description` topic
    fallback; SRDF name mismatch is non-fatal). mission_node now transforms the
    detection point (frame `d435_link`) into `base_link` via tf2 before IK, and
    SCAN recenters to `deploy` so DETECT/IK run from a stable pose.
  - [x] FINDING: the PROTON arm reach (~0.5 m) < corridor standoff to the row
    (~0.6–0.75 m), so `/compute_ik` returns NO_IK_SOLUTION (-31) for real
    detections (only points ~at the wrist are reachable). IK_POSITION skips
    gracefully. DECISION NEEDED: drive the base closer before IK, or redefine
    IK_POSITION as a camera "look-at" (joint_1 yaw + wrist tilt) instead of a
    reach — inspection only needs to aim the camera, not touch the pest.
- [x] `launch/mission.launch.py` (includes `nav.launch.py` + rosbridge + http web server + perception + mission; `use_moveit` arg)

### VR WebXR page

- [x] `plaguebot_mission/web/index.html`: x/y inputs, Start button, roslib.js, reconnect, publishes `/mission/start` PoseStamped (map, stamp=now)
- [x] Served via `python3 -m http.server 8080` from mission.launch.py
- [ ] Test: open page in Quest 3 browser, verify `/mission/start` in `ros2 topic echo` (needs rosbridge installed)

### End-to-end test (all runtime — pending)

- [ ] `colcon build` full workspace + launch `mission.launch.py headless:=true`
- [ ] Mock mission in sim: button press → navigate → deploy → scan → mock detect → (IK) → fold → return
- [ ] Swap `backend:=torch` with `best.pt`; later `hailo` on the real robot
