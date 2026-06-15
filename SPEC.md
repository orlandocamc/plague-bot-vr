# Plague-Bot VR — System Specification

## 1. Environment

- **OS**: Ubuntu 24.04
- **ROS**: ROS 2 Jazzy
- **Simulator**: Gazebo Harmonic (gz sim 8) — use `gz-sim-*` native plugin filenames throughout
- **Workspace**: `~/plaguebot_ws`

---

## 2. Package Architecture

```
src/
├── plaguebot_base_description/   # Mobile base URDF, chassis sensors (LiDAR, Kiyo, IMU), base ros2_control
├── plaguebot_description/        # PROTON Arm URDF (6-DOF), arm Gazebo config, arm ros2_control
├── plaguebot_robot/              # NEW: Unified URDF (base + arm), greenhouse world, sim launch
├── plaguebot_firmware/           # Real-robot hardware interface (serial → ESP32-C6)
├── plaguebot_arm_bridge/         # Real-robot arm serial bridge (ESP32 servo controller)
├── plaguebot_controller/         # Controller config and launch
├── plaguebot_moveit/             # MoveIt2 config for PROTON Arm
├── plaguebot_slam/               # NEW: slam_toolbox config and SLAM launch
├── plaguebot_nav/                # NEW: Nav2 config, EKF config, nav-arm coordinator
├── plaguebot_mission/            # NEW: Mission executor state machine
├── plaguebot_perception/         # NEW: YOLOv8n NCNN inference node
├── plaguebot_msgs/               # Custom message and service definitions
└── plaguebot_utils/              # Utility nodes
```

One package per concern. No monolithic launch packages.

---

## 3. Phase 2 — Full Simulation

### 3.1 Pre-requisite modifications to existing files

**`plaguebot_description/urdf/plaguebot.urdf.xacro`** — add `is_standalone` arg:
```xml
<xacro:arg name="is_standalone" default="true"/>
<!-- wrap ONLY these two elements: -->
<xacro:if value="$(arg is_standalone)">
  <link name="world"/>
  <joint name="virtual_joint" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
</xacro:if>
```
No other changes. Mesh collision geometry is untouched. See ADR-0001.

**`plaguebot_moveit/config/plaguebot.srdf`** — add two named states:
```xml
<group_state name="folded" group="arm">
  <joint name="joint_1" value="0"/>
  <joint name="joint_2" value="-1.5708"/>
  <joint name="joint_3" value="1.5708"/>
  <joint name="joint_4" value="0"/>
  <joint name="joint_5" value="0"/>
  <joint name="joint_6" value="0"/>
</group_state>

<group_state name="deploy" group="arm">
  <joint name="joint_1" value="0"/>
  <joint name="joint_2" value="0"/>
  <joint name="joint_3" value="0"/>
  <joint name="joint_4" value="0"/>
  <joint name="joint_5" value="-0.5"/>
  <joint name="joint_6" value="0"/>
</group_state>
```
Both are starting points subject to visual refinement in MoveIt2 Setup Assistant once the unified URDF is in RViz.

**`plaguebot_base_description/config/base_controllers.yaml`** — add documentation comment:
```yaml
# NOTE — wheel naming inversion (see docs/adr/0002-wheel-joint-naming-inversion.md):
# The URDF joint named "front_right_wheel_joint" is physically on the LEFT side of
# the chassis due to the original design. left_wheel_names / right_wheel_names are
# intentionally swapped to compensate. This is not a bug.
diff_drive_controller:
  ros__parameters:
    left_wheel_names:  ["front_right_wheel_joint", "rear_right_wheel_joint"]
    right_wheel_names: ["front_left_wheel_joint",  "rear_left_wheel_joint"]
    ...
```

### 3.2 New package: `plaguebot_robot`

Build type: `ament_cmake`. Dependencies: `plaguebot_base_description`, `plaguebot_description`, `plaguebot_moveit`.

#### 3.2.1 Unified URDF link tree

`urdf/plaguebot_robot.urdf.xacro` assembles the full robot:

```
base_footprint
└── base_track_link          (base_footprint_joint, z=+0.0762)
    ├── top_plate_link        (top_plate_joint, z=+0.15)           [already in base URDF]
    │   └── base_link [ARM]   (arm_mount_joint, fixed, xyz="0 0 0", is_standalone:=false)
    │       └── base_plate_link → upper_arm_link → elbow_1_link
    │           → forearm_link → wrist_1_link → wrist_2_link
    │               └── d435_link  (d435_joint, fixed, xyz="0 0 0.04")
    ├── laser                 (laser_joint)                         [already in base URDF]
    ├── kiyo_link             (kiyo_joint, fixed, xyz="-0.1 0.0 0.16")  [NEW in base URDF]
    └── imu_link              (imu_joint, fixed, xyz="0 0 0")           [NEW in base URDF]
```

The xacro passes `is_sim` to both included files and `is_standalone:=false` to the arm.

#### 3.2.2 Greenhouse world

`worlds/greenhouse.sdf` — SDF 1.9, Harmonic-native plugin filenames (`gz-sim-physics-system`, `gz-sim-user-commands-system`, `gz-sim-scene-broadcaster-system`, `gz-sim-sensors-system`).

World geometry (all static models):
| Element | Geometry | Position |
|---|---|---|
| Ground plane | plane 100×100 | z=0 |
| North wall | box 0.1 × 20.0 × 2.5 | x=5.0, y=10.0 |
| South wall | box 0.1 × 20.0 × 2.5 | x=5.0, y=-10.0 |
| East wall | box 10.0 × 0.1 × 2.5 | x=10.0, y=0 |
| West wall | box 10.0 × 0.1 × 2.5 | x=0.0, y=0 |
| Roof beam × 3 | cylinder r=0.05, l=10.0 | x∈{3,6,9}, y=0, z=2.5 |
| Plant stake (×40) | cylinder r=0.03, h=1.5 | 5 rows × 8 stakes; rows at y∈{-3,-1.5,0,1.5,3}; stakes at x∈{1..8} |
| Foliage box (×40) | box 0.3×0.3×0.5 | top of each stake, z=1.25 |

Robot spawn: `x=0.5, y=-5.0, z=0.30`.

#### 3.2.3 Sensor Gazebo plugins

Add to `plaguebot_base_description/urdf/plaguebot_base.urdf.xacro` (inside `<xacro:if value="$(arg is_sim)">`):

**RPLIDAR C1** (on existing `laser` link):
- Type: `gpu_lidar`
- Topic: `/scan`
- 360° horizontal, 10 Hz, range 0.15–12.0 m

**Razer Kiyo** (on new `kiyo_link`):
- Type: `camera`
- Topic: `/kiyo/image_raw`
- 1280×720 @ 30 Hz

**IMU** (on new `imu_link`):
- Type: `imu`
- Topic: `/imu/data`
- 100 Hz

Add to `plaguebot_robot/urdf/plaguebot_robot.urdf.xacro` (D435 sensor defined here, not in arm URDF):

**Intel RealSense D435** (on new `d435_link`, child of `wrist_2_link`):
- Type: `rgbd_camera`
- RGB topic: `/d435/image_raw`
- Depth topic: `/d435/depth/image_raw`
- PointCloud topic: `/d435/depth/points`
- 640×480 @ 30 Hz, depth range 0.1–10.0 m

#### 3.2.4 ros_gz_bridge topics

`launch/sim.launch.py` bridges:

| ROS topic | Message type | Direction |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | gz → ros |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ros → gz |
| `/odom` | `nav_msgs/msg/Odometry` | gz → ros |
| `/scan` | `sensor_msgs/msg/LaserScan` | gz → ros |
| `/imu/data` | `sensor_msgs/msg/Imu` | gz → ros |
| `/kiyo/image_raw` | `sensor_msgs/msg/Image` | gz → ros |
| `/d435/image_raw` | `sensor_msgs/msg/Image` | gz → ros |
| `/d435/depth/image_raw` | `sensor_msgs/msg/Image` | gz → ros |
| `/d435/depth/points` | `sensor_msgs/msg/PointCloud2` | gz → ros |

#### 3.2.5 Sim launch sequence

`launch/sim.launch.py`:
1. `robot_state_publisher` — `plaguebot_robot.urdf.xacro`, `use_sim_time:=true`
2. `gz_sim` — `greenhouse.sdf`
3. `ros_gz_sim/create` — spawn robot at defined pose
4. `ros_gz_bridge/parameter_bridge` — all 9 topic pairs
5. `TimerAction(3s)` — load `joint_state_broadcaster`
6. `TimerAction(5s)` — load `diff_drive_controller`

All nodes receive `use_sim_time: true`. The `is_sim:=true` arg is wired through to both URDF xacros so the Gazebo hardware plugin loads and the real hardware plugin does not.

### 3.3 Simulation wheel behavior

All 4 wheels are driven by `diff_drive_controller` in simulation. The real robot only commands the front pair; rear wheels follow mechanically. See ADR-0002.

---

## 4. Phase 3 — SLAM

New package: `plaguebot_slam` (ament_cmake).

- **Tool**: `slam_toolbox` online async mode
- **Config** (`config/slam_params.yaml`):
  - `mode: mapping`
  - `odom_frame: odom`, `map_frame: map`, `base_frame: base_footprint`
  - `scan_topic: /scan`
  - `use_sim_time: true`
- **Launch** (`launch/slam.launch.py`): includes `sim.launch.py` + slam_toolbox node + RViz
- **Map save**: `ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'greenhouse'}}"` → `plaguebot_slam/maps/greenhouse.{yaml,pgm}`

---

## 5. Phase 4 — Nav2

New package: `plaguebot_nav` (ament_cmake).

- **Stack**: `nav2_bringup` with pre-built greenhouse map
- **Localization**: AMCL using `plaguebot_slam/maps/greenhouse.yaml`
- **Pose fusion**: `robot_localization` EKF fusing `/odom` + `/imu/data` → `/odometry/filtered`
- **Launch** (`launch/nav2.launch.py`): includes `sim.launch.py` + nav2_bringup + EKF node

### 5.1 Nav2 arm coordinator

`plaguebot_nav/plaguebot_nav/nav_arm_coordinator.py`:
- On Nav2 **goal received** → MoveIt2 `move_group` call to `folded` named state
- On Nav2 **goal succeeded** → MoveIt2 call to `deploy` named state
- Uses `moveit_msgs/action/MoveGroup` action client

---

## 6. Phase 5 — Mission Execution

### 6.1 VR interface

- Package: `ros-jazzy-rosbridge-suite`
- Node: `rosbridge_websocket` on port 9090 (LAN-accessible from Quest 3)
- Quest 3 opens a WebXR HTML page (served from the robot or any LAN host)
- Page has a waypoint input (x, y) and a "Start Mission" button
- Button publishes to `/mission/start` (`geometry_msgs/msg/PoseStamped`, map frame) via rosbridge WebSocket JSON

### 6.2 Mission executor (`plaguebot_mission`)

Package type: `ament_python`. Subscribes to `/mission/start`.

State machine:

```
IDLE
  │  /mission/start received
  ▼
NAVIGATE  ← Nav2 NavigateToPose action goal sent; MoveIt2 → folded pose
  │  goal succeeded
  ▼
DEPLOY    ← MoveIt2 → deploy pose
  │  done
  ▼
SCAN      ← publish JointTrajectory: joint_1 sweeps -0.5 → +0.5 rad over 4s
  │  trajectory complete
  ▼
DETECT    ← service call /perception/detect
  │  detection returned
  ▼
IK_POSITION  ← compute gripper target from D435 PointCloud + YOLO bbox centroid
             ← MoveIt2 compute_ik → execute
  │  done (or no detection → skip)
  ▼
RETURN    ← MoveIt2 → folded pose; Nav2 → origin or next waypoint
  │
  ▼
IDLE
```

Launch: `launch/mission.launch.py` — includes `nav2.launch.py` + rosbridge_websocket + mission_node + perception_node.

### 6.3 Perception node (`plaguebot_perception`)

Package type: `ament_python`.

- Subscribes: `/d435/image_raw`, `/d435/depth/points`
- Model: YOLOv8n exported to NCNN format (`yolo export model=yolov8n.pt format=ncnn`)
- Inference: CPU via NCNN Python bindings (no Hailo, no MindSpore)
- Service: `/perception/detect` → returns list of detections with 3D position (bbox centroid reprojected into depth PointCloud)
- Publishes: `/perception/detections` (`visualization_msgs/msg/MarkerArray`)

---

## 7. Binding Constraints

| # | Constraint |
|---|---|
| 1 | `use_sim_time: true` for all simulation nodes |
| 2 | `is_sim:=true` → loads `gz_ros2_control/GazeboSimSystem`; `is_sim:=false` → loads `plaguebot_firmware/PlaguebotInterface` |
| 3 | Mesh collision geometry in `plaguebot_description/urdf/plaguebot.urdf.xacro` must not be modified (digital twin calibration) |
| 4 | One package per concern — no monolithic packages |
| 5 | AI inference: YOLOv8n via NCNN on CPU only |
| 6 | All 4 wheels driven in simulation; only front pair on real robot |
| 7 | Camera topics: `/kiyo/` namespace for chassis RGB; `/d435/` namespace for arm RGBD |

---

## 8. Additional APT Packages Required

```bash
sudo apt install \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization \
  ros-jazzy-rosbridge-suite \
  ros-jazzy-image-transport \
  ros-jazzy-cv-bridge

pip install ultralytics   # for yolo export to ncnn
# ncnn Python bindings: build from source or pip install ncnn
```
