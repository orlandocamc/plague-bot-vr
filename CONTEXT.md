# Plague-Bot VR

An autonomous agricultural inspection robot that navigates a tomato greenhouse, deploys a 6-DOF arm, and uses a depth camera to detect pests and diseases on plants.

## Language

### Robot structure

**Mobile Base**:
The tracked, 4-wheel skid-steer chassis. Contains the LiDAR, RGB camera, IMU, and ESP32-C6 motor controller. The arm mounts on top of it.
_Avoid_: rover, platform, cart

**PROTON Arm**:
The 6-DOF serial manipulator arm mounted on the mobile base's top plate. Joints are named joint_1 through joint_6.
_Avoid_: robot arm, manipulator, end-effector arm

**Top Plate**:
The flat mounting surface on the mobile base, 0.15m above base_track_link, where the PROTON Arm attaches.
_Avoid_: mounting plate, top surface

**Digital Twin**:
The Gazebo simulation model of the physical robot. Collision meshes intentionally match the physical geometry and must not be changed without consulting the mechanical design.
_Avoid_: sim model, virtual robot

### Environment

**Greenhouse**:
The simulated and real operating environment: a 10×20m enclosed structure with rows of tomato plants.
_Avoid_: field, grow space, facility

**Plant Row**:
A linear arrangement of tomato plants (stakes + foliage) running along the greenhouse's long axis. The robot navigates between rows to reach inspection positions.
_Avoid_: row, crop row, plant line

**Corridor**:
The traversable space between two adjacent Plant Rows, wide enough for the Mobile Base (~1.2m between row centers, 0.36m robot width).
_Avoid_: aisle, path, lane

### Mission

**Mission**:
A single complete inspection cycle: receive a waypoint → navigate → deploy arm → scan → detect → return.
_Avoid_: task, job, run

**Waypoint**:
A PoseStamped in the map frame identifying the inspection position for a Mission, published to `/mission/start` from the VR interface.
_Avoid_: goal, target, destination

**Scanning Routine**:
The predefined joint trajectory executed during a Mission that sweeps joint_1 from -0.5 to +0.5 rad over 4 seconds, moving the D435 camera across a Plant Row.
_Avoid_: scan trajectory, arm sweep, inspection sweep

**Detection**:
A YOLO-identified anomaly (pest or disease marker) on a plant, with an associated 3D position derived from the D435 depth point cloud.
_Avoid_: finding, result, hit, observation

### Arm states

**Folded Pose**:
The PROTON Arm configuration used during navigation (joint_2 = -π/2, joint_3 = +π/2, others = 0). Keeps the arm compact and low while the robot drives. Starting point subject to refinement in MoveIt2 Setup Assistant.
_Avoid_: travel pose, stowed, retracted

**Deploy Pose**:
The PROTON Arm configuration at the start of a Scanning Routine (joint_5 = -0.5, others = 0). Positions the D435 to face the Plant Row.
_Avoid_: ready pose, scan pose, inspection pose

### Hardware

**ESP32-C6**:
The microcontroller on the Mobile Base responsible for motor control. Communicated with over serial by the `plaguebot_firmware` hardware interface.
_Avoid_: MCU, microcontroller, board

**D435**:
The Intel RealSense D435 RGBD camera mounted on wrist_2_link of the PROTON Arm. Provides RGB, depth image, and point cloud for Detection.
_Avoid_: depth camera, wrist camera, RealSense

**Kiyo**:
The Razer Kiyo RGB camera mounted on the Mobile Base chassis near the LiDAR. Used for chassis-level vision (not for Detection).
_Avoid_: RGB camera, front camera, chassis camera
