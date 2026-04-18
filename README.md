# Plague-Bot VR

AI-guided robotic system for precision pest detection and targeted spraying in greenhouse tomato agriculture.

**Team:** Orlando Camacho · Eliot Calderón · Adán López  
**Advisor:** Mtro. Joel Arango  
**Institution:** Universidad Iberoamericana, Mexico City  
**Competition:** Huawei ICT Competition 2025–2026 — International Stage

---

## The Problem

Greenhouse tomato crops lose up to 40% of yield due to pest infestations. Conventional spraying applies pesticides uniformly across the entire crop — wasting chemicals, increasing costs, and harming the environment. Early detection is manual, slow, and error-prone.

**Target pests and diseases:** Spider mite · Whitefly · *Tuta absoluta* · Early blight

---

## Our Solution

Plague-Bot VR is a teleoperated robotic system that combines:

- Computer vision AI trained on Huawei Cloud ModelArts with MindSpore to detect pests in real time
- 6-DOF robotic arm with depth camera for precise localization and targeted spraying
- VR remote control via Meta Quest 3 for safe, intuitive operator interface
- WebRTC video streaming over 4G/5G for low-latency operation in greenhouses

---

## Demo Videos

| Demo | Link |
|---|---|
| Robot physical demo | Coming soon |
| RViz simulation + inverse kinematics | Coming soon |
| Orbbec → Meta Quest 3 streaming | Coming soon |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    OPERATOR (VR)                        │
│         Meta Quest 3 + Unity Digital Twin               │
└───────────────────────┬─────────────────────────────────┘
                        │ WebRTC (4G/5G)
┌───────────────────────▼─────────────────────────────────┐
│                  EDGE COMPUTING                         │
│     Raspberry Pi 5 + Hailo-8 AI Accelerator             │
│     YOLOv8 Inference → Pest Detection (up to 90 FPS)    │
└───────────────────────┬─────────────────────────────────┘
                        │ ROS 2 Humble / Jazzy
┌───────────────────────▼─────────────────────────────────┐
│                  ROBOT HARDWARE                         │
│   6-DOF Arm + Orbbec Depth Camera + Tracked Chassis     │
│   MoveIt! + KDL Kinematics Plugin                       │
└─────────────────────────────────────────────────────────┘
```

---

## AI Model — Huawei Cloud ModelArts + MindSpore

### Dataset

The dataset was compiled from multiple online sources, selecting only images relevant to common tomato pests. Web-scraped images were added and manually labeled with bounding box annotations. Data augmentation techniques (rotation, blur, high contrast) were applied, resulting in a robust dataset of **19,000 images**. 

The dataset is hosted on Roboflow as **[Jada_Tomato](https://universe.roboflow.com/eliotnecaxista21-gmail-com/jada_tomato-qiykq)**.

### Training Platform Comparison

| Platform | Epochs | Time |
|---|---|---|
| Huawei ModelArts | 50 | ~1 hour |
| Google Colab / Kaggle | 30 | ~2 hours |
| Local RTX 3050 | 30 | ~9 hours |

ModelArts significantly outperformed all alternatives in training speed and workflow efficiency, benefiting from tight integration with the Huawei ecosystem.

### Model Pipeline

- **Architecture:** YOLOv8 — selected for ease of deployment on hardware accelerators
- **Framework:** MindSpore / MindSpore Lite
- **Export pipeline:** `.pt` → ONNX → Hailo-compatible `.hef` format
- **Edge hardware:** Raspberry Pi 5 + Hailo-8 AI Kit (26 TOPS)
- **Inference speed:** up to 90 FPS on edge device
- **Classes:** Spider mite, Whitefly, *Tuta absoluta*, Early blight

---

## Robotic Arm — ROS 2

| Package | Description |
|---|---|
| `plaguebot_description` | URDF model, meshes, RViz configs |
| `plaguebot_controller` | C++ joint controllers |
| `plaguebot_moveit` | MoveIt! motion planning config |
| `plaguebot_msgs` | Custom ROS 2 messages and services |
| `plaguebot_utils` | Shared C++ utility library |

**Stack:** ROS 2 Humble · MoveIt! · KDL Kinematics Plugin · Orbbec depth camera in gripper

---

## Video Streaming — `src/plaguebot_vision`

ROS 2 package running on the Raspberry Pi that bridges the Orbbec Astra Pro Plus camera to WebRTC, streaming H.264 video to the Meta Quest 3 over Tailscale.

**Architecture:**
```
Orbbec Astra Pro Plus (USB)
        ↓
orbbec_camera_node (ROS 2)
        ↓ /color/image_raw
webrtc_bridge (aiortc + aiohttp)
        ↓ H.264 over Tailscale
Meta Quest 3 (Unity / Browser)
```

**Key specs:**
- Protocol: WebRTC with H.264 hardware decode on Quest 3
- Measured latency: ~8.7ms RTT on Universidad Iberoamericana network
- Framerate: ~29 FPS
- Networking: Tailscale VPN mesh between Pi and headset
- Auto-restart: systemd service on boot

**Run manually:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/plaguebot_ws/install/setup.bash
ros2 launch plaguebot_vision plaguebot.launch.py
```

---

## VR Interface — Unity + Meta Quest 3

- Digital twin of the robotic arm for real-time visualization
- Live camera feed from the Orbbec integrated in VR
- Operator controls for arm movement and spraying trigger
- Built with Unity + Meta XR SDK

---

## Reproduction Steps

### Prerequisites
- Ubuntu 22.04 / 24.04
- ROS 2 Humble (robot PC) / ROS 2 Jazzy (Raspberry Pi)
- Python 3.10+
- Raspberry Pi 5 with Hailo-8 AI Kit
- Orbbec Astra Pro Plus
- Meta Quest 3 + Unity 2022.3+

### 1. Clone the repository
```bash
git clone --recurse-submodules https://github.com/orlandocamc/plague-bot-vr.git
cd plague-bot-vr
```

### 2. Build the ROS 2 workspace
```bash
cd src
rosdep install --from-paths . --ignore-src -r -y
cd ..
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch the robot simulation
```bash
ros2 launch plaguebot_description display.launch.py
```

### 4. Run AI inference (on Raspberry Pi)
```bash
cd ai_model
python3 inference.py --source camera --model weights/best.hef
```

### 5. Start WebRTC streaming (on Raspberry Pi)
```bash
ros2 launch plaguebot_vision plaguebot.launch.py
# Stream available at http://<tailscale-ip>:8080
```

---

## Results

| Metric | Value |
|---|---|
| Inference speed (Hailo-8) | up to 90 FPS |
| Video latency (Ibero network) | ~8.7ms RTT |
| Streaming framerate | ~29 FPS |
| Dataset size | 19,000 images |
| Target pests detected | 4 classes |

---

## Tech Stack

| Layer | Technology |
|---|---|
| AI Training | Huawei Cloud ModelArts · MindSpore · YOLOv8 |
| Edge Inference | Raspberry Pi 5 · Hailo-8 (26 TOPS) · HailoRT |
| Robot Control | ROS 2 Humble · MoveIt! · KDL Kinematics Plugin |
| Streaming | ROS 2 Jazzy · WebRTC · aiortc · Tailscale |
| VR Interface | Unity · Meta Quest 3 · Meta XR SDK |

---

## Repository Structure

```
plague-bot-vr/
├── ai_model/              ← AI training & inference (submodule → Eliot Calderón)
├── src/
│   ├── plaguebot_description/   ← URDF, meshes, RViz
│   ├── plaguebot_controller/    ← C++ joint controllers
│   ├── plaguebot_moveit/        ← MoveIt! config
│   ├── plaguebot_msgs/          ← Custom messages & services
│   ├── plaguebot_utils/         ← Shared C++ utilities
│   └── plaguebot_vision/        ← WebRTC streaming (Orbbec → Quest 3)
└── README.md
```

---

## Team

| Name | Role |
|---|---|
| Orlando Camacho | Robotics & ROS 2 |
| Eliot Calderón | AI & Edge Computing |
| Adán López | Electronics & Hardware Assembly |
| Mtro. Joel Arango | Project Advisor |

---

## Related Repositories

- AI model and edge inference: [Eliotnecaxista21/Huawei](https://github.com/Eliotnecaxista21/Huawei)

---

## License

MIT License — see [LICENSE](LICENSE) for details.