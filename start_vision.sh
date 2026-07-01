#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/jada/plaguebot_vr_ws/install/setup.bash

echo "[Plaguebot] Iniciando cámaras + WebRTC server..."
ros2 launch plaguebot_vision webrtc_vision_launch.py &
VISION_PID=$!

echo "[Plaguebot] Esperando 8 segundos para que las cámaras inicien..."
sleep 8

echo "[Plaguebot] Iniciando LIDAR + Foxglove bridge..."
ros2 launch plaguebot_vision lidar_foxglove.launch.py &
LIDAR_PID=$!

echo "[Plaguebot] Todo listo. PIDs: vision=$VISION_PID lidar=$LIDAR_PID"

wait $VISION_PID $LIDAR_PID
