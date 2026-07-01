#!/bin/bash
set -e

echo "Starting RealSense installation for ROS 2 Jazzy on Ubuntu 24.04 (arm64/Raspberry Pi)..."

# Ensure universe repository is enabled
sudo apt-get update
sudo apt-get install -y software-properties-common curl git
sudo add-apt-repository universe -y

echo "Installing ROS 2 Jazzy RealSense packages and standard librealsense2..."
# We rely on the ROS and Ubuntu standard repositories rather than Intel's,
# which are often problematic on arm64/Noble.
if sudo apt-get install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-description librealsense2-utils librealsense2-dev; then
    echo "Installation via apt succeeded."
else
    echo "Apt installation failed. Building librealsense2 from source..."
    sudo apt-get install -y cmake libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
    
    cd /tmp
    rm -rf librealsense
    git clone https://github.com/IntelRealSense/librealsense.git
    cd librealsense
    mkdir build && cd build
    # FORCE_RSUSB_BACKEND=ON is recommended for arm64/Raspberry Pi to avoid kernel patching
    cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true -DFORCE_RSUSB_BACKEND=ON
    make -j$(nproc)
    sudo make install
    
    echo "librealsense2 installed from source."
    echo "Setting up ROS 2 wrappers from source..."
    WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    mkdir -p "$WS_DIR/src"
    cd "$WS_DIR/src"
    if [ ! -d "realsense-ros" ]; then
        git clone -b ros2-development https://github.com/IntelRealSense/realsense-ros.git
    else
        echo "realsense-ros already exists in $WS_DIR/src"
    fi
    echo "Please build the workspace using 'colcon build' afterwards."
fi

echo "Setting up udev rules for RealSense devices..."
sudo curl -s https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules -o /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "Install script finished successfully!"
