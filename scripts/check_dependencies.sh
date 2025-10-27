#!/bin/bash

echo "=== Checking ROS2 Jazzy Installation ==="
source /opt/ros/jazzy/setup.bash

echo "=== Checking for required packages ==="
packages=("cv_bridge" "image_transport" "camera_info_manager" "tf2_ros" "diagnostic_msgs")

for pkg in "${packages[@]}"; do
    if ros2 pkg list | grep -q "^${pkg}$"; then
        echo "✓ $pkg found"
    else
        echo "✗ $pkg missing"
    fi
done

echo ""
echo "=== Checking OpenCV installation ==="
if pkg-config --exists opencv4; then
    echo "✓ OpenCV4 found: $(pkg-config --modversion opencv4)"
elif pkg-config --exists opencv; then
    echo "✓ OpenCV found: $(pkg-config --modversion opencv)"
else
    echo "✗ OpenCV not found"
fi

echo ""
echo "=== Checking DepthAI installation ==="
if ldconfig -p | grep -q libdepthai; then
    echo "✓ DepthAI library found"
else
    echo "✗ DepthAI library not found"
fi

echo ""
echo "=== Recommended installation commands ==="
echo "sudo apt update"
echo "sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-camera-info-manager"
echo "sudo apt install ros-jazzy-tf2-ros ros-jazzy-diagnostic-msgs"
echo "sudo apt install libopencv-dev"
echo ""
echo "For DepthAI:"
echo "wget -qO- https://raw.githubusercontent.com/luxonis/depthai/main/install_dependencies.sh | bash"
echo "sudo apt install libdepthai-dev"