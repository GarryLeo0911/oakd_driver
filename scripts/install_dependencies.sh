#!/bin/bash

echo "=== OAK-D Driver Build Fix Script ==="
echo "This script will install the missing dependencies for ROS2 Jazzy"

# Update package lists
echo "Updating package lists..."
sudo apt update

# Install ROS2 packages
echo "Installing ROS2 packages..."
sudo apt install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-camera-info-manager \
    ros-jazzy-tf2-ros \
    ros-jazzy-diagnostic-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs

# Install OpenCV development packages
echo "Installing OpenCV development packages..."
sudo apt install -y libopencv-dev

# Install DepthAI dependencies
echo "Installing DepthAI dependencies..."
wget -qO- https://raw.githubusercontent.com/luxonis/depthai/main/install_dependencies.sh | bash

# Try to install DepthAI from package manager first
echo "Attempting to install DepthAI library..."
if ! sudo apt install -y libdepthai-dev; then
    echo "Package installation failed. Building DepthAI from source..."
    
    # Create temporary directory
    cd /tmp
    
    # Clone and build DepthAI
    git clone --recursive https://github.com/luxonis/depthai-core.git
    cd depthai-core
    mkdir build && cd build
    
    # Configure and build
    cmake .. -DBUILD_SHARED_LIBS=ON
    make -j$(nproc)
    sudo make install
    sudo ldconfig
    
    # Clean up
    cd /tmp
    rm -rf depthai-core
fi

echo "=== Installation Complete ==="
echo "Now you can build the package with:"
echo "cd ~/ros_ws"
echo "source /opt/ros/jazzy/setup.bash"
echo "colcon build --packages-select oakd_driver"