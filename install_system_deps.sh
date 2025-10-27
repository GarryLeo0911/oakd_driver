#!/bin/bash

echo "Installing OAK-D driver system dependencies..."

# Update package list
sudo apt update

# Install system packages
echo "Installing system packages..."
sudo apt install -y python3-pip python3-opencv python3-numpy

# Install ROS2 dependencies
echo "Installing ROS2 dependencies..."
sudo apt install -y ros-jazzy-cv-bridge ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-tf2-ros

# Install Python packages system-wide
echo "Installing Python packages system-wide..."
sudo pip3 install depthai numpy opencv-python

echo "Installation complete!"
echo ""
echo "You can now test with:"
echo "ros2 launch oakd_driver oakd_driver.launch.py"