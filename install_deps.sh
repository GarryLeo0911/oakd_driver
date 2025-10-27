#!/bin/bash

# OAK-D Driver Installation Script
echo "Installing OAK-D Driver dependencies..."

# Check if we're in a virtual environment
if [[ "$VIRTUAL_ENV" != "" ]]; then
    echo "Virtual environment detected: $VIRTUAL_ENV"
    echo "Installing dependencies with pip..."
    pip install depthai opencv-python numpy
elif command -v pipx &> /dev/null; then
    echo "Using pipx for isolated installation..."
    pipx install depthai
    # Note: pipx doesn't handle opencv and numpy well, so we'll use --user
    pip install --user opencv-python numpy
else
    echo "Installing with --user flag..."
    pip install --user depthai opencv-python numpy
fi

# Install ROS2 dependencies if available
if command -v apt &> /dev/null; then
    echo "Installing system dependencies..."
    sudo apt update
    sudo apt install -y python3-opencv python3-numpy ros-humble-cv-bridge
fi

echo "Installation complete!"
echo "You may need to restart your terminal or run 'source ~/.bashrc'"