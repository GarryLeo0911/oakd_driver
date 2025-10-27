#!/bin/bash

# OAK-D Driver Wrapper Script
# This script activates the virtual environment and runs the OAK-D driver

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_PATH="$HOME/ros_ws/oakd_venv"  # Update this path to match your venv location

echo "Activating virtual environment: $VENV_PATH"

# Check if virtual environment exists
if [ ! -d "$VENV_PATH" ]; then
    echo "Error: Virtual environment not found at $VENV_PATH"
    echo "Please update the VENV_PATH in this script or create the virtual environment"
    exit 1
fi

# Activate virtual environment and run the node
source "$VENV_PATH/bin/activate"

echo "Running OAK-D driver node..."
python3 "$SCRIPT_DIR/oakd_driver/oakd_node.py" "$@"