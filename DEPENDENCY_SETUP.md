# OAK-D Driver Dependency Installation Guide

This guide provides multiple solutions to resolve the DepthAI dependency issue with ROS2.

## Problem
The OAK-D driver requires DepthAI to be accessible to the ROS2 Python environment, but DepthAI is installed in a virtual environment while ROS2 uses the system Python.

## Solutions (Choose One)

### Solution 1: Install DepthAI System-Wide (Recommended for Linux)

```bash
# Install system packages
sudo apt update
sudo apt install -y python3-pip python3-opencv python3-numpy

# Install ROS2 dependencies
sudo apt install -y ros-jazzy-cv-bridge ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-tf2-ros

# Install DepthAI system-wide
sudo pip3 install depthai numpy opencv-python
```

**Test:**
```bash
# Test DepthAI installation
python3 -c "import depthai as dai; print('DepthAI version:', dai.__version__)"

# Run the driver
ros2 launch oakd_driver oakd_driver.launch.py
```

### Solution 2: Use the Automated Installation Script

Run the provided installation script:
```bash
chmod +x install_system_deps.sh
./install_system_deps.sh
```

### Solution 3: Use Virtual Environment Wrapper

Use the wrapper script to run the node with your virtual environment:

```bash
# Update the VENV_PATH in the script to match your environment
chmod +x run_oakd_with_venv.sh

# Run directly
./run_oakd_with_venv.sh

# Or use with ROS2 (modify the script path as needed)
```

### Solution 4: Install in Current Environment

If you want to keep using your virtual environment for development:

```bash
# Activate your virtual environment
source ~/ros_ws/oakd_venv/bin/activate

# Install ROS2 packages in the venv (may require additional setup)
pip install rclpy sensor-msgs geometry-msgs tf2-ros cv-bridge

# Ensure DepthAI is installed
pip install depthai

# Build and install the package in the venv
cd ~/ros_ws
colcon build --packages-select oakd_driver
source install/setup.bash
```

### Solution 5: Use Alternative Launch File

Use the virtual environment launch file:
```bash
ros2 launch oakd_driver oakd_driver_venv.launch.py venv_path:=/path/to/your/oakd_venv
```

## Verification

After applying any solution, test with:

```bash
# Test DepthAI import
python3 -c "import depthai as dai; print('DepthAI version:', dai.__version__)"

# Test camera detection
python3 -c "import depthai as dai; print('Devices:', len(dai.Device.getAllAvailableDevices()))"

# Test ROS2 launch
ros2 launch oakd_driver oakd_driver.launch.py
```

## Troubleshooting

### If you get "ModuleNotFoundError: No module named 'depthai'":
- Ensure DepthAI is installed in the Python environment that ROS2 is using
- Check which Python ROS2 is using: `which python3`

### If you get permission errors:
- Use `sudo pip3 install` for system-wide installation
- Or use `pip install --user` for user-local installation

### If ROS2 packages are missing:
```bash
sudo apt install ros-jazzy-desktop-full
source /opt/ros/jazzy/setup.bash
```

## Recommended Approach

For most users, **Solution 1** (system-wide installation) is recommended as it:
- Ensures compatibility with ROS2
- Avoids virtual environment conflicts
- Provides the most reliable setup

Choose the virtual environment solutions if you need to maintain isolation for development purposes.