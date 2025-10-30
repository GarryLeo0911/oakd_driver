# OAK-D Driver

This package provides a standalone ROS2 driver for OAK-D cameras that can run on a robot and publish camera data over the network.

## Features

- RGB and depth image publishing
- IMU data publishing  
- Visual-Inertial Odometry (VIO)
- Configurable camera parameters
- ROS2 Jazzy compatible

## Usage

### On the robot (where the OAK-D camera is connected):

```bash
# Build the package
colcon build --packages-select oakd_driver

# Source the workspace
source install/setup.bash

# Launch the camera driver
ros2 launch oakd_driver driver.launch.py name:=oak
```

### Parameters

- `name`: Camera node name (default: "oak")
- `params_file`: Path to camera configuration file
- `parent_frame`: Parent frame for camera transforms
- `use_rviz`: Whether to launch RViz for visualization

### Published Topics

- `/{name}/rgb/image_raw`: RGB camera images
- `/{name}/rgb/camera_info`: RGB camera info
- `/{name}/stereo/image_raw`: Depth images
- `/{name}/vio/odometry`: Visual-inertial odometry

## Network Configuration

Make sure both the robot and the laptop running map_builder are on the same ROS2 domain and can communicate over the network.

Set the same `ROS_DOMAIN_ID` on both machines:
```bash
export ROS_DOMAIN_ID=42
```

## Configuration

Edit `config/rtabmap.yaml` to modify camera settings:
- Frame rates
- Image resolution
- VIO parameters
- IMU settings