# OAK-D Driver for ROS2 Jazzy

A comprehensive C++ ROS2 driver for OAK-D cameras by Luxonis, designed for ROS2 Jazzy. This driver provides RGB images, depth images, point clouds, and camera calibration data from OAK-D cameras.

## Features

- **C++ Implementation**: High-performance, native C++ implementation using DepthAI library
- **Full Camera Support**: RGB camera, stereo depth, and point cloud generation
- **ROS2 Jazzy Compatible**: Built specifically for ROS2 Jazzy distribution
- **Comprehensive Parameters**: Extensive configuration options following Luxonis patterns
- **Diagnostics**: Built-in diagnostic monitoring and error reporting
- **Multiple Connection Types**: USB, PoE, and specific device targeting
- **TF Publishing**: Automatic transform publishing with calibration data
- **Service Interface**: Start/stop camera and data saving services

## Installation

### Prerequisites

First, ensure you have ROS2 Jazzy installed:

```bash
# Install ROS2 Jazzy (if not already installed)
sudo apt update
sudo apt install ros-jazzy-desktop
```

### Install Dependencies

#### System Dependencies
```bash
# Install ROS2 dependencies
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-camera-info-manager
sudo apt install ros-jazzy-tf2-ros ros-jazzy-diagnostic-msgs

# Install OpenCV and USB development libraries
sudo apt install libopencv-dev libusb-1.0-0-dev build-essential cmake
```

#### DepthAI Library Installation

**Option 1: Install via Package Manager (Recommended)**
```bash
# Install DepthAI dependencies
wget -qO- https://raw.githubusercontent.com/luxonis/depthai/main/install_dependencies.sh | bash

# Install DepthAI library
sudo apt install libdepthai-dev
```

**Option 2: Build from Source**
```bash
# Clone and build DepthAI Core
git clone --recursive https://github.com/luxonis/depthai-core.git
cd depthai-core
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j$(nproc)
sudo make install
sudo ldconfig
```

### Build the Driver

```bash
# Create workspace (if you don't have one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone <your-repo-url> oakd_driver

# Build the package
cd ~/ros2_ws
colcon build --packages-select oakd_driver

# Source the workspace
source install/setup.bash
```

## Usage

### Basic Launch

```bash
# Launch with default parameters
ros2 launch oakd_driver oakd_driver.launch.py

# Launch with specific camera name
ros2 launch oakd_driver oakd_driver.launch.py camera_name:=my_oak

# Launch with specific device MXID
ros2 launch oakd_driver oakd_driver.launch.py mx_id:=18443010D1462C1200

# Launch with PoE camera via IP
ros2 launch oakd_driver oakd_driver.launch.py ip:=192.168.1.10
```

### Advanced Configuration

```bash
# High accuracy depth with IR illumination
ros2 launch oakd_driver oakd_driver.launch.py \
    depth_preset_mode:=HIGH_ACCURACY \
    enable_ir:=true \
    laser_dot_brightness:=800 \
    confidence_threshold:=200

# 4K RGB with high density depth
ros2 launch oakd_driver oakd_driver.launch.py \
    rgb_resolution:=4K \
    depth_preset_mode:=HIGH_DENSITY \
    fps:=15
```

## Topics Published

### Images
- `/{camera_name}/rgb/image_raw` (sensor_msgs/Image) - RGB camera feed
- `/{camera_name}/depth/image_raw` (sensor_msgs/Image) - Depth image in meters

### Point Cloud
- `/{camera_name}/points` (sensor_msgs/PointCloud2) - 3D point cloud

### Camera Info
- `/{camera_name}/rgb/camera_info` (sensor_msgs/CameraInfo) - RGB camera calibration
- `/{camera_name}/depth/camera_info` (sensor_msgs/CameraInfo) - Depth camera calibration

### Diagnostics
- `/diagnostics` (diagnostic_msgs/DiagnosticArray) - System diagnostics

## Services

- `~/start_camera` (std_srvs/Trigger) - Start camera capture
- `~/stop_camera` (std_srvs/Trigger) - Stop camera capture
- `~/save_pipeline` (std_srvs/Trigger) - Save current pipeline configuration
- `~/save_calibration` (std_srvs/Trigger) - Save camera calibration data

## Parameters

### Device Connection
- `i_mx_id` (string): Device MXID to connect to
- `i_ip` (string): IP address for PoE cameras
- `i_usb_port_id` (string): USB port identifier
- `i_usb_speed` (string): USB speed (HIGH, SUPER, SUPER_PLUS)

### Camera Settings
- `i_fps` (int): Camera frame rate (default: 30)
- `i_rgb_resolution` (string): RGB resolution (720p, 1080p, 4K)
- `i_depth_resolution` (string): Depth resolution (400p, 720p, 800p)

### Depth Configuration
- `i_depth_confidence_threshold` (int): Confidence threshold 0-255 (default: 200)
- `i_depth_lr_check` (bool): Enable left-right consistency check
- `i_depth_subpixel` (bool): Enable subpixel depth calculation
- `i_depth_extended_disparity` (bool): Enable extended disparity range
- `i_depth_preset_mode` (string): Depth preset (HIGH_ACCURACY, HIGH_DENSITY)

### Frame Configuration
- `i_tf_camera_name` (string): Camera name for topics and frames
- `i_tf_base_frame` (string): Base frame for transforms
- `i_publish_tf_from_calibration` (bool): Publish TF from calibration

### IR Illumination
- `i_enable_ir` (bool): Enable IR illumination
- `i_laser_dot_brightness` (int): Laser dot projector brightness (0-1200mA)
- `i_floodlight_brightness` (int): IR flood light brightness (0-1500mA)

## Troubleshooting

### Common Issues

**1. "DepthAI library not available"**
```bash
# Ensure DepthAI is properly installed
sudo apt install libdepthai-dev
# Or build from source as shown above
```

**2. "No devices detected"**
```bash
# Check USB permissions
sudo usermod -a -G plugdev $USER
# Log out and back in for changes to take effect

# Check if device is detected
lsusb | grep 03e7
```

**3. "Device is already booted in different process"**
```bash
# Kill any existing processes using the camera
pkill -f oakd
# Or restart your system
```

**4. Build errors with OpenCV**
```bash
# Ensure OpenCV development headers are installed
sudo apt install libopencv-dev
```

### Performance Optimization

**For better performance:**
- Use 720p depth resolution for real-time applications
- Enable subpixel depth only when needed (reduces FPS)
- Adjust confidence threshold based on your environment
- Use HIGH_DENSITY mode for dense point clouds (slower)
- Use HIGH_ACCURACY mode for precise measurements

### Multiple Cameras

To run multiple cameras simultaneously:

```bash
# Terminal 1 - First camera
ros2 launch oakd_driver oakd_driver.launch.py \
    camera_name:=oak1 mx_id:=YOUR_FIRST_CAMERA_ID

# Terminal 2 - Second camera  
ros2 launch oakd_driver oakd_driver.launch.py \
    camera_name:=oak2 mx_id:=YOUR_SECOND_CAMERA_ID
```

## Example Usage with RViz

```bash
# Launch the driver
ros2 launch oakd_driver oakd_driver.launch.py

# In another terminal, launch RViz
rviz2

# Add displays:
# - Image: /oak/rgb/image_raw
# - Image: /oak/depth/image_raw  
# - PointCloud2: /oak/points
# - TF: All frames
```

## Test Mode (No Hardware Required)

For testing without physical hardware, you can use the test publisher:

```bash
# Run test publisher node
ros2 run oakd_driver oakd_publisher_node

# This publishes simulated point cloud and pose data
```

## Integration with Other Packages

This driver is designed to work seamlessly with:

- **Navigation2**: For mobile robot navigation
- **MoveIt2**: For robotic manipulation
- **RTABMap**: For SLAM applications
- **OpenCV**: For computer vision processing
- **PCL**: For point cloud processing

## Differences from Python Version

This C++ implementation offers several advantages over Python versions:

1. **Performance**: Native C++ provides better performance
2. **Dependencies**: No Python virtual environment issues
3. **Memory**: Lower memory footprint
4. **Integration**: Better integration with ROS2 ecosystem
5. **Reliability**: More stable for long-running applications

## Contributing

Please follow ROS2 C++ coding standards and ensure all code passes linting:

```bash
# Check code formatting
ament_cpplint src/
```

## License

MIT License - see LICENSE file for details.

## Support

For issues specific to this driver, please open an issue on the repository.
For DepthAI library issues, refer to the [official Luxonis documentation](https://docs.luxonis.com/).

---

*This driver is based on the official Luxonis depthai-ros implementation and adapted for specific use cases.*