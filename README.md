# OAK-D Driver for ROS2 Jazzy

A ROS2 Jazzy driver package for the OAK-D camera from Luxonis, designed following the official luxonis/depthai-ros patterns. This package provides real-time point clouds, RGB images, and depth data from the OAK-D stereo camera system with comprehensive device management and configuration options.

## Features

- **Real-time Point Cloud Publishing**: High-quality point clouds from stereo depth calculation
- **RGB Image Stream**: Full-resolution color images from the main camera  
- **Depth Image Publishing**: Accurate depth maps with configurable range
- **Camera Info Publishing**: Complete camera calibration data from device EEPROM
- **Configurable Parameters**: Extensive parameter system following luxonis patterns
- **Device Management**: Robust device detection, connection, and error handling
- **Service Interfaces**: Start/stop camera services and calibration utilities
- **Diagnostic Monitoring**: Comprehensive health monitoring and error reporting
- **Multiple Connection Types**: Support for USB and PoE (Power over Ethernet) devices
- **IR Illumination Control**: Configurable laser dot and flood light brightness
- **Test Publisher**: Simulated data publisher for testing without hardware

## Hardware Requirements

- OAK-D camera from Luxonis (any variant: OAK-D, OAK-D-Lite, OAK-D Pro, etc.)
- USB 3.0 connection (recommended for full performance) or PoE network connection
- Compatible with Raspberry Pi 4, desktop systems, or embedded platforms

## Software Dependencies

### System Dependencies
```bash
# ROS2 Jazzy
sudo apt install ros-jazzy-desktop-full

# Additional ROS2 packages
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport
sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs
sudo apt install ros-jazzy-diagnostic-msgs ros-jazzy-vision-msgs
sudo apt install ros-jazzy-camera-info-manager ros-jazzy-std-srvs
```

### Python Dependencies
```bash
# Method 1: Using pip (in virtual environment or with --user flag)
pip install --user depthai>=2.20.0 opencv-python>=4.5.0 numpy>=1.20.0

# Method 2: Using requirements.txt
pip install --user -r requirements.txt

# Method 3: For system-wide installation on Ubuntu (if allowed)
sudo apt install python3-opencv python3-numpy
pip install --user depthai>=2.20.0

# Method 4: Using conda (if conda environment is active)
conda install opencv numpy
pip install depthai>=2.20.0
```

## Installation

### 1. Clone Repository
```bash
# Create workspace
mkdir -p ~/oakd_ws/src
cd ~/oakd_ws/src

# Clone this package
git clone <repository-url> oakd_driver

# Install dependencies
cd ~/oakd_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build package
colcon build --packages-select oakd_driver

# Source workspace
source install/setup.bash
```

### 2. Test Camera Connection
```bash
# Test DepthAI installation and detect devices
python3 scripts/device_detector.py --detect

# Test specific device connection
python3 scripts/device_detector.py --test --mx-id <your-device-mx-id>
```

## Usage

### Launch OAK-D Driver
```bash
# Standard launch with real camera
ros2 launch oakd_driver oakd_driver.launch.py

# With custom parameters
ros2 launch oakd_driver oakd_driver.launch.py camera_name:=my_oak fps:=15 rgb_resolution:=720p

# Connect to specific device
ros2 launch oakd_driver oakd_driver.launch.py mx_id:=<device-mx-id>

# Connect to PoE camera
ros2 launch oakd_driver oakd_driver.launch.py ip:=192.168.1.100

# Enable IR illumination
ros2 launch oakd_driver oakd_driver.launch.py enable_ir:=true
```

### Run Individual Nodes

#### Real Camera Node
```bash
ros2 run oakd_driver oakd_node
```

#### Test Publisher (No Hardware Required)
```bash
ros2 run oakd_driver oakd_publisher
```

### Service Interfaces
```bash
# Start camera
ros2 service call /oakd_node/start_camera std_srvs/srv/Trigger

# Stop camera  
ros2 service call /oakd_node/stop_camera std_srvs/srv/Trigger

# Save pipeline configuration
ros2 service call /oakd_node/save_pipeline std_srvs/srv/Trigger

# Save calibration data
ros2 service call /oakd_node/save_calibration std_srvs/srv/Trigger
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/oak/rgb/image_raw` | `sensor_msgs/Image` | RGB camera feed |
| `/oak/depth/image_raw` | `sensor_msgs/Image` | Depth image |
| `/oak/points` | `sensor_msgs/PointCloud2` | 3D point cloud |
| `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` | RGB camera calibration |
| `/oak/depth/camera_info` | `sensor_msgs/CameraInfo` | Depth camera calibration |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Device health monitoring |

## Parameters

### Device Connection Parameters
```yaml
oakd_node:
  ros__parameters:
    # Device connection (leave empty for auto-detection)
    i_mx_id: ""                    # MX ID of specific device
    i_ip: ""                       # IP address for PoE cameras  
    i_usb_port_id: ""             # USB port ID
    i_usb_speed: "SUPER_PLUS"     # HIGH, SUPER, SUPER_PLUS
```

### Camera Configuration Parameters
```yaml
    # Camera settings
    i_fps: 30                      # Camera FPS
    i_rgb_resolution: "1080p"      # 720p, 1080p, 4K
    i_depth_resolution: "720p"     # 400p, 720p, 800p
    
    # Stereo depth settings
    i_depth_confidence_threshold: 200    # 0-255
    i_depth_lr_check: true              # Left-right consistency check
    i_depth_subpixel: false             # Subpixel accuracy
    i_depth_extended_disparity: false   # Extended disparity range
    i_depth_preset_mode: "HIGH_ACCURACY" # HIGH_ACCURACY, HIGH_DENSITY
```

### Frame and Transform Parameters
```yaml
    # Frame configuration following luxonis patterns
    i_tf_camera_name: "oak"           # Camera name prefix
    i_tf_camera_model: ""            # Auto-detected if empty
    i_tf_base_frame: "base_link"     # Base frame
    i_tf_parent_frame: "oak_camera_frame" # Parent frame
    i_publish_tf_from_calibration: true   # Use device calibration for TF
```

### IR Illumination Parameters
```yaml
    # IR illumination control
    i_enable_ir: false               # Enable IR projectors
    i_laser_dot_brightness: 800      # 0-1200mA laser dot
    i_floodlight_brightness: 0       # 0-1500mA flood light
```

### Pipeline and Diagnostic Parameters  
```yaml
    # Pipeline configuration
    i_pipeline_type: "RGBD"          # Pipeline type
    i_pipeline_dump: false           # Save pipeline to file
    i_calibration_dump: false        # Save calibration to file
    i_external_calibration_path: ""  # External calibration file
    
    # Error handling
    i_restart_on_diagnostics_error: false # Auto-restart on errors
```

## Device Management

### Device Detection
```bash
# List all available devices
python3 scripts/device_detector.py --detect

# Test connection to specific device
python3 scripts/device_detector.py --test --mx-id 14442C10D1545A0D00
```

### Multiple Devices
```bash
# Launch multiple cameras with different namespaces
ros2 launch oakd_driver oakd_driver.launch.py camera_name:=oak1 mx_id:=<device1-id>
ros2 launch oakd_driver oakd_driver.launch.py camera_name:=oak2 mx_id:=<device2-id>
```

### PoE Camera Setup
```bash
# Configure network interface for PoE camera
sudo ip addr add 192.168.1.10/24 dev eth0

# Launch with IP connection
ros2 launch oakd_driver oakd_driver.launch.py ip:=192.168.1.100
```

## Coordinate Frames

Following ROS and luxonis conventions:
```
base_link
└── oak_camera_frame
    ├── oak_rgb_camera_frame
    │   └── oak_rgb_camera_optical_frame
    ├── oak_left_camera_frame  
    │   └── oak_left_camera_optical_frame (depth frame)
    └── oak_right_camera_frame
        └── oak_right_camera_optical_frame
```

## Calibration and Accuracy

This driver uses the factory calibration stored in the device EEPROM, providing:
- Accurate camera intrinsics for both RGB and stereo cameras
- Precise stereo baseline and rotation matrices
- Distortion coefficients for lens correction
- Proper rectification for stereo processing

## Performance Optimization

### For Raspberry Pi 4
```yaml
# Optimized settings for embedded systems
oakd_node:
  ros__parameters:
    i_fps: 15
    i_rgb_resolution: "720p"
    i_depth_resolution: "720p"
    i_depth_confidence_threshold: 230
```

### For Desktop/High-Performance Systems
```yaml
# High performance settings
oakd_node:
  ros__parameters:
    i_fps: 30
    i_rgb_resolution: "1080p"  
    i_depth_resolution: "720p"
    i_depth_subpixel: true
```

### For PoE Cameras (Bandwidth Limited)
```yaml
# Bandwidth-optimized settings
oakd_node:
  ros__parameters:
    i_fps: 20
    i_rgb_resolution: "720p"
    i_depth_resolution: "400p"
```

## Visualization with RViz2

```bash
# Start RViz2
ros2 run rviz2 rviz2

# Add displays:
# - Image: /oak/rgb/image_raw
# - Image: /oak/depth/image_raw  
# - PointCloud2: /oak/points
# - TF: Enable to see camera frames
```

## Troubleshooting

### Device Detection Issues
```bash
# Check device detection
python3 scripts/device_detector.py --detect

# Check USB permissions
sudo usermod -a -G plugdev $USER
# Log out and log back in

# For PoE cameras, check network
ping 192.168.1.100
```

### Performance Issues
- Use USB 3.0 ports for best performance
- Reduce resolution and FPS for resource-constrained systems
- Enable left-right check for better depth quality
- Adjust confidence threshold to filter noisy depth

### Service and Diagnostic Monitoring
```bash
# Monitor diagnostics
ros2 topic echo /diagnostics

# Check camera services
ros2 service list | grep oakd

# Monitor camera status
ros2 topic hz /oak/rgb/image_raw
```

## Development and Customization

### Parameter Updates
The driver supports dynamic parameter updates following luxonis patterns:
```bash
# Update IR brightness
ros2 param set /oakd_node i_laser_dot_brightness 400

# Change confidence threshold  
ros2 param set /oakd_node i_depth_confidence_threshold 150
```

### Custom Pipeline Development
Extend the driver by modifying the pipeline creation methods to add:
- Neural network inference
- Additional camera outputs
- Custom image processing

## Integration Examples

### With Navigation Stack
```yaml
# In nav2 params
obstacle_layer:
  plugin: "nav2_costmap_2d::VoxelLayer"
  observation_sources: oak_points
  oak_points:
    topic: /oak/points
    data_type: "PointCloud2"
    min_obstacle_height: 0.1
    max_obstacle_height: 2.0
```

### With SLAM Systems
```python
# Example integration with SLAM
import rclpy
from sensor_msgs.msg import PointCloud2, Image

class SLAMIntegration(Node):
    def __init__(self):
        super().__init__('slam_integration')
        self.pc_sub = self.create_subscription(
            PointCloud2, '/oak/points', self.pointcloud_cb, 10)
        self.rgb_sub = self.create_subscription(
            Image, '/oak/rgb/image_raw', self.image_cb, 10)
```

## Contributing

1. Follow luxonis coding patterns and conventions
2. Test with multiple device types (USB, PoE)
3. Ensure parameter naming follows `i_` prefix convention
4. Add comprehensive error handling and logging
5. Update documentation for new features

## License

MIT License - see LICENSE file for details

## Acknowledgments

- Based on patterns from [luxonis/depthai-ros](https://github.com/luxonis/depthai-ros)
- Follows ROS2 conventions and best practices
- Inspired by the official DepthAI ROS driver architecture

## Related Projects

- [DepthAI Core](https://github.com/luxonis/depthai-core) - Core DepthAI library
- [Official depthai-ros](https://github.com/luxonis/depthai-ros) - Official ROS driver
- [OAK-D Hardware](https://docs.luxonis.com/projects/hardware/) - Hardware documentation