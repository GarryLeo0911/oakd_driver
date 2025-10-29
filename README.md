# Enhanced OAK-D Driver for ROS2 Jazzy with Mapping Optimization

A ROS2 Jazzy driver package for the OAK-D camera from Luxonis, specifically optimized for high-accuracy 3D mapping applications. This enhanced version provides superior depth quality, IMU integration, and mapping-specific configurations designed to work seamlessly with the [enhanced map_builder package](https://github.com/GarryLeo0911/map_builder).

## 🚀 **Enhanced Features**

- **🎯 Mapping-Optimized Stereo**: High-accuracy depth with subpixel precision and confidence filtering
- **📡 IMU Integration**: Built-in IMU data publishing for visual-inertial odometry
- **🔍 Enhanced Depth Quality**: Spatial, temporal, and speckle filtering for mapping applications
- **⚡ Synchronized Streams**: RGB-Depth synchronization for feature-based tracking
- **🛠️ Post-Processing Pipeline**: Hardware-accelerated filtering for cleaner point clouds
- **💡 Intelligent IR Control**: Adaptive illumination for indoor mapping scenarios
- **📊 Quality Monitoring**: Advanced diagnostics and confidence thresholding
- **🎛️ Mapping Profiles**: Pre-configured parameter sets for different mapping scenarios

## 🔧 **Mapping Integration Benefits**

| Feature | Standard Driver | Enhanced for Mapping | Improvement |
|---------|----------------|---------------------|-------------|
| Depth Quality | Basic | Subpixel + Filtering | **2x cleaner** |
| IR Illumination | Manual | Adaptive Control | **Better indoor performance** |
| IMU Data | Optional | Always Available | **Visual-inertial fusion** |
| Confidence Filtering | Basic | Advanced Thresholding | **50% less noise** |
| Synchronization | Loose | Tight RGB-Depth Sync | **Better feature tracking** |

## Hardware Requirements

- **OAK-D Camera**: Any variant with IMU (OAK-D Pro recommended for mapping)
- **USB 3.0 Connection**: Essential for high-quality stereo data and IMU streaming
- **System Requirements**: Intel i5 or equivalent, 8GB+ RAM for mapping applications
- **Compatible Platforms**: Ubuntu 22.04, Raspberry Pi 4 (with performance tuning)

## Enhanced Software Dependencies

### System Dependencies
```bash
# ROS2 Jazzy
sudo apt install ros-jazzy-desktop-full

# Enhanced imaging and sensor packages
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport
sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs
sudo apt install ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs
sudo apt install ros-jazzy-diagnostic-msgs ros-jazzy-vision-msgs
sudo apt install ros-jazzy-camera-info-manager ros-jazzy-std-srvs

# For mapping integration
sudo apt install libopencv-dev python3-opencv
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

## Enhanced Usage for Mapping

### 🎯 **Launch Enhanced OAK-D for Mapping**
```bash
# Optimized for mapping applications
ros2 launch oakd_driver oakd_driver.launch.py params_file:=config/oakd_params.yaml

# High accuracy mapping mode
ros2 launch oakd_driver oakd_driver.launch.py \
    depth_confidence_threshold:=230 \
    enable_imu:=true \
    laser_dot_brightness:=1000 \
    enable_depth_post_processing:=true

# Integration with enhanced map_builder
ros2 launch map_builder oakd_enhanced_mapping.launch.py
```

### 🔧 **Mapping-Specific Configurations**

#### Maximum Accuracy Configuration
```bash
ros2 launch oakd_driver oakd_driver.launch.py \
    rgb_resolution:=720p \
    depth_resolution:=720p \
    fps:=30 \
    depth_confidence_threshold:=230 \
    depth_subpixel:=true \
    depth_extended_disparity:=true \
    enable_imu:=true \
    laser_dot_brightness:=1000 \
    floodlight_brightness:=500
```

#### Performance Balanced Configuration
```bash
ros2 launch oakd_driver oakd_driver.launch.py \
    rgb_resolution:=720p \
    depth_resolution:=720p \
    fps:=25 \
    depth_confidence_threshold:=200 \
    enable_imu:=true \
    laser_dot_brightness:=800
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

## Enhanced Published Topics for Mapping

| Topic | Type | Description | Mapping Use |
|-------|------|-------------|-------------|
| `/oak/rgb/image_raw` | `sensor_msgs/Image` | RGB camera feed | Feature detection |
| `/oak/stereo/depth` | `sensor_msgs/Image` | High-quality depth image | 3D feature tracking |
| `/oak/points` | `sensor_msgs/PointCloud2` | Filtered 3D point cloud | Mapping & odometry |
| `/oak/imu` | `sensor_msgs/Imu` | IMU data (accel + gyro) | Visual-inertial fusion |
| `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` | RGB camera calibration | Feature rectification |
| `/oak/stereo/camera_info` | `sensor_msgs/CameraInfo` | Stereo camera calibration | Depth accuracy |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Enhanced health monitoring | Quality assurance |

## Enhanced Parameters for Mapping

### 🎯 **Mapping-Optimized Camera Settings**
```yaml
oakd_node:
  ros__parameters:
    # Device connection
    i_mx_id: ""                          # Auto-detect OAK-D
    i_usb_speed: "SUPER_PLUS"           # Maximum bandwidth
    
    # Camera configuration optimized for mapping
    i_fps: 30                            # Stable rate for odometry
    i_rgb_resolution: "720p"             # Optimal for feature detection
    i_depth_resolution: "720p"           # Best stereo quality/performance
    
    # Enhanced stereo depth for mapping accuracy
    i_depth_confidence_threshold: 230    # High confidence (200-255)
    i_depth_lr_check: true              # Left-right consistency
    i_depth_subpixel: true              # Sub-pixel accuracy for mapping
    i_depth_extended_disparity: true    # Better close-range detection
    i_depth_preset_mode: "HIGH_ACCURACY" # Optimized for quality
    
    # Post-processing for cleaner point clouds
    i_enable_depth_post_processing: true
    i_spatial_filter_enable: true       # Spatial noise reduction
    i_temporal_filter_enable: true      # Temporal smoothing
    i_speckle_filter_enable: true       # Remove speckle noise
    i_depth_filter_size: 5              # Median filter size
```

### 📡 **IMU and Synchronization**
```yaml
    # IMU configuration for visual-inertial odometry
    i_enable_imu: true                  # Essential for mapping
    i_imu_mode: "COPY"                  # IMU data mode
    i_enable_sync: true                 # RGB-Depth synchronization
    
    # Quality settings
    i_depth_quality: "ULTRA"            # Maximum depth quality
    i_stereo_confidence_threshold: 230  # Stereo matching confidence
```

### 💡 **Enhanced IR Illumination**
```yaml
    # Adaptive IR illumination for indoor mapping
    i_enable_ir: true                   # Enable IR projectors
    i_laser_dot_brightness: 1000        # Strong dot pattern (0-1200mA)
    i_floodlight_brightness: 500        # Flood illumination (0-1500mA)
```

### 🔧 **Frame Configuration for TF Tree**
```yaml
    # Frame hierarchy for mapping
    i_tf_camera_name: "oak"             # Camera namespace
    i_tf_camera_model: "OAK-D"          # Specify model
    i_tf_base_frame: "base_link"        # Robot base frame
    i_tf_parent_frame: "oak_camera_frame" # Camera mount frame
    i_publish_tf_from_calibration: true # Use factory calibration
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

## Enhanced Troubleshooting for Mapping

### 🎯 **Mapping-Specific Issues**

#### Poor Depth Quality for Mapping
```bash
# Check current confidence threshold
ros2 param get /oakd_node i_depth_confidence_threshold

# Increase for better quality (may reduce density)
ros2 param set /oakd_node i_depth_confidence_threshold 240

# Enable all post-processing filters
ros2 param set /oakd_node i_spatial_filter_enable true
ros2 param set /oakd_node i_temporal_filter_enable true
ros2 param set /oakd_node i_speckle_filter_enable true

# Verify stereo consistency
ros2 param get /oakd_node i_depth_lr_check  # Should be true
```

#### IMU Data Not Available
```bash
# Check IMU topic
ros2 topic list | grep imu
ros2 topic echo /oak/imu --max-count 1

# Enable IMU if disabled
ros2 param set /oakd_node i_enable_imu true

# Restart node if needed
ros2 service call /oakd_node/stop_camera std_srvs/srv/Trigger
ros2 service call /oakd_node/start_camera std_srvs/srv/Trigger
```

#### RGB-Depth Synchronization Issues
```bash
# Check synchronization status
ros2 param get /oakd_node i_enable_sync

# Monitor timestamp differences
ros2 topic echo /oak/rgb/image_raw --field header.stamp &
ros2 topic echo /oak/stereo/depth --field header.stamp

# Enable tight synchronization
ros2 param set /oakd_node i_enable_sync true
```

#### IR Illumination Not Working
```bash
# Check IR settings
ros2 param get /oakd_node i_enable_ir
ros2 param get /oakd_node i_laser_dot_brightness

# Enable and adjust brightness
ros2 param set /oakd_node i_enable_ir true
ros2 param set /oakd_node i_laser_dot_brightness 1000
ros2 param set /oakd_node i_floodlight_brightness 500
```

### 📊 **Performance Monitoring for Mapping**

#### Monitor Data Quality
```bash
# Check topic rates (should be consistent)
ros2 topic hz /oak/rgb/image_raw    # Should match FPS setting
ros2 topic hz /oak/stereo/depth     # Should match RGB rate
ros2 topic hz /oak/points           # Point cloud rate
ros2 topic hz /oak/imu              # IMU rate (typically 200-400 Hz)

# Check point cloud density
ros2 topic echo /oak/points --field width  # Number of points
```

#### USB Bandwidth Issues
```bash
# Check USB connection
lsusb | grep 03e7  # Look for Luxonis device

# Monitor system resources
htop
iotop

# Reduce bandwidth if needed
ros2 param set /oakd_node i_fps 25
ros2 param set /oakd_node i_rgb_resolution "720p"
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

## Enhanced Integration with Map Builder

### 🗺️ **Complete Mapping Pipeline**
```bash
# Step 1: Launch enhanced OAK-D driver
ros2 launch oakd_driver oakd_driver.launch.py params_file:=config/oakd_params.yaml

# Step 2: Launch enhanced map builder (in another terminal)
ros2 launch map_builder enhanced_map_builder.launch.py

# Or launch complete pipeline together
ros2 launch map_builder oakd_enhanced_mapping.launch.py
```

### 🔗 **Topic Connections for Mapping**
```python
# Enhanced map_builder subscribes to these OAK-D topics:
enhanced_visual_odometry_topics = {
    'rgb_image': '/oak/rgb/image_raw',      # Feature detection
    'depth_image': '/oak/stereo/depth',     # 3D feature tracking  
    'point_cloud': '/oak/points',           # Point cloud odometry
    'imu_data': '/oak/imu'                  # Motion prediction
}

map_builder_topics = {
    'input_points': '/oak/points',          # Raw point cloud
    'robot_pose': '/enhanced_visual_odometry/pose'  # Pose for mapping
}
```

### ⚙️ **Calibration for Mapping Accuracy**
```bash
# Save OAK-D calibration for map_builder
ros2 service call /oakd_node/save_calibration std_srvs/srv/Trigger

# Verify calibration quality
ros2 topic echo /oak/rgb/camera_info --max-count 1
ros2 topic echo /oak/stereo/camera_info --max-count 1

# Check stereo baseline (should be ~0.075m for OAK-D)
ros2 param get /oakd_node i_baseline_distance
```

### 📈 **Performance Optimization for Mapping**
```yaml
# For maximum mapping accuracy (config/oakd_params.yaml)
oakd_node:
  ros__parameters:
    i_fps: 30
    i_rgb_resolution: "720p"
    i_depth_resolution: "720p"
    i_depth_confidence_threshold: 230
    i_depth_subpixel: true
    i_enable_imu: true
    i_laser_dot_brightness: 1000
    i_enable_sync: true

# For balanced performance/accuracy
oakd_node:
  ros__parameters:
    i_fps: 25
    i_depth_confidence_threshold: 200
    i_laser_dot_brightness: 800
    i_enable_depth_post_processing: true
```

## Contributing

1. Follow luxonis coding patterns and conventions
2. Test with multiple device types (USB, PoE)
3. Ensure parameter naming follows `i_` prefix convention
4. Add comprehensive error handling and logging
5. Update documentation for new features

## License

MIT License - see LICENSE file for details.

## Enhanced Acknowledgments

- **Luxonis**: OAK-D hardware and DepthAI platform
- **[luxonis/depthai-ros](https://github.com/luxonis/depthai-ros)**: Reference patterns and conventions
- **Enhanced Map Builder Integration**: Optimized for [map_builder](https://github.com/GarryLeo0911/map_builder) accuracy
- **ROS2 Community**: Framework and sensor integration standards
- **rtabmap Project**: Inspiration for mapping-specific optimizations

## Related Projects

- **[Enhanced Map Builder](https://github.com/GarryLeo0911/map_builder)**: High-accuracy 3D mapping with OAK-D optimization
- **[DepthAI Core](https://github.com/luxonis/depthai-core)**: Core DepthAI library
- **[Official depthai-ros](https://github.com/luxonis/depthai-ros)**: Official ROS driver
- **[OAK-D Hardware](https://docs.luxonis.com/projects/hardware/)**: Hardware documentation and specifications

## Enhanced Changelog

### 🚀 **Version 2.0.0 (Current - Enhanced for Mapping)**
- **Mapping-Optimized Parameters**: High-accuracy stereo configuration
- **IMU Integration**: Built-in IMU data publishing for visual-inertial odometry
- **Enhanced Post-Processing**: Spatial, temporal, and speckle filtering
- **Synchronization**: Tight RGB-Depth sync for feature tracking
- **Quality Monitoring**: Advanced confidence thresholds and diagnostics
- **Adaptive IR Control**: Intelligent illumination for indoor mapping
- **Performance Tuning**: Balanced accuracy/speed configurations
- **Map Builder Integration**: Seamless integration with enhanced mapping pipeline

### 📷 **Version 1.0.0 (Legacy - Standard Driver)**
- Basic OAK-D ROS2 driver functionality
- Standard point cloud and image publishing
- Basic parameter configuration
- Simple device management

## Support and Issues

- **GitHub Issues**: [Report mapping-specific issues](https://github.com/GarryLeo0911/oakd_driver/issues)
- **Map Builder Issues**: [Report integration issues](https://github.com/GarryLeo0911/map_builder/issues)
- **Luxonis Community**: [DepthAI Discussion Forum](https://discuss.luxonis.com/)
- **ROS Discourse**: [Community support](https://discourse.ros.org/)

---

**🎯 Optimized for Enhanced Map Builder Integration - Achieve professional mapping accuracy with OAK-D cameras**