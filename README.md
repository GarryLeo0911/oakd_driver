# OAK-D Driver for ROS2 Jazzy

A ROS2 Jazzy driver package for the OAK-D camera from Luxonis. This package provides real-time point clouds, RGB images, and depth data from the OAK-D stereo camera system.

## Features

- **Real-time Point Cloud Publishing**: High-quality point clouds from stereo depth calculation
- **RGB Image Stream**: Full-resolution color images from the main camera
- **Depth Image Publishing**: Accurate depth maps with configurable range
- **Camera Info Publishing**: Complete camera calibration data
- **Configurable Parameters**: Adjustable resolution, FPS, and depth settings
- **Test Publisher**: Simulated data publisher for testing without hardware

## Hardware Requirements

- OAK-D camera from Luxonis
- USB 3.0 connection (recommended for full performance)
- Compatible with Raspberry Pi 4 or desktop systems

## Software Dependencies

### System Dependencies
```bash
# ROS2 Jazzy
sudo apt install ros-jazzy-desktop-full

# Additional ROS2 packages
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport
sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs
```

### Python Dependencies
```bash
# DepthAI SDK for OAK-D camera
pip install depthai opencv-python numpy
```

## Installation

### 1. Clone Repository
```bash
# Create workspace
mkdir -p ~/oakd_ws
cd ~/oakd_ws

# Clone this package
git clone <repository-url> oakd_driver

# Build package
colcon build --packages-select oakd_driver

# Source workspace
source install/setup.bash
```

### 2. Test Camera Connection
```bash
# Test DepthAI installation
python3 -c "import depthai as dai; print('DepthAI version:', dai.__version__)"

# Test camera detection
python3 -c "import depthai as dai; print('Devices:', len(dai.Device.getAllAvailableDevices()))"
```

## Usage

### Launch OAK-D Driver
```bash
# Standard launch with real camera
ros2 launch oakd_driver oakd_driver.launch.py

# With custom parameters
ros2 launch oakd_driver oakd_driver.launch.py resolution:=720p fps:=15
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

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/oakd/rgb/image_raw` | `sensor_msgs/Image` | RGB camera feed |
| `/oakd/depth/image_raw` | `sensor_msgs/Image` | Depth image |
| `/oakd/points` | `sensor_msgs/PointCloud2` | 3D point cloud |
| `/oakd/rgb/camera_info` | `sensor_msgs/CameraInfo` | RGB camera calibration |
| `/oakd/depth/camera_info` | `sensor_msgs/CameraInfo` | Depth camera info |

## Parameters

### Camera Configuration (`config/oakd_params.yaml`)

```yaml
oakd_node:
  ros__parameters:
    # Camera settings
    rgb_resolution: "1080p"    # Options: 1080p, 720p, 480p
    fps: 30                    # Frames per second
    depth_enabled: true        # Enable depth processing
    
    # Depth settings
    min_depth: 0.3            # Minimum depth in meters
    max_depth: 10.0           # Maximum depth in meters
    depth_filter_enabled: true # Enable depth filtering
    
    # Point cloud settings
    point_cloud_enabled: true # Enable point cloud publishing
    max_points: 100000        # Maximum points in cloud
    
    # TF frames
    base_frame: "oakd_frame"
    rgb_frame: "oakd_rgb_frame"
    depth_frame: "oakd_depth_frame"
```

## Coordinate Frames

```
oakd_frame (base)
├── oakd_rgb_frame (RGB camera)
└── oakd_depth_frame (Depth/stereo baseline)
```

## Visualization with RViz2

```bash
# Start RViz2
ros2 run rviz2 rviz2

# Add displays:
# - Image: /oakd/rgb/image_raw
# - Image: /oakd/depth/image_raw  
# - PointCloud2: /oakd/points
```

## Integration Examples

### With Mapping Systems
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/oakd/points',
            self.point_cloud_callback,
            10)

    def point_cloud_callback(self, msg):
        # Process point cloud for mapping
        pass
```

### With Navigation
```yaml
# In nav2 params file
global_costmap:
  global_costmap:
    plugins: ["obstacle_layer"]
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      observation_sources: oakd_points
      oakd_points:
        topic: /oakd/points
        data_type: "PointCloud2"
```

## Performance Tuning

### For Raspberry Pi 4
```yaml
# Reduced settings for Pi
oakd_node:
  ros__parameters:
    rgb_resolution: "720p"
    fps: 15
    max_points: 50000
    depth_filter_enabled: true
```

### For Desktop Systems
```yaml
# High performance settings
oakd_node:
  ros__parameters:
    rgb_resolution: "1080p"
    fps: 30
    max_points: 200000
    depth_filter_enabled: false
```

## Troubleshooting

### Camera Not Detected
```bash
# Check USB connection
lsusb | grep "Movidius"

# Check permissions
sudo usermod -a -G dialout $USER
# Log out and back in

# Test with DepthAI
python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"
```

### No Point Cloud Data
- Ensure adequate lighting for stereo cameras
- Check depth range parameters
- Verify USB 3.0 connection for full bandwidth

### Performance Issues
- Reduce resolution and FPS
- Enable depth filtering
- Limit max points in point cloud
- Use USB 3.0 port

### Network Issues (Remote Operation)
```bash
# Check ROS2 discovery
ros2 node list

# Test topic publishing
ros2 topic hz /oakd/points
```

## Development

### Building from Source
```bash
# Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build with debugging
colcon build --packages-select oakd_driver --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Run tests
colcon test --packages-select oakd_driver
```

### Adding Custom Features
1. Fork this repository
2. Create feature branch
3. Add your modifications
4. Test thoroughly
5. Submit pull request

## API Reference

### OakdNode Class
- `__init__()`: Initialize camera and ROS2 interface
- `setup_pipeline()`: Configure DepthAI pipeline
- `publish_rgb()`: Publish RGB images
- `publish_depth()`: Publish depth images  
- `publish_points()`: Publish point clouds

### Configuration Options
- RGB resolution: 1080p, 720p, 480p
- Frame rates: 1-30 FPS
- Depth range: 0.1-35 meters
- Point cloud density: adjustable

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

MIT License - see LICENSE file for details

## Support

- **Issues**: Report bugs and request features via GitHub Issues
- **Documentation**: Check the official DepthAI documentation
- **Community**: ROS Discourse for general ROS2 questions

## Related Projects

- [DepthAI Core](https://github.com/luxonis/depthai-core) - Core DepthAI library
- [OAK-D Hardware](https://docs.luxonis.com/projects/hardware/en/latest/pages/BW1098OAK.html) - Hardware documentation
- [ROS2 Examples](https://github.com/ros2/examples) - ROS2 example packages

## Changelog

### Version 1.0.0
- Initial release
- Basic RGB, depth, and point cloud publishing
- Configurable camera parameters
- Test publisher for development