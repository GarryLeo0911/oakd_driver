#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import subprocess
import sys
import os
import time
from threading import Lock

# Optional imports - will gracefully handle missing dependencies
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    
try:
    import depthai as dai
    DEPTHAI_AVAILABLE = True
except ImportError:
    DEPTHAI_AVAILABLE = False
    
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False
    
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

import sensor_msgs_py.point_cloud2 as pc2
from builtin_interfaces.msg import Time


class OAKDNode(Node):
    def __init__(self):
        super().__init__('oakd_node')
        
        # State management
        self.camera_running = False
        self.device = None
        self.pipeline = None
        self.queues = {}
        self.calibration_data = None
        self.device_info = {}
        self.startup_lock = Lock()
        
        # Check for required dependencies
        self._check_dependencies()
        
        # Declare and get parameters
        self._declare_parameters()
        self._get_parameters()
        
        # Publishers
        self._setup_publishers()
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Services
        self._setup_services()
        
        # Diagnostics
        self._setup_diagnostics()
        
        # Start camera with delay to allow node to fully initialize
        self.create_timer(1.0, self._delayed_start, clock=self.get_clock())
        
        self.get_logger().info('OAK-D driver node initialized')

    def _check_dependencies(self):
        """Check for required dependencies"""
        missing_deps = []
        if not CV2_AVAILABLE:
            missing_deps.append("opencv-python")
        if not DEPTHAI_AVAILABLE:
            missing_deps.append("depthai")
        if not NUMPY_AVAILABLE:
            missing_deps.append("numpy")
        if not CV_BRIDGE_AVAILABLE:
            missing_deps.append("cv_bridge")
            
        if missing_deps:
            self.get_logger().error(
                f"Missing required dependencies: {', '.join(missing_deps)}"
            )
            self.get_logger().error(
                "Please install missing dependencies with:"
            )
            self.get_logger().error(
                f"pip install {' '.join(missing_deps)}"
            )
            self.get_logger().error(
                "Or if using ROS2, install via apt:"
            )
            self.get_logger().error(
                "sudo apt install python3-opencv python3-numpy ros-jazzy-cv-bridge"
            )
            self.get_logger().error(
                "For depthai: pip install depthai"
            )
            raise RuntimeError(f"Missing dependencies: {', '.join(missing_deps)}")

    def _declare_parameters(self):
        """Declare ROS parameters following luxonis patterns"""
        # Device connection parameters
        self.declare_parameter('i_mx_id', '')
        self.declare_parameter('i_ip', '')
        self.declare_parameter('i_usb_port_id', '')
        self.declare_parameter('i_usb_speed', 'SUPER_PLUS')  # SUPER, SUPER_PLUS, HIGH
        
        # Camera parameters
        self.declare_parameter('i_fps', 30)
        self.declare_parameter('i_rgb_resolution', '1080p')  # 720p, 1080p, 4K
        self.declare_parameter('i_depth_resolution', '720p')
        
        # Stereo depth parameters
        self.declare_parameter('i_depth_confidence_threshold', 200)
        self.declare_parameter('i_depth_lr_check', True)
        self.declare_parameter('i_depth_subpixel', False)
        self.declare_parameter('i_depth_extended_disparity', False)
        self.declare_parameter('i_depth_preset_mode', 'HIGH_ACCURACY')  # HIGH_ACCURACY, HIGH_DENSITY
        
        # Frame parameters
        self.declare_parameter('i_tf_camera_name', 'oak')
        self.declare_parameter('i_tf_camera_model', '')
        self.declare_parameter('i_tf_base_frame', 'base_link')
        self.declare_parameter('i_tf_parent_frame', 'oak_camera_frame')
        self.declare_parameter('i_publish_tf_from_calibration', True)
        
        # IR parameters
        self.declare_parameter('i_enable_ir', False)
        self.declare_parameter('i_laser_dot_brightness', 800)  # 0-1200mA
        self.declare_parameter('i_floodlight_brightness', 0)   # 0-1500mA
        
        # Pipeline parameters
        self.declare_parameter('i_pipeline_type', 'RGBD')
        self.declare_parameter('i_pipeline_dump', False)
        self.declare_parameter('i_calibration_dump', False)
        self.declare_parameter('i_external_calibration_path', '')
        
        # Restart parameters
        self.declare_parameter('i_restart_on_diagnostics_error', False)

    def _get_parameters(self):
        """Get parameter values"""
        # Device connection
        self.mx_id = self.get_parameter('i_mx_id').get_parameter_value().string_value
        self.ip = self.get_parameter('i_ip').get_parameter_value().string_value
        self.usb_port_id = self.get_parameter('i_usb_port_id').get_parameter_value().string_value
        self.usb_speed_str = self.get_parameter('i_usb_speed').get_parameter_value().string_value
        
        # Camera settings
        self.fps = self.get_parameter('i_fps').get_parameter_value().integer_value
        self.rgb_resolution = self.get_parameter('i_rgb_resolution').get_parameter_value().string_value
        self.depth_resolution = self.get_parameter('i_depth_resolution').get_parameter_value().string_value
        
        # Stereo depth
        self.confidence_threshold = self.get_parameter('i_depth_confidence_threshold').get_parameter_value().integer_value
        self.lr_check = self.get_parameter('i_depth_lr_check').get_parameter_value().bool_value
        self.subpixel = self.get_parameter('i_depth_subpixel').get_parameter_value().bool_value
        self.extended_disparity = self.get_parameter('i_depth_extended_disparity').get_parameter_value().bool_value
        self.depth_preset_mode = self.get_parameter('i_depth_preset_mode').get_parameter_value().string_value
        
        # Frame parameters
        self.camera_name = self.get_parameter('i_tf_camera_name').get_parameter_value().string_value
        self.camera_model = self.get_parameter('i_tf_camera_model').get_parameter_value().string_value
        self.base_frame = self.get_parameter('i_tf_base_frame').get_parameter_value().string_value
        self.parent_frame = self.get_parameter('i_tf_parent_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('i_publish_tf_from_calibration').get_parameter_value().bool_value
        
        # IR settings
        self.enable_ir = self.get_parameter('i_enable_ir').get_parameter_value().bool_value
        self.laser_dot_brightness = self.get_parameter('i_laser_dot_brightness').get_parameter_value().integer_value
        self.floodlight_brightness = self.get_parameter('i_floodlight_brightness').get_parameter_value().integer_value
        
        # Pipeline settings
        self.pipeline_type = self.get_parameter('i_pipeline_type').get_parameter_value().string_value
        self.pipeline_dump = self.get_parameter('i_pipeline_dump').get_parameter_value().bool_value
        self.calibration_dump = self.get_parameter('i_calibration_dump').get_parameter_value().bool_value
        self.external_calibration_path = self.get_parameter('i_external_calibration_path').get_parameter_value().string_value
        
        # Other settings
        self.restart_on_error = self.get_parameter('i_restart_on_diagnostics_error').get_parameter_value().bool_value

    def _setup_publishers(self):
        """Setup ROS publishers"""
        self.rgb_pub = self.create_publisher(Image, f'{self.camera_name}/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, f'{self.camera_name}/depth/image_raw', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, f'{self.camera_name}/points', 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, f'{self.camera_name}/rgb/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, f'{self.camera_name}/depth/camera_info', 10)

    def _setup_services(self):
        """Setup ROS services following luxonis patterns"""
        self.start_srv = self.create_service(Trigger, '~/start_camera', self._start_camera_srv)
        self.stop_srv = self.create_service(Trigger, '~/stop_camera', self._stop_camera_srv)
        self.save_pipeline_srv = self.create_service(Trigger, '~/save_pipeline', self._save_pipeline_srv)
        self.save_calib_srv = self.create_service(Trigger, '~/save_calibration', self._save_calibration_srv)

    def _setup_diagnostics(self):
        """Setup diagnostic monitoring"""
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.diagnostics_timer = self.create_timer(1.0, self._publish_diagnostics)

    def _delayed_start(self):
        """Start camera after a delay to ensure node is fully initialized"""
        self._delayed_start.cancel()  # Cancel the timer
        self.start_camera()

    def start_camera(self):
        """Start the camera following luxonis patterns"""
        with self.startup_lock:
            if self.camera_running:
                self.get_logger().info('Camera already running!')
                return
                
            self.get_logger().info('Starting camera...')
            
            try:
                self._detect_and_connect_device()
                self._setup_pipeline()
                self._start_device()
                self._setup_queues()
                self._configure_ir()
                
                # Start processing timer
                self.processing_timer = self.create_timer(1.0 / self.fps, self._process_frames)
                
                self.camera_running = True
                self.get_logger().info('Camera started successfully!')
                
            except Exception as e:
                self.get_logger().error(f'Failed to start camera: {str(e)}')
                self.stop_camera()
                raise

    def stop_camera(self):
        """Stop the camera"""
        with self.startup_lock:
            if not self.camera_running:
                self.get_logger().info('Camera already stopped!')
                return
                
            self.get_logger().info('Stopping camera...')
            
            try:
                # Stop processing timer
                if hasattr(self, 'processing_timer'):
                    self.processing_timer.cancel()
                
                # Close queues
                self.queues.clear()
                
                # Close device
                if self.device:
                    self.device.close()
                    self.device = None
                
                self.pipeline = None
                self.camera_running = False
                
                self.get_logger().info('Camera stopped successfully!')
                
            except Exception as e:
                self.get_logger().error(f'Error stopping camera: {str(e)}')

    def _detect_and_connect_device(self):
        """Detect and connect to DepthAI device following luxonis patterns"""
        usb_speed_map = {
            'HIGH': dai.UsbSpeed.HIGH,
            'SUPER': dai.UsbSpeed.SUPER,
            'SUPER_PLUS': dai.UsbSpeed.SUPER_PLUS
        }
        usb_speed = usb_speed_map.get(self.usb_speed_str, dai.UsbSpeed.SUPER_PLUS)
        
        connected = False
        while not connected and rclpy.ok():
            try:
                if not self.mx_id and not self.ip and not self.usb_port_id:
                    # Connect to any available device
                    self.get_logger().info('No device ID specified, connecting to next available device...')
                    device_info = dai.Device.getAnyAvailableDevice()
                    self.device = dai.Device(device_info[1], usb_speed)
                    connected = True
                else:
                    # Connect to specific device
                    available_devices = dai.Device.getAllAvailableDevices()
                    
                    if not available_devices and self.ip:
                        # Try direct IP connection
                        self.get_logger().info(f'No devices detected, trying direct IP connection: {self.ip}')
                        device_info = dai.DeviceInfo(self.ip)
                        available_devices = [device_info]
                    
                    if not available_devices:
                        raise RuntimeError('No devices detected!')
                    
                    for device_info in available_devices:
                        if self.mx_id and device_info.getMxId() == self.mx_id:
                            self._connect_device(device_info, usb_speed, f'MXID: {self.mx_id}')
                            connected = True
                            break
                        elif self.ip and device_info.name == self.ip:
                            self._connect_device(device_info, usb_speed, f'IP: {self.ip}')
                            connected = True
                            break
                        elif self.usb_port_id and device_info.name == self.usb_port_id:
                            self._connect_device(device_info, usb_speed, f'USB ID: {self.usb_port_id}')
                            connected = True
                            break
                
                if connected:
                    # Get device information
                    self.device_info = {
                        'mxid': self.device.getMxId(),
                        'name': self.device.getDeviceInfo().name,
                        'device_name': self.device.getDeviceName(),
                        'usb_speed': self.device.getUsbSpeed() if self.device.getDeviceInfo().getXLinkDeviceDesc().protocol != dai.XLinkProtocol.X_LINK_TCP_IP else None
                    }
                    
                    # Set camera model if not specified
                    if not self.camera_model:
                        self.camera_model = self.device.getDeviceName()
                    
                    self.get_logger().info(f'Connected to device: {self.device_info["device_name"]} (MXID: {self.device_info["mxid"]})')
                    
                    # Get calibration data
                    self.calibration_data = self.device.readCalibration()
                    
                    # Log camera sensors
                    for socket, sensor_name in self.device.getCameraSensorNames().items():
                        self.get_logger().debug(f'Socket {socket}: {sensor_name}')
                
            except Exception as e:
                self.get_logger().error(f'Device connection failed: {str(e)}')
                time.sleep(1.0)
                
        if not connected:
            raise RuntimeError('Failed to connect to any device')

    def _connect_device(self, device_info, usb_speed, description):
        """Connect to a specific device"""
        self.get_logger().info(f'Connecting to device using {description}')
        
        if device_info.state in [dai.XLinkDeviceState.X_LINK_UNBOOTED, dai.XLinkDeviceState.X_LINK_BOOTLOADER]:
            if device_info.name.startswith('192.168') or device_info.name.startswith('10.') or device_info.name.startswith('172.'):
                # IP device
                self.device = dai.Device(device_info)
            else:
                # USB device
                self.device = dai.Device(device_info, usb_speed)
        elif device_info.state == dai.XLinkDeviceState.X_LINK_BOOTED:
            raise RuntimeError('Device is already booted in different process')
        else:
            raise RuntimeError(f'Unknown device state: {device_info.state}')

    def _setup_pipeline(self):
        """Setup DepthAI pipeline"""
        self.get_logger().info('Setting up DepthAI pipeline...')
        
        # Create pipeline
        self.pipeline = dai.Pipeline()
        
        # Load external calibration if specified
        if self.external_calibration_path:
            self._load_external_calibration()
        
        # Create nodes
        self._create_camera_nodes()
        self._create_stereo_node()
        self._setup_outputs()
        
        if self.pipeline_dump:
            self._save_pipeline()
            
        if self.calibration_dump:
            self._save_calibration()

    def _create_camera_nodes(self):
        """Create camera nodes in pipeline"""
        # RGB camera
        self.cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        self.cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.cam_rgb.setResolution(self._get_rgb_resolution())
        self.cam_rgb.setInterleaved(False)
        self.cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        self.cam_rgb.setFps(self.fps)
        
        # Mono cameras for stereo
        self.mono_left = self.pipeline.create(dai.node.MonoCamera)
        self.mono_left.setResolution(self._get_mono_resolution())
        self.mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        
        self.mono_right = self.pipeline.create(dai.node.MonoCamera)
        self.mono_right.setResolution(self._get_mono_resolution())
        self.mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    def _create_stereo_node(self):
        """Create stereo depth node"""
        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        
        # Set preset mode
        preset_map = {
            'HIGH_ACCURACY': dai.node.StereoDepth.PresetMode.HIGH_ACCURACY,
            'HIGH_DENSITY': dai.node.StereoDepth.PresetMode.HIGH_DENSITY
        }
        preset = preset_map.get(self.depth_preset_mode, dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        self.stereo.setDefaultProfilePreset(preset)
        
        # Configure stereo
        self.stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        self.stereo.setLeftRightCheck(self.lr_check)
        self.stereo.setSubpixel(self.subpixel)
        self.stereo.setExtendedDisparity(self.extended_disparity)
        self.stereo.initialConfig.setConfidenceThreshold(self.confidence_threshold)
        
        # Link cameras to stereo
        self.mono_left.out.link(self.stereo.left)
        self.mono_right.out.link(self.stereo.right)

    def _setup_outputs(self):
        """Setup pipeline outputs"""
        # RGB output
        self.rgb_out = self.pipeline.create(dai.node.XLinkOut)
        self.rgb_out.setStreamName("rgb")
        self.cam_rgb.isp.link(self.rgb_out.input)
        
        # Depth output
        self.depth_out = self.pipeline.create(dai.node.XLinkOut)
        self.depth_out.setStreamName("depth")
        self.stereo.depth.link(self.depth_out.input)

    def _get_rgb_resolution(self):
        """Get RGB camera resolution"""
        resolution_map = {
            '720p': dai.ColorCameraProperties.SensorResolution.THE_720_P,
            '1080p': dai.ColorCameraProperties.SensorResolution.THE_1080_P,
            '4K': dai.ColorCameraProperties.SensorResolution.THE_4_K
        }
        return resolution_map.get(self.rgb_resolution, dai.ColorCameraProperties.SensorResolution.THE_1080_P)

    def _get_mono_resolution(self):
        """Get mono camera resolution"""
        resolution_map = {
            '720p': dai.MonoCameraProperties.SensorResolution.THE_720_P,
            '800p': dai.MonoCameraProperties.SensorResolution.THE_800_P,
            '400p': dai.MonoCameraProperties.SensorResolution.THE_400_P
        }
        return resolution_map.get(self.depth_resolution, dai.MonoCameraProperties.SensorResolution.THE_720_P)

    def _start_device(self):
        """Start the device pipeline"""
        self.device.startPipeline(self.pipeline)

    def _setup_queues(self):
        """Setup output queues"""
        self.queues['rgb'] = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.queues['depth'] = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    def _configure_ir(self):
        """Configure IR illumination"""
        if self.enable_ir and self.device.getIrDrivers():
            # Normalize brightness values
            laser_brightness = min(self.laser_dot_brightness / 1200.0, 1.0)
            flood_brightness = min(self.floodlight_brightness / 1500.0, 1.0)
            
            self.device.setIrLaserDotProjectorIntensity(laser_brightness)
            self.device.setIrFloodLightIntensity(flood_brightness)
            
            self.get_logger().info(f'IR configured - Laser: {laser_brightness:.2f}, Flood: {flood_brightness:.2f}')

    def _load_external_calibration(self):
        """Load external calibration file"""
        try:
            self.get_logger().info(f'Loading external calibration: {self.external_calibration_path}')
            calib_handler = dai.CalibrationHandler(self.external_calibration_path)
            self.pipeline.setCalibrationData(calib_handler)
        except Exception as e:
            self.get_logger().error(f'Failed to load external calibration: {str(e)}')

    def _process_frames(self):
        """Process camera frames and publish messages"""
        try:
            # Process RGB
            if 'rgb' in self.queues:
                rgb_frame = self.queues['rgb'].tryGet()
                if rgb_frame is not None:
                    self._publish_rgb_image(rgb_frame)
                    self._publish_rgb_camera_info()
            
            # Process depth
            if 'depth' in self.queues:
                depth_frame = self.queues['depth'].tryGet()
                if depth_frame is not None:
                    self._publish_depth_image(depth_frame)
                    self._publish_depth_camera_info()
                    self._publish_pointcloud(depth_frame)
            
            # Publish TF
            if self.publish_tf:
                self._publish_transform()
                
        except Exception as e:
            self.get_logger().error(f'Error processing frames: {str(e)}')

    def _publish_rgb_image(self, frame):
        """Publish RGB image"""
        try:
            cv_image = frame.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(cv_image, 'rgb8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'{self.camera_name}_rgb_camera_optical_frame'
            self.rgb_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing RGB image: {str(e)}')

    def _publish_depth_image(self, frame):
        """Publish depth image"""
        try:
            depth_image = frame.getFrame()
            # Convert to 32-bit float in meters
            depth_float = depth_image.astype(np.float32) / 1000.0
            msg = self.bridge.cv2_to_imgmsg(depth_float, '32FC1')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'{self.camera_name}_left_camera_optical_frame'
            self.depth_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing depth image: {str(e)}')

    def _publish_pointcloud(self, frame):
        """Generate and publish point cloud"""
        try:
            depth_image = frame.getFrame()
            height, width = depth_image.shape
            
            # Get camera intrinsics from calibration
            if self.calibration_data:
                intrinsics = self.calibration_data.getCameraIntrinsics(dai.CameraBoardSocket.LEFT, dai.Size2f(width, height))
                fx, fy = intrinsics[0][0], intrinsics[1][1]
                cx, cy = intrinsics[0][2], intrinsics[1][2]
            else:
                # Fallback values
                fx = fy = 883.15
                cx, cy = width / 2.0, height / 2.0
            
            # Generate point cloud (subsample for performance)
            points = []
            step = 4
            for v in range(0, height, step):
                for u in range(0, width, step):
                    z = depth_image[v, u] / 1000.0  # Convert to meters
                    
                    if 0.3 < z < 10.0:  # Filter valid depth range
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy
                        points.append([x, y, z])
            
            if points:
                header = self._create_header(f'{self.camera_name}_left_camera_optical_frame')
                pointcloud_msg = pc2.create_cloud_xyz32(header, points)
                self.pointcloud_pub.publish(pointcloud_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing point cloud: {str(e)}')

    def _publish_rgb_camera_info(self):
        """Publish RGB camera info using calibration data"""
        try:
            if self.calibration_data:
                # Get RGB camera intrinsics
                intrinsics = self.calibration_data.getCameraIntrinsics(dai.CameraBoardSocket.RGB, dai.Size2f(1920, 1080))
                distortion = self.calibration_data.getDistortionCoefficients(dai.CameraBoardSocket.RGB)
                
                info = CameraInfo()
                info.header = self._create_header(f'{self.camera_name}_rgb_camera_optical_frame')
                info.width = 1920
                info.height = 1080
                info.distortion_model = 'plumb_bob'
                
                # Intrinsic matrix
                info.k = [
                    intrinsics[0][0], intrinsics[0][1], intrinsics[0][2],
                    intrinsics[1][0], intrinsics[1][1], intrinsics[1][2],
                    intrinsics[2][0], intrinsics[2][1], intrinsics[2][2]
                ]
                
                # Rectification matrix (identity for RGB)
                info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                
                # Projection matrix
                info.p = [
                    intrinsics[0][0], 0.0, intrinsics[0][2], 0.0,
                    0.0, intrinsics[1][1], intrinsics[1][2], 0.0,
                    0.0, 0.0, 1.0, 0.0
                ]
                
                # Distortion coefficients
                info.d = distortion
                
                self.rgb_info_pub.publish(info)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing RGB camera info: {str(e)}')

    def _publish_depth_camera_info(self):
        """Publish depth camera info using calibration data"""
        try:
            if self.calibration_data:
                # Get left camera intrinsics (depth is based on left camera)
                intrinsics = self.calibration_data.getCameraIntrinsics(dai.CameraBoardSocket.LEFT, dai.Size2f(1280, 720))
                distortion = self.calibration_data.getDistortionCoefficients(dai.CameraBoardSocket.LEFT)
                
                info = CameraInfo()
                info.header = self._create_header(f'{self.camera_name}_left_camera_optical_frame')
                info.width = 1280
                info.height = 720
                info.distortion_model = 'plumb_bob'
                
                # Intrinsic matrix
                info.k = [
                    intrinsics[0][0], intrinsics[0][1], intrinsics[0][2],
                    intrinsics[1][0], intrinsics[1][1], intrinsics[1][2],
                    intrinsics[2][0], intrinsics[2][1], intrinsics[2][2]
                ]
                
                # Rectification matrix (identity for depth)
                info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                
                # Projection matrix
                info.p = [
                    intrinsics[0][0], 0.0, intrinsics[0][2], 0.0,
                    0.0, intrinsics[1][1], intrinsics[1][2], 0.0,
                    0.0, 0.0, 1.0, 0.0
                ]
                
                # Distortion coefficients
                info.d = distortion
                
                self.depth_info_pub.publish(info)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing depth camera info: {str(e)}')

    def _publish_transform(self):
        """Publish TF transform"""
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.base_frame
            t.child_frame_id = f'{self.camera_name}_camera_frame'
            
            # Default transform (can be enhanced with calibration data)
            t.transform.translation.x = 0.1
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.2
            
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing transform: {str(e)}')

    def _publish_diagnostics(self):
        """Publish diagnostic information"""
        try:
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            # Camera status
            status = DiagnosticStatus()
            status.name = f'{self.get_name()}: Camera Status'
            status.hardware_id = self.device_info.get('mxid', 'unknown')
            
            if self.camera_running and self.device:
                status.level = DiagnosticStatus.OK
                status.message = 'Camera running normally'
                
                # Add device information
                status.values.append(KeyValue(key='Device Name', value=self.device_info.get('device_name', 'unknown')))
                status.values.append(KeyValue(key='MXID', value=self.device_info.get('mxid', 'unknown')))
                status.values.append(KeyValue(key='Connection', value=self.device_info.get('name', 'unknown')))
                
                if self.device_info.get('usb_speed'):
                    status.values.append(KeyValue(key='USB Speed', value=str(self.device_info['usb_speed'])))
                
                # Add temperature if available
                try:
                    temps = self.device.getChipTemperature()
                    for temp_source, temp_value in temps.items():
                        status.values.append(KeyValue(key=f'Temperature {temp_source}', value=f'{temp_value:.1f}°C'))
                except:
                    pass
                    
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = 'Camera not running'
            
            diag_array.status.append(status)
            self.diagnostics_pub.publish(diag_array)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing diagnostics: {str(e)}')

    def _create_header(self, frame_id):
        """Create ROS header"""
        from std_msgs.msg import Header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header

    def _save_pipeline(self):
        """Save pipeline schema to file"""
        try:
            import tempfile
            save_path = f"{tempfile.gettempdir()}/{self.device_info.get('mxid', 'unknown')}_pipeline.json"
            self.get_logger().info(f'Saving pipeline schema to: {save_path}')
            
            with open(save_path, 'w') as f:
                f.write(str(self.pipeline.serializeToJson()["pipeline"]))
                
        except Exception as e:
            self.get_logger().error(f'Error saving pipeline: {str(e)}')

    def _save_calibration(self):
        """Save calibration data to file"""
        try:
            import tempfile
            save_path = f"{tempfile.gettempdir()}/{self.device_info.get('mxid', 'unknown')}_calibration.json"
            self.get_logger().info(f'Saving calibration to: {save_path}')
            
            if self.calibration_data:
                self.calibration_data.eepromToJsonFile(save_path)
                
        except Exception as e:
            self.get_logger().error(f'Error saving calibration: {str(e)}')

    # Service callbacks
    def _start_camera_srv(self, request, response):
        """Start camera service"""
        try:
            self.start_camera()
            response.success = True
            response.message = 'Camera started successfully'
        except Exception as e:
            response.success = False
            response.message = f'Failed to start camera: {str(e)}'
        return response

    def _stop_camera_srv(self, request, response):
        """Stop camera service"""
        try:
            self.stop_camera()
            response.success = True
            response.message = 'Camera stopped successfully'
        except Exception as e:
            response.success = False
            response.message = f'Failed to stop camera: {str(e)}'
        return response

    def _save_pipeline_srv(self, request, response):
        """Save pipeline service"""
        try:
            self._save_pipeline()
            response.success = True
            response.message = 'Pipeline saved successfully'
        except Exception as e:
            response.success = False
            response.message = f'Failed to save pipeline: {str(e)}'
        return response

    def _save_calibration_srv(self, request, response):
        """Save calibration service"""
        try:
            self._save_calibration()
            response.success = True
            response.message = 'Calibration saved successfully'
        except Exception as e:
            response.success = False
            response.message = f'Failed to save calibration: {str(e)}'
        return response

    def destroy_node(self):
        """Clean up resources"""
        try:
            self.stop_camera()
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {str(e)}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OAKDNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()