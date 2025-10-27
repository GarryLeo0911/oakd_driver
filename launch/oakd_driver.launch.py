#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('oakd_driver')
    
    # Declare launch arguments following luxonis patterns
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'oakd_params.yaml'),
        description='Path to the config file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='oak',
        description='Camera name for topics and frames'
    )
    
    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='',
        description='Camera model name (auto-detected if empty)'
    )
    
    mx_id_arg = DeclareLaunchArgument(
        'mx_id',
        default_value='',
        description='MX ID of the device to connect to'
    )
    
    ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='',
        description='IP address of the device (for PoE cameras)'
    )
    
    usb_port_id_arg = DeclareLaunchArgument(
        'usb_port_id',
        default_value='',
        description='USB port ID of the device'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Camera FPS'
    )
    
    rgb_resolution_arg = DeclareLaunchArgument(
        'rgb_resolution',
        default_value='1080p',
        description='RGB camera resolution (720p, 1080p, 4K)'
    )
    
    depth_resolution_arg = DeclareLaunchArgument(
        'depth_resolution',
        default_value='720p',
        description='Depth camera resolution'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='200',
        description='Depth confidence threshold (0-255)'
    )
    
    lr_check_arg = DeclareLaunchArgument(
        'lr_check',
        default_value='true',
        description='Enable left-right check for depth'
    )
    
    enable_ir_arg = DeclareLaunchArgument(
        'enable_ir',
        default_value='false',
        description='Enable IR illumination'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish TF transforms from calibration'
    )
    
    # OAK-D driver node with comprehensive parameter passing
    oakd_node = Node(
        package='oakd_driver',
        executable='oakd_node',
        name='oakd_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'i_tf_camera_name': LaunchConfiguration('camera_name'),
                'i_tf_camera_model': LaunchConfiguration('camera_model'),
                'i_mx_id': LaunchConfiguration('mx_id'),
                'i_ip': LaunchConfiguration('ip'),
                'i_usb_port_id': LaunchConfiguration('usb_port_id'),
                'i_fps': LaunchConfiguration('fps'),
                'i_rgb_resolution': LaunchConfiguration('rgb_resolution'),
                'i_depth_resolution': LaunchConfiguration('depth_resolution'),
                'i_depth_confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'i_depth_lr_check': LaunchConfiguration('lr_check'),
                'i_enable_ir': LaunchConfiguration('enable_ir'),
                'i_publish_tf_from_calibration': LaunchConfiguration('publish_tf'),
            }
        ],
        output='screen',
        respawn=True,
        respawn_delay=2
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        camera_name_arg,
        camera_model_arg,
        mx_id_arg,
        ip_arg,
        usb_port_id_arg,
        fps_arg,
        rgb_resolution_arg,
        depth_resolution_arg,
        confidence_threshold_arg,
        lr_check_arg,
        enable_ir_arg,
        publish_tf_arg,
        oakd_node
    ])