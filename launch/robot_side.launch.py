#!/usr/bin/env python3
"""
Robot-side launch: OAK-D driver optimized for RTABmap SLAM integration
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Use your existing OAK-D driver with proper parameters for SLAM
    oakd_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('oakd_driver'),
                'launch',
                'oakd_driver.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'camera_name': 'oak',
            'publish_tf': 'true',
            'fps': '30',
            'rgb_resolution': '720p',  # Lower resolution for better performance
            'depth_resolution': '720p',
            'confidence_threshold': '200',
            'lr_check': 'true'
        }.items()
    )
    
    # Static transform publisher for base_link to camera
    # This is crucial for RTABmap to work properly
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '0.0', '0.0', '0.1',  # x, y, z (camera 10cm above base)
            '0.0', '0.0', '0.0', '1.0',  # quaternion (no rotation)
            'base_link',
            'oak_rgb_camera_optical_frame'
        ],
        output='screen'
    )
    
    # RGBD Synchronization node - this is what RTABmap expects
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'approx_sync': True,
            'approx_sync_max_interval': 0.01,
            'topic_queue_size': 10,
            'sync_queue_size': 10
        }],
        remappings=[
            # Map your OAK-D topics to what rgbd_sync expects
            ('rgb/image', '/oak/rgb/image_raw'),
            ('depth/image', '/oak/depth/image_raw'), 
            ('rgb/camera_info', '/oak/rgb/camera_info')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        oakd_driver,
        base_to_camera_tf,
        rgbd_sync
    ])