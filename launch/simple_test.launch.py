#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    
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
    
    # Simple OAK-D driver node (test mode without hardware dependencies)
    simple_oakd_node = Node(
        package='oakd_driver',
        executable='simple_oakd_driver',
        name='simple_oakd_driver',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'camera_name': LaunchConfiguration('camera_name'),
            }
        ],
        output='screen',
        respawn=False,
        respawn_delay=2
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        camera_name_arg,
        simple_oakd_node
    ])