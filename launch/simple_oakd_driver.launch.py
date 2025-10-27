#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Simple OAK-D driver node (test mode, no hardware required)
    simple_oakd_node = Node(
        package='oakd_driver',
        executable='simple_oakd_driver',
        name='simple_oakd_driver',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
        respawn=True,
        respawn_delay=2
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        simple_oakd_node
    ])