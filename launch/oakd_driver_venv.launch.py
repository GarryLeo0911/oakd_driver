#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('oakd_driver')
    
    # Declare launch arguments
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
    
    venv_path_arg = DeclareLaunchArgument(
        'venv_path',
        default_value='/home/garry/ros_ws/oakd_venv',  # Update this path as needed
        description='Path to the virtual environment'
    )
    
    # OAK-D driver node using virtual environment
    oakd_node = ExecuteProcess(
        cmd=[
            'bash', '-c', 
            f'source {LaunchConfiguration("venv_path")}/bin/activate && '
            f'ros2 run oakd_driver oakd_node '
            f'--ros-args --params-file {LaunchConfiguration("config_file")} '
            f'-p use_sim_time:={LaunchConfiguration("use_sim_time")}'
        ],
        name='oakd_node',
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        venv_path_arg,
        oakd_node
    ])