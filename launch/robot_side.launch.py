#!/usr/bin/env python3
"""
Robot-side launch: Just your existing OAK-D driver
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    # Use your existing OAK-D driver - publishes all camera topics
    oakd_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('oakd_driver'),
                'launch',
                'oakd_driver.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        oakd_driver
    ])