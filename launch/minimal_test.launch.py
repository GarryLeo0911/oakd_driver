#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oakd_driver',
            executable='minimal_camera_test',
            name='minimal_camera_test',
            output='both',
            parameters=[],
        )
    ])