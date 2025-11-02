#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    oakd_driver_prefix = get_package_share_directory("oakd_driver")

    # Arguments
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(oakd_driver_prefix, "config", "driver.yaml"),
        ),
    ]

    # Get launch configuration values
    name = LaunchConfiguration("name")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    # Container with minimal configuration for testing
    container = ComposableNodeContainer(
        name=f"oak_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="oakd_driver",
                plugin="depthai_ros_driver::Driver", 
                name=name,
                namespace=namespace,
                parameters=[params_file],
            )
        ],
        arguments=["--ros-args", "--log-level", "info"],
        output="both",
    )

    return LaunchDescription(declared_arguments + [container])