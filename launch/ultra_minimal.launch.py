#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    oakd_driver_prefix = get_package_share_directory("oakd_driver")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(oakd_driver_prefix, "config", "driver.yaml"),
        ),
    ]

    name = LaunchConfiguration("name")
    namespace = LaunchConfiguration("namespace") 
    params_file = LaunchConfiguration("params_file")

    # Ultra-minimal container - just test camera connection without ROS nodes
    container = ComposableNodeContainer(
        name=f"oak_minimal_container",
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
                extra_arguments=[
                    {"use_intra_process_comms": False},
                ],
            )
        ],
        arguments=[
            "--ros-args", 
            "--log-level", "info",
        ],
        output="both",
    )

    # Very delayed start to ensure system stability
    delayed_container = TimerAction(
        period=2.0,  # 2 second delay
        actions=[container]
    )

    return LaunchDescription(declared_arguments + [delayed_container])