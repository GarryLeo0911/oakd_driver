import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration("name").perform(context)
    oakd_driver_prefix = get_package_share_directory("oakd_driver")

    params_file = LaunchConfiguration("params_file")

    # RGBD Odometry parameters
    rgbd_odom_parameters = [
        {
            "frame_id": "oak_parent_frame",
            "odom_frame_id": "odom", 
            "publish_tf": True,
            "subscribe_rgb": True,
            "subscribe_depth": True,
            "approx_sync": True,
            # RTAB-Map odometry parameters
            "Odom/Strategy": 1,  # 0=Frame-to-Map, 1=Frame-to-Frame
            "Vis/EstimationType": 1,  # 0=3D->3D, 1=3D->2D (PnP)
            "Vis/MaxFeatures": 400,
            "Vis/MinInliers": 15,
            "Odom/ResetCountdown": 0,
            "Odom/GuessmotionEnabled": False,
        }
    ]

    # Topic remappings for odometry node
    rgbd_odom_remappings = [
        ("rgb/image", f"/{name}/rgb/image_raw"),
        ("rgb/camera_info", f"/{name}/rgb/camera_info"),
        ("depth/image", f"/{name}/stereo/image_raw"),
    ]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(oakd_driver_prefix, "launch", "simple_driver.launch.py")
            ),
            launch_arguments={
                "name": name, 
                "params_file": params_file,
            }.items(),
        ),
        
        # RGBD Odometry node - computes visual odometry and publishes /odom
        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="rgbd_odometry",
            parameters=rgbd_odom_parameters,
            remappings=rgbd_odom_remappings,
            output="screen",
        ),
    ]


def generate_launch_description():
    oakd_driver_prefix = get_package_share_directory("oakd_driver")
    
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(oakd_driver_prefix, "config", "rtabmap_minimal.yaml"),
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )