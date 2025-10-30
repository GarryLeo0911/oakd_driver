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


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration("name").perform(context)
    oakd_driver_prefix = get_package_share_directory("oakd_driver")

    params_file = LaunchConfiguration("params_file")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(oakd_driver_prefix, "launch", "driver.launch.py")
            ),
            launch_arguments={
                "name": name, 
                "params_file": params_file,
                "publish_tf_from_calibration": "true",
                "use_rviz": "false"
            }.items(),
        ),
    ]


def generate_launch_description():
    oakd_driver_prefix = get_package_share_directory("oakd_driver")
    
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(oakd_driver_prefix, "config", "rtabmap.yaml"),
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )