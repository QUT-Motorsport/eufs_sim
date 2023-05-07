import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launcher_share = get_package_share_directory("eufs_launcher")
    return LaunchDescription(
        [
            Node(
                name="eufs_launcher",
                package="eufs_launcher",
                executable="eufs_launcher",
                output="both",
            ),
        ]
    )
