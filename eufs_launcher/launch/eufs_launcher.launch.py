import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launcher_share = get_package_share_directory('eufs_launcher')
    return LaunchDescription([
        DeclareLaunchArgument(
            name="config",
            default_value=os.path.join(launcher_share, "config/eufs_launcher.yaml"),
            description="Path to config file"),

        DeclareLaunchArgument(
            name="gui",
            default_value="true",
            description="Condition for GUI, if false, auto-launches as per defaults in config"),

        # Launch the launcher
        Node(
            name='eufs_launcher',
            package='eufs_launcher',
            executable='eufs_launcher',
            output='both',
            parameters=[{
                'config': LaunchConfiguration("config"),
                'gui': LaunchConfiguration("gui")
            }],
        ),

    ])
