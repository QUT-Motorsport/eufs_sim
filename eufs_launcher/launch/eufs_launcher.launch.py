from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(name="config",
                              default_value="config/eufs_launcher",
                              description="Relative path to config file ("
                                          "within package supplied by "
                                          "config_loc)"),

        DeclareLaunchArgument(name="config_loc",
                              default_value="eufs_launcher",
                              description="Decides which package to load "
                                          "config file from"),

        DeclareLaunchArgument(name="gui",
                              default_value="true",
                              description="Condition for GUI launch, "
                                          "if false, GUI auto-launches "
                                          "according to "
                                          "defaults in the config file"),

        # Launch the launcher
        Node(
            name='eufs_launcher',
            package='eufs_launcher',
            executable='eufs_launcher',
            output='screen',
            parameters=[{
                'config': LaunchConfiguration("config"),
                'config_loc': LaunchConfiguration("config_loc"),
                'gui': LaunchConfiguration("gui")
            }],
        ),

    ])
