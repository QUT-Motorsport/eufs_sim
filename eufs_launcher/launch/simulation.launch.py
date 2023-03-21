from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="track",
                default_value="small_track",
                description="Determines which track is launched",
            ),
            DeclareLaunchArgument(
                name="vehicleModel",
                default_value="DynamicBicycle",
                description="Determines which vehicle model is used",
            ),
            DeclareLaunchArgument(
                name="vehicleModelConfig",
                default_value="configDry.yaml",
                description="Determines the file from which the vehicle model parameters are read",
            ),
            DeclareLaunchArgument(
                name="commandMode",
                default_value="acceleration",
                description="Determines the vehicle control mode (acceleration or velocity)",
            ),
            DeclareLaunchArgument(
                name="robot_name",
                default_value="eufs",
                description="Determines which robot urdf is used in the sim",
            ),
            DeclareLaunchArgument(
                name="gazebo_gui",
                default_value="false",
                description="Condition to launch the Gazebo GUI",
            ),
            DeclareLaunchArgument(
                name="rviz",
                default_value="true",
                description="Condition to launch the Rviz GUI",
            ),
            DeclareLaunchArgument(
                name="publish_gt_tf",
                default_value="false",
                description="Condition to use ground truth transform",
            ),
            DeclareLaunchArgument(
                name="pub_ground_truth",
                default_value="true",
                description="Condition to publish ground truth",
            ),
            DeclareLaunchArgument(
                name="sim_perception",
                default_value="true",
                description="Condition to enable sim perception cones",
            ),
            DeclareLaunchArgument(
                name="sim_slam",
                default_value="true",
                description="Condition to enable sim SLAM cones",
            ),
            DeclareLaunchArgument(
                name="enable_camera",
                default_value="false",
                description="Condition to enable camera",
            ),
            DeclareLaunchArgument(
                name="enable_lidar",
                default_value="false",
                description="Condition to enable lidar",
            ),
            DeclareLaunchArgument(
                name="enable_laserscan",
                default_value="false",
                description="Condition to enable laserscan",
            ),
            DeclareLaunchArgument(
                name="robostack",
                default_value="false",
                description="Condition to use robostack",
            ),
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("eufs_tracks"),
                            "launch",
                            PythonExpression(
                                ["'", LaunchConfiguration("track"), "'", "+ '.launch'"]
                            ),
                        ]
                    ),
                ),
                launch_arguments=[
                    ("vehicleModel", LaunchConfiguration("vehicleModel")),
                    ("vehicleModelConfig", LaunchConfiguration("vehicleModelConfig")),
                    ("commandMode", LaunchConfiguration("commandMode")),
                    ("robot_name", LaunchConfiguration("robot_name")),
                    ("gazebo_gui", LaunchConfiguration("gazebo_gui")),
                    ("rviz", LaunchConfiguration("rviz")),
                    ("publish_gt_tf", LaunchConfiguration("publish_gt_tf")),
                    ("pub_ground_truth", LaunchConfiguration("pub_ground_truth")),
                    ("sim_perception", LaunchConfiguration("sim_perception")),
                    ("sim_slam", LaunchConfiguration("sim_slam")),
                    ("enable_camera", LaunchConfiguration("enable_camera")),
                    ("enable_lidar", LaunchConfiguration("enable_lidar")),
                    ("enable_laserscan", LaunchConfiguration("enable_laserscan")),
                    ("robostack", LaunchConfiguration("robostack")),
                ],
            ),
        ]
    )
