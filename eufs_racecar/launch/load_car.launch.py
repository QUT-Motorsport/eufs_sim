import os
from os.path import isfile, join

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_argument(context, arg):
    return LaunchConfiguration(arg).perform(context)

def spawn_car(context, *args, **kwargs):
    robot_name = get_argument(context, "robot_name")
    vehicle_model_config = get_argument(context, "vehicle_model_config")
    enable_camera = get_argument(context, "enable_camera")
    enable_lidar = get_argument(context, "enable_lidar")
    enable_laserscan = get_argument(context, "enable_laserscan")
    x = get_argument(context, "x")
    y = get_argument(context, "y")
    yaw = get_argument(context, "yaw")

    vehicle_config = join(
        get_package_share_directory("eufs_config"),
        "config",
        vehicle_model_config,
    )
    xacro_path = join(
        get_package_share_directory("eufs_racecar"),
        "urdf",
        "robot.urdf.xacro",
    )
    urdf_path = join(
        get_package_share_directory("eufs_racecar"),
        "urdf",
        "robot.urdf",
    )

    if not isfile(urdf_path):
        os.mknod(urdf_path)

    doc = xacro.process_file(
        xacro_path,
        mappings={
            "vehicle_config": vehicle_config,
            "enable_camera": enable_camera,
            "enable_lidar": enable_lidar,
            "enable_laserscan": enable_laserscan,
        },
    )
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent="  "))

    with open(urdf_path, "r") as urdf_file:
        robot_description = urdf_file.read()

    return [
        Node(
            name="spawn_robot",
            package="gazebo_ros",
            executable="spawn_entity.py",
            output="screen",
            arguments=[
                "-entity",
                robot_name,
                "-file",
                urdf_path,
                "-x",
                x,
                "-y",
                y,
                "-Y",
                yaw,
                "-spawn_service_timeout",
                "60.0",
                "--ros-args",
                "--log-level",
                "warn",
            ],
        ),
        Node(
            name="joint_state_publisher",
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "rate": 200,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
            arguments=[urdf_path],
        ),
        Node(
            name="robot_state_publisher",
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "rate": 200,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
            arguments=["--ros-args", "--log-level", "warn"],
        ),
    ]


def generate_launch_description():
    rqt_perspective_file = join(
        get_package_share_directory("eufs_config"),
        "ui",
        "control.perspective",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="qev3",
                description="The name of the robot",
            ),
            DeclareLaunchArgument(
                "vehicle_model_config",
                default_value="configDry.yaml",
                description="Determines the file from which the vehicle model parameters are read",
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
                "x",
                default_value="0",
                description="Vehicle initial x position",
            ),
            DeclareLaunchArgument(
                "y",
                default_value="0",
                description="Vehicle initial y position",
            ),
            DeclareLaunchArgument(
                "yaw",
                default_value="0",
                description="Vehicle initial yaw",
            ),
            # perspective launches the state controller and robot steering GUIs
            Node(
                name="eufs_sim_rqt",
                package="rqt_gui",
                executable="rqt_gui",
                output="screen",
                arguments=[
                    "--force-discover",
                    "--perspective-file",
                    str(rqt_perspective_file),
                ],
            ),
            Node(
                package="vehicle_supervisor",
                executable="vehicle_supervisor_slim_node",        
            ),
            # Spawn the car
            OpaqueFunction(function=spawn_car),
        ]
    )
