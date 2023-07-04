import os
from os.path import isfile, join

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn_car(context, *args, **kwargs):
    robot_name = get_argument(context, "robot_name")
    vehicle_model = get_argument(context, "vehicle_model")
    command_mode = get_argument(context, "command_mode")
    vehicle_model_config = get_argument(context, "vehicle_model_config")
    publish_transform = get_argument(context, "publish_transform")
    publish_ground_truth = get_argument(context, "publish_ground_truth")
    x = get_argument(context, "x")
    y = get_argument(context, "y")
    z = get_argument(context, "z")
    roll = get_argument(context, "roll")
    pitch = get_argument(context, "pitch")
    yaw = get_argument(context, "yaw")
    enable_camera = get_argument(context, "enable_camera")
    enable_lidar = get_argument(context, "enable_lidar")
    enable_laserscan = get_argument(context, "enable_laserscan")
    simulate_perception = get_argument(context, "simulate_perception")
    simulate_slam = get_argument(context, "simulate_slam")

    vehicle_config = join(
        get_package_share_directory("config"),
        "config",
        vehicle_model_config,
    )
    noise_config = join(
        get_package_share_directory("config"),
        "config",
        "motionNoise.yaml",
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
            "robot_name": robot_name,
            "vehicle_model": vehicle_model,
            "command_mode": command_mode,
            "vehicle_config": vehicle_config,
            "noise_config": noise_config,
            "publish_transform": publish_transform,
            "simulate_perception": simulate_perception,
            "simulate_slam": simulate_slam,
            "enable_camera": enable_camera,
            "enable_lidar": enable_lidar,
            "enable_laserscan": enable_laserscan,
            "publish_ground_truth": publish_ground_truth,
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
                "-z",
                z,
                "-R",
                roll,
                "-P",
                pitch,
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
                }
            ],
            arguments=["--ros-args", "--log-level", "warn"],
        ),
    ]


def generate_launch_description():
    rqt_perspective_file = join(
        get_package_share_directory("eufs_rqt"),
        "config",
        "eufs_sim.perspective",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value="qev3",
                description="The name of the robot",
            ),
            DeclareLaunchArgument(
                "vehicle_model",
                default_value="DynamicBicycle",
                description="The vehicle model class to use on the robot",
            ),
            DeclareLaunchArgument(
                "command_mode",
                default_value="acceleration",
                description="Determines whether to use acceleration or velocity to control the vehicle",
            ),
            DeclareLaunchArgument(
                "vehicle_model_config",
                default_value="configDry.yaml",
                description="Determines the file from which the vehicle model parameters are read",
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
                "z",
                default_value="0",
                description="Vehicle initial z position",
            ),
            DeclareLaunchArgument(
                "roll",
                default_value="0",
                description="Vehicle initial roll",
            ),
            DeclareLaunchArgument(
                "pitch",
                default_value="0",
                description="Vehicle initial pitch",
            ),
            DeclareLaunchArgument(
                "yaw",
                default_value="0",
                description="Vehicle initial yaw",
            ),
            DeclareLaunchArgument(
                "publish_transform",
                default_value="false",
                description="Condition to publish the transform ground truth vehicle transform",
            ),
            DeclareLaunchArgument(
                "publish_ground_truth",
                default_value="false",
                description="Condition to publish ground truth vehicle and track data",
            ),
            DeclareLaunchArgument(
                name="simulate_perception",
                default_value="false",
                description="Condition to enable sim perception cones",
            ),
            DeclareLaunchArgument(
                name="simulate_slam",
                default_value="false",
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
            # Spawn the car!!!
            OpaqueFunction(function=spawn_car),
        ]
    )


def get_argument(context, arg):
    return LaunchConfiguration(arg).perform(context)
