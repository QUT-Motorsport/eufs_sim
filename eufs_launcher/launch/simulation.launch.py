import os
from os.path import isfile, join

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_argument(context, arg):
    return LaunchConfiguration(arg).perform(context)


def gen_world(context, *args, **kwargs):
    use_robostack = get_argument(context, "robostack")
    track = str(get_argument(context, "track") + ".world")
    gui = str(get_argument(context, "gazebo_gui"))

    tracks = get_package_share_directory("eufs_tracks")
    racecar = get_package_share_directory("eufs_racecar")
    PLUGINS = os.environ.get("GAZEBO_PLUGIN_PATH")
    MODELS = os.environ.get("GAZEBO_MODEL_PATH")
    RESOURCES = os.environ.get("GAZEBO_RESOURCE_PATH")
    EUFS = os.path.expanduser(os.environ.get("EUFS_MASTER"))
    DISTRO = os.environ.get("ROS_DISTRO")

    if use_robostack == "true":
        os.environ["GAZEBO_PLUGIN_PATH"] = EUFS + "/install/eufs_plugins:" + PLUGINS
    else:
        os.environ["GAZEBO_PLUGIN_PATH"] = (
            EUFS + "/install/eufs_plugins:" + "/opt/ros/" + DISTRO
        )
    os.environ["GAZEBO_MODEL_PATH"] = tracks + "/models:" + str(MODELS)
    os.environ["GAZEBO_RESOURCE_PATH"] = (
        tracks
        + "/materials:"
        + tracks
        + "/meshes:"
        + racecar
        + "/materials:"
        + racecar
        + "/meshes:"
        + str(RESOURCES)
    )

    world_path = join(tracks, "worlds", track)

    gazebo_launch = join(
        get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
    )
    params_file = join(
        get_package_share_directory("eufs_config"), "config", "pluginUserParams.yaml"
    )

    return [
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments=[
                ("verbose", "false"),
                ("pause", "false"),
                ("gui", gui),
                ("world", world_path),
                ("params_file", params_file),
            ],
        ),
    ]


def spawn_car(context, *args, **kwargs):
    # get x,y,z,roll,pitch,yaw from track csv file
    tracks = get_package_share_directory("eufs_tracks")
    track = get_argument(context, "track")

    with open(join(tracks, "csv", track + ".csv"), "r") as f:
        # car position is last line of csv file
        for line in f:
            pass
        car_pos = line.split(",")
        x = car_pos[1]
        y = car_pos[2]
        yaw = car_pos[3]

    robot_name = get_argument(context, "robot_name")
    vehicle_model_config = get_argument(context, "vehicle_model_config")
    base_frame = get_argument(context, "base_frame")
    enable_camera = get_argument(context, "enable_camera")
    enable_lidar = get_argument(context, "enable_lidar")
    enable_laserscan = get_argument(context, "enable_laserscan")

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
            "base_frame": base_frame,
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
                    "source_list": ["joint_states/steering"],
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
    rviz_config_file = join(
        get_package_share_directory("eufs_config"), "rviz", "default.rviz"
    )
    rqt_perspective_file = join(
        get_package_share_directory("eufs_config"),
        "ui",
        "control.perspective",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                name="robostack",
                default_value="false",
                description="Condition to use robostack",
            ),
            DeclareLaunchArgument(
                name="track",
                default_value="small_track",
                description="Determines which track is launched",
            ),
            DeclareLaunchArgument(
                name="vehicle_model_config",
                default_value="configDry.yaml",
                description="Determines the file from which the vehicle model parameters are read",
            ),
            DeclareLaunchArgument(
                name="robot_name",
                default_value="qev3",
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
                name="base_frame",
                default_value="base_link",
                description="ROS transform frame for the vehicle base",
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
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        arguments=["-d", rviz_config_file],
                        condition=IfCondition(LaunchConfiguration("rviz")),
                        parameters=[
                            {"use_sim_time": LaunchConfiguration("use_sim_time")}
                        ],
                    ),
                ],
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
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            # Node(
            #     package="vehicle_supervisor",
            #     executable="supervisor_node",
            #     output="screen",
            #     parameters=[
            #         {"use_sim_time": LaunchConfiguration("use_sim_time")},
            #         {"manual_override": False},
            #     ],
            # ),
            # launch the gazebo world
            OpaqueFunction(function=gen_world),
            # launch the car
            OpaqueFunction(function=spawn_car),
        ]
    )
