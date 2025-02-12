import os
import cv2
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
import time


def get_argument(context, arg):
    return LaunchConfiguration(arg).perform(context)


def gen_world(context, *args, **kwargs):
    print("Generating World... ")

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
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )
    params_file = join(
        get_package_share_directory("eufs_config"), "config", "pluginUserParams.yaml"
    )
    print("Sigma Online")
    return [
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments=[
                ("verbose", "false"),
                ("pause", "false"),
                ("gui", gui),
                ("gz_args", track),
                ("urdf_model", "qev-3d.urdf.xacro"),
                ("base_frame", "base_link"),
                ("display_car", "true"),
                ("params_file", params_file),
            ],
        ),
    ]


def spawn_car(context, *args, **kwargs):
    # get x,y,z,roll,pitch,yaw from track csv file
    tracks = get_package_share_directory("eufs_tracks")
    track = get_argument(context, "track")
    urdf_model = get_argument(context, "urdf_model")
    base_frame = get_argument(context, "base_frame")
    display_car = get_argument(context, "display_car")

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
    vehicle_config = join(
        get_package_share_directory("eufs_config"),
        "config",
        vehicle_model_config,
    )  
    xacro_path = join(
        get_package_share_directory("vehicle_urdf"),
        "urdf",
        urdf_model,
    )
    urdf_path = join(
        get_package_share_directory("vehicle_urdf"),
        "urdf",
        "processed.urdf",
    )

    doc = xacro.process_file(
        xacro_path,
        mappings={
            "base_frame": base_frame,
            "display_car": display_car,
        },
    )
    # out = xacro.open_output(urdf_path)
    urdf_xml = doc.toprettyxml(indent="  ")

    with open(urdf_path, "w") as f:
        f.write(urdf_xml)

    with open(urdf_path, "r") as urdf_file:
        robot_description = urdf_file.read()
    
    # very important, trust
    time.sleep(1)

    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='track_to_chassis_broadcaster',
            output='screen',
            arguments=[
                 x,   y,  '0',    # xyz
                '0', '0', yaw,    # roll, pitch, yaw
                'track', 'chassis'
            ]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="track_to_base_broadcaster",
            arguments=[
                "0", "0", "0",      # x y z
                "0", "0", "0",      # roll pitch yaw
                "track", "base_link"
            ]
        ),
        Node(
            name="spawn_robot",
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-entity", robot_name,
                "-file", urdf_path,
                "-x", x,
                "-y", y,
                "-Y", yaw,
                "-spawn_service_timeout", "60.0",
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
                    # "use_sim_time": LaunchConfiguration("use_sim_time"),
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
                    # "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
            arguments=["--ros-args", "--log-level", "warn"],
        ),
    ]


def generate_launch_description():
    config_package = get_package_share_directory("eufs_config")

    rviz_config_file = join(
        config_package, "rviz", "default.rviz"
    )
    rqt_perspective_file = join(
        config_package, "ui", "control.perspective",
    )
    bridge_params_file = join(
        config_package, "config", "bridgeParams.yaml"
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
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_bridge",
                output="screen",
                parameters=[bridge_params_file],   # Load YAML
                remappings=[
                    # Remap topics cause it looks nice
                    ("/gazebo/lidar_scan",         "/ros2/lidar_scan"),
                    ("/gazebo/cmd_vel",            "/ros2/cmd_vel"),
                    ("/gazebo/ground_truth_map",   "/ros2/ground_truth_map")
                ],
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
