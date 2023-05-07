from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter, Node
from launch.conditions import IfCondition
import os

def get_argument(context, arg):
    return LaunchConfiguration(arg).perform(context)

def gen_world(context, *args, **kwargs):
    use_robostack = get_argument(context, "robostack")
    track = str(get_argument(context, "track") + ".world")
    gui = str(get_argument(context, "gazebo_gui"))

    tracks = get_package_share_directory('eufs_tracks')
    racecar = get_package_share_directory('eufs_racecar')
    PLUGINS = os.environ.get('GAZEBO_PLUGIN_PATH')
    MODELS = os.environ.get('GAZEBO_MODEL_PATH')
    RESOURCES = os.environ.get('GAZEBO_RESOURCE_PATH')
    EUFS = os.path.expanduser(os.environ.get('EUFS_MASTER'))
    DISTRO = os.environ.get('ROS_DISTRO')
    
    if use_robostack == "true":
        os.environ['GAZEBO_PLUGIN_PATH'] = EUFS + '/install/eufs_plugins:' + PLUGINS
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = EUFS + '/install/eufs_plugins:' + '/opt/ros/' + DISTRO
    os.environ['GAZEBO_MODEL_PATH'] = tracks + '/models:' + str(MODELS)
    os.environ['GAZEBO_RESOURCE_PATH'] = tracks + '/materials:' + tracks + '/meshes:' + racecar + '/materials:' + racecar + '/meshes:' + str(RESOURCES)

    world_path = os.path.join(tracks, "worlds", track)
    
    gazebo_launch = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    params_file = os.path.join(get_package_share_directory('config'), 'config', 'sim_params.yaml')

    return [
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                gazebo_launch
            ),
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
    car_launch = os.path.join(get_package_share_directory("eufs_racecar"), "launch", "load_car.launch.py")

    # get x,y,z,roll,pitch,yaw from track csv file
    tracks = get_package_share_directory("eufs_tracks")
    track = get_argument(context, "track")

    with open(os.path.join(tracks, "csv", track + ".csv"), "r") as f:
        # car position is last line of csv file
        for line in f:
            pass
        car_pos = line.split(",")
        x = car_pos[1]
        y = car_pos[2]
        z = car_pos[3]
        roll = car_pos[4]
        pitch = car_pos[5]
        yaw = car_pos[6]

    return[
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                car_launch
            ),
            launch_arguments=[
                ("vehicle_model", LaunchConfiguration("vehicleModel")),
                ("vehicle_model_config", LaunchConfiguration("vehicleModelConfig")),
                ("command_mode", LaunchConfiguration("commandMode")),
                ("robot_name", LaunchConfiguration("robot_name")),
                ("publish_transform", LaunchConfiguration("publish_gt_tf")),
                ("publish_ground_truth", LaunchConfiguration("pub_ground_truth")),
                ("simulate_perception", LaunchConfiguration("sim_perception")),
                ("simulate_slam", LaunchConfiguration("sim_slam")),
                ("enable_camera", LaunchConfiguration("enable_camera")),
                ("enable_lidar", LaunchConfiguration("enable_lidar")),
                ("enable_laserscan", LaunchConfiguration("enable_laserscan")),
                ("x", x),
                ("y", y),
                ("z", z),
                ("roll", roll),
                ("pitch", pitch),
                ("yaw", yaw),
            ],
        ),
    ]


def generate_launch_description():
    rviz_config_file = os.path.join(
        get_package_share_directory("eufs_launcher"), "config", "default.rviz"
    )

    return LaunchDescription(
        [
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
            Node(
                name="rviz",
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_file],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            # launch the gazebo world
            OpaqueFunction(function=gen_world),
            # launch the car
            OpaqueFunction(function=spawn_car),
        ]
    )
