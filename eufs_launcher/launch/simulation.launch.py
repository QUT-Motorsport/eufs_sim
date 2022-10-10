from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.launch_description_sources import FrontendLaunchDescriptionSource


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            name='track',
            default_value='small_track',
            description="Determines which track is launched"),

        DeclareLaunchArgument(
            name='vehicleModel',
            default_value='DynamicBicycle',
            description="Determines which vehicle model is used"),

        DeclareLaunchArgument(
            name='vehicleModelConfig',
            default_value='configDry.yaml',
            description="Determines the file from which the vehicle model parameters are read"),

        DeclareLaunchArgument(
            name='commandMode',
            default_value='acceleration',
            description="Determines the vehicle control mode (acceleration or velocity)"),

        DeclareLaunchArgument(
            name='robot_name',
            default_value='eufs',
            description="Determines which robot urdf is used in the sim"),

        DeclareLaunchArgument(
            name='gazebo_gui',
            default_value='false',
            description="Condition to launch the Gazebo GUI"),

        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description="Condition to launch the Rviz GUI"),

        DeclareLaunchArgument(
            name='publish_gt_tf',
            default_value='false',
            description="Condition to use ground truth transform"),

        DeclareLaunchArgument(
            name='pub_ground_truth',
            default_value='true',
            description="Condition to publish ground truth"),

        # Set to 'no_perception' to turn off the perception code and use ground truth cones.
        DeclareLaunchArgument(
            name='launch_group',
            default_value='default',
            description="Determines which launch files are used in the state_machine node"),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('eufs_tracks'),
                    'launch',
                    PythonExpression(["'", LaunchConfiguration('track'), "'", "+ '.launch'"])
                ]),
            ),
            launch_arguments=[
                ('vehicleModel', LaunchConfiguration('vehicleModel')),
                ('vehicleModelConfig', LaunchConfiguration('vehicleModelConfig')),
                ('commandMode', LaunchConfiguration('commandMode')),
                ('robot_name', LaunchConfiguration('robot_name')),
                ('gazebo_gui', LaunchConfiguration('gazebo_gui')),
                ('rviz', LaunchConfiguration('rviz')),
                ('publish_gt_tf', LaunchConfiguration('publish_gt_tf')),
                ('pub_ground_truth', LaunchConfiguration('pub_ground_truth')),
                ('launch_group', LaunchConfiguration('launch_group')),
            ]
        ),
    ])
