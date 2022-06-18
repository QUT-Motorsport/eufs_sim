import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, \
    IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


# Note: Due to some python launch file subtlety, the launch_setup function
# must be kept seperate from
# generate_launch_description, otherwise it is not possible to access the
# parameters that are passed into the simulation
def launch_setup(context, *args, **kwargs):
    # Get arguments and store them so they can be passed into the simulation
    track = get_argument(context, 'track')

    launch_file_path = os.path.join(get_package_share_directory('eufs_tracks'),
                                    'launch', str(track) + '.launch')
    launch_description = AnyLaunchDescriptionSource(launch_file_path)

    return [

        # Launch the simulation
        IncludeLaunchDescription(launch_description)
    ]


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(name='track',
                              default_value='small_track',
                              description="Determines which track is "
                                          "launched"),

        DeclareLaunchArgument(name='vehicleModel',
                              default_value='DynamicBicycle',
                              description="Determines which vehicle model is"
                                          " used"),

        DeclareLaunchArgument(name='commandMode',
                              default_value='acceleration',
                              description="Determines the vehicle control "
                                          "mode (acceleration or velocity)"),

        DeclareLaunchArgument(name='vehicleModelConfig',
                              default_value='configDry.yaml',
                              description="Determines the file from which "
                                          "the vehicle model parameters are "
                                          "read"),

        DeclareLaunchArgument(name='gazebo_gui',
                              default_value='false',
                              description="Condition to launch the Gazebo "
                                          "GUI"),

        DeclareLaunchArgument(name='rviz',
                              default_value='true',
                              description="Condition to launch the Rviz GUI"),

        DeclareLaunchArgument(name='publish_gt_tf',
                              default_value='false',
                              description="Condition to use ground truth "
                                          "transform"),

        DeclareLaunchArgument(name='pub_ground_truth',
                              default_value='true',
                              description="Condition to publish ground "
                                          "truth"),

        # Set to 'no_perception' to turn off the perception code and use
        # ground truth cones.
        DeclareLaunchArgument(name='launch_group',
                              default_value='default',
                              description="Determines which launch files are "
                                          "used in the state_machine node"),

        DeclareLaunchArgument(name='robot_name',
                              default_value='eufs',
                              description="Determines which robot urdf is"
                                          "used in the sim"),

        OpaqueFunction(function=launch_setup),
    ])


def get_argument(context, arg):
    return LaunchConfiguration(arg).perform(context)
