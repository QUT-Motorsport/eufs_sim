import os
from os.path import join
from os.path import isfile

import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def spawn_car(context, *args, **kwargs):
    # Get the values of the arguments
    launch_group = get_argument(context, 'launch_group')
    namespace = get_argument(context, 'namespace')
    robot_name = get_argument(context, 'robot_name')
    vehicle_model = get_argument(context, 'vehicleModel')
    command_mode = get_argument(context, 'commandMode')
    vehicle_model_config = get_argument(context, "vehicleModelConfig")
    publish_tf = get_argument(context, 'publish_gt_tf')
    pub_ground_truth = get_argument(context, 'pub_ground_truth')
    x = get_argument(context, 'x')
    y = get_argument(context, 'y')
    z = get_argument(context, 'z')
    roll = get_argument(context, 'roll')
    pitch = get_argument(context, 'pitch')
    yaw = get_argument(context, 'yaw')

    simulate_perception = 'true' if launch_group == 'no_perception' else 'false'
    config_file = join(get_package_share_directory('eufs_racecar'), 'robots', robot_name,
                       vehicle_model_config)
    noise_file = join(get_package_share_directory('eufs_models'), 'config', 'noise.yaml')
    recolor_config = join(get_package_share_directory('eufs_plugins'), 'config',
                          'cone_recolor.yaml')
    bounding_boxes_file = os.path.join(get_package_share_directory('eufs_plugins'),
                                       'config', 'boundingBoxes.yaml')

    xacro_path = join(get_package_share_directory('eufs_racecar'),
                      'robots', robot_name, 'robot.urdf.xacro')
    urdf_path = join(get_package_share_directory('eufs_racecar'),
                     'robots', robot_name, 'robot.urdf')

    if not isfile(urdf_path):
        os.mknod(urdf_path)

    doc = xacro.process_file(xacro_path,
                             mappings={
                                 'robot_name': robot_name,
                                 'vehicle_model': vehicle_model,
                                 'command_mode': command_mode,
                                 'config_file': config_file,
                                 'noise_config': noise_file,
                                 'recolor_config': recolor_config,
                                 'publish_tf': publish_tf,
                                 'simulate_perception': simulate_perception,
                                 'pub_ground_truth': pub_ground_truth,
                                 'bounding_box_settings': bounding_boxes_file,
                             })
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    return [
        Node(
            name='spawn_robot',
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', namespace,
                '-file', urdf_path,
                '-x', x,
                '-y', y,
                '-z', z,
                '-R', roll,
                '-P', pitch,
                '-Y', yaw,
                '-spawn_service_timeout', '60.0',
                '--ros-args', '--log-level', 'warn'
            ]
        ),

        Node(
            name='joint_state_publisher',
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'rate': 200,
            }],
            arguments=[urdf_path],
            remappings=[('/joint_states', '/eufs/joint_states')]
        ),

        Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'rate': 200,
            }],
            remappings=[('/joint_states', '/eufs/joint_states')],
            arguments=['--ros-args', '--log-level', 'warn']
        ),
    ]


def generate_launch_description():
    rqt_perspective_file = join(get_package_share_directory('eufs_rqt'),
                                'config', 'eufs_sim.perspective')

    rviz_config_file = join(
        get_package_share_directory('eufs_launcher'), 'config', 'default.rviz')

    default_user_config_file = join(os.path.expanduser("~"),
                                    ".rviz2", "default.rviz")
    if os.path.isfile(default_user_config_file):
        rviz_config_file = default_user_config_file

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('launch_group', default_value='default',
                              description='The launch group (default or '
                                          'no_perception)'),

        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz'),

        DeclareLaunchArgument('show_rqt_gui', default_value='true',
                              description='Show the RQT GUI (with '
                                          'ros_can_sim GUI and EUFS Robot '
                                          'Steering GUI)'),

        DeclareLaunchArgument('namespace', default_value='eufs',
                              description='Namespace of the gazebo robot'),

        DeclareLaunchArgument('robot_name', default_value='eufs',
                              description='The name of the robot (must be '
                                          'directory in eufs_racecar/robots '
                                          'called '
                                          '{robot_name} with '
                                          'robot.urdf.xacro and {'
                                          'vehicle_model_config}'),

        DeclareLaunchArgument('vehicleModel', default_value='DynamicBicycle',
                              description='The vehicle model class to use in '
                                          'the gazebo_ros_race_car_model'),

        DeclareLaunchArgument('commandMode', default_value='acceleration',
                              description='Determines whether to use '
                                          'acceleration or velocity to '
                                          'control the vehicle'),

        DeclareLaunchArgument('vehicleModelConfig',
                              default_value='configDry.yaml',
                              description="Determines the file from which "
                                          "the vehicle model parameters are "
                                          "read"),

        DeclareLaunchArgument('publish_gt_tf', default_value='false',
                              description='If the gazebo_ros_race_car_model '
                                          'should publish the ground truth '
                                          'tf'),

        DeclareLaunchArgument('x', default_value='0',
                              description='Vehicle initial x position'),
        DeclareLaunchArgument('y', default_value='0',
                              description='Vehicle initial y position'),
        DeclareLaunchArgument('z', default_value='0',
                              description='Vehicle initial z position'),
        DeclareLaunchArgument('roll', default_value='0',
                              description='Vehicle initial roll'),
        DeclareLaunchArgument('pitch', default_value='0',
                              description='Vehicle initial pitch'),
        DeclareLaunchArgument('yaw', default_value='0',
                              description='Vehicle initial yaw'),

        Node(
            name='rviz',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),

        Node(
            name='eufs_sim_rqt',
            package='rqt_gui',
            executable='rqt_gui',
            output='screen',
            arguments=['--force-discover', '--perspective-file',
                       str(rqt_perspective_file)],
            condition=IfCondition(LaunchConfiguration('show_rqt_gui'))
        ),

        # Spawn the car!!!
        OpaqueFunction(function=spawn_car)
    ])


def get_argument(context, arg):
    return LaunchConfiguration(arg).perform(context)
