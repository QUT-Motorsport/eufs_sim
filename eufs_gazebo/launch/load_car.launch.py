import os

import xacro

import launch
import launch.actions
import launch.conditions
import launch.substitutions
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def spawn_car(context, *args, **kwargs):
    # Get the values of the arguments
    launch_group = get_argument(context, 'launch_group')
    namespace = get_argument(context, 'namespace')
    robot_name = get_argument(context, 'robot_name')
    vehicle_model = get_argument(context, 'vehicleModel')
    command_mode = get_argument(context, 'commandMode')
    publish_tf = get_argument(context, 'publish_gt_tf')
    x = get_argument(context, 'x')
    y = get_argument(context, 'y')
    z = get_argument(context, 'z')
    roll = get_argument(context, 'roll')
    pitch = get_argument(context, 'pitch')
    yaw = get_argument(context, 'yaw')

    simulate_perception = 'true' if launch_group == 'no_perception' else 'false'
    config_file = str(os.path.join(get_package_share_directory('eufs_description'), 'robots', robot_name, 'config.yaml'))

    xacro_path = os.path.join(get_package_share_directory('eufs_description'), 'robots', robot_name, 'robot.urdf.xacro')
    urdf_path = os.path.join(get_package_share_directory('eufs_description'), 'robots', robot_name, 'robot.urdf')

    if not os.path.isfile(urdf_path):
        os.mknod(urdf_path)

    doc = xacro.process_file(xacro_path,
                             mappings={
                                 'robot_name': robot_name,
                                 'vehicle_model': vehicle_model,
                                 'command_mode': command_mode,
                                 'config_file': config_file,
                                 'publish_tf': publish_tf,
                                 'simulate_perception': simulate_perception
                             })
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    return [
        launch_ros.actions.Node(
            name='spawn_robot',
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=[
                '-entity', namespace,
                '-file', urdf_path,
                '-x', x,
                '-y', y,
                '-z', z,
                '-R', roll,
                '-P', pitch,
                '-Y', yaw,
                '-spawn_service_timeout', '30.0',
                '--ros-args', '--log-level', 'warn'
            ]
        ),

        launch_ros.actions.Node(
            name='joint_state_publisher',
            package='joint_state_publisher', executable='joint_state_publisher', output='screen',
            parameters=[{
                'robot_description': robot_description,
                'rate': 200,
            }],
            arguments=[urdf_path],
            remappings=[('/joint_states', '/eufs/joint_states')]
        ),

        launch_ros.actions.Node(
            name='robot_state_publisher',
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            parameters=[{
                'robot_description': robot_description,
                'rate': 200,
            }],
            remappings=[('/joint_states', '/eufs/joint_states')],
            arguments=['--ros-args', '--log-level', 'warn']
        ),
    ]


def generate_launch_description():
    rqt_perspective_file = os.path.join(get_package_share_directory('eufs_gazebo'), 'config', 'eufs_sim.perspective')

    return launch.LaunchDescription([
        # Launch Arguments
        launch.actions.DeclareLaunchArgument('launch_group', default_value='default',
                                             description='The launch group (default or no_perception)'),

        launch.actions.DeclareLaunchArgument('rviz', default_value='false',
                                             description='Launch RViz'),
        launch.actions.DeclareLaunchArgument('show_rqt_gui', default_value='true',
                                             description='Show the RQT GUI (with ros_can_sim GUI and EUFS Robot Steering GUI)'),

        launch.actions.DeclareLaunchArgument('namespace', default_value='eufs',
                                             description='Namespace of the gazebo robot'),
        launch.actions.DeclareLaunchArgument('robot_name', default_value='eufs',
                                             description='The name of the robot (must be directory in eufs_description/robots called {robot_name} with robot.urdf.xacro and config.yaml'),

        launch.actions.DeclareLaunchArgument('vehicleModel', default_value='DynamicBicycle',
                                             description='The vehicle model class to use in the gazebo_ros_race_car_model'),
        launch.actions.DeclareLaunchArgument('commandMode', default_value='acceleration',
                                             description='Determines whether to use acceleration or velocity to control the vehicle'),
        launch.actions.DeclareLaunchArgument('publish_gt_tf', default_value='false',
                                             description='If the gazebo_ros_race_car_model should publish the ground truth tf'),

        launch.actions.DeclareLaunchArgument('x', default_value='0',
                                             description='Vehicle initial x position'),
        launch.actions.DeclareLaunchArgument('y', default_value='0',
                                             description='Vehicle initial y position'),
        launch.actions.DeclareLaunchArgument('z', default_value='0',
                                             description='Vehicle initial z position'),
        launch.actions.DeclareLaunchArgument('roll', default_value='0',
                                             description='Vehicle initial roll'),
        launch.actions.DeclareLaunchArgument('pitch', default_value='0',
                                             description='Vehicle initial pitch'),
        launch.actions.DeclareLaunchArgument('yaw', default_value='0',
                                             description='Vehicle initial yaw'),

        launch_ros.actions.Node(
            name='rviz',
            package='rviz2', executable='rviz2',
            # TODO: Add path to RViz config as argument
            condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('rviz'))
        ),
        launch_ros.actions.Node(
            name='eufs_sim_rqt',
            package='rqt_gui', executable='rqt_gui', output='screen',
            arguments=['--force-discover', '--perspective-file', str(rqt_perspective_file)],
            condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('show_rqt_gui'))
        ),

        # Spawn the car!!!
        launch.actions.OpaqueFunction(function = spawn_car)
    ])

def get_argument(context, arg):
    return launch.substitutions.LaunchConfiguration(arg).perform(context)
