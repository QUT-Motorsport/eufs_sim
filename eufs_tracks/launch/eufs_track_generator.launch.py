from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Launch the track generator
        Node(
            name='eufs_track_generator',
            package='eufs_tracks',
            executable='eufs_tracks',
            output='screen',
        ),
    ])
