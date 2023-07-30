from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # TODO: Read from globals configuration file

    return LaunchDescription([
        Node(
            package='network_systems',
            namespace='example',
            executable='example',
            name='cached_fib'
        )
    ])
