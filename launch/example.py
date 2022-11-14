from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
        Node(
            package='network_systems',
            node_namespace='example',
            node_executable='example',
            node_name='cached_fib'
        )
   ])
