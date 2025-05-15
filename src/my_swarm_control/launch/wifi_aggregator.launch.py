from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_swarm_control',
            executable='wifi_aggregator',
            name='wifi_aggregator'
        )
    ])