from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_hardware',
            executable='wifi_mesh_node',
            name='wifi_mesh_node',
            output='screen'
        )
    ])
