from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_hardware',
            executable='wifi_ibss_node',
            name='wifi_ibss_node',
            namespace='robot1',  # TODO Change per robot
            parameters=[{'interface': 'wlan0'}],
            output='screen'
        )
    ])