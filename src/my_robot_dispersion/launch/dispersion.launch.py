from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_dispersion',
            executable='rssi_simulator',
            name='rssi_simulator'
        ),
        Node(
            package='robot_dispersion',
            executable='robot_node',
            namespace='robot_0',
            arguments=['0'],
            name='robot_0_node'
        ),
        Node(
            package='robot_dispersion',
            executable='robot_node',
            namespace='robot_1',
            arguments=['1'],
            name='robot_1_node'
        ),
        Node(
            package='robot_dispersion',
            executable='robot_node',
            namespace='robot_2',
            arguments=['2'],
            name='robot_2_node'
        ),
    ])