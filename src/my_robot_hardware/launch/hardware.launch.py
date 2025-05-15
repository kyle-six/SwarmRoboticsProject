from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # urdf_path = os.path.join(
    #     get_package_share_directory('my_robot_description'),
    #     'urdf',
    #     'my_robot.urdf'
    # )

    return LaunchDescription([
        Node(
            package='my_robot_hardware',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
        ),
        # Add any other hardware-specific nodes here
        # ...
    ])
