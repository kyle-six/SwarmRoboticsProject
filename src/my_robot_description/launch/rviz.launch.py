from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from pathlib import Path

def generate_launch_description():
    # Declare RViz launch toggle
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )

    # Paths to important files
    description_package_path = Path(__file__).parent.parent
    urdf_file = description_package_path / 'urdf' / 'my_robot.urdf'
    rviz_file = description_package_path / 'rviz' / 'my_robot.rviz'

    # Read the URDF into a string
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': False}]
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_file)],
        output='screen'
    )

    return LaunchDescription([
        use_rviz_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
