from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, PushRosNamespace
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from pathlib import Path
from xacro import process_file

def generate_launch_description():
    # Declare launch arguments
    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Whether to launch Gazebo'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Namespace for this robot'
    )

    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='x position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.1', description='z position')

    # Load URDF
    description_package_path = Path(__file__).parent.parent
    urdf_file = description_package_path / 'urdf' / 'my_robot.urdf.xacro'

    # with open(urdf_file, 'r') as infp:
    #     robot_description_content = infp.read()
    robot_description_content = process_file(
            str(urdf_file),
            mappings={"prefix": f"{namespace}/"}
        ).toxml()

    namespace = LaunchConfiguration('namespace')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')

    # Robot-specific nodes grouped under namespace
    robot_group = GroupAction([
        PushRosNamespace(namespace),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description_content},
                {'frame_prefix': [namespace, '/']}
            ]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[{'use_gui': False}]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', namespace,
                '-topic', 'robot_description',
                '-x', x, '-y', y, '-z', z
            ],
            output='screen'
        )
    ])

    # Start Gazebo only once, outside of robot group
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    return LaunchDescription([
        use_gazebo_arg,
        namespace_arg,
        x_arg,
        y_arg,
        z_arg,
        gazebo_process,
        robot_group
    ])
