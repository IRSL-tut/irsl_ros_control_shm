import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('irsl_ros_control_shm')
    urdf_path = os.path.join(package_share, 'test', '5links.urdf')
    controller_config = os.path.join(package_share, 'test', 'ros_control.yaml')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            remappings=[('~/robot_description', '/robot_description')],
            parameters=[
                {'robot_description': robot_description},
                controller_config,
            ],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['trajectory_controller'],
            output='screen',
        ),
    ])