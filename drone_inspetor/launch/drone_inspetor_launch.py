from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_inspetor',
            executable='camera_node',
            name='camera_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='drone_inspetor',
            executable='drone_node',
            name='drone_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='drone_inspetor',
            executable='cv_node',
            name='cv_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='drone_inspetor',
            executable='depth_node',
            name='depth_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='drone_inspetor',
            executable='fsm_node',
            name='fsm_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='drone_inspetor',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='drone_inspetor',
            executable='dashboard_node',
            name='dashboard_node',
            output='screen',
            emulate_tty=True,
        ),
    ])


