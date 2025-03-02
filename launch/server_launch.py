from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_demo',
            executable='start_docker.sh',
            name='server',
            shell=True,
            output="both"
        ),
        Node(
            package='ros2_demo',
            executable='serial_writer',
            name='serial_writer'
        )
    ])