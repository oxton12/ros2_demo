from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_demo',
            executable='yolo_vision',
            name='yolo_vision'
        ),
        Node(
            package='ros2_demo',
            executable='position_estimator',
            name='position_estimator'
        ),
        Node(
            package='ros2_demo',
            executable='camera_driver',
            name='camera_driver'
        )
    ])