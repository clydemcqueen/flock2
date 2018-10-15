from launch import LaunchDescription
from launch_ros.actions import Node

# USB camera launch, useful for testing the image processing pipeline without flying a drone


def generate_launch_description():
    return LaunchDescription([
        Node(package='flock2', node_executable='usb_camera.py', output='screen'),
    ])
