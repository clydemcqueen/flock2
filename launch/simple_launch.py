from launch import LaunchDescription
from launch_ros.actions import Node

# The simplest launch


def generate_launch_description():
    return LaunchDescription([
        Node(package='flock2', node_executable='tf2_test.py', output='screen'),
    ])
