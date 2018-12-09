import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# USB camera launch, useful for testing the image processing pipeline without flying a drone


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('flock2'), 'urdf', 'tello.urdf')
    return LaunchDescription([
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen', arguments=[urdf]),
        Node(package='joy', node_executable='joy_node', output='screen'),
        Node(package='flock2', node_executable='usb_camera.py', output='screen'),
        Node(package='flock2', node_executable='filter.py', output='screen'),
        Node(package='flock2', node_executable='flock_base.py', output='screen'),
        Node(package='flock2', node_executable='flock_simple_path.py', output='screen'),
    ])
