import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# USB camera launch, useful for testing the image processing pipeline without flying a drone


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('flock2'), 'urdf', 'tello.urdf')
    return LaunchDescription([
        ExecuteProcess(cmd=['rviz2', '-d', 'src/flock2/launch/default.rviz'], output='screen'),
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen', arguments=[urdf]),
        Node(package='joy', node_executable='joy_node', output='screen'),
        Node(package='flock2', node_executable='usb_camera', output='screen'),
        Node(package='flock2', node_executable='detect_markers', output='screen'),
        Node(package='flock2', node_executable='filter.py', output='screen'),
        Node(package='flock2', node_executable='flock_base.py', output='screen'),
        Node(package='flock2', node_executable='flock_simple_path.py', output='screen'),
    ])
