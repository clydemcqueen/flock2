import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# Launch a single drone.


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('flock2'), 'urdf', 'tello.urdf')
    return LaunchDescription([
        # Rviz
        ExecuteProcess(cmd=['rviz2', '-d', 'install/flock2/share/flock2/launch/one.rviz'], output='screen'),

        # Publish static transforms
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             arguments=[urdf]),

        # Driver
        Node(package='tello_driver', node_executable='tello_driver', output='screen',
             node_name='tello_driver', node_namespace='solo'),

        # Joystick
        Node(package='joy', node_executable='joy_node', output='screen'),

        # Flock controller
        Node(package='flock2', node_executable='flock_base', output='screen'),

        # Drone controller
        Node(package='flock2', node_executable='drone_base', output='screen',
             node_name='drone_base', node_namespace='solo'),

        # Mapper
        Node(package='flock_vlam', node_executable='vmap_node', output='screen'),

        # Visual localizer
        Node(package='flock_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_node', node_namespace='solo'),

        # Kalman filter
        Node(package='flock2', node_executable='filter_node', output='screen',
             node_name='filter_node', node_namespace='solo'),

        # Global planner
        Node(package='flock2', node_executable='global_planner', output='screen'),
    ])
