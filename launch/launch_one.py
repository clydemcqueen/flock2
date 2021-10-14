import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# Launch a single drone.


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello.urdf')
    return LaunchDescription([
        # Rviz
        ExecuteProcess(cmd=['rviz2', '-d', 'install/flock2/share/flock2/launch/one.rviz'], output='screen'),

        # Publish static transforms
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             arguments=[urdf]),

        # Driver
        Node(package='tello_driver', executable='tello_driver_main', output='screen',
             name='tello_driver', namespace='solo'),

        # Joystick
        Node(package='joy', executable='joy_node', output='screen'),

        # Flock controller
        Node(package='flock2', executable='flock_base', output='screen'),

        # Drone controller
        Node(package='flock2', executable='drone_base', output='screen',
             name='drone_base', namespace='solo'),

        # Mapper
        Node(package='fiducial_vlam', executable='vmap_main', output='screen'),

        # Visual localizer
        Node(package='fiducial_vlam', executable='vloc_main', output='screen',
             name='vloc_main', namespace='solo'),

        # WIP: planner
        Node(package='flock2', executable='planner_node', output='screen'),
    ])
