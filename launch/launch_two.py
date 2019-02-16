import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# Launch a flock of drones

# TODO use xacro or something similar to modify urdf files on-the-fly


def generate_launch_description():
    urdf1 = os.path.join(get_package_share_directory('flock2'), 'urdf', 'drone1.urdf')
    urdf2 = os.path.join(get_package_share_directory('flock2'), 'urdf', 'drone2.urdf')

    dr1_ns = 'drone1'
    dr2_ns = 'drone2'

    dr1_params = [{
        'drone_ip': '192.168.86.206',
        'command_port': '11001',
        'drone_port': '12001',
        'data_port': '13001',
        'video_port': '14001'
    }]

    dr2_params = [{
        'drone_ip': '192.168.86.212',
        'command_port': '11002',
        'drone_port': '12002',
        'data_port': '13002',
        'video_port': '14002'
    }]

    base_params = [{
        'drones': [dr1_ns, dr2_ns]
    }]

    filter1_params = [{
        'map_frame': 'map',
        'base_frame': 'base1'
    }]

    filter2_params = [{
        'map_frame': 'map',
        'base_frame': 'base2'
    }]

    return LaunchDescription([
        # Rviz
        ExecuteProcess(cmd=['rviz2', '-d', 'install/flock2/share/flock2/launch/two.rviz'], output='screen'),

        # Publish N sets of static transforms
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             arguments=[urdf1]),
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
             arguments=[urdf2]),

        # N drivers
        Node(package='tello_driver', node_executable='tello_driver', output='screen',
             node_name='driver1', node_namespace=dr1_ns, parameters=dr1_params),
        Node(package='tello_driver', node_executable='tello_driver', output='screen',
             node_name='driver2', node_namespace=dr2_ns, parameters=dr2_params),

        # Joystick
        Node(package='joy', node_executable='joy_node', output='screen'),

        # Flock controller
        Node(package='flock2', node_executable='flock_base', output='screen',
             node_name='flock_base', parameters=base_params),

        # N drone controllers
        Node(package='flock2', node_executable='drone_base', output='screen',
             node_name='base1', node_namespace=dr1_ns),
        Node(package='flock2', node_executable='drone_base', output='screen',
             node_name='base2', node_namespace=dr2_ns),

        # Mapper
        Node(package='flock_vlam', node_executable='vmap_node', output='screen'),

        # N visual localizers
        Node(package='flock_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc1', node_namespace=dr1_ns),
        Node(package='flock_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc2', node_namespace=dr2_ns),

        # N Kalman filters
        Node(package='flock2', node_executable='filter_node', output='screen',
             node_name='filter1', node_namespace=dr1_ns, parameters=filter1_params),
        Node(package='flock2', node_executable='filter_node', output='screen',
             node_name='filter2', node_namespace=dr2_ns, parameters=filter2_params),
    ])
