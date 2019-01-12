from launch import LaunchDescription
from launch_ros.actions import Node

# Launch a flock of drones


def generate_launch_description():
    p = [{'drones': ['dr1', 'dr2', 'dr3']}]

    return LaunchDescription([
        Node(package='joy', node_executable='joy_node', output='screen'),
        Node(package='flock2', node_executable='flock_base', node_name='flock_base', parameters=p, output='screen'),
    ])
