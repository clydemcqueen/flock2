"""Simulate one or more Tello drones in Gazebo, using ArUco markers and fiducial_vlam for localization"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    # 1 or more drones:
    drones = ['drone1', 'drone2', 'drone3', 'drone4']

    # Starting locations:
    starting_locations = [
        # Face the wall of markers in fiducial.world
        ['-2.5', '1.5', '1', '0'],
        ['-1.5', '0.5', '1', '0.785'],
        ['-0.5', '1.5', '1', '0'],
        ['-1.5', '2.5', '1', '-0.785']

        # Face all 4 directions in f2.world
        # ['-2.5', '1.5', '1', '0'],
        # ['-1.5', '0.5', '1', '1.57'],
        # ['-0.5', '1.5', '1', '3.14'],
        # ['-1.5', '2.5', '1', '-1.57']
    ]

    tello_gazebo_path = get_package_share_directory('tello_gazebo')
    tello_description_path = get_package_share_directory('tello_description')

    world_path = os.path.join(tello_gazebo_path, 'worlds', 'fiducial.world')
    map_path = os.path.join(tello_gazebo_path, 'worlds', 'fiducial_map.yaml')

    # Global entities
    entities = [
        # Launch Gazebo, loading tello.world
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',      # Publish /clock
            '-s', 'libgazebo_ros_factory.so',   # Provide gazebo_ros::Node
            world_path
        ], output='screen'),

        # Load and publish a known map
        Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
             node_name='vmap_main', parameters=[{
                'use_sim_time': True,                           # Use /clock if available
                'publish_tfs': 1,                               # Publish marker /tf
                'marker_length': 0.1778,                        # Marker length
                'marker_map_load_full_filename': map_path,      # Load a pre-built map from disk
                'make_not_use_map': 0                           # Don't save a map to disk
            }]),

        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', node_executable='joy_node', output='screen',
             node_name='joy_node', parameters=[{
                'use_sim_time': True,                           # Use /clock if available
            }]),

        # Flock controller (basically a joystick multiplexer, also starts/stops missions)
        Node(package='flock2', node_executable='flock_base', output='screen',
             node_name='flock_base', parameters=[{
                'use_sim_time': True,                           # Use /clock if available
                'drones': drones
            }]),

        # WIP: planner
        Node(package='flock2', node_executable='planner_node', output='screen',
             node_name='planner_node', parameters=[{
                'use_sim_time': True,                           # Use /clock if available
                'drones': drones,
                'arena_x': -5.0,
                'arena_y': -5.0,
                'arena_z': 10.0,
            }]),
    ]

    # Per-drone entities
    for idx, namespace in enumerate(drones):
        suffix = '_' + str(idx + 1)
        urdf_path = os.path.join(tello_description_path, 'urdf', 'tello' + suffix + '.urdf')

        entities.extend([
            # Add a drone to the simulation
            Node(package='tello_gazebo', node_executable='inject_entity.py', output='screen',
                 arguments=[urdf_path]+starting_locations[idx]),

            # Publish base_link to camera_link tf
            # Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen',
            #      node_name=namespace+'_tf_pub', arguments=[urdf_path], parameters=[{
            #         'use_sim_time': True,                       # Use /clock if available
            #     }]),

            # Localize this drone against the map
            # Future: need odometry for base_link, not camera_link
            Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
                 node_name='vloc_main', node_namespace=namespace, parameters=[{
                    'use_sim_time': True,                           # Use /clock if available
                    'publish_tfs': 1,                               # Publish drone and camera /tf
                    'stamp_msgs_with_current_time': 0,              # Use incoming message time, not now()
                    'base_frame_id': 'base_link' + suffix,
                    'map_init_pose_z': -0.035,
                    'camera_frame_id': 'camera_link' + suffix,
                    'base_odometry_pub_topic': 'base_odom',
                    'sub_camera_info_best_effort_not_reliable': 1,  # Gazebo camera uses 'best effort'
                }]),

            # Drone controller
            Node(package='flock2', node_executable='drone_base', output='screen',
                 node_name='drone_base', node_namespace=namespace, parameters=[{
                    'use_sim_time': True,                       # Use /clock if available
                }]),
        ])

    return LaunchDescription(entities)
