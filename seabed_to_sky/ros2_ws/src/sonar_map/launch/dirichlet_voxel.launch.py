#!/usr/bin/env python3
"""
dirichlet_voxel.launch.py
-------------------------
Launches the Dirichlet-Categorical voxel mapping pipeline end to end:

  0. sonar_scan_ned     — transforms raw sonar returns into the odom frame
  1. seabed_estimator   — dual scalar Kalman filter (seabed depth + intensity)
  2. return_classifier  — labels returns FREE / SEABED / OBJECT / STRUCTURE
  3. voxel_mapper       — Dirichlet voxel map + event-based promotion protocol
  4. seabed_surface     — coarse seabed patches + smoothed Delaunay mesh

All nodes are configured from a single file: config/dirichlet_voxel.yaml.

To swap sonar at launch time:
  ros2 launch sonar_map dirichlet_voxel.launch.py sonar_frame:=oculus

External inputs (not started here): the registered LiDAR cloud on
`/cloud_registered` and `odometry`, both from the rest of the seabed_to_sky
stack (or replayed from a bag).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('sonar_map')
    config = os.path.join(pkg, 'config', 'dirichlet_voxel.yaml')

    # ------------------------------------------------------------------ #
    # Arguments                                                           #
    # ------------------------------------------------------------------ #
    arg_sonar_frame = DeclareLaunchArgument(
        'sonar_frame',
        default_value='blueview_sonar',
        description='Active sonar TF frame: "blueview_sonar" or "oculus"',
    )
    arg_sonar_topic = DeclareLaunchArgument(
        'sonar_cloud_topic',
        default_value='/blueview/point2/leading',
        description='Raw sonar point cloud input topic',
    )

    # ------------------------------------------------------------------ #
    # 0. sonar_scan_ned  — raw sonar returns -> odom-frame `sonar_scan`   #
    # ------------------------------------------------------------------ #
    node_sonar_scan = Node(
        package='sonar_map',
        executable='sonar_scan_ned',
        name='sonar_scan_ned',
        output='screen',
        parameters=[
            config,
            {'sonar_frame':       LaunchConfiguration('sonar_frame')},
            {'sonar_cloud_topic': LaunchConfiguration('sonar_cloud_topic')},
        ],
    )

    # ------------------------------------------------------------------ #
    # 1. seabed_estimator                                                 #
    # ------------------------------------------------------------------ #
    node_seabed_estimator = Node(
        package='sonar_map',
        executable='seabed_estimator',
        name='seabed_estimator',
        output='screen',
        parameters=[config],
    )

    # ------------------------------------------------------------------ #
    # 2. return_classifier                                                #
    # ------------------------------------------------------------------ #
    node_return_classifier = Node(
        package='sonar_map',
        executable='return_classifier',
        name='return_classifier',
        output='screen',
        parameters=[config],
    )

    # ------------------------------------------------------------------ #
    # 3. voxel_mapper                                                     #
    # ------------------------------------------------------------------ #
    node_voxel_mapper = Node(
        package='sonar_map',
        executable='voxel_mapper',
        name='voxel_mapper',
        output='screen',
        parameters=[config],
    )

    # ------------------------------------------------------------------ #
    # 4. seabed_surface                                                   #
    # ------------------------------------------------------------------ #
    node_seabed_surface = Node(
        package='sonar_map',
        executable='seabed_surface',
        name='seabed_surface',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        arg_sonar_frame,
        arg_sonar_topic,
        node_sonar_scan,
        node_seabed_estimator,
        node_return_classifier,
        node_voxel_mapper,
        node_seabed_surface,
    ])
