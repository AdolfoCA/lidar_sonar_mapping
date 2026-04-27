#!/usr/bin/env python3
"""
sonar_mapping_no_semantic.launch.py
------------------------------------
Launches the lightweight (no-semantic) sonar mapping pipeline:
  1. sonar_scan_ned           — transforms sonar scans into the odom frame via TF
  2. sonar_map_ned_no_semantic — accumulates scans into a basic voxel map
  3. save_map                 — saves map to PCD on request

No object clustering or SDF fitting nodes are launched.

To swap sonar at launch time:
  ros2 launch sonar_map sonar_mapping_no_semantic.launch.py sonar_frame:=oculus
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('sonar_map')
    config = os.path.join(pkg, 'config', 'sonar_no_semantic.yaml')

    # ------------------------------------------------------------------ #
    # Arguments                                                           #
    # ------------------------------------------------------------------ #

    arg_sonar_frame = DeclareLaunchArgument(
        'sonar_frame',
        default_value='blueview',
        description='Active sonar TF frame: "blueview" or "oculus"',
    )

    arg_sonar_topic = DeclareLaunchArgument(
        'sonar_cloud_topic',
        default_value='/blueview/point2/leading',
        description='Sonar point cloud input topic',
    )

    # ------------------------------------------------------------------ #
    # 1. sonar_scan_ned                                                   #
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
    # 2. sonar_map_ned (no semantic)                                      #
    # ------------------------------------------------------------------ #

    node_sonar_map = Node(
        package='sonar_map',
        executable='sonar_map_ned_no_semantic',
        name='sonar_map_ned',
        output='screen',
        parameters=[config],
    )

    # ------------------------------------------------------------------ #
    # 3. save_map                                                         #
    # ------------------------------------------------------------------ #

    node_save_map = Node(
        package='sonar_map',
        executable='save_map',
        name='save_map',
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # Assemble                                                            #
    # ------------------------------------------------------------------ #

    return LaunchDescription([
        arg_sonar_frame,
        arg_sonar_topic,
        node_sonar_scan,
        node_sonar_map,
        node_save_map,
    ])
