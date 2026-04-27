#!/usr/bin/env python3
"""
sonar_mapping_semantic.launch.py
---------------------------------
Full semantic sonar mapping pipeline:
  1. sonar_scan_ned          — transforms leading-edge points to the odom frame
  2. sonar_map_ned_semantic  — Beta-Bernoulli voxel map with semantic labelling
  3. save_map                — saves map to PCD on service call
  4. object_cluster_node     — DBSCAN clustering on structure/object voxels
  5. sdf_fitting_node        — SDF fitting on clusters

Config: sonar_semantic.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg    = get_package_share_directory('sonar_map')
    config = os.path.join(pkg, 'config', 'sonar_semantic.yaml')

    arg_sonar_frame = DeclareLaunchArgument(
        'sonar_frame',
        default_value='blueview',
        description='Active sonar TF frame: "blueview" or "oculus"',
    )

    arg_sonar_topic = DeclareLaunchArgument(
        'sonar_cloud_topic',
        default_value='/blueview/point2/leading',
        description='Sonar leading-edge point cloud input topic',
    )

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

    node_sonar_map = Node(
        package='sonar_map',
        executable='sonar_map_ned_semantic',
        name='sonar_map_ned',
        output='screen',
        parameters=[config],
    )

    node_save_map = Node(
        package='sonar_map',
        executable='save_map',
        name='save_map',
        output='screen',
    )

    node_object_cluster = Node(
        package='sonar_map',
        executable='object_cluster_node',
        name='object_cluster_node',
        output='screen',
        parameters=[config],
    )

    node_sdf_fitting = Node(
        package='sonar_map',
        executable='sdf_fitting_node',
        name='sdf_fitting_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        arg_sonar_frame,
        arg_sonar_topic,
        node_sonar_scan,
        node_sonar_map,
        node_save_map,
        node_object_cluster,
        node_sdf_fitting,
    ])
