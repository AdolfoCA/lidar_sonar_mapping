#!/usr/bin/env python3
"""Launch the seabed_to_sky_viz publisher with the package config.

  ros2 launch seabed_to_sky_viz seabed_to_sky_viz.launch.py
  # override the chunk dirs / config for a different mission:
  ros2 launch seabed_to_sky_viz seabed_to_sky_viz.launch.py config:=/path/to/my.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('seabed_to_sky_viz'), 'config', 'seabed_to_sky_viz.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_cfg,
                              description='params YAML for seabed_to_sky_viz'),
        Node(
            package='seabed_to_sky_viz',
            executable='seabed_to_sky_viz',
            name='seabed_to_sky_viz',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
