#!/usr/bin/env python3
"""
mission_start.launch.py
-----------------------
One-shot long-mission launcher. Brings up:
  * the perf_tools capture loggers (latency/resource/map-growth) -> mission/perf_data/<run_id>
  * the mission_supervisor (latency-triggered sonar save+reset / lidar save+prune)

It does NOT start the mapping stack itself (spark-fast-lio + sonar_map + sonar_scan)
so you can keep launching those exactly as you do today; start them first, then this.
Set `with_sonar_map:=true` to also bring up the non-semantic sonar mapping here.

  ros2 launch mission_supervisor mission_start.launch.py
  # then, at mission end, Ctrl-C and plot:
  python3 src/perf_tools/perf_tools/plot_perf_mission.py \
      --run-dir <mission_dir>/perf_data/<run_id> --mission-dir <mission_dir>

At end of mission you have: mission/sonar/*.pcd, mission/lidar/*.pcd,
mission/perf_data/<run_id>/{messages,resources,map_growth}.csv, mission/events.csv,
and mission/plots/*.png.
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def _setup(context, *a, **k):
    mission_dir = LaunchConfiguration('mission_dir').perform(context)
    run_id = LaunchConfiguration('run_id').perform(context).strip() or time.strftime('%Y%m%d_%H%M%S')
    perf_share = get_package_share_directory('perf_tools')
    sup_share = get_package_share_directory('mission_supervisor')
    sup_cfg = os.path.join(sup_share, 'config', 'mission_supervisor.yaml')

    perf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(perf_share, 'launch', 'perf_capture.launch.py')),
        launch_arguments={'run_id': run_id,
                          'out_root': os.path.join(mission_dir, 'perf_data')}.items())

    supervisor = Node(
        package='mission_supervisor', executable='mission_supervisor',
        name='mission_supervisor', output='screen',
        parameters=[sup_cfg, {'mission_dir': mission_dir,
                              'enable_actions': LaunchConfiguration('enable_actions')}])

    out = [LogInfo(msg=f'[mission] run_id={run_id} mission_dir={mission_dir}'), perf, supervisor]

    if LaunchConfiguration('with_sonar_map').perform(context).lower() in ('true', '1'):
        sonar_share = get_package_share_directory('sonar_map')
        out.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(sonar_share, 'launch', 'sonar_mapping_no_semantic.launch.py'))))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mission_dir', default_value='/home/rosdev/ros2_ws/mission'),
        DeclareLaunchArgument('run_id', default_value=''),
        DeclareLaunchArgument('enable_actions', default_value='true'),
        DeclareLaunchArgument('with_sonar_map', default_value='false',
                              description='also launch sonar_mapping_no_semantic here'),
        OpaqueFunction(function=_setup),
    ])
