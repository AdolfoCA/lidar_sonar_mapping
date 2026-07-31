#!/usr/bin/env python3
"""
perf_capture.launch.py
----------------------
Standalone capture workflow: brings up the three perf_tools loggers
(latency_sniffer, resource_logger, map_growth_logger) that subscribe to the
mapping-stack topics. It does NOT play the bag — you run the bag yourself in a
separate terminal:

    # Terminal 1: the mapping pipeline (use_sim_time:=true)
    # Terminal 2: this launch file
    ros2 launch perf_tools perf_capture.launch.py
    # Terminal 3: the replay
    ros2 bag play <bag> --clock --rate 1.0

All loggers share one run_id. Output goes to <out_root>/<run_id>/:
    messages.csv  resources.csv  map_growth.csv

Override anything at launch time, e.g.:
    ros2 launch perf_tools perf_capture.launch.py \
        run_id:=mission_1h out_root:=/data/perf_data interval:=1.0

Notes
-----
* The loggers timestamp arrivals with time.monotonic_ns() (wall clock), which is
  exactly what we want for processing-latency measurement, so they do NOT need
  use_sim_time. Stage latencies are matched offline by header.stamp.
* Stop capture with Ctrl-C in this terminal once the bag finishes, then analyse:
    python3 src/perf_tools/perf_tools/analyze_perf.py --run-dir <out_root>/<run_id>
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    # Resolve all launch configurations to concrete strings here, so we can
    # build plain Python argument lists for each Node.
    run_id = LaunchConfiguration('run_id').perform(context).strip()
    config = LaunchConfiguration('config').perform(context)
    out_root = LaunchConfiguration('out_root').perform(context)
    interval = LaunchConfiguration('interval').perform(context)

    # Share one run_id across all three loggers. If the user did not provide
    # one, mint a timestamp HERE (rather than letting each logger pick its own,
    # which would scatter the CSVs across three different folders).
    if not run_id:
        run_id = time.strftime('%Y%m%d_%H%M%S')

    common = ['--run-id', run_id, '--config', config, '--out-root', out_root]

    nodes = [
        Node(package='perf_tools', executable='latency_sniffer',
             name='latency_sniffer', output='screen', arguments=list(common)),
        Node(package='perf_tools', executable='map_growth_logger',
             name='map_growth_logger', output='screen', arguments=list(common)),
        Node(package='perf_tools', executable='resource_logger',
             name='resource_logger', output='screen',
             arguments=list(common) + ['--interval', interval]),
    ]

    return [
        LogInfo(msg=f'[perf_tools] capture run_id={run_id}  '
                    f'out={os.path.join(out_root, run_id)}'),
        LogInfo(msg='[perf_tools] loggers up. Now start the replay in another '
                    'terminal:  ros2 bag play <bag> --clock --rate 1.0  '
                    '(Ctrl-C here to stop capture.)'),
        *nodes,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('perf_tools')
    default_config = os.path.join(pkg_share, 'config.yaml')
    default_out_root = os.path.abspath(os.path.join(os.getcwd(), 'perf_data'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'run_id', default_value='',
            description='Shared run id (folder). Empty => a timestamp is minted '
                        'and shared across all three loggers.'),
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Path to perf_tools config.yaml'),
        DeclareLaunchArgument(
            'out_root', default_value=default_out_root,
            description='Root directory for perf_data/<run_id> '
                        '(default: <cwd>/perf_data)'),
        DeclareLaunchArgument(
            'interval', default_value='1.0',
            description='resource_logger sampling period (seconds)'),
        OpaqueFunction(function=_launch_setup),
    ])
