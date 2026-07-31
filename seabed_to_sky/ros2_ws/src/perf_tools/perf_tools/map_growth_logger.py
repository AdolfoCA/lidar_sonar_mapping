#!/usr/bin/env python3
"""map_growth_logger.py — per-update map size logger.

Subscribes to the sky map (/cloud_registered) and the seabed map (/sonar_map),
both PointCloud2, and logs one CSV row per published update:

    topic, arrival_monotonic_ns, header_stamp_ns, point_count, serialized_size_bytes

/cloud_registered is a per-scan registered cloud; /sonar_map is the cumulative
accumulated seabed map, so its point_count IS the map-growth curve over the run.

The topics with role 'fastlio_skymap' and 'seabed_map' are read from config.yaml.

Standalone:
    python3 map_growth_logger.py --run-id 20260615_120000 [--config config.yaml]
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

from perf_tools.qos_util import load_config, qos_for, default_config_path


CSV_HEADER = ['topic', 'arrival_monotonic_ns', 'header_stamp_ns',
              'point_count', 'serialized_size_bytes']

MAP_ROLES = ('fastlio_skymap', 'seabed_map')


def _stamp_ns(msg) -> str:
    hdr = getattr(msg, 'header', None)
    if hdr is None:
        return ''
    return str(int(hdr.stamp.sec) * 1_000_000_000 + int(hdr.stamp.nanosec))


class MapGrowthLogger(Node):
    def __init__(self, run_dir: str, cfg: dict):
        super().__init__('map_growth_logger')
        os.makedirs(run_dir, exist_ok=True)
        self.csv_path = os.path.join(run_dir, 'map_growth.csv')
        self._fh = open(self.csv_path, 'w', newline='')
        self._writer = csv.writer(self._fh)
        self._writer.writerow(CSV_HEADER)
        self._n = 0

        self._subs = []
        for t in cfg.get('topics', []):
            if t.get('role') not in MAP_ROLES:
                continue
            name, type_str = t['name'], t['type']
            try:
                msg_cls = get_message(type_str)
            except Exception as exc:
                self.get_logger().error(f'Cannot resolve {type_str} for {name}: {exc}')
                continue
            sub = self.create_subscription(
                msg_cls, name,
                lambda msg, topic=name: self._on_msg(topic, msg),
                qos_for(t.get('qos', 'sensor')))
            self._subs.append(sub)
            self.get_logger().info(f'map_growth subscribed: {name}')

        if not self._subs:
            self.get_logger().warn('no map topics matched roles %s' % str(MAP_ROLES))
        self.get_logger().info(f'map_growth_logger writing {self.csv_path}')

    def _on_msg(self, topic: str, msg) -> None:
        arrival = time.monotonic_ns()
        try:
            pc = int(msg.width) * int(msg.height)
        except Exception:
            pc = ''
        try:
            size = len(serialize_message(msg))
        except Exception:
            size = ''
        self._writer.writerow([topic, arrival, _stamp_ns(msg), pc, size])
        self._n += 1
        if self._n % 20 == 0:
            self._fh.flush()

    def close(self):
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:
            pass
        self.get_logger().info(f'map_growth_logger wrote {self._n} rows')


def _resolve_run_dir(out_root, run_id) -> str:
    if not out_root:
        here = os.path.dirname(os.path.abspath(__file__))
        out_root = os.path.abspath(os.path.join(here, '..', '..', '..', 'perf_data'))
    return os.path.join(out_root, run_id)


def _strip_ros_args():
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    return argv


def main(argv=None):
    parser = argparse.ArgumentParser(description='Per-update map size logger.')
    parser.add_argument('--run-id', default=None,
                        help='Run id (folder). Defaults to PERF_RUN_ID env or timestamp.')
    parser.add_argument('--config', default=None,
                        help=f'config.yaml (default: {default_config_path()})')
    parser.add_argument('--out-root', default=None, help='Root dir for perf_data')
    args = parser.parse_args(argv if argv is not None else _strip_ros_args())

    run_id = args.run_id or os.environ.get('PERF_RUN_ID') or time.strftime('%Y%m%d_%H%M%S')
    cfg = load_config(args.config)
    run_dir = _resolve_run_dir(args.out_root, run_id)

    rclpy.init()
    node = MapGrowthLogger(run_dir, cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
