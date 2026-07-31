#!/usr/bin/env python3
"""latency_sniffer.py — zero-touch arrival logger for the seabed-to-sky stack.

Subscribes to every topic listed in config.yaml and, for each message, writes ONE
CSV row with the raw facts needed to derive per-stage latency OFFLINE:

    topic, header_stamp_ns, arrival_monotonic_ns, msg_size_bytes, point_count

It performs NO latency math online — stage latencies A-D are reconstructed later by
analyze_perf.py via header.stamp matching. A buffered writer keeps per-callback
overhead negligible.

Run standalone (no colcon needed):
    python3 latency_sniffer.py --run-id 20260615_120000 [--config config.yaml]
                               [--out-root ../../perf_data]

Or via ros2:  ros2 run perf_tools latency_sniffer --ros-args -p run_id:=...
(the standalone argparse path is the supported one and what run_perf_capture.sh uses)
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


CSV_HEADER = [
    'topic',
    'header_stamp_ns',
    'arrival_monotonic_ns',
    'msg_size_bytes',
    'point_count',
]


def _stamp_ns(msg) -> str:
    """Return header.stamp in ns, or '' if the message has no header."""
    hdr = getattr(msg, 'header', None)
    if hdr is None:
        return ''
    st = hdr.stamp
    return str(int(st.sec) * 1_000_000_000 + int(st.nanosec))


def _point_count(msg) -> str:
    """width*height for PointCloud2-like messages, else ''."""
    if hasattr(msg, 'width') and hasattr(msg, 'height'):
        try:
            return str(int(msg.width) * int(msg.height))
        except Exception:
            return ''
    return ''


class LatencySniffer(Node):
    def __init__(self, run_dir: str, cfg: dict):
        super().__init__('latency_sniffer')
        self.flush_rows = int(cfg.get('run', {}).get('csv_flush_rows', 200))

        os.makedirs(run_dir, exist_ok=True)
        self.csv_path = os.path.join(run_dir, 'messages.csv')
        # line-buffered file; we flush explicitly every flush_rows
        self._fh = open(self.csv_path, 'w', newline='')
        self._writer = csv.writer(self._fh)
        self._writer.writerow(CSV_HEADER)
        self._rows_since_flush = 0
        self._total_rows = 0

        self._subs = []
        for t in cfg.get('topics', []):
            name = t['name']
            type_str = t['type']            # "pkg/msg/Type"
            qos_kind = t.get('qos', 'sensor')
            try:
                msg_cls = get_message(type_str)
            except Exception as exc:
                self.get_logger().error(
                    f'Cannot resolve message type "{type_str}" for {name}: {exc}')
                continue
            # bind topic name into the callback via default arg
            sub = self.create_subscription(
                msg_cls, name,
                lambda msg, topic=name: self._on_msg(topic, msg),
                qos_for(qos_kind))
            self._subs.append(sub)
            self.get_logger().info(f'subscribed: {name} [{type_str}] qos={qos_kind}')

        self.get_logger().info(
            f'latency_sniffer writing {self.csv_path} '
            f'({len(self._subs)} topics, flush every {self.flush_rows} rows)')

    def _on_msg(self, topic: str, msg) -> None:
        # arrival captured FIRST, before any other work
        arrival = time.monotonic_ns()
        try:
            size = len(serialize_message(msg))
        except Exception:
            size = ''
        self._writer.writerow([
            topic,
            _stamp_ns(msg),
            arrival,
            size,
            _point_count(msg),
        ])
        self._total_rows += 1
        self._rows_since_flush += 1
        if self._rows_since_flush >= self.flush_rows:
            self._fh.flush()
            self._rows_since_flush = 0

    def close(self) -> None:
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:
            pass
        self.get_logger().info(
            f'latency_sniffer wrote {self._total_rows} rows to {self.csv_path}')


def _resolve_run_dir(args) -> str:
    out_root = args.out_root
    if not out_root:
        # default: <workspace>/src/perf_tools/../../perf_data  i.e. ws/perf_data
        here = os.path.dirname(os.path.abspath(__file__))
        out_root = os.path.abspath(os.path.join(here, '..', '..', '..', 'perf_data'))
    return os.path.join(out_root, args.run_id)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Zero-touch arrival logger; one CSV row per message.')
    parser.add_argument('--run-id', required=False, default=None,
                        help='Run id (folder name). Defaults to PERF_RUN_ID env or a timestamp.')
    parser.add_argument('--config', default=None,
                        help=f'Path to config.yaml (default: {default_config_path()})')
    parser.add_argument('--out-root', default=None,
                        help='Root dir for perf_data (default: <ws>/perf_data)')
    args = parser.parse_args(argv if argv is not None else _strip_ros_args())

    if not args.run_id:
        args.run_id = os.environ.get('PERF_RUN_ID') or time.strftime('%Y%m%d_%H%M%S')

    cfg = load_config(args.config)
    run_dir = _resolve_run_dir(args)

    rclpy.init()
    node = LatencySniffer(run_dir, cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _strip_ros_args():
    """Drop ROS-injected args (after --ros-args) so argparse sees only ours."""
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    return argv


if __name__ == '__main__':
    main()
