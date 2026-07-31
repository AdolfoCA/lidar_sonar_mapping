#!/usr/bin/env python3
"""
sonar_map_ned.py
----------------
Accumulates per-scan sonar point clouds (already in the odom frame, published
by sonar_scan_ned.py) into a growing map and re-publishes it at a fixed rate.

All tunable values are read from ROS2 parameters supplied by the unified
config file  seabed_to_sky.yaml  (section sonar_map_NED).

Parameters (with defaults):
    input_topic   "sonar_scan"
    output_topic  "sonar_map"
    map_frame     "odom"
    max_points    10000000
    publish_rate  1.0          (Hz)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger

import numpy as np
from pathlib import Path
from datetime import datetime


class SonarMapNode(Node):

    def __init__(self):
        super().__init__('sonar_map_ned')

        # ------------------------------------------------------------------ #
        # Parameters — all sourced from seabed_to_sky.yaml                   #
        # ------------------------------------------------------------------ #
        self.declare_parameter('input_topic',  'sonar_scan')
        self.declare_parameter('output_topic', 'sonar_map')
        self.declare_parameter('map_frame',    'odom')
        self.declare_parameter('max_points',   10_000_000)
        self.declare_parameter('publish_rate', 1.0)
        # Long-mission: where save_and_reset dumps the map before clearing RAM.
        self.declare_parameter('mission_dir', '/home/rosdev/ros2_ws/mission')

        input_topic  = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.map_frame_  = self.get_parameter('map_frame').value
        self.max_points_ = self.get_parameter('max_points').value
        publish_rate     = self.get_parameter('publish_rate').value
        self.mission_dir_ = Path(self.get_parameter('mission_dir').value)

        # ------------------------------------------------------------------ #
        # State                                                               #
        # ------------------------------------------------------------------ #
        self.accumulated_: np.ndarray | None = None

        # ------------------------------------------------------------------ #
        # Pub / Sub / Timer                                                   #
        # ------------------------------------------------------------------ #
        self.map_pub_ = self.create_publisher(PointCloud2, output_topic, 10)

        # perf-measurement (stage D): emit the just-integrated input scan's
        # header.stamp on a debug topic the instant the scan is folded into the
        # map. An external latency sniffer matches this debug stamp to the
        # leading-edge input by header.stamp to recover the EXACT per-scan insert
        # latency (the timer-published /sonar_map below carries no single input
        # stamp, so it is only used for cumulative map size / refresh interval).
        self.insert_dbg_pub_ = self.create_publisher(
            PointStamped, output_topic + '/last_insert_stamp', 10)

        self.cloud_sub_ = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, 1)

        self.timer_ = self.create_timer(
            1.0 / publish_rate, self.publish_map)

        # Long-mission health: save the current map to disk then start a FRESH
        # empty map WITHOUT restarting the node (subscriptions/timer stay alive,
        # so no scans are lost beyond the save itself).
        self.save_reset_srv_ = self.create_service(
            Trigger, 'sonar_map/save_and_reset', self.save_and_reset_cb)

        self.get_logger().info(
            f'sonar_map_NED started\n'
            f'  map frame   : {self.map_frame_}\n'
            f'  max points  : {self.max_points_}\n'
            f'  publish rate: {publish_rate} Hz\n'
            f'  sub  : {input_topic}\n'
            f'  pub  : {output_topic}'
        )

    # ---------------------------------------------------------------------- #
    # Callbacks                                                               #
    # ---------------------------------------------------------------------- #

    def cloud_callback(self, msg: PointCloud2):
        """Append incoming points to the accumulated map."""
        try:
            raw = pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'))
            pts = np.array(list(raw))
        except Exception as exc:
            self.get_logger().error(f'Failed to read PointCloud2: {exc}')
            return

        if pts.size == 0:
            return

        if self.accumulated_ is None:
            self.accumulated_ = pts.copy()
        else:
            self.accumulated_ = np.concatenate((self.accumulated_, pts), axis=0)

        # Ring-buffer: keep only the newest max_points
        n = len(self.accumulated_)
        if n > self.max_points_:
            self.accumulated_ = self.accumulated_[-self.max_points_:]
            self.get_logger().info(
                f'Map capped at {self.max_points_} points (oldest discarded)')

        # perf-measurement (stage D): the scan is now integrated — echo its input
        # header.stamp so the sniffer can timestamp the moment of insertion.
        dbg = PointStamped()
        dbg.header.stamp = msg.header.stamp
        dbg.header.frame_id = self.map_frame_
        self.insert_dbg_pub_.publish(dbg)

    def publish_map(self):
        """Publish the current accumulated map."""
        if self.accumulated_ is None or len(self.accumulated_) == 0:
            return

        header           = Header()
        header.frame_id  = self.map_frame_
        header.stamp     = self.get_clock().now().to_msg()

        out_msg = pc2.create_cloud(
            header,
            [
                pc2.PointField(name='x',         offset=0,  datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y',         offset=4,  datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z',         offset=8,  datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
            ],
            self.accumulated_,
        )

        self.map_pub_.publish(out_msg)
        self.get_logger().debug(
            f'Published sonar map: {len(self.accumulated_)} points')

    # ---------------------------------------------------------------------- #
    # Long-mission save + reset                                               #
    # ---------------------------------------------------------------------- #

    def save_and_reset_cb(self, request, response):
        """Save the accumulated map to <mission_dir>/sonar/ then clear it so a
        fresh empty map starts growing. The node, subscriptions and timer stay
        alive (no re-init), so only the scans during the save are missed."""
        try:
            pts = self.accumulated_
            n = 0 if pts is None else len(pts)
            out_dir = self.mission_dir_ / 'sonar'
            out_dir.mkdir(parents=True, exist_ok=True)
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = out_dir / f'sonar_map_{ts}.pcd'
            if n > 0:
                self._save_pcd_xyzi(path, pts)             # pts may be a structured array
            self.accumulated_ = None                       # fresh empty map
            response.success = True
            response.message = f'Saved {n} pts -> {path}; map reset' if n else 'Map empty; reset (nothing saved)'
            self.get_logger().info(response.message)
        except Exception as exc:                            # noqa: BLE001
            response.success = False
            response.message = f'save_and_reset failed: {exc!r}'
            self.get_logger().error(response.message)
        return response

    @staticmethod
    def _save_pcd_xyzi(path, pts):
        """Write (x,y,z,intensity) points to an ASCII PCD (MATLAB-readable).

        Accepts EITHER a structured array (pc2.read_points output, dtype with named
        fields x/y/z/intensity) OR a plain Nx>=4 numeric array, and normalises to an
        Nx4 float64 before writing -- the accumulated map is the structured form, so
        a plain np.asarray(..., float64) cast would raise on the structured dtype."""
        if getattr(pts, 'dtype', None) is not None and pts.dtype.names is not None:
            xyzi = np.column_stack([np.asarray(pts[c], dtype=np.float64)
                                    for c in ('x', 'y', 'z', 'intensity')])
        else:
            xyzi = np.asarray(pts, dtype=np.float64)[:, :4]
        n = len(xyzi)
        with open(path, 'w') as f:
            f.write('# .PCD v.7 - Point Cloud Data file format\n')
            f.write('VERSION .7\n')
            f.write('FIELDS x y z intensity\n')
            f.write('SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n')
            f.write(f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {n}\nDATA ascii\n')
            for x, y, z, i in xyzi:
                f.write(f'{x:.6f} {y:.6f} {z:.6f} {i:.6f}\n')


# ---------------------------------------------------------------------------- #
# Entry point                                                                   #
# ---------------------------------------------------------------------------- #

def main(args=None):
    rclpy.init(args=args)
    node = SonarMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()