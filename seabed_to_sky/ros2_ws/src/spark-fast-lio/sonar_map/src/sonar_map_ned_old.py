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

import numpy as np


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

        input_topic  = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.map_frame_  = self.get_parameter('map_frame').value
        self.max_points_ = self.get_parameter('max_points').value
        publish_rate     = self.get_parameter('publish_rate').value

        # ------------------------------------------------------------------ #
        # State                                                               #
        # ------------------------------------------------------------------ #
        self.accumulated_: np.ndarray | None = None

        # ------------------------------------------------------------------ #
        # Pub / Sub / Timer                                                   #
        # ------------------------------------------------------------------ #
        self.map_pub_ = self.create_publisher(PointCloud2, output_topic, 10)

        self.cloud_sub_ = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, 10)

        self.timer_ = self.create_timer(
            1.0 / publish_rate, self.publish_map)

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