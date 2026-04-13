#!/usr/bin/env python3
"""
sonar_scan_ned — transform leading-edge sonar points into the odom frame.

Uses Time(0) (latest available TF) because the sonar hardware clock (Unix wall
time) and the SLAM clock (Ouster device-boot time) live in different domains and
cannot be correlated by timestamp.

Queue size 1 ensures we always process the freshest scan with the current pose.

A sliding time window keeps the last `window_seconds` worth of transformed points
and re-publishes them combined on every new scan, giving a continuous visual flow
in Foxglove/RViz instead of a flashing single-scan display.
"""

import collections
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial.transform import Rotation
import tf2_ros


class SonarScanNode(Node):
    def __init__(self):
        super().__init__('sonar_scan_ned')
        self.declare_parameter('odom_frame',        'odom')
        self.declare_parameter('sonar_frame',       'blueview')
        self.declare_parameter('sonar_cloud_topic', '/blueview/point2/leading')
        self.declare_parameter('output_topic',      'sonar_scan')
        self.declare_parameter('tf_timeout',        0.5)
        # How many seconds of scans to keep and re-publish as one cloud.
        # Increase for a longer trail; set to 0.0 to publish only the latest scan.
        self.declare_parameter('window_seconds',    30.0)

        self.odom_frame      = self.get_parameter('odom_frame').value
        self.sonar_frame     = self.get_parameter('sonar_frame').value
        sonar_topic          = self.get_parameter('sonar_cloud_topic').value
        output_topic         = self.get_parameter('output_topic').value
        self.tf_timeout      = Duration(seconds=self.get_parameter('tf_timeout').value)
        self.window_seconds  = self.get_parameter('window_seconds').value

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # queue_size=1: always process freshest scan, never stale ones
        self.pub = self.create_publisher(PointCloud2, output_topic, 1)
        self.sub = self.create_subscription(PointCloud2, sonar_topic, self._cb, 1)

        # Sliding window: deque of (timestamp_sec: float, points: np.ndarray shape Nx4)
        self._window: collections.deque = collections.deque()

        self._recv = self._dropped = 0
        self.create_timer(5.0, self._diag)

        self._fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.get_logger().info(
            f'sonar_scan_ned: {sonar_topic} [{self.sonar_frame}] -> '
            f'{output_topic} [{self.odom_frame}]  window={self.window_seconds}s')

    def _lookup_tf(self):
        """Return 4x4 float64 T_{odom<-sonar} or None on failure.

        Time(0) requests the latest available transform, which is correct because
        the sonar hardware clock (Unix wall time) and SLAM clock (Ouster boot time)
        are in completely different time domains and cannot be matched by timestamp.
        """
        try:
            ts = self.tf_buffer.lookup_transform(
                self.odom_frame, self.sonar_frame, Time(), self.tf_timeout)
        except Exception as exc:
            self.get_logger().warn(f'TF lookup failed: {exc}', throttle_duration_sec=5.0)
            return None
        t = ts.transform.translation
        q = ts.transform.rotation
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3,  3] = [t.x, t.y, t.z]
        return T

    def _read_points(self, msg):
        """Safe PointCloud2 read — unpacks structured array rows by field name.

        NOTE: pc2.read_points returns a numpy structured array with dtype like
        [('x', '<f4'), ('y', '<f4'), ...]. Casting that directly to float32 raises
        TypeError in ROS2 Humble's numpy. We must unpack via named field access.
        """
        rows = list(pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                                    skip_nans=True))
        if not rows:
            return np.empty((0, 4), dtype=np.float32)
        arr = np.array(rows)   # structured array: dtype=[('x','f4'),('y','f4'),...]
        return np.column_stack(
            [arr['x'], arr['y'], arr['z'], arr['intensity']]
        ).astype(np.float32)

    def _prune_window(self, now_sec: float):
        """Remove entries older than window_seconds from the left of the deque."""
        cutoff = now_sec - self.window_seconds
        while self._window and self._window[0][0] < cutoff:
            self._window.popleft()

    def _cb(self, msg: PointCloud2):
        self._recv += 1
        T = self._lookup_tf()
        if T is None:
            self._dropped += 1
            return

        pts = self._read_points(msg)
        if pts.shape[0] == 0:
            return

        # Transform XYZ into odom frame; keep original intensity
        xyz  = pts[:, :3].astype(np.float64)
        ones = np.ones((len(xyz), 1), dtype=np.float64)
        xyz_w = (T @ np.hstack([xyz, ones]).T).T[:, :3].astype(np.float32)

        out = np.zeros(len(xyz_w),
                       dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4')])
        out['x'] = xyz_w[:, 0]
        out['y'] = xyz_w[:, 1]
        out['z'] = xyz_w[:, 2]
        out['intensity'] = pts[:, 3]

        # Add to sliding window
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._window.append((now_sec, out))

        if self.window_seconds > 0.0:
            self._prune_window(now_sec)
            # Concatenate all scans in the window into one cloud
            combined = np.concatenate([p for _, p in self._window])
        else:
            combined = out

        h = Header()
        h.stamp = msg.header.stamp
        h.frame_id = self.odom_frame
        self.pub.publish(pc2.create_cloud(h, self._fields, combined))

    def _diag(self):
        total_pts = sum(p.shape[0] for _, p in self._window)
        self.get_logger().info(
            f'[DIAG] sonar_scan_ned: recv={self._recv} '
            f'ok={self._recv - self._dropped} dropped={self._dropped} '
            f'window_scans={len(self._window)} window_pts={total_pts}')
        self._recv = self._dropped = 0


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SonarScanNode())


if __name__ == '__main__':
    main()
