#!/usr/bin/env python3
"""
sonar_scan_ned — transform leading-edge sonar points into the odom frame.

Uses Time(0) (latest available TF) because the sonar hardware clock (Unix wall
time) and the SLAM clock (Ouster device-boot time) live in different domains and
cannot be correlated by timestamp.

Queue size 1 ensures we always process the freshest scan with the current pose.
Each scan is published individually; the map node accumulates scans over time.

FIELD-GENERIC PASSTHROUGH: the eq-24 pipeline attaches per-return measurement
channels upstream of this node — most importantly the six shape/morphology
descriptors of the leading-edge extractor (shape_flag, region_*; paper
eqs 39–40) that ride on the same cloud as x/y/z/intensity. Rather than
enumerate them here (and break every time a channel is added), this node reads
ALL float32 fields of the incoming cloud by name, transforms only x/y/z into
odom, and copies every other field through unchanged, rebuilding the PointField
layout dynamically. With the classic 4-field input the output is byte-identical
to the fixed-layout version.

NaN GATING is restricted to the x/y/z/intensity core: the shape descriptor
channels are NaN BY DESIGN on bare-seabed returns (the hurdle factor of eq 42
abstains on NaN), so a whole-row skip_nans read would silently drop exactly the
returns the shape factor needs to see. The core mask mirrors the library's
skip_nans contract (applied only when the cloud is not flagged dense).
"""

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
        self.declare_parameter('sonar_frame',       'blueview_sonar')
        self.declare_parameter('sonar_cloud_topic', '/blueview/leading_edge')
        self.declare_parameter('output_topic',      'sonar_scan')
        self.declare_parameter('tf_timeout',        0.5)
        self.declare_parameter('scan_publish_rate', 5.0)
        self.declare_parameter('latency_compensation_ms', 0.0)

        self.odom_frame  = self.get_parameter('odom_frame').value
        self.sonar_frame = self.get_parameter('sonar_frame').value
        sonar_topic      = self.get_parameter('sonar_cloud_topic').value
        output_topic     = self.get_parameter('output_topic').value
        self.tf_timeout  = Duration(seconds=self.get_parameter('tf_timeout').value)

        # NEW: Throttle configuration
        scan_rate = self.get_parameter('scan_publish_rate').value
        self.min_interval = Duration(seconds=1.0 / scan_rate)
        self.last_publish_time = None  # Will be set on first publish

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # queue_size=1: always process freshest scan, never stale ones
        self.pub = self.create_publisher(PointCloud2, output_topic, 1)
        self._throttled = 0  # NEW: Track throttled scans
        self.sub = self.create_subscription(PointCloud2, sonar_topic, self._cb, 1)

        self._recv = self._dropped = 0
        self.create_timer(5.0, self._diag)

        self.get_logger().info(
            f'sonar_scan_ned: {sonar_topic} [{self.sonar_frame}] -> '
            f'{output_topic} [{self.odom_frame}] @ {scan_rate} Hz')

    def _lookup_tf(self):
        """Return 4x4 float64 T_{odom<-sonar} or None on failure.

        Uses Time(0) to get the latest transform first (the only way to bridge
        the sonar Unix-wall-clock / Ouster-boot-clock mismatch), then steps back
        by latency_compensation_ms in the Ouster-clock domain so we use the pose
        the vehicle was actually at when the ping was transmitted.
        """
        try:
            # Step 1: get latest TF — this also gives us the current Ouster-clock stamp
            ts = self.tf_buffer.lookup_transform(
                self.odom_frame, self.sonar_frame, Time(), self.tf_timeout)

            latency_s = self.get_parameter('latency_compensation_ms').value / 1000.0
            if latency_s > 0.0:
                # Step 2: subtract latency in Ouster-clock domain and look up again
                stamp = ts.header.stamp
                t_ns = stamp.sec * 1_000_000_000 + stamp.nanosec - int(latency_s * 1e9)
                compensated = Time(nanoseconds=max(t_ns, 0))
                ts = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.sonar_frame, compensated, self.tf_timeout)

        except Exception as exc:
            self.get_logger().warn(f'TF lookup failed: {exc}', throttle_duration_sec=5.0)
            return None
        t = ts.transform.translation
        q = ts.transform.rotation
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3,  3] = [t.x, t.y, t.z]
        return T

    def _read_points(self, msg, names):
        """Safe PointCloud2 read — unpacks structured array rows by field name.

        Returns an (N, len(names)) float32 array, columns in `names` order.

        NOTE: pc2.read_points returns a numpy structured array with dtype like
        [('x', '<f4'), ('y', '<f4'), ...]. Casting that directly to float32 raises
        TypeError in ROS2 Humble's numpy. We must unpack via named field access.
        skip_nans stays OFF here: NaN is data in the descriptor channels (hurdle
        abstention) — the caller masks the geometric core itself.
        """
        arr = np.asarray(pc2.read_points(msg, field_names=names, skip_nans=False))
        if arr.size == 0:
            return np.empty((0, len(names)), dtype=np.float32)
        return np.column_stack(
            [np.asarray(arr[nm], dtype=np.float32) for nm in names])

    def _cb(self, msg: PointCloud2):
        self._recv += 1

        # NEW: Time-based throttle - skip if not enough time has elapsed
        now = self.get_clock().now()
        if self.last_publish_time is not None:
            elapsed = now - self.last_publish_time
            if elapsed < self.min_interval:
                self._throttled += 1
                return  # Skip this scan

        T = self._lookup_tf()
        if T is None:
            self._dropped += 1
            return

        # Field-generic layout: every scalar float32 field of the incoming
        # cloud, in its original order. x/y/z get transformed; everything else
        # (intensity, shape descriptors, future channels) is copied verbatim.
        names = [f.name for f in msg.fields
                 if f.datatype == PointField.FLOAT32 and f.count == 1]
        if not {'x', 'y', 'z'}.issubset(names):
            self.get_logger().warn(
                f'Cloud missing x/y/z float32 fields (got {names}) — dropping.',
                throttle_duration_sec=5.0)
            self._dropped += 1
            return

        pts = self._read_points(msg, names)

        # NaN gate on the x/y/z/intensity core only (mirrors read_points'
        # skip_nans contract, incl. its is_dense gate). Descriptor-channel NaNs
        # must survive — they are the shape factor's abstention signal (eq 42).
        core = [names.index(nm) for nm in ('x', 'y', 'z', 'intensity')
                if nm in names]
        if pts.shape[0] and not msg.is_dense:
            pts = pts[~np.isnan(pts[:, core]).any(axis=1)]
        if pts.shape[0] == 0:
            return

        # Transform XYZ into odom frame; keep all other channels untouched
        ix, iy, iz = names.index('x'), names.index('y'), names.index('z')
        xyz  = pts[:, (ix, iy, iz)].astype(np.float64)
        ones = np.ones((len(xyz), 1), dtype=np.float64)
        xyz_w = (T @ np.hstack([xyz, ones]).T).T[:, :3].astype(np.float32)

        out = np.zeros(len(xyz_w), dtype=[(nm, 'f4') for nm in names])
        for j, nm in enumerate(names):
            out[nm] = pts[:, j]
        out['x'] = xyz_w[:, 0]
        out['y'] = xyz_w[:, 1]
        out['z'] = xyz_w[:, 2]

        fields = [PointField(name=nm, offset=4 * j,
                             datatype=PointField.FLOAT32, count=1)
                  for j, nm in enumerate(names)]

        h = Header()
        h.stamp = msg.header.stamp
        h.frame_id = self.odom_frame
        self.pub.publish(pc2.create_cloud(h, fields, out))

        self.last_publish_time = now  # NEW: Update last publish time

    def _diag(self):
        published = self._recv - self._dropped - self._throttled
        self.get_logger().info(
            f'[DIAG] sonar_scan_ned: recv={self._recv} '
            f'published={published} dropped={self._dropped} throttled={self._throttled}')
        self._recv = self._dropped = self._throttled = 0


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SonarScanNode())


if __name__ == '__main__':
    main()
