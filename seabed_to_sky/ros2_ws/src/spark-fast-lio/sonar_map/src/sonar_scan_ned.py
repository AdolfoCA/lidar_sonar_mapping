#!/usr/bin/env python3
"""
sonar_scan_ned.py
-----------------
Subscribes to a sonar PointCloud2 (in the sonar sensor frame) and republishes
each scan transformed into the odometry (world/NED) frame.

The full TF chain is resolved automatically by tf2 using the static transforms
broadcast by tf_static_publisher:

    odom  ──►  base_link  ──►  otter  ──►  otter_frd
          ──►  blueview_joint1  ──►  blueview_joint2  ──►  blueview  (or oculus)

Parameters (set in sonar_scan_ned.yaml):
    sonar_frame        "blueview"                — source TF frame
    odom_frame         "odom"                    — target TF frame
    sonar_cloud_topic  "/blueview/point2/leading"— input topic
    output_topic       "sonar_scan"              — output topic
    tf_timeout         0.0                       — seconds; 0 = non-blocking
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy.time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
from scipy.spatial.transform import Rotation
import tf2_ros


class SonarScanNode(Node):

    def __init__(self):
        super().__init__('sonar_scan_ned')

        # ------------------------------------------------------------------ #
        # Parameters                                                          #
        # ------------------------------------------------------------------ #
        self.declare_parameter('sonar_frame',       'blueview')
        self.declare_parameter('odom_frame',        'odom')
        self.declare_parameter('sonar_cloud_topic', '/blueview/point2/leading')
        self.declare_parameter('output_topic',      'sonar_scan')
        self.declare_parameter('tf_timeout',        0.0)

        self.sonar_frame_ = self.get_parameter('sonar_frame').value
        self.odom_frame_  = self.get_parameter('odom_frame').value
        sonar_topic       = self.get_parameter('sonar_cloud_topic').value
        output_topic      = self.get_parameter('output_topic').value
        tf_timeout_s      = self.get_parameter('tf_timeout').value

        self.tf_timeout_ = Duration(seconds=tf_timeout_s)

        # ------------------------------------------------------------------ #
        # TF2 — reads the tree published by tf_static_publisher              #
        # ------------------------------------------------------------------ #
        self.tf_buffer_   = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        # ------------------------------------------------------------------ #
        # Publisher / Subscriber                                              #
        # ------------------------------------------------------------------ #
        self.cloud_pub_ = self.create_publisher(PointCloud2, output_topic, 10)
        self.sonar_sub_ = self.create_subscription(
            PointCloud2, sonar_topic, self._sonar_callback, 10)

        # Suppress repeated TF warnings while tree is not yet ready
        self._tf_warn_suppressed = False

        self.get_logger().info(
            f'sonar_scan_NED ready\n'
            f'  sonar frame : {self.sonar_frame_}\n'
            f'  odom  frame : {self.odom_frame_}\n'
            f'  tf timeout  : {tf_timeout_s} s\n'
            f'  subscribing : {sonar_topic}\n'
            f'  publishing  : {output_topic}'
        )

    # ---------------------------------------------------------------------- #
    # TF lookup                                                               #
    # ---------------------------------------------------------------------- #

    def _lookup_tf(self, target: str, source: str, stamp) -> np.ndarray | None:
        """
        Look up T_{target <- source} from the TF tree.
        tf2 resolves the full chain automatically — no manual chaining needed.
        Returns a 4x4 numpy homogeneous matrix, or None if lookup fails.
        """
        try:
            ts = self.tf_buffer_.lookup_transform(
                target, source, stamp, self.tf_timeout_)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            if not self._tf_warn_suppressed:
                self.get_logger().warn(
                    f'TF [{source}] -> [{target}] failed: {exc}\n'
                    f'  Is tf_static_publisher running? '
                    f'  Skipping frame (further warnings suppressed until TF is available).'
                )
                self._tf_warn_suppressed = True
            return None

        # TF is available — re-enable warnings for future failures
        self._tf_warn_suppressed = False

        t = ts.transform.translation
        q = ts.transform.rotation

        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3,  3] = [t.x, t.y, t.z]
        return T

    # ---------------------------------------------------------------------- #
    # Sonar callback                                                          #
    # ---------------------------------------------------------------------- #

    def _sonar_callback(self, msg: PointCloud2):
        """
        On each incoming sonar scan:
          1. Look up odom <- sonar_frame from the TF tree
          2. Apply transform to all points
          3. Publish result in odom frame
        """

        # 1. TF lookup — tf2 resolves the full chain automatically
        T = self._lookup_tf(
            target=self.odom_frame_,
            source=self.sonar_frame_,
            stamp=rclpy.time.Time(),
        )
        if T is None:
            return  # skip this scan, already warned

        # 2. Read points from incoming cloud
        try:
            raw = pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'))
            pts = np.array(list(raw))
        except Exception as exc:
            self.get_logger().error(f'Failed to read PointCloud2: {exc}')
            return

        if pts.size == 0:
            return

        # 3. Apply transform: xyz_world = T * xyz_sonar
        xyz   = np.column_stack((pts['x'], pts['y'], pts['z']))
        xyz_h = np.hstack((xyz, np.ones((len(xyz), 1), dtype=np.float64)))
        xyz_world = (T @ xyz_h.T).T[:, :3]

        # 4. Pack into output structured array
        out = np.zeros(len(xyz_world), dtype=np.dtype([
            ('x',         np.float32),
            ('y',         np.float32),
            ('z',         np.float32),
            ('intensity', np.float32),
        ]))
        out['x']         = xyz_world[:, 0].astype(np.float32)
        out['y']         = xyz_world[:, 1].astype(np.float32)
        out['z']         = xyz_world[:, 2].astype(np.float32)
        out['intensity'] = pts['intensity']

        # 5. Publish
        header          = Header()
        header.frame_id = self.odom_frame_
        header.stamp    = msg.header.stamp

        self.cloud_pub_.publish(pc2.create_cloud(
            header,
            [
                pc2.PointField(name='x',         offset=0,  datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y',         offset=4,  datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z',         offset=8,  datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
            ],
            out,
        ))


# ---------------------------------------------------------------------------- #
# Entry point                                                                   #
# ---------------------------------------------------------------------------- #

def main(args=None):
    rclpy.init(args=args)
    node = SonarScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()