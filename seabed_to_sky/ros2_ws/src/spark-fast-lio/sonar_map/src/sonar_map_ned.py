#!/usr/bin/env python3
"""
sonar_map_ned.py  (v3 — full density + voxel deduplication)
------------------------------------------------------------
Stores every point from every scan (no downsampling within a scan),
but deduplicates across scans using a voxel hashmap — new points
overwrite old ones in the same voxel cell.

This gives you the dense point cloud of the original ring-buffer
approach, with bounded memory from the voxel structure.

Parameters (sonar.yaml → sonar_map_ned):
    input_topic    "sonar_scan"
    output_topic   "sonar_map"
    odom_topic     "odometry"
    map_frame      "odom"
    voxel_size     0.05          ← tune to sonar resolution
    window_radius  50.0          ← sliding window in metres
    publish_rate   1.0
    source_id      0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np
import numpy.lib.recfunctions as rfn


class VoxelMap:
    """
    2D voxel hashmap — stores the LATEST point per voxel cell.
    No averaging: each new point fully replaces the old one in its cell.
    This preserves full scan density while bounding memory.

    Key:   (ix, iy)  — 2D integer voxel index
    Value: [x, y, z, intensity]  — most recent point in that cell
    """

    def __init__(self, voxel_size: float):
        self.voxel_size = voxel_size
        self._cells: dict = {}          # (ix,iy) -> np.array([x,y,z,intensity])

    def insert_batch(self, pts: np.ndarray):
        """
        pts: (N, 4) float32 [x, y, z, intensity]
        Vectorized voxel assignment — no Python loop over points.
        Latest point wins within each voxel.
        """
        inv_v = 1.0 / self.voxel_size
        ix = np.floor(pts[:, 0] * inv_v).astype(np.int32)
        iy = np.floor(pts[:, 1] * inv_v).astype(np.int32)

        # One dict update per unique voxel hit by this scan.
        # If multiple points fall in the same voxel, last one wins
        # (numpy unique keeps the last occurrence with [::-1] trick).
        keys_int = ix.astype(np.int64) * (1 << 30) + iy.astype(np.int64)

        # Reverse so that np.unique keeps the LAST point per voxel
        rev_idx  = np.arange(len(pts) - 1, -1, -1)
        _, first_of_rev = np.unique(keys_int[rev_idx], return_index=True)
        keep_idx = rev_idx[first_of_rev]

        for i in keep_idx:
            k_int = int(keys_int[i])
            k_iy  = k_int % (1 << 30)
            k_ix  = k_int >> 30
            if k_iy > (1 << 29):
                k_iy -= (1 << 30)
            self._cells[(k_ix, k_iy)] = pts[i]   # plain (4,) array

    def prune_outside_radius(self, cx: float, cy: float, radius: float) -> int:
        """Remove voxels outside sliding window."""
        r2 = radius * radius
        to_delete = [
            k for k, v in self._cells.items()
            if (v[0] - cx) ** 2 + (v[1] - cy) ** 2 > r2
        ]
        for k in to_delete:
            del self._cells[k]
        return len(to_delete)

    def to_numpy(self) -> np.ndarray:
        """Return (N, 4) float32 array [x, y, z, intensity]."""
        if not self._cells:
            return np.empty((0, 4), dtype=np.float32)
        return np.array(list(self._cells.values()), dtype=np.float32)

    def __len__(self):
        return len(self._cells)


class SonarMapNode(Node):

    def __init__(self):
        super().__init__('sonar_map_ned')

        self.declare_parameter('input_topic',   'sonar_scan')
        self.declare_parameter('output_topic',  'sonar_map')
        self.declare_parameter('odom_topic',    'odometry')
        self.declare_parameter('map_frame',     'odom')
        self.declare_parameter('voxel_size',    0.05)
        self.declare_parameter('window_radius', 50.0)
        self.declare_parameter('publish_rate',  1.0)
        self.declare_parameter('source_id',     0)

        input_topic         = self.get_parameter('input_topic').value
        output_topic        = self.get_parameter('output_topic').value
        odom_topic          = self.get_parameter('odom_topic').value
        self.map_frame_     = self.get_parameter('map_frame').value
        voxel_size          = self.get_parameter('voxel_size').value
        self.window_radius_ = self.get_parameter('window_radius').value
        publish_rate        = self.get_parameter('publish_rate').value
        self.source_id_     = float(self.get_parameter('source_id').value)

        self.voxel_map_          = VoxelMap(voxel_size)
        self.vehicle_xy_         = None
        self._last_prune_xy_     = None
        self._prune_threshold_   = self.window_radius_ * 0.1

        self._fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='source',    offset=16, datatype=PointField.FLOAT32, count=1),
        ]

        self.map_pub_   = self.create_publisher(PointCloud2, output_topic, 10)
        self.cloud_sub_ = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, 10)
        self.odom_sub_  = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        self.timer_     = self.create_timer(1.0 / publish_rate, self.publish_map)

        self.get_logger().info(
            f'sonar_map_NED v3 | voxel={voxel_size}m | '
            f'window={self.window_radius_}m | rate={publish_rate}Hz'
        )

    def odom_callback(self, msg: Odometry):
        self.vehicle_xy_ = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def cloud_callback(self, msg: PointCloud2):
        try:
            raw       = pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                                        skip_nans=True)
            pts_struct = np.array(list(raw))
            if pts_struct.size == 0:
                return
            pts = rfn.structured_to_unstructured(pts_struct, dtype=np.float32)
        except Exception as exc:
            self.get_logger().error(f'Failed to read PointCloud2: {exc}')
            return

        # Reject any remaining non-finite values
        valid = np.isfinite(pts).all(axis=1)
        pts   = pts[valid]
        if len(pts) == 0:
            return

        self.voxel_map_.insert_batch(pts)

        # Prune only when vehicle has moved 10% of window radius
        if self.vehicle_xy_ is not None:
            cx, cy = self.vehicle_xy_
            if self._last_prune_xy_ is None:
                self._last_prune_xy_ = (cx, cy)
            else:
                dx = cx - self._last_prune_xy_[0]
                dy = cy - self._last_prune_xy_[1]
                if dx*dx + dy*dy > self._prune_threshold_**2:
                    removed = self.voxel_map_.prune_outside_radius(
                        cx, cy, self.window_radius_)
                    self._last_prune_xy_ = (cx, cy)
                    if removed > 0:
                        self.get_logger().debug(
                            f'Pruned {removed} voxels | '
                            f'map size: {len(self.voxel_map_)}')

    def publish_map(self):
        if len(self.voxel_map_) == 0:
            return

        pts  = self.voxel_map_.to_numpy()              # (N, 4)
        n    = pts.shape[0]
        src  = np.full((n, 1), self.source_id_, dtype=np.float32)
        pts5 = np.hstack([pts, src])                   # (N, 5)

        header          = Header()
        header.frame_id = self.map_frame_
        header.stamp    = self.get_clock().now().to_msg()

        self.map_pub_.publish(pc2.create_cloud(header, self._fields, pts5))
        self.get_logger().debug(f'Published sonar map: {n} voxels')


def main(args=None):
    rclpy.init(args=args)
    node = SonarMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()