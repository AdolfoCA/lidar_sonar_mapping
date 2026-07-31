#!/usr/bin/env python3
"""
lidar_count_debug — observability node for the per-return LiDAR support count.
===============================================================================

A PURE OBSERVER. It does not feed the pipeline and never changes production
state. It exists to answer one question visually: for every sonar leading-edge
return, how many LiDAR points sit in the support disc of its CLOSEST LiDAR
point?

This is exactly the ``count`` k the classifier ships as ``lidar_count`` and that
voxel_mapper feeds into the count factor ``1 - exp(-beta_k * k)``. This node
re-derives k and the nearest LiDAR point ``q*`` for visualisation: it rebuilds
the SAME XY (horizontal) KD-tree the classifier builds from ``lidar_topic`` and
re-runs the nearest-neighbour + radius query with the SAME ``eps_L``. With those
matched, the count shown here equals the count the classifier shipped.

Per sonar return p (XY/horizontal distance, matching return_classifier):
    q*    = nearest LiDAR point to p (in XY)
    k     = | { LiDAR pts c : || c_xy - q*_xy || <= eps_L
                              [and |c_z - q*_z| <= eps_Lz] } |     (the count)
When eps_Lz > 0 the disc becomes a cylinder (z band around q*), matching the
classifier; the emitted LiDAR cluster is filtered to the same set. eps_Lz <= 0
recovers the pure-XY disc. MUST match return_classifier.eps_Lz.

OUTPUT — one unified cloud ``/debug/lidar_count`` (PointCloud2):
  For every sonar return in the latest classified batch we emit, sharing one
  ``group_id = 0, 1, 2, …`` (reset each frame):
    - the sonar return itself                         (kind = KIND_SONAR = 0)
    - EVERY LiDAR point within r_lidar of its q*       (kind = KIND_LIDAR = 1)
  i.e. the actual LiDAR cluster that makes up the return's count — not just q*
  and a number. Every row in a group carries the same ``count`` field = k (the
  cluster size). In Foxglove: colour by ``group_id`` to see which LiDAR points
  belong to which sonar return, and by ``kind`` to tell sonar from LiDAR. A
  return with no LiDAR support appears alone (k = 0, sonar row only).
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy.lib.recfunctions as rfn
from scipy.spatial import cKDTree

from sonar_map_msgs.msg import ClassifiedReturns

# ── Row "kind" tags (which of the paired points a row is) ─────────────────────
KIND_SONAR = 0
KIND_LIDAR = 1

# ── Defaults — overridable from the YAML config / launch arguments ────────────
INPUT_TOPIC  = 'classified_returns'   # classified batch (gives us the returns)
LIDAR_TOPIC  = '/cloud_registered'    # same LiDAR cloud the classifier uses
MAP_FRAME    = 'odom'
EPS_L        = 0.15                    # [m] MUST match return_classifier.eps_L
EPS_LZ       = 0.0                     # [m] z half-height; ≤0 = pure XY. MUST match classifier
PUBLISH_RATE = 5.0                     # [Hz] throttle on the debug cloud


def _read_xyz(msg: PointCloud2) -> np.ndarray:
    """Read x,y,z into an (N, 3) float64 array, NaN/inf rows dropped."""
    raw = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    if isinstance(raw, np.ndarray):
        if raw.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        if raw.dtype.names is not None:
            xyz = rfn.structured_to_unstructured(raw, dtype=np.float64)
        else:
            xyz = raw.reshape(-1, 3).astype(np.float64)
    else:
        rows = list(raw)
        if not rows:
            return np.empty((0, 3), dtype=np.float64)
        xyz = np.array(rows, dtype=np.float64).reshape(-1, 3)
    return xyz[np.isfinite(xyz).all(axis=1)] if xyz.size else xyz


class LidarCountDebugNode(Node):

    def __init__(self):
        super().__init__('lidar_count_debug')

        self.declare_parameter('input_topic',  INPUT_TOPIC)
        self.declare_parameter('lidar_topic',   LIDAR_TOPIC)
        self.declare_parameter('map_frame',     MAP_FRAME)
        self.declare_parameter('eps_L',         EPS_L)
        self.declare_parameter('eps_Lz',        EPS_LZ)
        self.declare_parameter('publish_rate',  PUBLISH_RATE)

        def gp(n):
            return self.get_parameter(n).value

        self._map_frame = str(gp('map_frame'))
        self._eps_l     = float(gp('eps_L'))
        self._eps_lz    = float(gp('eps_Lz'))   # ≤0 ⇒ z band off (pure XY)
        rate            = max(float(gp('publish_rate')), 1e-6)
        self._min_period_ns = int(1e9 / rate)
        self._last_pub_ns   = 0

        # Latest LiDAR, kept as both an XY tree (for the matched query — z is
        # ignored, matching the classifier) and the full XYZ (so we can emit the
        # supporting points at their true 3D position).
        self._lidar_tree = None    # cKDTree over LiDAR XY (horizontal)
        self._lidar_xyz  = None    # (M, 3) float64

        # PointCloud2 layout: x, y, z, count, group_id, kind (all float32).
        self._fields = [
            PointField(name='x',        offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',        offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',        offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='count',    offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='group_id', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='kind',     offset=20, datatype=PointField.FLOAT32, count=1),
        ]

        self._cloud_pub = self.create_publisher(
            PointCloud2, '/debug/lidar_count', 10)
        self.sub_lidar_ = self.create_subscription(
            PointCloud2, str(gp('lidar_topic')), self._lidar_cb, 1)
        self.sub_returns_ = self.create_subscription(
            ClassifiedReturns, str(gp('input_topic')), self._returns_cb, 1)

        self.get_logger().info(
            f'lidar_count_debug: {gp("input_topic")} + {gp("lidar_topic")} '
            f'-> /debug/lidar_count | eps_L={self._eps_l}m (XY)'
            + (f' ∩ |Δz|≤{self._eps_lz}m' if self._eps_lz > 0.0
               else ' [z band off]')
            + f' rate={rate}Hz')

    # ── input caches ──────────────────────────────────────────────────────────

    def _lidar_cb(self, msg: PointCloud2):
        xyz = _read_xyz(msg)
        if xyz.shape[0] == 0:
            self._lidar_tree = None
            self._lidar_xyz  = None
            return
        self._lidar_xyz  = xyz                  # full 3D, for emitting points
        self._lidar_tree = cKDTree(xyz[:, :2])  # XY-only, matches classifier

    # ── main: re-derive count + q* per return, emit paired cloud ──────────────

    def _returns_cb(self, msg: ClassifiedReturns):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_pub_ns < self._min_period_ns:
            return
        self._last_pub_ns = now_ns

        sx = np.asarray(msg.x, dtype=np.float64)
        sy = np.asarray(msg.y, dtype=np.float64)
        sz = np.asarray(msg.z, dtype=np.float64)
        n = sx.shape[0]
        if n == 0:
            return

        sonar_xyz = np.column_stack((sx, sy, sz))

        # No LiDAR yet: still show the returns, count = 0, no q* partners.
        if self._lidar_tree is None:
            rows = np.empty((n, 6), dtype=np.float32)
            rows[:, 0:3] = sonar_xyz.astype(np.float32)
            rows[:, 3]   = 0.0                                   # count
            rows[:, 4]   = np.arange(n, dtype=np.float32)        # group_id
            rows[:, 5]   = KIND_SONAR
            self._publish_cloud(rows)
            self.get_logger().warn('No LiDAR cloud yet — returns shown with '
                                   'count = 0.', throttle_duration_sec=5.0)
            return

        # Matched query: nearest LiDAR point q* in XY, then ALL LiDAR points
        # within eps_L of q* (XY) — exactly the set return_classifier counts as
        # k. We keep the full index lists (not just the length) so we can emit
        # the supporting LiDAR points themselves (at their true 3D position).
        sonar_xy = sonar_xyz[:, :2]
        _, idx = self._lidar_tree.query(sonar_xy, k=1)
        idx = np.asarray(idx, dtype=np.int64).reshape(n)
        tree_xy = np.asarray(self._lidar_tree.data, dtype=np.float64)  # (M, 2)
        qstar_xy = tree_xy[idx]
        ball_idx = self._lidar_tree.query_ball_point(qstar_xy, r=self._eps_l)

        # One sonar row per return + one LiDAR row for every point in its ball,
        # all sharing the return's group_id. `count` field = size of the ball
        # (same for every row in a group), so colouring by count still works.
        # When eps_Lz > 0 the ball is trimmed to the z band around q* (a
        # cylinder), so both the count and the emitted cluster match the
        # classifier's cylinder count.
        out = []
        for i in range(n):
            members = ball_idx[i]
            if self._eps_lz > 0.0 and len(members) > 0:
                qz = self._lidar_xyz[idx[i], 2]
                members = [j for j in members
                           if abs(self._lidar_xyz[j, 2] - qz) <= self._eps_lz]
            k = float(len(members))
            # the sonar leading-edge return itself
            sp = sonar_xyz[i]
            out.append((float(sp[0]), float(sp[1]), float(sp[2]),
                        k, float(i), float(KIND_SONAR)))
            # every LiDAR point that supports it (the count cluster around q*),
            # emitted at its true 3D position from the cached full cloud.
            for j in members:
                lp = self._lidar_xyz[j]
                out.append((float(lp[0]), float(lp[1]), float(lp[2]),
                            k, float(i), float(KIND_LIDAR)))

        self._publish_cloud(np.asarray(out, dtype=np.float32))

    # ── output ────────────────────────────────────────────────────────────────

    def _publish_cloud(self, rows: np.ndarray):
        header = Header()
        header.frame_id = self._map_frame
        header.stamp = self.get_clock().now().to_msg()
        # create_cloud expects an iterable of per-point sequences; passing the
        # raw 2D ndarray can mis-pack (each row is itself an ndarray), so hand it
        # an explicit list of tuples matching self._fields' 6 float32 columns.
        points = [tuple(float(v) for v in r) for r in rows]
        cloud = pc2.create_cloud(header, self._fields, points)
        self._cloud_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = LidarCountDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
