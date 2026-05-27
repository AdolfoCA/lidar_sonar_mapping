#!/usr/bin/env python3
"""
return_classifier — Node 2 of the Dirichlet voxel mapping pipeline.
===============================================================================

Consumes raw sonar returns (``input_topic``, odom frame) together with the
LiDAR point cloud (``lidar_topic``) and the seabed estimator state
(``state_topic``), and emits one batch of semantically labelled returns per
scan on ``output_topic``.

ADAPTIVE INTENSITY THRESHOLD  (derived from the estimator's mu_I)
      tau_str_obj = lambda_str_obj * mu_I
  Single threshold separating SEABED from non-SEABED (STRUCTURE / OBJECT).
  Hard label is only used for FREE vs non-FREE routing in voxel_mapper and
  for logging; the Dirichlet weights are driven by the soft responsibility
  (controlled by alpha_i), not by this threshold.

GEOMETRIC LiDAR SUPPORT — two separate factors, both shipped per return:
  For each sonar return p (XY-plane only):
      q*           = nearest LiDAR point to p
      d            = || p_xy - q*_xy ||
      d_factor     = exp(- d / d_scale)                          (sharp gate)
      count        = | { LiDAR pts c : || c_xy - q*_xy || <= r_lidar } |
      count_factor = 1 - exp(- alpha_count * count)              (saturates)

  d_factor    ∈ [0, 1], 1 at LiDAR, sharp decay.
  count_factor ∈ [0, 1), saturating with cluster density.

  Both are shipped on ClassifiedReturns. voxel_mapper combines them into
        f_L = d_factor * count_factor
  for the soft STRUCTURE responsibility, and uses d_factor alone for the
  per-voxel LiDAR-derived STRUCTURE prior and for the structure-neighbour
  filter in the intensity-similarity gate.

ORDERED CLASSIFICATION RULE (hard label is only used for routing — voxel_mapper
distinguishes FREE vs non-FREE; the actual class evidence is the soft
per-return responsibility computed there):
      FREE       if intensity == 0
      STRUCTURE  if intensity >= tau_str_obj AND any LiDAR support
      OBJECT     if intensity >= tau_str_obj AND no LiDAR support
      SEABED     otherwise
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import numpy.lib.recfunctions as rfn
from scipy.spatial import cKDTree

from sonar_map_msgs.msg import ClassifiedReturns, SeabedEstimatorState


# ── Class labels (kept in sync with ClassifiedReturns.msg constants) ──────────
FREE      = 0
SEABED    = 1
OBJECT    = 2
STRUCTURE = 3

# ── Defaults — overridable from the YAML config / launch arguments ────────────
INPUT_TOPIC   = 'sonar_scan'
LIDAR_TOPIC   = '/cloud_registered'
STATE_TOPIC   = 'seabed_estimator_state'
ODOM_TOPIC    = 'odometry'
OUTPUT_TOPIC  = 'classified_returns'

LAMBDA_STR_OBJ = 1.5     # single SEABED vs (STRUCTURE/OBJECT) threshold multiplier on mu_I
R_LIDAR        = 1.5     # [m]  disc radius around the nearest LiDAR (count input)
D_SCALE       = 0.2      # [m]  sharp distance gate length for d_factor
ALPHA_COUNT   = 0.1      # [-]  count saturation rate for count_factor


def _read_fields(msg: PointCloud2, fields: tuple) -> np.ndarray:
    """Read the requested fields into an (N, len(fields)) float32 array."""
    raw = pc2.read_points(msg, field_names=fields, skip_nans=True)
    if isinstance(raw, np.ndarray):
        if raw.size == 0:
            return np.empty((0, len(fields)), dtype=np.float32)
        if raw.dtype.names is not None:
            return rfn.structured_to_unstructured(raw, dtype=np.float32)
        return raw.reshape(-1, len(fields)).astype(np.float32)
    rows = list(raw)
    if not rows:
        return np.empty((0, len(fields)), dtype=np.float32)
    return np.array(rows, dtype=np.float32).reshape(-1, len(fields))


class ReturnClassifierNode(Node):

    def __init__(self):
        super().__init__('return_classifier')

        self.declare_parameter('input_topic',   INPUT_TOPIC)
        self.declare_parameter('lidar_topic',   LIDAR_TOPIC)
        self.declare_parameter('state_topic',   STATE_TOPIC)
        self.declare_parameter('odom_topic',    ODOM_TOPIC)
        self.declare_parameter('output_topic',   OUTPUT_TOPIC)
        self.declare_parameter('lambda_str_obj', LAMBDA_STR_OBJ)
        self.declare_parameter('r_lidar',        R_LIDAR)
        self.declare_parameter('d_scale',        D_SCALE)
        self.declare_parameter('alpha_count',    ALPHA_COUNT)

        def gp(n):
            return self.get_parameter(n).value

        self.lambda_str_obj_ = float(gp('lambda_str_obj'))
        self.r_lidar_        = float(gp('r_lidar'))
        self.d_scale_        = max(float(gp('d_scale')), 1e-6)
        self.alpha_count_    = max(float(gp('alpha_count')), 0.0)

        # Latest inputs cached between callbacks
        self._estimator_state = None     # SeabedEstimatorState
        self._lidar_tree      = None     # cKDTree of LiDAR XY
        self._usv_xyz         = None     # (3,) float64 sonar origin

        self.pub_ = self.create_publisher(
            ClassifiedReturns, gp('output_topic'), 10)
        self.sub_scan_  = self.create_subscription(
            PointCloud2, gp('input_topic'), self._scan_cb, 1)
        self.sub_lidar_ = self.create_subscription(
            PointCloud2, str(gp('lidar_topic')), self._lidar_cb, 1)
        self.sub_state_ = self.create_subscription(
            SeabedEstimatorState, gp('state_topic'), self._state_cb, 10)
        self.sub_odom_  = self.create_subscription(
            Odometry, gp('odom_topic'), self._odom_cb, 1)

        self._scan_count_ = 0
        self.get_logger().info(
            f'return_classifier: {gp("input_topic")} + {gp("lidar_topic")} '
            f'-> {gp("output_topic")} | lambda_str_obj={self.lambda_str_obj_} | '
            f'r_lidar={self.r_lidar_}m d_scale={self.d_scale_}m '
            f'alpha_count={self.alpha_count_}')

    # ── input caches ──────────────────────────────────────────────────────────

    def _state_cb(self, msg: SeabedEstimatorState):
        self._estimator_state = msg

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._usv_xyz = np.array([p.x, p.y, p.z], dtype=np.float64)

    def _lidar_cb(self, msg: PointCloud2):
        try:
            xyz = _read_fields(msg, ('x', 'y', 'z'))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Bad LiDAR cloud: {exc}',
                                   throttle_duration_sec=5.0)
            return
        xyz = xyz[np.isfinite(xyz).all(axis=1)] if xyz.size else xyz
        self._lidar_tree = cKDTree(xyz[:, :2]) if xyz.shape[0] > 0 else None

    # ── geometric LiDAR support: d_factor + count_factor (vectorised) ─────────

    def _lidar_support_factors(self, xy: np.ndarray):
        """Return (d_factor, count_factor) per return, both float32 (N,).

        d_factor     = exp(-d / d_scale)                  in [0, 1]
        count_factor = 1 - exp(-alpha_count * count)      in [0, 1)
        """
        n = xy.shape[0]
        if self._lidar_tree is None or n == 0:
            return (np.zeros(n, dtype=np.float32),
                    np.zeros(n, dtype=np.float32))

        d_arr, idx = self._lidar_tree.query(xy, k=1)
        d_arr = np.asarray(d_arr, dtype=np.float64).reshape(n)
        idx   = np.asarray(idx,   dtype=np.int64 ).reshape(n)

        qstar = np.asarray(self._lidar_tree.data, dtype=np.float64)[idx]
        counts = self._lidar_tree.query_ball_point(
            qstar, r=self.r_lidar_, return_length=True)
        counts = np.asarray(counts, dtype=np.float64)

        d_factor     = np.exp(-d_arr / self.d_scale_)
        count_factor = 1.0 - np.exp(-self.alpha_count_ * counts)
        return d_factor.astype(np.float32), count_factor.astype(np.float32)

    # ── main per-scan classification ──────────────────────────────────────────

    def _scan_cb(self, msg: PointCloud2):
        if self._estimator_state is None:
            self.get_logger().warn('No estimator state yet — skipping scan.',
                                   throttle_duration_sec=5.0)
            return
        if self._usv_xyz is None:
            self.get_logger().warn('No odometry yet — skipping scan.',
                                   throttle_duration_sec=5.0)
            return

        try:
            pts = _read_fields(msg, ('x', 'y', 'z', 'intensity'))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to read PointCloud2: {exc}',
                                    throttle_duration_sec=5.0)
            return
        pts = pts[np.isfinite(pts).all(axis=1)] if pts.size else pts
        if pts.shape[0] == 0:
            return

        xyz       = pts[:, :3].astype(np.float64)
        intensity = pts[:, 3].astype(np.float64)
        n         = xyz.shape[0]

        mu_i        = float(self._estimator_state.mu_i)
        tau_str_obj = self.lambda_str_obj_ * mu_i

        # ── Slant range = distance from the sonar (USV) to each return ─────────
        slant = np.linalg.norm(xyz - self._usv_xyz[None, :], axis=1)

        # ── Geometric LiDAR support: two separate factors ────────────────────
        d_factor, count_factor = self._lidar_support_factors(xyz[:, :2])
        # Hard-label "has support" — any product > 0. The actual soft-class
        # evidence is computed downstream in voxel_mapper.
        has_support = (d_factor.astype(np.float64) *
                       count_factor.astype(np.float64)) > 0.0

        # ── Hard classification rule (used only for FREE vs non-FREE routing
        # downstream — voxel_mapper computes soft responsibilities itself) ────
        label = np.full(n, SEABED, dtype=np.uint8)          # default: SEABED
        is_free   = intensity <= 0.0
        is_bright = (~is_free) & (intensity >= tau_str_obj)
        is_struct = is_bright & has_support
        is_object = is_bright & (~has_support)
        label[is_object] = OBJECT
        label[is_struct] = STRUCTURE
        label[is_free]   = FREE

        self._publish(msg, xyz, intensity, slant, label,
                      d_factor, count_factor, mu_i)

        self._scan_count_ += 1
        if self._scan_count_ <= 5 or self._scan_count_ % 50 == 0:
            df_pos = d_factor[d_factor > 0]
            cf_pos = count_factor[count_factor > 0]
            d_med = float(np.median(df_pos)) if df_pos.size else 0.0
            c_med = float(np.median(cf_pos)) if cf_pos.size else 0.0
            self.get_logger().info(
                f'scan #{self._scan_count_}: {n} returns | '
                f'FREE={int(is_free.sum())} '
                f'SEABED={int((label == SEABED).sum())} '
                f'OBJECT={int(is_object.sum())} '
                f'STRUCTURE={int(is_struct.sum())} | '
                f'd_factor med={d_med:.2f} count_factor med={c_med:.2f}')

    def _publish(self, src_msg, xyz, intensity, slant, label,
                 d_factor, count_factor, mu_i):
        out = ClassifiedReturns()
        out.header.stamp = src_msg.header.stamp
        out.header.frame_id = src_msg.header.frame_id
        out.mu_i = float(mu_i)
        out.x             = xyz[:, 0].astype(np.float32).tolist()
        out.y             = xyz[:, 1].astype(np.float32).tolist()
        out.z             = xyz[:, 2].astype(np.float32).tolist()
        out.intensity     = intensity.astype(np.float32).tolist()
        out.slant_range   = slant.astype(np.float32).tolist()
        out.label         = label.astype(np.uint8).tolist()
        out.d_factor      = d_factor.astype(np.float32).tolist()
        out.count_factor  = count_factor.astype(np.float32).tolist()
        self.pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ReturnClassifierNode()
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
