#!/usr/bin/env python3
"""
return_classifier — Node 2 of the Dirichlet voxel mapping pipeline.
===============================================================================

Consumes raw sonar returns (``input_topic``, odom frame) together with the
LiDAR point cloud (``lidar_topic``) and the seabed estimator state
(``state_topic``), and emits one batch of semantically labelled returns per
scan on ``output_topic``.

ADAPTIVE INTENSITY THRESHOLDS  (derived from the estimator's mu_I)
      tau_struct = lambda_struct * mu_I
      tau_obj    = lambda_obj    * mu_I            (lambda_struct < lambda_obj)

GEOMETRIC LiDAR SUPPORT SCORE  (continuous, replaces the old ball-count)
  For each sonar return p (XY-plane only):
      q*       = nearest LiDAR point to p
      d        = || p_xy - q*_xy ||
      v_count  = | { LiDAR points c : || c_xy - q*_xy || <= r_lidar } |
      v        = v_count * exp( -d / d_scale )

  Geometry knobs (config):
      r_lidar  — full-disc radius around q*.
      d_scale  — exponential decay length on the distance from the return
                 to its nearest LiDAR point.

  Properties:
      - v in [0, +inf); 0 when no LiDAR cloud is available.
      - v_count >= 1 whenever a LiDAR cloud exists (q* itself is in the disc).
      - Distance gates everything: a return far from any LiDAR returns
        v ≈ 0 (and therefore zero STRUCTURE responsibility downstream).
      - Density boosts: dense LiDAR around q* → large v_count → large v.
  Voxel_mapper turns v into a soft STRUCTURE responsibility via
        f_L = 1 - exp(-alpha_L * v).

ORDERED CLASSIFICATION RULE  (the hard label is used only for routing —
voxel_mapper distinguishes FREE vs non-FREE; the actual class evidence is the
soft per-return responsibility computed there):
      FREE       if intensity == 0
      STRUCTURE  if intensity >= tau_struct AND v > 0
      OBJECT     if intensity >= tau_obj    AND v == 0
      SEABED     otherwise

Each return keeps its position, intensity, slant range (distance from the
sonar, taken as the USV position from odometry), assigned hard label and
LiDAR support score v. They are shipped as parallel arrays in a
ClassifiedReturns message — see sonar_map_msgs/msg/ClassifiedReturns.msg.
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

LAMBDA_STRUCT = 1.5      # structure threshold multiplier on mu_I
LAMBDA_OBJ    = 2.5      # object    threshold multiplier on mu_I
R_LIDAR       = 1.5      # [m]  full-disc radius around the nearest LiDAR q*
D_SCALE       = 1.0      # [m]  exponential decay length on distance p -> q*


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
        self.declare_parameter('output_topic',  OUTPUT_TOPIC)
        self.declare_parameter('lambda_struct', LAMBDA_STRUCT)
        self.declare_parameter('lambda_obj',    LAMBDA_OBJ)
        self.declare_parameter('r_lidar',       R_LIDAR)
        self.declare_parameter('d_scale',       D_SCALE)

        def gp(n):
            return self.get_parameter(n).value

        self.lambda_struct_ = float(gp('lambda_struct'))
        self.lambda_obj_    = float(gp('lambda_obj'))
        self.r_lidar_       = float(gp('r_lidar'))
        self.d_scale_       = max(float(gp('d_scale')), 1e-6)
        if self.lambda_struct_ >= self.lambda_obj_:
            self.get_logger().warn(
                f'lambda_struct ({self.lambda_struct_}) should be < '
                f'lambda_obj ({self.lambda_obj_}); classification may misbehave.')

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
            f'-> {gp("output_topic")} | lambda_struct={self.lambda_struct_} '
            f'lambda_obj={self.lambda_obj_} | '
            f'r_lidar={self.r_lidar_}m d_scale={self.d_scale_}m')

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

    # ── geometric LiDAR support v ─────────────────────────────────────────────

    def _lidar_support_v(self, xy: np.ndarray) -> np.ndarray:
        """v = v_count_around_qstar(R) * exp(-d_to_qstar / d_scale), vectorised.

        xy: (N, 2) float; returns (N,) float32 >= 0.
        """
        n = xy.shape[0]
        if self._lidar_tree is None or n == 0:
            return np.zeros(n, dtype=np.float32)

        # 1-NN distance + index of nearest LiDAR for each return.
        d_arr, idx = self._lidar_tree.query(xy, k=1)
        d_arr = np.asarray(d_arr, dtype=np.float64).reshape(n)
        idx   = np.asarray(idx,   dtype=np.int64 ).reshape(n)

        # Coordinates of the nearest LiDAR points (q*).
        qstar = np.asarray(self._lidar_tree.data, dtype=np.float64)[idx]

        # Full-disc count of LiDAR points within r_lidar of q*.
        counts = self._lidar_tree.query_ball_point(
            qstar, r=self.r_lidar_, return_length=True)
        counts = np.asarray(counts, dtype=np.float64)

        v = counts * np.exp(-d_arr / self.d_scale_)
        return v.astype(np.float32)

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

        mu_i       = float(self._estimator_state.mu_i)
        tau_struct = self.lambda_struct_ * mu_i
        tau_obj    = self.lambda_obj_    * mu_i

        # ── Slant range = distance from the sonar (USV) to each return ─────────
        slant = np.linalg.norm(xyz - self._usv_xyz[None, :], axis=1)

        # ── Geometric LiDAR support score v ──────────────────────────────────
        v_arr = self._lidar_support_v(xyz[:, :2])
        has_support = v_arr > 0.0

        # ── Hard classification rule (used only for FREE vs non-FREE routing
        # downstream — voxel_mapper computes soft responsibilities itself) ────
        label = np.full(n, SEABED, dtype=np.uint8)          # default: SEABED
        is_free   = intensity <= 0.0
        is_struct = (~is_free) & (intensity >= tau_struct) & has_support
        is_object = (~is_free) & (~is_struct) & (intensity >= tau_obj) & (~has_support)
        label[is_object] = OBJECT
        label[is_struct] = STRUCTURE
        label[is_free]   = FREE

        self._publish(msg, xyz, intensity, slant, label, v_arr,
                      tau_struct, tau_obj, mu_i)

        self._scan_count_ += 1
        if self._scan_count_ <= 5 or self._scan_count_ % 50 == 0:
            vpos = v_arr[v_arr > 0]
            vmax = float(vpos.max()) if vpos.size else 0.0
            vmed = float(np.median(vpos)) if vpos.size else 0.0
            self.get_logger().info(
                f'scan #{self._scan_count_}: {n} returns | '
                f'FREE={int(is_free.sum())} '
                f'SEABED={int((label == SEABED).sum())} '
                f'OBJECT={int(is_object.sum())} '
                f'STRUCTURE={int(is_struct.sum())} | '
                f'v: max={vmax:.1f} med={vmed:.1f} | '
                f'tau_obj={tau_obj:.0f}')

    def _publish(self, src_msg, xyz, intensity, slant, label, v_arr,
                 tau_struct, tau_obj, mu_i):
        out = ClassifiedReturns()
        out.header.stamp = src_msg.header.stamp
        out.header.frame_id = src_msg.header.frame_id
        out.seabed_depth = 0.0                # depth KF removed; field kept = 0
        out.tau_struct   = float(tau_struct)
        out.tau_obj      = float(tau_obj)
        out.mu_i         = float(mu_i)
        out.x             = xyz[:, 0].astype(np.float32).tolist()
        out.y             = xyz[:, 1].astype(np.float32).tolist()
        out.z             = xyz[:, 2].astype(np.float32).tolist()
        out.intensity     = intensity.astype(np.float32).tolist()
        out.slant_range   = slant.astype(np.float32).tolist()
        out.label         = label.astype(np.uint8).tolist()
        out.lidar_support = v_arr.astype(np.float32).tolist()
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
