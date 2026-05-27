#!/usr/bin/env python3
"""
seabed_surface — Node 4 of the Dirichlet voxel mapping pipeline.
===============================================================================

Owns the promotion protocol. On a periodic sweep (`sweep_rate`, default 0.2 Hz)
it takes the latest Dirichlet voxel-map snapshot from voxel_mapper, finds
every voxel whose dominant class is SEABED and which clears the promotion
criterion, marks them as promoted, and sends the keys back to voxel_mapper so
it can retire them.

NOTE: patch aggregation, the smoothed Delaunay mesh, the patch point cloud and
the cumulative /debug/promoted cloud were intentionally stripped from this
node to accelerate the pipeline and isolate the source of slowness. Only the
promotion sweep + feedback remain. The Patch / Kalman / smoothing code lives
in git history if you want it back.

PROMOTION-READINESS CHECK  (applied here, once per sweep)
  Posterior class probabilities under the symmetric Dirichlet prior alpha_0
  plus the per-voxel LiDAR-derived STRUCTURE prior alpha_str_prior (only
  enters pi_str's numerator and the shared denominator):
      pi_c    = (alpha_0 + w_c) / Z              for c != STRUCTURE
      pi_str  = (alpha_0 + alpha_str_prior + w_str) / Z
      Z       = 4*alpha_0 + alpha_str_prior + W
  A voxel is promoted when:
      argmax_c pi_c == SEABED  AND  pi_SEABED >= pi_conf  AND  W >= w_conf
  Promotion is irreversible — a voxel is promoted at most once.

TOPICS
  voxel-map snapshot  (`voxel_map_topic`)    — subscribed; DirichletVoxelMap.
  promotion feedback  (`promoted_keys_topic`) — published; PromotedVoxelKeys.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from sonar_map_msgs.msg import DirichletVoxelMap, PromotedVoxelKeys


SEABED = 1   # index of SEABED in the weight vector [free, sb, obj, str]

# ── Defaults — overridable from the YAML config / launch arguments ────────────
VOXEL_MAP_TOPIC     = 'dirichlet_voxel_map'
PROMOTED_KEYS_TOPIC = 'promoted_voxel_keys'
MAP_FRAME           = 'odom'

ALPHA_0    = 0.01    # [-]  symmetric Dirichlet prior (<< 1)
PI_CONF    = 0.70    # [-]  posterior confidence threshold
W_CONF     = 25.0    # [-]  evidence weight threshold
SWEEP_RATE = 0.2     # [Hz] promotion sweep / feedback rate


class SeabedSurfaceNode(Node):

    def __init__(self):
        super().__init__('seabed_surface')

        self.declare_parameter('voxel_map_topic',     VOXEL_MAP_TOPIC)
        self.declare_parameter('promoted_keys_topic', PROMOTED_KEYS_TOPIC)
        self.declare_parameter('map_frame',           MAP_FRAME)
        self.declare_parameter('alpha_0',             ALPHA_0)
        self.declare_parameter('pi_conf',             PI_CONF)
        self.declare_parameter('w_conf',              W_CONF)
        self.declare_parameter('sweep_rate',          SWEEP_RATE)

        def gp(n):
            return self.get_parameter(n).value

        self._map_frame = str(gp('map_frame'))
        self._a0        = float(gp('alpha_0'))
        self._four_a0   = 4.0 * self._a0
        self._pi_conf   = float(gp('pi_conf'))
        self._w_conf    = float(gp('w_conf'))

        self._promoted: set = set()   # voxel keys already promoted (irreversible)
        self._snapshot = None         # latest DirichletVoxelMap

        self._keys_pub = self.create_publisher(
            PromotedVoxelKeys, gp('promoted_keys_topic'), 10)
        self._sub = self.create_subscription(
            DirichletVoxelMap, gp('voxel_map_topic'), self._snapshot_cb, 10)

        rate = max(float(gp('sweep_rate')), 1e-3)
        self._timer = self.create_timer(1.0 / rate, self._sweep)

        self._sweep_count = 0
        self._n_promoted = 0
        self.get_logger().info(
            f'seabed_surface: sweeps {gp("voxel_map_topic")} @ {rate}Hz | '
            f'promote SEABED voxels with pi>={self._pi_conf} & W>={self._w_conf} | '
            f'feedback -> {gp("promoted_keys_topic")}')

    # ── snapshot intake ───────────────────────────────────────────────────────

    def _snapshot_cb(self, msg: DirichletVoxelMap):
        self._snapshot = msg

    # ── periodic promotion sweep ──────────────────────────────────────────────

    def _sweep(self):
        """Promote every SEABED-ready voxel in the latest snapshot and feed
        the keys back to voxel_mapper so it can retire them."""
        self._sweep_count += 1
        snap = self._snapshot
        n_new = 0
        new_promotion_w = []

        if snap is not None and len(snap.voxel_key) > 0:
            keys = np.asarray(snap.voxel_key, dtype=np.int64)
            W    = np.asarray(snap.weight,    dtype=np.float64)
            w4   = np.stack([np.asarray(snap.w_free, dtype=np.float64),
                             np.asarray(snap.w_sb,   dtype=np.float64),
                             np.asarray(snap.w_obj,  dtype=np.float64),
                             np.asarray(snap.w_str,  dtype=np.float64)], axis=1)
            aps  = np.asarray(snap.alpha_str_prior, dtype=np.float64)

            # Vectorised promotion-readiness check. The per-voxel
            # alpha_str_prior contribution lifts pi_str only, so SEABED
            # promotion gets correspondingly harder near LiDAR-supported
            # structures (which is the point).
            num = self._a0 + w4
            num[:, 3] += aps                                # STRUCTURE numerator
            denom = self._four_a0 + aps + W                  # shared denominator
            pi    = num / denom[:, None]
            dom   = np.argmax(pi, axis=1)
            pmax  = pi[np.arange(len(keys)), dom]
            ready = (dom == SEABED) & (pmax >= self._pi_conf) & (W >= self._w_conf)

            for i in np.nonzero(ready)[0]:
                k = int(keys[i])
                if k in self._promoted:
                    continue            # already promoted — irreversible
                self._promoted.add(k)
                new_promotion_w.append(float(W[i]))
                self._n_promoted += 1
                n_new += 1

            # Feedback: every promoted key still in the snapshot — voxel_mapper
            # removes them. Self-healing if a message is dropped.
            snap_keys = set(int(k) for k in keys)
            to_retire = [k for k in self._promoted if k in snap_keys]
            if to_retire:
                fb = PromotedVoxelKeys()
                fb.header.stamp = self.get_clock().now().to_msg()
                fb.header.frame_id = self._map_frame
                fb.keys = to_retire
                self._keys_pub.publish(fb)

        if self._sweep_count <= 5 or self._sweep_count % 10 == 0:
            w_hint = ''
            if new_promotion_w:
                w_hint = (f' | W@promotion='
                          f'[{min(new_promotion_w):.1f}..{max(new_promotion_w):.1f}]')
            self.get_logger().info(
                f'sweep #{self._sweep_count}: +{n_new} promoted '
                f'(total={self._n_promoted}){w_hint}')


def main(args=None):
    rclpy.init(args=args)
    node = SeabedSurfaceNode()
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
