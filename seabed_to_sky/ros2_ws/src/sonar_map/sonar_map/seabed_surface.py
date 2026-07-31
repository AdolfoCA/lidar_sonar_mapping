#!/usr/bin/env python3
"""
seabed_surface — Node 4 of the eq-24 Dirichlet voxel mapping pipeline.
===============================================================================

Owns the promotion protocol. On a periodic sweep (`sweep_rate`, default 0.2 Hz)
it takes the latest Dirichlet voxel-map snapshot from voxel_mapper, finds
every voxel whose dominant class is the promoted class and which clears the
promotion criterion, marks them as promoted, and sends the keys back to
voxel_mapper so it can retire them.

The class set is NOT hardcoded: the snapshot carries the full inventory
(n_classes, class_names, flattened soft counts w) as defined by classes.yaml
plus the trailing catch-all "other". The promoted class is matched by NAME
(`seabed_class_name`, default 'seabed') against class_names of EVERY snapshot,
so editing/reordering classes.yaml never silently promotes the wrong column —
an absent name logs an error and disables promotion instead.

NOTE: patch aggregation, the smoothed Delaunay mesh, the patch point cloud and
the cumulative /debug/promoted cloud were intentionally stripped from this
node to accelerate the pipeline and isolate the source of slowness. Only the
promotion sweep + feedback remain. The Patch / Kalman / smoothing code lives
in git history if you want it back.

PROMOTION-READINESS CHECK  (applied here, once per sweep)
  Posterior class probabilities (paper eq 21) under the symmetric Dirichlet
  prior alpha_0 over the K̃ = n_classes categories of the snapshot:
      pi_c    = (alpha_0 + w_c) / Z
      Z       = K̃*alpha_0 + W
  A voxel is promoted when (c* = index of `seabed_class_name`):
      argmax_c pi_c == c*  AND  pi_c* >= pi_conf  AND  W >= w_conf
  Promotion is irreversible — a voxel is promoted at most once.

TOPICS
  voxel-map snapshot  (`voxel_map_topic`)    — subscribed; DirichletVoxelMap.
  promotion feedback  (`promoted_keys_topic`) — published; PromotedVoxelKeys.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from sonar_map_msgs.msg import DirichletVoxelMap, PromotedVoxelKeys


# ── Defaults — overridable from the YAML config / launch arguments ────────────
VOXEL_MAP_TOPIC     = 'dirichlet_voxel_map'
PROMOTED_KEYS_TOPIC = 'promoted_voxel_keys'
MAP_FRAME           = 'odom'

SEABED_CLASS_NAME = 'seabed'  # promoted class, matched by NAME in class_names

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
        self.declare_parameter('seabed_class_name',   SEABED_CLASS_NAME)
        self.declare_parameter('alpha_0',             ALPHA_0)
        self.declare_parameter('pi_conf',             PI_CONF)
        self.declare_parameter('w_conf',              W_CONF)
        self.declare_parameter('sweep_rate',          SWEEP_RATE)

        def gp(n):
            return self.get_parameter(n).value

        self._map_frame  = str(gp('map_frame'))
        self._class_name = str(gp('seabed_class_name'))
        self._a0         = float(gp('alpha_0'))
        self._pi_conf    = float(gp('pi_conf'))
        self._w_conf     = float(gp('w_conf'))

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
            f'promote "{self._class_name}" voxels with pi>={self._pi_conf} '
            f'& W>={self._w_conf} | feedback -> {gp("promoted_keys_topic")}')

    # ── snapshot intake ───────────────────────────────────────────────────────

    def _snapshot_cb(self, msg: DirichletVoxelMap):
        self._snapshot = msg

    # ── periodic promotion sweep ──────────────────────────────────────────────

    def _sweep(self):
        """Promote every ready voxel of the promoted class in the latest
        snapshot and feed the keys back to voxel_mapper so it can retire
        them."""
        self._sweep_count += 1
        snap = self._snapshot
        n_new = 0
        new_promotion_w = []

        if snap is not None and len(snap.voxel_key) > 0:
            # The class axis is snapshot-defined (classes.yaml + "other") —
            # resolve the promoted column by NAME on every snapshot so a
            # reordered/edited inventory can never promote the wrong class.
            names = list(snap.class_names)
            if self._class_name not in names:
                self.get_logger().error(
                    f'seabed_class_name "{self._class_name}" is not in the '
                    f'snapshot class inventory {names} — check classes.yaml / '
                    f'the seabed_class_name parameter; skipping promotion sweep')
                return
            c_star = names.index(self._class_name)

            kt   = int(snap.n_classes)                       # K̃ = K + 1
            keys = np.asarray(snap.voxel_key, dtype=np.int64)
            W    = np.asarray(snap.weight,    dtype=np.float64)
            if kt != len(names) or len(snap.w) != kt * len(keys):
                self.get_logger().error(
                    f'malformed DirichletVoxelMap: n_classes={kt}, '
                    f'|class_names|={len(names)}, |w|={len(snap.w)}, '
                    f'|voxel_key|={len(keys)} — skipping promotion sweep')
                return
            w = np.asarray(snap.w, dtype=np.float64).reshape(len(keys), kt)

            # Vectorised promotion-readiness check (posterior mean, eq 21)
            # under the symmetric prior: Z = K̃·alpha_0 + W per voxel.
            num   = self._a0 + w
            denom = kt * self._a0 + W                        # shared denominator
            pi    = num / denom[:, None]
            dom   = np.argmax(pi, axis=1)
            pmax  = pi[np.arange(len(keys)), dom]
            ready = (dom == c_star) & (pmax >= self._pi_conf) & (W >= self._w_conf)

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
