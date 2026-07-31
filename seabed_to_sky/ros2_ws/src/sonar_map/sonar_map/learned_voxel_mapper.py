#!/usr/bin/env python3
"""
learned_voxel_mapper — Dirichlet voxel map with ONLINE-LEARNED, DYNAMIC classes.
===============================================================================

A parallel, ADDITIVE alternative to voxel_mapper.py. It keeps the paper's
per-voxel Dirichlet–categorical update (eqs 16–23) and the 21-bit voxel hashing,
but replaces the FIXED hand-tuned class likelihoods with the online-learned
conjugate class models of `class_model` / `class_registry` (paper §II-E–G).
voxel_mapper.py, its messages, save_map and plot_dirichlet are untouched; this
node publishes on its OWN topics.

CLASSES ARE JUST NUMBERED (0, 1, 2, …). There is NO seabed/object/structure
labelling. The node starts with ZERO classes; each distinct INTENSITY signature
that sustains enough novelty is born as a new class (stick-breaking creation,
eq 53), and every class keeps learning its intensity Gaussian N(μ_c, σ_c²) online
(eq 52) on every scan.

VOXEL MAP (per-class)
  Each voxel accumulates Dirichlet soft-counts over the DYNAMIC class set. A
  voxel's label is the argmax class id — i.e. which learned class dominates it.
  The `learned_voxels` cloud carries that class id per voxel, so colouring by
  `class_id` shows the map segmented by learned intensity class.

FIRST-CUT SCOPE (intensity only): only the intensity channel is active (eq 33
with the height-lift λ·h dropped); height / LiDAR-hurdle / shape channels abstain.
The neighbourhood spatial prior (eq 23) is off (eps=0), so class generation is
driven purely by intensity.

TOPICS
  `voxel_topic`        (default learned_voxels)        — PointCloud2, per-voxel
      dominant class id + confidence, at `active_publish_rate`.
  `class_states_topic` (default learned_class_states)  — live per-class μ/σ/
      evidence telemetry (Foxglove/PlotJuggler), one message per scan.
"""

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

from sonar_map_msgs.msg import ClassifiedReturns, LearnedClassStates

# Reuse the EXACT voxel hashing of voxel_mapper so keys/centres match.
from sonar_map.voxel_mapper import _pack, _unpack
from sonar_map.conjugate import NIG
from sonar_map.class_registry import ClassRegistry


class Voxel:
    """One persistent voxel: Dirichlet soft-counts over the DYNAMIC class set
    (fixed-capacity vector, index == class id) + total evidence."""

    __slots__ = ('w', 'W', 'last_i')

    def __init__(self, ncap: int):
        self.w = np.zeros(ncap, dtype=np.float64)   # m_v[class_id]
        self.W = 0.0
        self.last_i = 0.0


class LearnedVoxelMapperNode(Node):

    def __init__(self):
        super().__init__('learned_voxel_mapper')
        d = self.declare_parameter

        # -- topics / frame / voxel geometry --
        d('input_topic', 'classified_returns')
        d('voxel_topic', 'learned_voxels')
        d('class_states_topic', 'learned_class_states')   # live class telemetry (Foxglove)
        d('map_frame', 'odom')
        d('r_v', 0.05)
        d('alpha_0', 0.01)              # symmetric Dirichlet prior per class
        d('eps', 0.0)                   # spatial-prior radius; 0 = uniform (intensity-only)

        # -- base prior Λ_H: the template EVERY class is born from --
        d('h_m0', 800.0)                # central intensity for the new-class predictive
        d('h_kappa0', 0.01)             # tiny → uncertain mean → wide novelty predictive
        d('h_sigma0', 200.0)            # expected within-class intensity spread → new-class sharpness
        d('h_a0', 2.0)
        d('seed_lambda_h_from_mu_i', True)  # center Λ_H on live μ_I on the first scan

        # -- class-generation controls --
        d('gamma', 0.5)                 # stick-breaking concentration for the new column
        d('novelty_return_floor', 0.5)  # r_new a return needs to count toward creation
        d('novelty_min_count', 25)      # sustained novel returns per intensity-bin before birth
        d('novelty_mass_thresh', 12.0)  # total novelty mass per intensity-bin before birth
        d('novelty_bin', 150.0)         # intensity-space bin novelty accrues in
        d('create_merge_sigmas', 3.0)   # don't spawn a class within kσ of an existing one
        d('max_classes', 32)            # capacity of the dynamic inventory / per-voxel vector

        # -- publishing / display --
        d('active_publish_rate', 2.0)
        d('w_ref', 5.0)
        d('w_display_min', 1.0)
        d('log_class_every', 50)        # log the full inventory every N scans

        gp = lambda n: self.get_parameter(n).value

        self._map_frame = str(gp('map_frame'))
        self._r_v = float(gp('r_v'))
        self._inv_r_v = 1.0 / self._r_v
        self._a0 = float(gp('alpha_0'))
        self._eps = max(float(gp('eps')), 0.0)
        self._use_prior = self._eps > 0.0
        self._w_ref = max(float(gp('w_ref')), 1e-6)
        self._w_disp_min = float(gp('w_display_min'))
        self._ncap = int(gp('max_classes'))
        self._novelty_bin = max(float(gp('novelty_bin')), 1.0)
        self._log_every = max(int(gp('log_class_every')), 1)
        self._seed_lh_from_mu_i = bool(gp('seed_lambda_h_from_mu_i'))
        self._lh_seeded = False

        # -- build the EMPTY dynamic registry: every class is born from Λ_H,
        #    and every class keeps learning online (eq 52) on every scan. --
        h_sigma0 = float(gp('h_sigma0'))
        h_a0 = float(gp('h_a0'))
        self._lambda_h = NIG(float(gp('h_m0')), float(gp('h_kappa0')),
                             h_a0, h_a0 * h_sigma0 * h_sigma0)
        self._reg = ClassRegistry(
            [], self._lambda_h,
            gamma=float(gp('gamma')),
            novelty_return_floor=float(gp('novelty_return_floor')),
            novelty_min_count=int(gp('novelty_min_count')),
            novelty_mass_thresh=float(gp('novelty_mass_thresh')),
            create_merge_sigmas=float(gp('create_merge_sigmas')))

        self._store: dict = {}          # key -> Voxel (per-class soft counts)

        self._cloud_fields = [
            PointField(name='x',          offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',          offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',          offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='class_id',   offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='class_prob', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='confidence', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='weight',     offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity',  offset=28, datatype=PointField.FLOAT32, count=1),
        ]

        self._voxel_pub = self.create_publisher(PointCloud2, gp('voxel_topic'), 10)
        self._states_pub = self.create_publisher(
            LearnedClassStates, gp('class_states_topic'), 10)
        self._sub = self.create_subscription(
            ClassifiedReturns, gp('input_topic'), self._returns_cb, 10)

        cloud_rate = max(float(gp('active_publish_rate')), 1e-3)
        self.create_timer(1.0 / cloud_rate, self._publish_active)

        self._scan_count = 0
        if self._use_prior:
            self.get_logger().warn(
                'eps>0 (spatial prior) is not wired in this intensity-only first '
                'cut; falling back to the uniform prior. Set eps=0 to silence.')
            self._use_prior = False
        self.get_logger().info(
            f'learned_voxel_mapper: {gp("input_topic")} -> {gp("voxel_topic")} '
            f'(+ {gp("class_states_topic")} telemetry) | UNSUPERVISED, INTENSITY-ONLY, '
            f'NUMBERED CLASSES: starts with 0 classes, all born from Λ_H '
            f'(m0={self._lambda_h.m:.0f}, sigma0={h_sigma0:.0f}) | r_v={self._r_v}m '
            f'gamma={gp("gamma")} novelty(bin={self._novelty_bin:.0f}, '
            f'min_count={gp("novelty_min_count")}, mass={gp("novelty_mass_thresh")}) '
            f'merge={gp("create_merge_sigmas")}σ cap={self._ncap}')

    # ── per-scan processing ───────────────────────────────────────────────────
    def _returns_cb(self, msg: ClassifiedReturns):
        n = len(msg.intensity)
        if n == 0:
            return
        inten = np.asarray(msg.intensity, dtype=np.float64)
        x = np.asarray(msg.x, dtype=np.float64)
        y = np.asarray(msg.y, dtype=np.float64)
        z = np.asarray(msg.z, dtype=np.float64)

        # Center Λ_H on the live seabed intensity μ_I once, so the new-class
        # predictive sits where the data is before ANY class exists.
        if self._seed_lh_from_mu_i and not self._lh_seeded and msg.mu_i > 0.0:
            self._lambda_h.m = float(msg.mu_i)
            self._lh_seeded = True
            self.get_logger().info(f'centered Λ_H mean <- mu_i={msg.mu_i:.0f}')

        ix = np.floor(x * self._inv_r_v).astype(np.int64)
        iy = np.floor(y * self._inv_r_v).astype(np.int64)
        iz = np.floor(z * self._inv_r_v).astype(np.int64)

        # Class creation accrues novelty per INTENSITY BIN, not per voxel: on real
        # data returns are spread ~1-per-voxel, so novelty must group by intensity
        # SIGNATURE for classes to form.
        nkeys = np.floor(inten / self._novelty_bin).astype(np.int64)

        # Assignment over active ∪ {new}; then EVERY active class is updated online
        # (eq 52), and a new class is born from Λ_H when an intensity bin gathers
        # sustained novelty (eq 53).
        r_active, r_new = self._reg.assign(inten)
        created = []
        if self._reg.n_classes < self._ncap:
            created = self._reg.accumulate(inten, nkeys, r_active, r_new)
        else:
            for c, m in enumerate(self._reg.classes):
                m.update(inten, r_active[:, c])

        for cid in created:
            cm = next(c for c in self._reg.classes if c.class_id == cid)
            self.get_logger().info(
                f'BORN class {cid} I~N({cm.intensity_mean:.0f}, {cm.intensity_std:.0f}) '
                f'| inventory now {self._reg.n_classes} classes')

        # Deposit per-class responsibilities into voxels. r_active has only the
        # pre-creation columns (class_id == column index); a class born this scan
        # already absorbed its triggering returns via ⊕ and carries no column.
        self._deposit(r_active, ix, iy, iz, inten)

        self._publish_class_states()                 # live telemetry every scan

        self._scan_count += 1
        if self._scan_count <= 3 or self._scan_count % self._log_every == 0:
            self._log_inventory(n)

    def _deposit(self, r_active, ix, iy, iz, inten):
        """Sum per-return, per-class responsibility into voxels (one voxel per
        return; grouped and summed once per distinct voxel). Column c → class id c."""
        C = r_active.shape[1]
        idx = np.stack([ix, iy, iz], axis=1)
        uniq, inv = np.unique(idx, axis=0, return_inverse=True)
        inv = inv.ravel()
        if C > 0:
            sums = np.zeros((uniq.shape[0], C), dtype=np.float64)
            np.add.at(sums, inv, r_active)
        last_i = np.zeros(uniq.shape[0], dtype=np.float64)
        last_i[inv] = inten
        for j in range(uniq.shape[0]):
            key = _pack(int(uniq[j, 0]), int(uniq[j, 1]), int(uniq[j, 2]))
            v = self._store.get(key)
            if v is None:
                v = self._store[key] = Voxel(self._ncap)
            if C > 0:
                v.w[:C] += sums[j]
                v.W += float(sums[j].sum())
            v.last_i = float(last_i[j])

    def _log_inventory(self, n):
        parts = [f'#{c.class_id}~N({c.intensity_mean:.0f},{c.intensity_std:.0f})'
                 f'[k={c.evidence:.0f}]' for c in self._reg.classes]
        self.get_logger().info(
            f'scan #{self._scan_count}: {n} returns | {self._reg.n_classes} classes '
            f'| active voxels {len(self._store)} | ' + '  '.join(parts))

    # ── live class telemetry (Foxglove/PlotJuggler) ───────────────────────────
    def _publish_class_states(self):
        """Publish the current learned inventory: per-class μ, σ, evidence — one
        message per scan so μ/σ/n_classes can be plotted over time."""
        classes = self._reg.classes
        msg = LearnedClassStates()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.n_classes = len(classes)
        msg.class_id = [int(c.class_id) for c in classes]
        msg.semantic_tag = [0 for _ in classes]      # deprecated: classes are numbered
        msg.mu = [float(c.intensity_mean) for c in classes]
        msg.sigma = [float(c.intensity_std) for c in classes]
        msg.evidence = [float(c.evidence) for c in classes]
        self._states_pub.publish(msg)

    # ── active voxel cloud (per-voxel dominant class id) ───────────────────────
    def _publish_active(self):
        N = self._reg.n_classes
        items = list(self._store.items())
        if N == 0 or not items:
            return
        keys = np.fromiter((k for k, _ in items), dtype=np.int64, count=len(items))
        wmat = np.array([v.w[:N] for _, v in items], dtype=np.float64)   # (M, N)
        Ws = np.array([v.W for _, v in items], dtype=np.float64)
        last_i = np.array([v.last_i for _, v in items], dtype=np.float64)

        # Posterior mean over the N classes (eq 21); argmax = dominant class id.
        pi = (self._a0 + wmat) / (N * self._a0 + Ws)[:, None]           # (M, N)
        cls = np.argmax(pi, axis=1)
        pmax = pi[np.arange(len(keys)), cls]
        conf = pmax * np.tanh(Ws / self._w_ref)

        keep = Ws >= self._w_disp_min
        if not np.any(keep):
            return
        ix, iy, iz = _unpack(keys[keep])
        out = np.empty((int(keep.sum()), 8), dtype=np.float32)
        out[:, 0] = (ix.astype(np.float64) + 0.5) * self._r_v
        out[:, 1] = (iy.astype(np.float64) + 0.5) * self._r_v
        out[:, 2] = (iz.astype(np.float64) + 0.5) * self._r_v
        out[:, 3] = cls[keep].astype(np.float32)          # class_id (dominant)
        out[:, 4] = pmax[keep].astype(np.float32)         # class_prob
        out[:, 5] = conf[keep].astype(np.float32)
        out[:, 6] = Ws[keep].astype(np.float32)
        out[:, 7] = last_i[keep].astype(np.float32)

        header = Header()
        header.frame_id = self._map_frame
        header.stamp = self.get_clock().now().to_msg()
        self._voxel_pub.publish(pc2.create_cloud(header, self._cloud_fields, out))


def main(args=None):
    rclpy.init(args=args)
    node = LearnedVoxelMapperNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
