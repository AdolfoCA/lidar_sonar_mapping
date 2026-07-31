#!/usr/bin/env python3
"""
voxel_mapper — Node 3 of the Dirichlet voxel mapping pipeline.
===============================================================================

Maintains the central Dirichlet–Categorical voxel map over the FIXED class set
defined in classes.yaml: K predefined classes plus the trailing catch-all
"other" — the paper's base measure Λ_H made concrete (Alg. 1 line 8). There is
NO online class learning here: every parameter of the paper's Table I is a
fixed value read from the config, and the class axis (length K+1, "other"
LAST) is set once at startup. This node implements the per-voxel posterior
update derived in the paper (eqs 16–23); the class-conditional likelihoods now
come from the eq-24 factorisation implemented in the pure `dirichlet` package.
The map is PERSISTENT — voxels accumulate evidence indefinitely and are never
retired; the per-voxel label is the MAP argmax of the posterior.

PER-RETURN UPDATE  (paper eqs 16–24, applied per return i hitting voxel v)
  Class-conditional likelihood (eq 24), assembled in LOG space by
  dirichlet.likelihood from the measurement channels of ClassifiedReturns:

      L_{i,c} = p(p_i|c) · p(I_i|p_i,c) · p(ℓ_i|c) · p(m_i|c)           (24)
        position  p(p_i|c)     = N(h_i; ν_c, ω²_c + ϱ_i)                (31)
        intensity p(I_i|p_i,c) = N(I_i; μ_c + λ·h_i, σ²_c + λ²·ϱ_i)     (34)
        lidar     p(ℓ_i|c)     = hurdle π^s(1−π)^{1−s}·[Pois>0·Exp]^s   (37)
        shape     p(m_i|c)     = hurdle ρ^s(1−ρ)^{1−s}·p(m_R|c)^{s/N_R} (42)

  with h_i = z_i − ẑ_bed(x_i, y_i) (eq 25) and the elevation-ambiguity
  variance ϱ_i = (r_i·cosθ·Δϕ)²/12 (eqs 29–30), both computed here from the
  message channels via dirichlet.geometry. A factor whose channel is missing
  (NaN — e.g. no bottom reference yet, no LiDAR cloud, old bag without the
  shape extractor) ABSTAINS with a log-likelihood row of zeros, so the
  remaining factors still combine correctly. The "other" column receives
  log(other_weight) once on the total (the analogue of the paper's
  stick-breaking γ, Alg. 1 line 9).

  Spatial prior (eq 23), neighbourhood N_v = voxels within radius ε of v, read
  from the PREVIOUS sweep's frozen snapshot (Jacobi update — no within-sweep
  feedback); uniform fallback when the neighbourhood carries no evidence:

      π_c(N_v) = Σ_{u∈N_v} M_u · θ̂^{(t-1)}_{u,c} / Σ_{u∈N_v} M_u
               = 1/|C|   when Σ_u M_u = 0

  Responsibility (eq 16) and soft-count accumulation (eqs 18–20), evaluated as
  a numerically stable row-softmax of log L + log π (dirichlet.likelihood):

      r_{i,c} = L_{i,c} · π_c(N_v) / Σ_c' L_{i,c'} · π_c'(N_v),  Σ_c r_{i,c}=1
      m_{v,c} += r_{i,c}

  ELEVATION BAND (engineering detail, kept from the prior implementation; the
  follow-on work-stream replaces it with a principled beam kernel): each return's
  unit responsibility mass is spread uniformly over a vertical band of width
  k_elev · slant_range (top-hat, 1/N_band per band voxel). k_elev = 0 collapses
  to a single voxel.

POSTERIOR (eqs 21–22)
      θ̂_{v,c} = (α_0 + m_{v,c}) / (|C|·α_0 + M_v)         posterior mean
      ĉ_MAP   = argmax_c (α_0 + m_{v,c})                  MAP label

TOPICS
  active voxel cloud (`voxel_topic`)   — PointCloud2 for visualisation, at
      `active_publish_rate`. Confidence = max_c θ̂_c · tanh(W / w_ref); `weight`
      carries raw W; a display-only floor (W ≥ w_display_min) is applied. The
      `semantic` field is the argmax class id (index into class_names, "other"
      last); one `theta_<name>` field per class carries the full posterior θ̂.
  voxel-map snapshot (`voxel_map_topic`) — full store as a DirichletVoxelMap
      (n_classes, class_names, row-major-flattened soft counts w, weight), at
      `snapshot_rate` (kept for inspection/logging).
  eq-24 telemetry (`telemetry_topic`) — one Eq24Telemetry per processed scan:
      per-factor mean log-likelihood matrix (which factor pulls which class),
      mean responsibility share r̄_c, and the hard-argmax share per class.
"""

import os

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from ament_index_python.packages import get_package_share_directory

from sonar_map_msgs.msg import ClassifiedReturns, DirichletVoxelMap, Eq24Telemetry
from sonar_map.dirichlet import geometry, likelihood
from sonar_map.dirichlet.classes import load_classes


# ── 21-bit-per-axis voxel hashing ─────────────────────────────────────────────
_BITS = 21
_MASK = (1 << _BITS) - 1
_SIGN = 1 << (_BITS - 1)
_SPAN = 1 << _BITS


def _pack(ix: int, iy: int, iz: int) -> int:
    """Pack signed voxel indices into a single int64-range key."""
    return ((ix & _MASK) << (2 * _BITS)) | ((iy & _MASK) << _BITS) | (iz & _MASK)


def _unpack(keys: np.ndarray):
    """Vectorised inverse of _pack — returns signed (ix, iy, iz) int arrays."""
    iz = (keys & _MASK).astype(np.int64)
    iy = ((keys >> _BITS) & _MASK).astype(np.int64)
    ix = ((keys >> (2 * _BITS)) & _MASK).astype(np.int64)
    iz = np.where(iz >= _SIGN, iz - _SPAN, iz)
    iy = np.where(iy >= _SIGN, iy - _SPAN, iy)
    ix = np.where(ix >= _SIGN, ix - _SPAN, ix)
    return ix, iy, iz


# ── Defaults — overridable from the YAML config / launch arguments ────────────
INPUT_TOPIC         = 'classified_returns'
VOXEL_TOPIC         = 'dirichlet_voxels'
VOXEL_MAP_TOPIC     = 'dirichlet_voxel_map'
TELEMETRY_TOPIC     = 'eq24_telemetry'
MAP_FRAME           = 'odom'
# '' resolves to the INSTALLED share copy of classes.yaml (edits to the src
# copy need a colcon build to take effect, same as every other config file).
CLASSES_FILE        = ''

R_V                 = 0.20      # [m]  voxel resolution
ALPHA_0             = 0.01      # [-]  symmetric Dirichlet prior α (<< 1)
K_ELEV              = 0.0       # [-]  elevation-band factor (Δz = k_elev·slant)
EPS                 = 0.5       # [m]  ε — spatial-prior neighbourhood radius
ACTIVE_PUBLISH_RATE = 5.0       # [Hz] active voxel cloud republish rate
SNAPSHOT_RATE       = 1.0       # [Hz] voxel-map snapshot rate
W_REF               = 10.0      # [-]  tanh scale for the visualisation confidence
W_DISPLAY_MIN       = 1.0       # [-]  display-only lower bound on W


class Voxel:
    """One persistent Dirichlet voxel: soft counts m_v and total evidence W.

    last_i is a reference-only annotation: the intensity of the most recent
    sonar return that hit this voxel (not used in the posterior)."""

    __slots__ = ('w', 'W', 'last_i')

    def __init__(self, n_classes: int):
        self.w = np.zeros(n_classes, dtype=np.float64)   # m_v, class axis = cs.names
        self.W = 0.0                                     # M_v = Σ_c m_{v,c}
        self.last_i = 0.0                                # last hit intensity (display only)


class VoxelMapperNode(Node):

    def __init__(self):
        super().__init__('voxel_mapper')

        self.declare_parameter('input_topic',         INPUT_TOPIC)
        self.declare_parameter('voxel_topic',         VOXEL_TOPIC)
        self.declare_parameter('voxel_map_topic',     VOXEL_MAP_TOPIC)
        self.declare_parameter('telemetry_topic',     TELEMETRY_TOPIC)
        self.declare_parameter('map_frame',           MAP_FRAME)
        self.declare_parameter('classes_file',        CLASSES_FILE)
        self.declare_parameter('r_v',                 R_V)
        self.declare_parameter('alpha_0',             ALPHA_0)
        self.declare_parameter('k_elev',              K_ELEV)
        self.declare_parameter('eps',                 EPS)
        self.declare_parameter('active_publish_rate', ACTIVE_PUBLISH_RATE)
        self.declare_parameter('snapshot_rate',       SNAPSHOT_RATE)
        self.declare_parameter('w_ref',               W_REF)
        self.declare_parameter('w_display_min',       W_DISPLAY_MIN)

        def gp(n):
            return self.get_parameter(n).value

        # ── The fixed class inventory (classes.yaml — Table I, "other" last) ──
        classes_file = str(gp('classes_file')).strip()
        if not classes_file:
            classes_file = os.path.join(
                get_package_share_directory('sonar_map'), 'config', 'classes.yaml')
        self._cs        = load_classes(classes_file)
        self._n_classes = self._cs.n_classes            # K+1, "other" = column K

        self._map_frame   = str(gp('map_frame'))
        self._r_v         = float(gp('r_v'))
        self._inv_r_v     = 1.0 / self._r_v
        self._a0          = float(gp('alpha_0'))
        self._Ka0         = self._n_classes * self._a0
        self._k_elev      = max(float(gp('k_elev')), 0.0)
        # Spatial-prior neighbourhood radius ε (metres). eps <= 0 DISABLES the
        # neighbour prior: π_c becomes the symmetric-α_0 uniform prior (1/|C|),
        # so the responsibility reduces to the normalised likelihood r = L/ΣL.
        # This skips both the ε-ball query and the (t-1) snapshot, and lets the
        # whole per-scan update run vectorised — by far the cheapest mode.
        self._eps         = max(float(gp('eps')), 0.0)
        self._use_prior   = self._eps > 0.0
        self._w_ref       = max(float(gp('w_ref')), 1e-6)
        self._w_disp_min  = float(gp('w_display_min'))

        self._uniform     = np.full(self._n_classes, 1.0 / self._n_classes,
                                    dtype=np.float64)
        self._log_uniform = np.log(self._uniform)

        # Precompute the ε-ball neighbour offsets (a true Euclidean ball, to
        # match the paper's "voxels within radius ε of v"). Includes (0,0,0): a
        # voxel's own previous-sweep belief is a legitimate prior, and the (t-1)
        # freeze means it introduces no within-sweep feedback.
        R = int(np.ceil(self._eps * self._inv_r_v))
        eps2 = self._eps * self._eps
        self._nbr_offsets = [
            (dx, dy, dz)
            for dx in range(-R, R + 1)
            for dy in range(-R, R + 1)
            for dz in range(-R, R + 1)
            if (dx * dx + dy * dy + dz * dz) * self._r_v * self._r_v <= eps2
        ]

        # ── Persistent Dirichlet store + frozen (t-1) prior snapshot ──────────
        self._store: dict = {}        # key -> Voxel
        self._prior_snap: dict = {}   # key -> (theta_hat (K+1,), M)  frozen at t-1

        # Per-class posteriors θ̂_c carried as one float32 `theta_<name>` field
        # per class, built dynamically from the class inventory. `intensity` is
        # reference-only: the last sonar return that hit the voxel (does NOT
        # affect the posterior or the semantic label).
        field_names = (['x', 'y', 'z', 'confidence', 'weight', 'semantic',
                        'intensity']
                       + [f'theta_{name}' for name in self._cs.names])
        self._cloud_fields = [
            PointField(name=nm, offset=4 * i,
                       datatype=PointField.FLOAT32, count=1)
            for i, nm in enumerate(field_names)
        ]

        self._voxel_pub = self.create_publisher(
            PointCloud2, gp('voxel_topic'), 10)
        self._map_pub = self.create_publisher(
            DirichletVoxelMap, gp('voxel_map_topic'), 10)
        self._telemetry_pub = self.create_publisher(
            Eq24Telemetry, gp('telemetry_topic'), 10)
        self._sub = self.create_subscription(
            ClassifiedReturns, gp('input_topic'), self._returns_cb, 10)

        cloud_rate = max(float(gp('active_publish_rate')), 1e-3)
        snap_rate  = max(float(gp('snapshot_rate')), 1e-3)
        self._cloud_timer = self.create_timer(1.0 / cloud_rate, self._publish_active)
        self._snap_timer  = self.create_timer(1.0 / snap_rate, self._publish_snapshot)

        enabled = [f for f in likelihood.FACTOR_NAMES
                   if self._cs.factors_enabled.get(f, True)]
        self._scan_count_ = 0
        self.get_logger().info(
            f'voxel_mapper: {gp("input_topic")} -> {gp("voxel_topic")} (cloud) '
            f'+ {gp("voxel_map_topic")} (snapshot @ {snap_rate}Hz) '
            f'+ {gp("telemetry_topic")} (eq-24 telemetry) | '
            f'PERSISTENT {self._n_classes}-class Dirichlet (eq 24: L·π) | '
            f'classes={list(self._cs.names)} factors={enabled} '
            f'({classes_file}) | r_v={self._r_v}m alpha_0={self._a0} '
            f'spatial_prior={"ON eps=%.2fm (%d nbr voxels)" % (self._eps, len(self._nbr_offsets)) if self._use_prior else "OFF (uniform alpha_0 prior, vectorised)"} '
            f'k_elev={self._k_elev}')

    # ── (t-1) prior snapshot — frozen at the start of each sweep ──────────────

    def _build_prior_snapshot(self):
        """Freeze the per-voxel posterior mean θ̂ and evidence M for the spatial
        prior, so the whole sweep reads a consistent (t-1) neighbourhood."""
        a0, Ka0 = self._a0, self._Ka0
        snap = {}
        for key, v in self._store.items():
            M = v.W
            snap[key] = ((a0 + v.w) / (Ka0 + M), M)
        self._prior_snap = snap

    def _spatial_prior(self, ix, iy, iz):
        """π_c(N_v) (eq 23): evidence-weighted mean of neighbour posteriors over
        the ε-ball, read from the frozen (t-1) snapshot. Uniform fallback when
        the neighbourhood carries no evidence."""
        snap = self._prior_snap
        num   = np.zeros(self._n_classes, dtype=np.float64)
        denom = 0.0
        for dx, dy, dz in self._nbr_offsets:
            e = snap.get(_pack(ix + dx, iy + dy, iz + dz))
            if e is None:
                continue
            theta, M = e
            num   += M * theta
            denom += M
        if denom <= 0.0:
            return self._uniform
        return num / denom

    # ── voxel evidence accumulation (persistent — never retired) ──────────────

    def _add(self, key, dvec, last_i=None):
        """Add a length-(K+1) responsibility-mass vector to one voxel. Optionally
        record the intensity of the hitting return (display-only annotation)."""
        v = self._store.get(key)
        if v is None:
            v = Voxel(self._n_classes)
            self._store[key] = v
        v.w += dvec
        v.W += float(dvec.sum())
        if last_i is not None:
            v.last_i = float(last_i)

    def _column_iz(self, z_lo, z_hi):
        """Voxel iz indices spanning [z_lo, z_hi]."""
        iz0 = int(np.floor(z_lo * self._inv_r_v))
        iz1 = int(np.floor(z_hi * self._inv_r_v))
        if iz1 < iz0:
            iz0, iz1 = iz1, iz0
        return range(iz0, iz1 + 1)

    def _deposit(self, r, ix_arr, iy_arr, iz_arr, z, slant, inten):
        """Accumulate the per-return responsibility mass into the store.

        k_elev = 0 (single voxel): fully vectorised — returns are grouped by
        voxel key and summed once per distinct voxel. k_elev > 0: per-return
        elevation-band spread (top-hat). ``inten`` records the last hitting
        intensity per voxel (display-only)."""
        if self._k_elev <= 0.0:
            idx = np.stack([ix_arr, iy_arr, iz_arr], axis=1)         # (N, 3)
            uniq, inv = np.unique(idx, axis=0, return_inverse=True)
            inv = inv.ravel()
            sums = np.zeros((uniq.shape[0], self._n_classes), dtype=np.float64)
            np.add.at(sums, inv, r)
            # Last hitting intensity per unique voxel (last write wins).
            last_i = np.zeros(uniq.shape[0], dtype=np.float64)
            last_i[inv] = inten
            for j in range(uniq.shape[0]):
                self._add(_pack(int(uniq[j, 0]), int(uniq[j, 1]), int(uniq[j, 2])),
                          sums[j], last_i=last_i[j])
        else:
            n = r.shape[0]
            for i in range(n):
                delta_z = self._k_elev * slant[i]
                izs = list(self._column_iz(z[i] - 0.5 * delta_z,
                                           z[i] + 0.5 * delta_z))
                dvec = r[i] * (1.0 / len(izs))
                for izc in izs:
                    self._add(_pack(int(ix_arr[i]), int(iy_arr[i]), izc), dvec,
                              last_i=inten[i])

    # ── per-scan processing ───────────────────────────────────────────────────

    def _returns_cb(self, msg: ClassifiedReturns):
        n = len(msg.intensity)
        if n == 0:
            return

        x      = np.asarray(msg.x,            dtype=np.float64)
        y      = np.asarray(msg.y,            dtype=np.float64)
        z      = np.asarray(msg.z,            dtype=np.float64)
        inten  = np.asarray(msg.intensity,    dtype=np.float64)
        slant  = np.asarray(msg.slant_range,  dtype=np.float64)
        z_bed  = np.asarray(msg.z_bed,        dtype=np.float64)

        # ── eq-24 features: geometry (eqs 25, 29–30) + raw channels ───────────
        # NaN channels flow through — the factors ABSTAIN on those returns
        # (bottom grid warming up, no LiDAR cloud yet, no shape extractor).
        rho = geometry.height_variance(slant, self._cs)         # ϱ_i  (eq 30)
        h   = geometry.height_above_bed(z, z_bed)               # h_i  (eq 25)
        feats = likelihood.Features(
            h=h,
            rho=rho,
            intensity=inten,
            over_count=np.asarray(msg.over_count,      dtype=np.float64),
            over_gap=np.asarray(msg.over_gap,          dtype=np.float64),
            shape_flag=np.asarray(msg.shape_flag,      dtype=np.float64),
            region_size=np.asarray(msg.region_size,    dtype=np.float64),
            region_logarea=np.asarray(msg.region_logarea,   dtype=np.float64),
            region_solidity=np.asarray(msg.region_solidity, dtype=np.float64),
            region_shadow=np.asarray(msg.region_shadow,     dtype=np.float64),
            region_texture=np.asarray(msg.region_texture,   dtype=np.float64),
        )
        log_L, by_factor = likelihood.log_likelihood(feats, self._cs)  # (N, K+1)

        ix_arr = np.floor(x * self._inv_r_v).astype(np.int64)
        iy_arr = np.floor(y * self._inv_r_v).astype(np.int64)
        iz_arr = np.floor(z * self._inv_r_v).astype(np.int64)

        if not self._use_prior:
            # ── Uniform symmetric-α_0 prior: r = softmax(log L) (vectorised) ──
            r = likelihood.responsibilities(log_L, self._log_uniform)
            self._deposit(r, ix_arr, iy_arr, iz_arr, z, slant, inten)
        else:
            # ── Paper spatial-prior path (eq 16/23): per-return, (t-1) frozen ─
            self._build_prior_snapshot()
            r = np.empty((n, self._n_classes), dtype=np.float64)
            for i in range(n):
                ix, iy, iz = int(ix_arr[i]), int(iy_arr[i]), int(iz_arr[i])
                # π > 0 always (α_0 > 0 floors θ̂; uniform fallback) — log is safe.
                pi = self._spatial_prior(ix, iy, iz)
                ri = likelihood.responsibilities(log_L[i:i + 1], np.log(pi))[0]
                r[i] = ri
                delta_z = self._k_elev * slant[i]
                izs = list(self._column_iz(z[i] - 0.5 * delta_z,
                                           z[i] + 0.5 * delta_z))
                dvec = ri * (1.0 / len(izs))
                for izc in izs:
                    self._add(_pack(ix, iy, izc), dvec, last_i=inten[i])

        self._publish_telemetry(msg.header.stamp, n, by_factor, r)

        self._scan_count_ += 1
        if self._scan_count_ <= 5 or self._scan_count_ % 50 == 0:
            share = ' '.join(f'{nm}={s:.2f}'
                             for nm, s in zip(self._cs.names, r.mean(axis=0)))
            self.get_logger().info(
                f'scan #{self._scan_count_}: {n} returns | '
                f'active={len(self._store)} voxels (persistent) | r̄: {share}')

    # ── eq-24 telemetry (one message per processed scan) ──────────────────────

    def _publish_telemetry(self, stamp, n, by_factor, r):
        """Which factor drives which class this scan: per-factor mean log-lik
        matrix (factor-major flatten), mean responsibility share, and the
        hard-argmax share. Disabled factors keep their all-zero row so the
        indexing stays stable across configurations."""
        msg = Eq24Telemetry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._map_frame
        msg.n_returns    = int(n)
        msg.n_classes    = int(self._n_classes)
        msg.class_names  = list(self._cs.names)
        msg.factor_names = list(likelihood.FACTOR_NAMES)

        mean_ll = np.concatenate(
            [by_factor[f].mean(axis=0) for f in likelihood.FACTOR_NAMES])
        msg.mean_log_lik = mean_ll.astype(np.float32).tolist()
        msg.resp_share   = r.mean(axis=0).astype(np.float32).tolist()
        counts = np.bincount(np.argmax(r, axis=1), minlength=self._n_classes)
        msg.map_share    = (counts / float(n)).astype(np.float32).tolist()
        self._telemetry_pub.publish(msg)

    # ── voxel-map snapshot (full store, for inspection/logging) ───────────────

    def _publish_snapshot(self):
        msg = DirichletVoxelMap()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.n_classes = int(self._n_classes)
        msg.class_names = list(self._cs.names)

        n = len(self._store)
        keys = np.empty(n, dtype=np.int64)
        wmat = np.empty((n, self._n_classes), dtype=np.float32)
        Ws   = np.empty(n,                    dtype=np.float32)
        for i, (k, v) in enumerate(self._store.items()):
            keys[i] = k
            wmat[i] = v.w
            Ws[i]   = v.W

        msg.voxel_key = keys.tolist()
        msg.w         = wmat.reshape(-1).tolist()   # row-major (N × n_classes)
        msg.weight    = Ws.tolist()
        self._map_pub.publish(msg)

    # ── active voxel cloud (visualisation) ────────────────────────────────────

    def _publish_active(self):
        items = list(self._store.items())
        if not items:
            return

        keys = np.fromiter((k for k, _ in items), dtype=np.int64, count=len(items))
        wmat = np.array([v.w for _, v in items], dtype=np.float64)        # (N, K+1)
        Ws   = np.array([v.W for _, v in items], dtype=np.float64)        # (N,)
        last_i = np.array([v.last_i for _, v in items], dtype=np.float64) # (N,)

        # Posterior mean θ̂_c = (α_0 + m_c) / (|C|·α_0 + M)  (eq 21).
        theta = (self._a0 + wmat) / (self._Ka0 + Ws)[:, None]        # (N, K+1)
        sem   = np.argmax(theta, axis=1)                             # class id
        pmax  = theta[np.arange(len(keys)), sem]
        conf  = pmax * np.tanh(Ws / self._w_ref)

        keep = Ws >= self._w_disp_min
        if not np.any(keep):
            return

        ix, iy, iz = _unpack(keys[keep])
        out = np.empty((int(keep.sum()), 7 + self._n_classes), dtype=np.float32)
        out[:, 0] = (ix.astype(np.float64) + 0.5) * self._r_v
        out[:, 1] = (iy.astype(np.float64) + 0.5) * self._r_v
        out[:, 2] = (iz.astype(np.float64) + 0.5) * self._r_v
        out[:, 3] = conf[keep].astype(np.float32)
        out[:, 4] = Ws[keep].astype(np.float32)
        out[:, 5] = sem[keep].astype(np.float32)
        out[:, 6] = last_i[keep].astype(np.float32)      # last hit intensity
        out[:, 7:] = theta[keep].astype(np.float32)      # theta_<name> per class

        header = Header()
        header.frame_id = self._map_frame
        header.stamp = self.get_clock().now().to_msg()
        self._voxel_pub.publish(
            pc2.create_cloud(header, self._cloud_fields, out))


def main(args=None):
    rclpy.init(args=args)
    node = VoxelMapperNode()
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
