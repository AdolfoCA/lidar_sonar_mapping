#!/usr/bin/env python3
"""
voxel_mapper — Node 3 of the Dirichlet voxel mapping pipeline.
===============================================================================

Maintains the central Dirichlet-Categorical voxel map over the four semantic
classes FREE, SEABED, OBJECT, STRUCTURE. This is a TRANSIENT working layer:
voxels accumulate evidence and are periodically swept out by seabed_surface.

PROMOTION OWNERSHIP
  voxel_mapper does NOT decide promotions. It publishes the full active voxel
  store as a snapshot (`voxel_map_topic`); seabed_surface sweeps that snapshot
  every few seconds, applies the promotion-readiness check, and sends the keys
  of the voxels it promoted back on `promoted_keys_topic`. voxel_mapper then
  removes those voxels from the active store and retires their locations —
  returns that later fall in a retired voxel are dropped (promotion is
  irreversible).

EVIDENCE PER CLASSIFIED RETURN  (soft Dirichlet update)
  Every return is spread uniformly across its elevation-uncertainty band
  z +/- 0.5 * k_elev * slant_range (top-hat, 1/N per band voxel). At close
  range or k_elev = 0 the band collapses to a single voxel.

  The total mass per return is 1.0 (unit). For non-FREE returns the mass is
  split between (SEABED, OBJECT, STRUCTURE) by the soft responsibility
  vector:

      u     = max(I - mu_i, 0) / mu_i           intensity excess
      r_sb  = exp(-alpha_i * u)                  SEABED
      r_obj = (1 - r_sb) * exp(-alpha_l * v)     OBJECT
      r_str = (1 - r_sb) * (1 - exp(-alpha_l * v))   STRUCTURE

  with `v` = the geometric LiDAR-support score computed in return_classifier.
  Sum = 1. Hierarchical: intensity decides "is this seabed?"; LiDAR decides
  "object or structure within not-seabed". Smooth exponential decays in
  both gates.

  FREE returns (I = 0) deposit unit mass into w_free only (hard branch).

  The return's z-coordinate is appended to the ring buffer of the voxel that
  contains it (for every non-FREE class).

TOPICS
  active voxel cloud (`voxel_topic`)   — PointCloud2 for visualisation, at
      `active_publish_rate`. Confidence = max_c pi_c * tanh(W / W_ref); `weight`
      carries raw W; a display-only floor (W >= W_display_min) is applied.
  voxel-map snapshot (`voxel_map_topic`) — full store as a DirichletVoxelMap,
      at `snapshot_rate`, consumed by seabed_surface.
  promotion feedback (`promoted_keys_topic`) — subscribed; PromotedVoxelKeys
      from seabed_surface, telling this node which voxels to retire.
"""

from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

from sonar_map_msgs.msg import ClassifiedReturns, DirichletVoxelMap, PromotedVoxelKeys


# ── Class indices into the Dirichlet weight vector w = [free, sb, obj, str] ───
FREE      = 0
SEABED    = 1
OBJECT    = 2
STRUCTURE = 3

# ── 21-bit-per-axis voxel hashing (matches sonar_map_ned_semantic) ────────────
_BITS = 21
_MASK = (1 << _BITS) - 1
_SIGN = 1 << (_BITS - 1)
_SPAN = 1 << _BITS


def _pack(ix: int, iy: int, iz: int) -> int:
    """Pack signed voxel indices into a single int64-range key."""
    return ((ix & _MASK) << (2 * _BITS)) | ((iy & _MASK) << _BITS) | (iz & _MASK)


def _unpack_one(key: int):
    """Inverse of _pack for a single key — returns signed (ix, iy, iz)."""
    iz = key & _MASK
    iy = (key >> _BITS) & _MASK
    ix = (key >> (2 * _BITS)) & _MASK
    if iz >= _SIGN:
        iz -= _SPAN
    if iy >= _SIGN:
        iy -= _SPAN
    if ix >= _SIGN:
        ix -= _SPAN
    return ix, iy, iz


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
PROMOTED_KEYS_TOPIC = 'promoted_voxel_keys'
MAP_FRAME           = 'odom'

R_V                 = 0.20      # [m]  voxel resolution
RING_BUFFER_LEN     = 10        # [-]  recent return depths kept per voxel
ALPHA_0             = 0.01      # [-]  symmetric Dirichlet prior (<< 1)
K_ELEV              = 0.3       # [-]  elevation-uncertainty factor (Delta_z = k_elev * slant)
ALPHA_I             = 1.5       # [-]  intensity-gate decay rate in the soft responsibility
BETA_STRUCT_PRIOR   = 3.0       # [-]  per-voxel LiDAR-derived STRUCTURE prior strength
SIGMA_SIM           = 0.3       # [-]  intensity similarity tolerance (fraction of mu_str_local)
TAU_SIM             = 0.5       # [-]  d_factor threshold for "structure-plausible" voxel
R_SIM_VOXELS        = 2         # [-]  neighbour search radius in voxels (cube side = 2R+1)
N_MIN_NEIGHBORS     = 3         # [-]  min structure-neighbours with history before similarity kicks in
F_L_SIM_GATE        = 0.01      # [-]  skip similarity query when d_factor*count_factor below this
U_OBJ_THRESH        = 0.5       # [-]  u must exceed this before the "very bright = OBJECT" boost engages
ALPHA_OBJ           = 1.5       # [-]  OBJECT-boost saturation rate: f_obj_boost = 1 - exp(-alpha_obj * (u - u_obj_thresh)+)
ACTIVE_PUBLISH_RATE = 5.0       # [Hz] active voxel cloud republish rate
SNAPSHOT_RATE       = 1.0       # [Hz] voxel-map snapshot rate (for seabed_surface)
W_REF               = 10.0      # [-]  tanh scale for the visualisation confidence
W_DISPLAY_MIN       = 1.0       # [-]  display-only lower bound on W


class Voxel:
    """One active Dirichlet voxel."""

    __slots__ = ('w', 'W', 'ring', 'sum_r', 'n_r',
                 'intensity_mean', 'n_obs',
                 'd_factor_cached', 'alpha_str_prior')

    def __init__(self, ring_len: int):
        self.w = np.zeros(4, dtype=np.float64)   # [free, sb, obj, str]
        self.W = 0.0                              # total accumulated weight
        self.ring = deque(maxlen=ring_len)        # recent return z-coords
        self.sum_r = 0.0                          # sum of contributing slant ranges
        self.n_r = 0                              # count of slant-range samples
        # NEW — per-voxel intensity history for the similarity gate
        self.intensity_mean = 0.0                 # running mean of observed intensities
        self.n_obs = 0                            # # observations contributing to the mean
        # NEW — LiDAR-derived spatial prior (set on creation, static thereafter)
        self.d_factor_cached = 0.0                # d_factor at voxel creation
        self.alpha_str_prior = 0.0                # beta_struct * d_factor_cached


class VoxelMapperNode(Node):

    def __init__(self):
        super().__init__('voxel_mapper')

        self.declare_parameter('input_topic',         INPUT_TOPIC)
        self.declare_parameter('voxel_topic',         VOXEL_TOPIC)
        self.declare_parameter('voxel_map_topic',     VOXEL_MAP_TOPIC)
        self.declare_parameter('promoted_keys_topic', PROMOTED_KEYS_TOPIC)
        self.declare_parameter('map_frame',           MAP_FRAME)
        self.declare_parameter('r_v',                 R_V)
        self.declare_parameter('ring_buffer_len',     RING_BUFFER_LEN)
        self.declare_parameter('alpha_0',             ALPHA_0)
        self.declare_parameter('k_elev',              K_ELEV)
        self.declare_parameter('alpha_i',             ALPHA_I)
        self.declare_parameter('beta_struct_prior',   BETA_STRUCT_PRIOR)
        self.declare_parameter('sigma_sim',           SIGMA_SIM)
        self.declare_parameter('tau_sim',             TAU_SIM)
        self.declare_parameter('r_sim_voxels',        R_SIM_VOXELS)
        self.declare_parameter('n_min_neighbors',     N_MIN_NEIGHBORS)
        self.declare_parameter('u_obj_thresh',        U_OBJ_THRESH)
        self.declare_parameter('alpha_obj',           ALPHA_OBJ)
        self.declare_parameter('active_publish_rate', ACTIVE_PUBLISH_RATE)
        self.declare_parameter('snapshot_rate',       SNAPSHOT_RATE)
        self.declare_parameter('w_ref',               W_REF)
        self.declare_parameter('w_display_min',       W_DISPLAY_MIN)

        def gp(n):
            return self.get_parameter(n).value

        self._map_frame   = str(gp('map_frame'))
        self._r_v         = float(gp('r_v'))
        self._inv_r_v     = 1.0 / self._r_v
        self._ring_len    = max(1, int(gp('ring_buffer_len')))
        self._a0          = float(gp('alpha_0'))
        self._four_a0     = 4.0 * self._a0
        # Elevation-angle uncertainty: each return's unit mass is spread
        # uniformly over a vertical band of width k_elev * slant_range. k_elev=0
        # disables the spread and reverts to the single-voxel update.
        self._k_elev      = max(float(gp('k_elev')), 0.0)
        # Soft-responsibility gate decay rates (intensity only — LiDAR factors
        # come from the message as d_factor and count_factor).
        self._alpha_i     = max(float(gp('alpha_i')), 0.0)
        # LiDAR-derived STRUCTURE prior + intensity-similarity gate.
        self._beta_struct = max(float(gp('beta_struct_prior')), 0.0)
        self._sigma_sim   = max(float(gp('sigma_sim')), 1e-6)
        self._tau_sim     = float(gp('tau_sim'))
        self._r_sim_vox   = max(0, int(gp('r_sim_voxels')))
        self._n_min_nbr   = max(1, int(gp('n_min_neighbors')))
        # "Very bright -> OBJECT" boost: when u > u_obj_thresh the boost
        # transfers part of the LiDAR-supported (STRUCTURE) mass into OBJECT.
        self._u_obj_th    = max(float(gp('u_obj_thresh')), 0.0)
        self._alpha_obj   = max(float(gp('alpha_obj')), 0.0)
        self._w_ref       = max(float(gp('w_ref')), 1e-6)
        self._w_disp_min  = float(gp('w_display_min'))

        # Precomputed (dx, dy, dz) offsets for the structure-neighbour cube
        # — avoids the triple-loop overhead inside the hot path.
        R = self._r_sim_vox
        self._nbr_offsets = [
            (dx, dy, dz)
            for dx in range(-R, R + 1)
            for dy in range(-R, R + 1)
            for dz in range(-R, R + 1)
        ]

        # ── The active Dirichlet store and the set of retired voxel keys ──────
        self._store: dict = {}        # key -> Voxel
        self._promoted: set = set()   # keys retired by seabed_surface (irreversible)

        # Per-class posteriors pi_c are carried as float32 in [0, 1] (e.g.
        # 0.87 == 87 %). Colour the cloud by any of them in rviz to read each
        # voxel's per-class probability directly.
        self._cloud_fields = [
            PointField(name='x',              offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',              offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',              offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='confidence',     offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='weight',         offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='semantic',       offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='pi_free',        offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name='pi_sb',          offset=28, datatype=PointField.FLOAT32, count=1),
            PointField(name='pi_obj',         offset=32, datatype=PointField.FLOAT32, count=1),
            PointField(name='pi_str',         offset=36, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity_mean', offset=40, datatype=PointField.FLOAT32, count=1),
        ]

        self._voxel_pub = self.create_publisher(
            PointCloud2, gp('voxel_topic'), 10)
        self._map_pub = self.create_publisher(
            DirichletVoxelMap, gp('voxel_map_topic'), 10)
        self._sub = self.create_subscription(
            ClassifiedReturns, gp('input_topic'), self._returns_cb, 10)
        self._keys_sub = self.create_subscription(
            PromotedVoxelKeys, gp('promoted_keys_topic'), self._promoted_keys_cb, 10)

        cloud_rate = max(float(gp('active_publish_rate')), 1e-3)
        snap_rate  = max(float(gp('snapshot_rate')), 1e-3)
        self._cloud_timer = self.create_timer(1.0 / cloud_rate, self._publish_active)
        self._snap_timer  = self.create_timer(1.0 / snap_rate, self._publish_snapshot)

        self._scan_count_ = 0
        self._n_retired = 0
        self.get_logger().info(
            f'voxel_mapper: {gp("input_topic")} -> {gp("voxel_topic")} (cloud) '
            f'+ {gp("voxel_map_topic")} (snapshot @ {snap_rate}Hz) | '
            f'feedback <- {gp("promoted_keys_topic")} | r_v={self._r_v}m '
            f'alpha_0={self._a0} k_elev={self._k_elev} alpha_i={self._alpha_i} '
            f'beta_struct={self._beta_struct} sigma_sim={self._sigma_sim} '
            f'tau_sim={self._tau_sim} R_sim={self._r_sim_vox} '
            f'u_obj_th={self._u_obj_th} alpha_obj={self._alpha_obj} '
            f'ring={self._ring_len} cloud@{cloud_rate}Hz')

    # ── voxel evidence update (no promotion — that lives in seabed_surface) ───

    def _apply(self, key, cls_idx, dweight, z_ring=None, slant=None):
        """Add evidence to one voxel. Retired (promoted) voxels are skipped."""
        if dweight <= 0.0 or key in self._promoted:
            return
        v = self._store.get(key)
        if v is None:
            v = Voxel(self._ring_len)
            self._store[key] = v
        v.w[cls_idx] += dweight
        v.W += dweight
        if z_ring is not None:
            v.ring.append(float(z_ring))
        if slant is not None:
            v.sum_r += float(slant)
            v.n_r += 1

    def _apply_multi(self, key, dw_vec, d_factor_for_create=0.0,
                     z_ring=None, slant=None, intensity_for_mean=None):
        """Add a length-4 evidence vector to one voxel in a single op.

        Faster than three sequential ``_apply`` calls when the per-return
        update is a soft distribution over classes — one dict lookup, one
        numpy in-place add, one scalar ``W`` bump.

        On voxel creation, captures ``d_factor_for_create`` into the voxel's
        static LiDAR-derived prior (``alpha_str_prior = beta_struct * d_factor``).
        If ``intensity_for_mean`` is provided, updates the running
        ``intensity_mean`` (used downstream by the similarity gate).
        """
        if key in self._promoted:
            return
        total = float(dw_vec.sum())
        if total <= 0.0:
            return
        v = self._store.get(key)
        if v is None:
            v = Voxel(self._ring_len)
            v.d_factor_cached = float(d_factor_for_create)
            v.alpha_str_prior = self._beta_struct * float(d_factor_for_create)
            self._store[key] = v
        v.w += dw_vec
        v.W += total
        if z_ring is not None:
            v.ring.append(float(z_ring))
        if slant is not None:
            v.sum_r += float(slant)
            v.n_r += 1
        if intensity_for_mean is not None:
            # Welford-style running mean.
            v.intensity_mean += (float(intensity_for_mean) - v.intensity_mean) / (v.n_obs + 1)
            v.n_obs += 1

    def _promoted_keys_cb(self, msg: PromotedVoxelKeys):
        """seabed_surface has promoted these voxels — retire them."""
        for k in msg.keys:
            k = int(k)
            self._promoted.add(k)
            if self._store.pop(k, None) is not None:
                self._n_retired += 1

    # ── structure-neighbour helper (for the intensity-similarity gate) ────────

    def _structure_neighbour_mean_intensity(self, ix, iy, iz):
        """Mean of ``intensity_mean`` over structure-plausible voxels in a
        cube of side ``2*r_sim_voxels + 1`` around ``(ix, iy, iz)``.

        "Structure-plausible" means the neighbour has been observed at least
        once and was created with ``d_factor_cached >= tau_sim`` (i.e. it's
        right at a LiDAR cluster). Returns ``None`` when fewer than
        ``n_min_neighbors`` qualify — the caller treats that as the bootstrap
        case and sets ``similarity = 1`` so the geometric gates alone decide.
        """
        store     = self._store
        tau       = self._tau_sim
        n_count   = 0
        i_sum     = 0.0
        for dx, dy, dz in self._nbr_offsets:
            v_n = store.get(_pack(ix + dx, iy + dy, iz + dz))
            if v_n is None or v_n.n_obs == 0 or v_n.d_factor_cached < tau:
                continue
            n_count += 1
            i_sum   += v_n.intensity_mean
        if n_count < self._n_min_nbr:
            return None
        return i_sum / n_count

    # ── column helpers ────────────────────────────────────────────────────────

    def _column(self, z_lo, z_hi):
        """Voxel iz indices and centre z-values spanning [z_lo, z_hi]."""
        iz0 = int(np.floor(z_lo * self._inv_r_v))
        iz1 = int(np.floor(z_hi * self._inv_r_v))
        if iz1 < iz0:
            iz0, iz1 = iz1, iz0
        izs = np.arange(iz0, iz1 + 1, dtype=np.int64)
        centres = (izs.astype(np.float64) + 0.5) * self._r_v
        return izs, centres

    # ── per-scan processing ───────────────────────────────────────────────────

    def _returns_cb(self, msg: ClassifiedReturns):
        n = len(msg.label)
        if n == 0:
            return

        x       = np.asarray(msg.x,             dtype=np.float64)
        y       = np.asarray(msg.y,             dtype=np.float64)
        z       = np.asarray(msg.z,             dtype=np.float64)
        inten   = np.asarray(msg.intensity,     dtype=np.float64)
        slant   = np.asarray(msg.slant_range,   dtype=np.float64)
        d_fac   = np.asarray(msg.d_factor,      dtype=np.float64)
        c_fac   = np.asarray(msg.count_factor,  dtype=np.float64)
        label   = np.asarray(msg.label,         dtype=np.uint8)

        # ── Vectorised pre-computation (the bits that don't depend on
        # the structure-neighbour query). r_sb is the SEABED responsibility,
        # which only depends on intensity excess u.
        mu_i = max(float(msg.mu_i), 1e-9)
        u    = np.maximum(inten - mu_i, 0.0) / mu_i
        r_sb = np.exp(-self._alpha_i * u)               # exp(-alpha_i * u)
        f_L_geom = d_fac * c_fac                         # geometric STRUCTURE gate

        # "Very bright -> OBJECT" boost: transfers part of the LiDAR-supported
        # STRUCTURE share into OBJECT once u exceeds u_obj_thresh.
        u_obj_exc   = np.maximum(u - self._u_obj_th, 0.0)
        f_obj_boost = 1.0 - np.exp(-self._alpha_obj * u_obj_exc)

        # Per-return loop. FREE returns are a hard branch (all mass to w_free);
        # non-FREE returns go through the soft responsibility split with the
        # intensity-similarity gate.
        dw_vec = np.zeros(4, dtype=np.float64)
        for i in range(n):
            lab = int(label[i])
            ix  = int(np.floor(x[i] * self._inv_r_v))
            iy  = int(np.floor(y[i] * self._inv_r_v))
            iz  = int(np.floor(z[i] * self._inv_r_v))

            delta_z = self._k_elev * slant[i]
            izs, _  = self._column(z[i] - 0.5 * delta_z,
                                   z[i] + 0.5 * delta_z)
            inv_N   = 1.0 / len(izs)

            if lab == FREE:
                dw_vec[0] = inv_N
                dw_vec[1] = 0.0
                dw_vec[2] = 0.0
                dw_vec[3] = 0.0
                # FREE doesn't update intensity_mean (zero-intensity).
                for izc in izs:
                    self._apply_multi(_pack(ix, iy, int(izc)),
                                      dw_vec,
                                      d_factor_for_create=d_fac[i])
                continue

            # ── Non-FREE: compute the intensity-similarity gate at the centre
            # voxel. The similarity multiplies into the STRUCTURE responsibility
            # and the complement goes to OBJECT — total per-return mass is 1.
            if f_L_geom[i] > F_L_SIM_GATE:
                mu_str_local = self._structure_neighbour_mean_intensity(ix, iy, iz)
                if mu_str_local is None or mu_str_local <= 0.0:
                    similarity = 1.0
                else:
                    sigma = self._sigma_sim * mu_str_local
                    similarity = float(np.exp(
                        -(inten[i] - mu_str_local) ** 2 / (2.0 * sigma * sigma)
                    ))
            else:
                # Far from any LiDAR: STRUCTURE is already negligible — skip
                # the neighbour query.
                similarity = 1.0

            f_L_eff = f_L_geom[i] * similarity            # final STRUCTURE gate
            r_sb_i   = r_sb[i]
            one_minus = 1.0 - r_sb_i
            # Apply the "very bright -> OBJECT" boost by stealing a fraction
            # f_obj_boost of the STRUCTURE mass for OBJECT. Sum still = 1.
            fob_i    = f_obj_boost[i]
            r_str_i  = one_minus * f_L_eff * (1.0 - fob_i)
            r_obj_i  = one_minus * ((1.0 - f_L_eff) + f_L_eff * fob_i)

            dw_vec[0] = 0.0
            dw_vec[1] = r_sb_i  * inv_N
            dw_vec[2] = r_obj_i * inv_N
            dw_vec[3] = r_str_i * inv_N

            for izc in izs:
                key = _pack(ix, iy, int(izc))
                if int(izc) == iz:
                    # Centre voxel: attach ring-buffer z, slant, and the
                    # intensity sample to update intensity_mean.
                    self._apply_multi(key, dw_vec,
                                      d_factor_for_create=d_fac[i],
                                      z_ring=z[i], slant=slant[i],
                                      intensity_for_mean=inten[i])
                else:
                    self._apply_multi(key, dw_vec,
                                      d_factor_for_create=d_fac[i])

        self._scan_count_ += 1
        if self._scan_count_ <= 5 or self._scan_count_ % 50 == 0:
            self.get_logger().info(
                f'scan #{self._scan_count_}: {n} returns | '
                f'active={len(self._store)} voxels | retired(total)={self._n_retired}')

    # ── voxel-map snapshot (for seabed_surface to sweep) ──────────────────────

    def _publish_snapshot(self):
        msg = DirichletVoxelMap()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        items = self._store.items()
        n = len(self._store)

        # Pre-allocate the parallel arrays we publish — a single pass over the
        # store fills them, avoiding the per-attribute list-comprehension passes.
        keys = np.empty(n, dtype=np.int64)
        wmat = np.empty((n, 4), dtype=np.float32)
        Ws   = np.empty(n,     dtype=np.float32)
        aps  = np.empty(n,     dtype=np.float32)
        for i, (k, v) in enumerate(items):
            keys[i]    = k
            wmat[i, 0] = v.w[0]
            wmat[i, 1] = v.w[1]
            wmat[i, 2] = v.w[2]
            wmat[i, 3] = v.w[3]
            Ws[i]      = v.W
            aps[i]     = v.alpha_str_prior

        msg.voxel_key       = keys.tolist()         # int64[] expects a python list
        msg.w_free          = wmat[:, 0].tolist()
        msg.w_sb            = wmat[:, 1].tolist()
        msg.w_obj           = wmat[:, 2].tolist()
        msg.w_str           = wmat[:, 3].tolist()
        msg.weight          = Ws.tolist()
        msg.alpha_str_prior = aps.tolist()
        self._map_pub.publish(msg)

    # ── active voxel cloud (visualisation) ────────────────────────────────────

    def _publish_active(self):
        items = list(self._store.items())
        if not items:
            return

        keys = np.fromiter((k for k, _ in items), dtype=np.int64,
                           count=len(items))
        wmat = np.array([v.w for _, v in items], dtype=np.float64)   # (N, 4)
        Ws   = np.array([v.W for _, v in items], dtype=np.float64)   # (N,)
        aps  = np.array([v.alpha_str_prior for _, v in items],
                        dtype=np.float64)                            # (N,)
        imean = np.array([v.intensity_mean for _, v in items],
                         dtype=np.float64)                           # (N,)

        # Per-voxel LiDAR-derived STRUCTURE prior enters pi_str's numerator
        # and once in the shared denominator:
        #   pi_c    = (alpha_0 + w_c) / Z          for c != STRUCTURE
        #   pi_str  = (alpha_0 + alpha_str_prior + w_str) / Z
        #   Z       = 4*alpha_0 + alpha_str_prior + W
        num            = self._a0 + wmat                             # (N, 4)
        num[:, STRUCTURE] += aps
        denom          = self._four_a0 + aps + Ws                    # (N,)
        pi   = num / denom[:, None]
        cmax = np.argmax(pi, axis=1)
        pmax = pi[np.arange(len(keys)), cmax]
        conf = pmax * np.tanh(Ws / self._w_ref)

        keep = Ws >= self._w_disp_min
        if not np.any(keep):
            return

        ix, iy, iz = _unpack(keys[keep])
        out = np.empty((int(keep.sum()), 11), dtype=np.float32)
        out[:, 0] = (ix.astype(np.float64) + 0.5) * self._r_v
        out[:, 1] = (iy.astype(np.float64) + 0.5) * self._r_v
        out[:, 2] = (iz.astype(np.float64) + 0.5) * self._r_v
        out[:, 3] = conf[keep].astype(np.float32)
        out[:, 4] = Ws[keep].astype(np.float32)
        out[:, 5] = cmax[keep].astype(np.float32)
        # pi[keep] is (N_keep, 4) in [free, sb, obj, str] order — drop straight in.
        out[:, 6:10] = pi[keep].astype(np.float32)
        out[:, 10]   = imean[keep].astype(np.float32)

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
