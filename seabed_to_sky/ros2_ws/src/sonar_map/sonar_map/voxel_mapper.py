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
ALPHA_L             = 0.1       # [-]  LiDAR-gate decay rate in the soft responsibility
ACTIVE_PUBLISH_RATE = 5.0       # [Hz] active voxel cloud republish rate
SNAPSHOT_RATE       = 1.0       # [Hz] voxel-map snapshot rate (for seabed_surface)
W_REF               = 10.0      # [-]  tanh scale for the visualisation confidence
W_DISPLAY_MIN       = 1.0       # [-]  display-only lower bound on W


class Voxel:
    """One active Dirichlet voxel."""

    __slots__ = ('w', 'W', 'ring', 'sum_r', 'n_r')

    def __init__(self, ring_len: int):
        self.w = np.zeros(4, dtype=np.float64)   # [free, sb, obj, str]
        self.W = 0.0                              # total accumulated weight
        self.ring = deque(maxlen=ring_len)        # recent return z-coords
        self.sum_r = 0.0                          # sum of contributing slant ranges
        self.n_r = 0                              # count of slant-range samples


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
        self.declare_parameter('alpha_l',             ALPHA_L)
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
        # Soft-responsibility gate decay rates (intensity, LiDAR support).
        self._alpha_i     = max(float(gp('alpha_i')), 0.0)
        self._alpha_l     = max(float(gp('alpha_l')), 0.0)
        self._w_ref       = max(float(gp('w_ref')), 1e-6)
        self._w_disp_min  = float(gp('w_display_min'))

        # ── The active Dirichlet store and the set of retired voxel keys ──────
        self._store: dict = {}        # key -> Voxel
        self._promoted: set = set()   # keys retired by seabed_surface (irreversible)

        # Per-class posteriors pi_c are carried as float32 in [0, 1] (e.g.
        # 0.87 == 87 %). Colour the cloud by any of them in rviz to read each
        # voxel's per-class probability directly.
        self._cloud_fields = [
            PointField(name='x',          offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',          offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',          offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='confidence', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='weight',     offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='semantic',   offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='pi_free',    offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name='pi_sb',      offset=28, datatype=PointField.FLOAT32, count=1),
            PointField(name='pi_obj',     offset=32, datatype=PointField.FLOAT32, count=1),
            PointField(name='pi_str',     offset=36, datatype=PointField.FLOAT32, count=1),
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
            f'alpha_0={self._a0} k_elev={self._k_elev} '
            f'alpha_i={self._alpha_i} alpha_l={self._alpha_l} '
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

    def _apply_multi(self, key, dw_vec, z_ring=None, slant=None):
        """Add a length-4 evidence vector to one voxel in a single op.

        Faster than three sequential ``_apply`` calls when the per-return
        update is a soft distribution over classes — one dict lookup, one
        numpy in-place add, one scalar ``W`` bump.
        """
        if key in self._promoted:
            return
        total = float(dw_vec.sum())
        if total <= 0.0:
            return
        v = self._store.get(key)
        if v is None:
            v = Voxel(self._ring_len)
            self._store[key] = v
        v.w += dw_vec
        v.W += total
        if z_ring is not None:
            v.ring.append(float(z_ring))
        if slant is not None:
            v.sum_r += float(slant)
            v.n_r += 1

    def _promoted_keys_cb(self, msg: PromotedVoxelKeys):
        """seabed_surface has promoted these voxels — retire them."""
        for k in msg.keys:
            k = int(k)
            self._promoted.add(k)
            if self._store.pop(k, None) is not None:
                self._n_retired += 1

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

        x     = np.asarray(msg.x,             dtype=np.float64)
        y     = np.asarray(msg.y,             dtype=np.float64)
        z     = np.asarray(msg.z,             dtype=np.float64)
        inten = np.asarray(msg.intensity,     dtype=np.float64)
        slant = np.asarray(msg.slant_range,   dtype=np.float64)
        v_arr = np.asarray(msg.lidar_support, dtype=np.float64)
        label = np.asarray(msg.label,         dtype=np.uint8)

        # ── Vectorised soft responsibilities (Dirichlet–Categorical) ──────────
        # Each non-FREE return contributes ONE unit of total mass to the voxel,
        # split between (SEABED, OBJECT, STRUCTURE) by:
        #     r_sb  = exp(-alpha_i * u)                     (SEABED)
        #     r_obj = (1 - r_sb) * exp(-alpha_l * v)        (OBJECT)
        #     r_str = (1 - r_sb) * (1 - exp(-alpha_l * v))  (STRUCTURE)
        # with u = max(I - mu_i, 0) / mu_i (intensity excess above the
        # estimator's mean) and v = the geometric LiDAR support score.
        # Sum = 1 by construction. Hierarchical: intensity decides "is this
        # seabed?", LiDAR decides "if not, is it object or structure?".
        mu_i = max(float(msg.mu_i), 1e-9)
        u    = np.maximum(inten - mu_i, 0.0) / mu_i
        f_I  = 1.0 - np.exp(-self._alpha_i * u)
        f_L  = 1.0 - np.exp(-self._alpha_l * v_arr)
        r_sb  = 1.0 - f_I
        r_obj = f_I * (1.0 - f_L)
        r_str = f_I * f_L

        # Per-return loop: FREE → unit mass to w_free; non-FREE → unit mass
        # split by (r_sb, r_obj, r_str). All four classes use the same
        # elevation-band spread (weight / N per band voxel).
        dw_vec = np.zeros(4, dtype=np.float64)   # reused across iterations
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
                attach_ring = False
            else:
                dw_vec[0] = 0.0
                dw_vec[1] = r_sb[i]  * inv_N
                dw_vec[2] = r_obj[i] * inv_N
                dw_vec[3] = r_str[i] * inv_N
                attach_ring = True

            for izc in izs:
                key = _pack(ix, iy, int(izc))
                if int(izc) == iz and attach_ring:
                    self._apply_multi(key, dw_vec,
                                      z_ring=z[i], slant=slant[i])
                else:
                    self._apply_multi(key, dw_vec)

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

        items = list(self._store.items())
        keys, cx, cy, cz = [], [], [], []
        wf, ws, wo, wt, W, sl = [], [], [], [], [], []
        ring_count, ring_flat = [], []
        for k, v in items:
            ix, iy, iz = _unpack_one(k)
            keys.append(int(k))
            cx.append((ix + 0.5) * self._r_v)
            cy.append((iy + 0.5) * self._r_v)
            cz.append((iz + 0.5) * self._r_v)
            wf.append(float(v.w[0]))
            ws.append(float(v.w[1]))
            wo.append(float(v.w[2]))
            wt.append(float(v.w[3]))
            W.append(float(v.W))
            sl.append(v.sum_r / v.n_r if v.n_r > 0 else 0.0)
            ring_count.append(len(v.ring))
            ring_flat.extend(float(d) for d in v.ring)

        msg.voxel_key = keys
        msg.center_x = cx
        msg.center_y = cy
        msg.center_z = cz
        msg.w_free = wf
        msg.w_sb = ws
        msg.w_obj = wo
        msg.w_str = wt
        msg.weight = W
        msg.mean_slant_range = sl
        msg.ring_count = ring_count
        msg.ring_depths_flat = ring_flat
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

        pi   = (self._a0 + wmat) / (self._four_a0 + Ws)[:, None]
        cmax = np.argmax(pi, axis=1)
        pmax = pi[np.arange(len(keys)), cmax]
        conf = pmax * np.tanh(Ws / self._w_ref)

        keep = Ws >= self._w_disp_min
        if not np.any(keep):
            return

        ix, iy, iz = _unpack(keys[keep])
        out = np.empty((int(keep.sum()), 10), dtype=np.float32)
        out[:, 0] = (ix.astype(np.float64) + 0.5) * self._r_v
        out[:, 1] = (iy.astype(np.float64) + 0.5) * self._r_v
        out[:, 2] = (iz.astype(np.float64) + 0.5) * self._r_v
        out[:, 3] = conf[keep].astype(np.float32)
        out[:, 4] = Ws[keep].astype(np.float32)
        out[:, 5] = cmax[keep].astype(np.float32)
        # pi[keep] is (N_keep, 4) in [free, sb, obj, str] order — drop straight in.
        out[:, 6:10] = pi[keep].astype(np.float32)

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
