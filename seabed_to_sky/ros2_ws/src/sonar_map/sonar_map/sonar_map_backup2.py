#!/usr/bin/env python3
"""
prob_sonar_map_ros2.py  (v8 — two-sided Beta-Bernoulli, uniform depth prior)
-----------------------------------------------------------------------------
Beta-Bernoulli probabilistic occupancy map for imaging sonar.

═══════════════════════════════════════════════════════════════════════════════
PARAMETERS  (set at the top of this file or via ROS2 launch)
═══════════════════════════════════════════════════════════════════════════════

  GEOMETRY
  ─────────────────────────────────────────────────────────────────────────
  voxel_size          float   Side length of each cubic voxel [m].
                              Default: 0.10

  SEABED PRIOR  (uniform depth band)
  ─────────────────────────────────────────────────────────────────────────
  seabed_depth        float   Expected depth of the bare seabed below the
                              USV [m].  Defines the bottom of the band.
                              Default: 6.0

  tallest_object      float   Height of the tallest object expected on the
                              seabed [m].  Defines the top of the band.
                              band = [seabed_depth - tallest_object,
                                      seabed_depth]
                              Default: 1.0

  The prior p(d) is UNIFORM inside this band and zero outside:
      p(d) = 1 / tallest_object   for d ∈ [d_lo, d_hi]
  where d_lo = seabed_depth - tallest_object
        d_hi = seabed_depth

  INTENSITY GATE
  ─────────────────────────────────────────────────────────────────────────
  min_intensity       float   Raw intensity threshold (uint16).
                              Beams with I >= min_intensity  → HIT  → α++
                              Beams with I <  min_intensity  → MISS → β++
                              Default: 50.0

  intensity_scale     float   Normalisation reference for the intensity
                              weight on hits:
                                w_I = 0.5 + 0.5 * clip(I / intensity_scale)
                              Default: 2000.0

  BAYESIAN UPDATE
  ─────────────────────────────────────────────────────────────────────────
  lambda_hit          float   Alpha increment gain per hit.
                              Δα = lambda_hit * w_I
                              Default: 1.0

  lambda_miss         float   Beta increment gain per miss.
                              Δβ = lambda_miss   (constant inside band)
                              Default: 0.1
                              Rule of thumb: lambda_hit / lambda_miss gives
                              the number of consecutive misses needed to
                              erase one hit.

  alpha_min           float   Initial α prior (voxel creation).
                              Default: 1e-3

  beta_min            float   Initial β prior (voxel creation).
                              Default: 1e-3

  PUBLICATION FILTER
  ─────────────────────────────────────────────────────────────────────────
  pub_threshold       float   Minimum p = α/(α+β) to publish a voxel.
                              Voxels that drop below this after a miss
                              update are DELETED from the map.
                              Default: 0.5

  min_hits            int     Minimum hit count before a voxel is published.
                              Default: 2

  SPATIAL WINDOW
  ─────────────────────────────────────────────────────────────────────────
  window_radius       float   Voxels further than this from the USV in XY
                              are pruned every prune_every_n_scans scans.
                              Default: 200.0

  prune_every_n_scans int     How often to run the spatial prune.
                              Default: 50

  PERSISTENCE
  ─────────────────────────────────────────────────────────────────────────
  save_path           str     Path for manual save service and autosave.
  load_path           str     Path to load a prior map on startup.
  autosave_on_shutdown bool   Autosave on node shutdown.
                              Default: True

═══════════════════════════════════════════════════════════════════════════════
WHAT CHANGED IN v8 vs v7
═══════════════════════════════════════════════════════════════════════════════

1.  TWO-SIDED Beta-Bernoulli update
    ─────────────────────────────────
    v7 only incremented α (hits). β was fixed forever at beta_min.
    v8 updates β on misses:

        HIT  (I >= min_intensity, d ∈ [d_lo, d_hi]):
            Δα = lambda_hit * w_I
            w_I = 0.5 + 0.5 * clip(I / intensity_scale, 0, 1)

        MISS (I <  min_intensity, d ∈ [d_lo, d_hi]):
            Δβ = lambda_miss

    A beam carries information about a voxel ONLY if the sonar physically
    fired through its (x,y) column this scan. Voxels in the depth band
    that were NOT insonified this scan receive NO update at all.

2.  UNIFORM depth prior
    ─────────────────────
    v7 used a Gaussian weight w_d = exp(-0.5*((d-mu_d)/sigma_d)^2).
    v8 replaces this with a uniform prior over [d_lo, d_hi]:
        w_d = 1  inside the band
        w_d = 0  outside  (hard gate, same as before)
    This is more honest: we know the seabed is somewhere in the band
    but we do not know where exactly. Every depth in the band is equally
    plausible. The only thing that differentiates returns inside the band
    is acoustic intensity.

3.  DELETION on probability drop
    ────────────────────────────
    After a β update, if p = α/(α+β) < pub_threshold the voxel is
    removed from the store entirely, freeing memory.

4.  PARAMETERS renamed / simplified
    ──────────────────────────────────
    seabed_depth_h  → seabed_depth   (bare seabed depth below USV)
    max_object_height_hh → tallest_object  (height of tallest object)
    sigma_r_m removed  (was unused in practice)
    lambda_miss added  (new)

FILE FORMAT (.sonarmap, little-endian) — version 8:
  [0]   magic     uint32   0x534F4E52
  [4]   version   uint8    8
  [5]   voxel_sz  float32
  [9]   n_voxels  uint64
  [17]  per voxel: key(i64) alpha(f32) beta(f32) sumI(f32) hits(f32) = 24 bytes
"""

import math
import os
import struct
import time
from typing import Dict, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
import numpy.lib.recfunctions as rfn


# ══════════════════════════════════════════════════════════════════════════════
# DEFAULT PARAMETERS — edit here or override via ROS2 launch arguments
# ══════════════════════════════════════════════════════════════════════════════

# Geometry
VOXEL_SIZE          = 0.01      # [m] side length of each voxel

# Seabed prior (uniform band)
SEABED_DEPTH        = 4.0       # [m] depth of bare seabed below USV
TALLEST_OBJECT      = 2.0       # [m] height of tallest object on seabed
#   → band = [SEABED_DEPTH - TALLEST_OBJECT, SEABED_DEPTH]
#           = [5.0, 6.0] m with the values above

# Intensity gate
MIN_INTENSITY       = 100.0      # [uint16] gate threshold
INTENSITY_SCALE     = 2000.0    # [uint16] normalisation reference

# Bayesian update gains
LAMBDA_HIT          = 1.0       # alpha increment gain per hit
LAMBDA_MISS         = 0.1       # beta  increment gain per miss
#   → LAMBDA_HIT / LAMBDA_MISS = 10 consecutive misses cancel one hit

# Priors
ALPHA_MIN           = 1e-3      # initial alpha at voxel creation
BETA_MIN            = 1e-3      # initial beta  at voxel creation

# Publication filter
PUB_THRESHOLD       = 0.9       # min p to publish; voxels below this are deleted
MIN_HITS            = 1         # min hit count to publish

# Spatial window
WINDOW_RADIUS       = 200.0     # [m] spatial pruning radius
PRUNE_EVERY_N_SCANS = 50        # how often to prune

# Topics / frames
INPUT_TOPIC         = 'sonar_scan'
OUTPUT_TOPIC        = 'sonar_map'
ODOM_TOPIC          = 'odometry'
MAP_FRAME           = 'odom'

# Publishing
PUBLISH_RATE        = 10.0      # [Hz]
SOURCE_ID           = 0.0       # tag added to every published point

# Persistence
SAVE_PATH           = '/home/rosdev/ros2_ws/saved_maps/voxels/sonar_map.sonarmap'
LOAD_PATH           = ''        # leave empty to skip loading on startup
AUTOSAVE_ON_SHUTDOWN = True
AUTOSAVE_PATH       = ''        # if empty, uses SAVE_PATH


# ══════════════════════════════════════════════════════════════════════════════
# INTERNALS — do not edit below unless you know what you are doing
# ══════════════════════════════════════════════════════════════════════════════

_BITS    = 21
_MASK    = np.int64((1 << _BITS) - 1)
_SIGN    = 1 << (_BITS - 1)
_MAGIC   = 0x534F4E52   # "SONR"
_VERSION = 8

_RECORD_DTYPE = np.dtype([
    ('key',   np.int64),
    ('alpha', np.float32),
    ('beta',  np.float32),
    ('sumI',  np.float32),
    ('hits',  np.float32),
])


def _pack(ix: np.ndarray, iy: np.ndarray, iz: np.ndarray) -> np.ndarray:
    return ((ix & _MASK) << (2 * _BITS)) | ((iy & _MASK) << _BITS) | (iz & _MASK)


def _unpack(keys: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    iz = (keys & _MASK).astype(np.int32)
    iy = ((keys >> _BITS) & _MASK).astype(np.int32)
    ix = ((keys >> (2 * _BITS)) & _MASK).astype(np.int32)
    iz = np.where(iz >= _SIGN, iz - (1 << _BITS), iz)
    iy = np.where(iy >= _SIGN, iy - (1 << _BITS), iy)
    ix = np.where(ix >= _SIGN, ix - (1 << _BITS), ix)
    return ix, iy, iz


# ── Robust PointCloud2 reading ───────────────────────────────────────────────

def _read_xyz_intensity(msg: PointCloud2) -> np.ndarray:
    """Return (N,4) float32 [x,y,z,intensity]. Works for all sensor_msgs_py versions."""
    raw = pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)
    if isinstance(raw, np.ndarray):
        if raw.size == 0:
            return np.empty((0, 4), dtype=np.float32)
        if raw.dtype.names is not None:
            return rfn.structured_to_unstructured(raw, dtype=np.float32)
        return raw.reshape(-1, 4).astype(np.float32)
    rows = list(raw)
    if not rows:
        return np.empty((0, 4), dtype=np.float32)
    try:
        return np.array(rows, dtype=np.float32).reshape(-1, 4)
    except Exception:
        pts = np.empty((len(rows), 4), dtype=np.float32)
        for i, r in enumerate(rows):
            pts[i, 0] = float(r[0]); pts[i, 1] = float(r[1])
            pts[i, 2] = float(r[2]); pts[i, 3] = float(r[3])
        return pts


# ── Preallocated voxel store ──────────────────────────────────────────────────

class VoxelStore:
    """
    Preallocated parallel arrays for O(1) voxel lookup and zero-copy publishing.

      keys_arr : (capacity,)   int64    packed voxel key
      data_arr : (capacity, 4) float32  [alpha, beta, sumI, hits]
      _index   : dict[int -> int]       key -> row
      _n       : int                    number of live voxels
    """

    _INIT_CAP = 8_192

    def __init__(self, alpha_min: float, beta_min: float):
        self.alpha_min = np.float32(alpha_min)
        self.beta_min  = np.float32(beta_min)
        cap = self._INIT_CAP
        self.keys_arr = np.empty(cap, dtype=np.int64)
        self.data_arr = np.zeros((cap, 4), dtype=np.float32)
        self._index: Dict[int, int] = {}
        self._n = 0

    def _grow(self):
        old_cap = len(self.keys_arr)
        new_cap = old_cap * 2
        new_keys = np.empty(new_cap, dtype=np.int64)
        new_data = np.zeros((new_cap, 4), dtype=np.float32)
        new_keys[:old_cap] = self.keys_arr
        new_data[:old_cap] = self.data_arr
        self.keys_arr = new_keys
        self.data_arr = new_data

    def __len__(self) -> int:
        return self._n

    # ── hit update ────────────────────────────────────────────────────────────

    def update_hits(self,
                    hit_keys:  np.ndarray,   # (K,) int64
                    hit_da:    np.ndarray,   # (K,) float64  Δα per voxel
                    hit_inten: np.ndarray):  # (K,) float64  mean intensity
        am  = self.alpha_min
        bm  = self.beta_min
        idx = self._index
        ka  = self.keys_arr
        da  = self.data_arr
        n   = self._n

        for i in range(len(hit_keys)):
            key = int(hit_keys[i])
            inc = float(hit_da[i])
            it  = float(hit_inten[i])
            row = idx.get(key, -1)
            if row == -1:
                if n >= len(ka):
                    self._grow()
                    ka = self.keys_arr
                    da = self.data_arr
                ka[n]    = key
                da[n, 0] = am + inc   # alpha
                da[n, 1] = bm         # beta
                da[n, 2] = it         # sumI
                da[n, 3] = 1.0        # hits
                idx[key] = n
                n += 1
            else:
                da[row, 0] += inc
                da[row, 2] += it
                da[row, 3] += 1.0

        self._n = n

    # ── miss update ───────────────────────────────────────────────────────────

    def update_misses(self,
                      miss_keys:   np.ndarray,   # (M,) int64  existing voxel keys
                      delta_beta:  float,         # constant Δβ increment
                      pub_threshold: float) -> int:
        """
        Increment β for each voxel in miss_keys.
        If p = α/(α+β) drops below pub_threshold after the update,
        the voxel is DELETED from the store.
        Returns the number of voxels deleted.
        """
        idx      = self._index
        da       = self.data_arr
        to_delete = []

        for i in range(len(miss_keys)):
            key = int(miss_keys[i])
            row = idx.get(key, -1)
            if row == -1:
                continue                        # voxel not in map, skip
            da[row, 1] += delta_beta            # β grows
            alpha = da[row, 0]
            beta  = da[row, 1]
            p     = alpha / max(alpha + beta, 1e-9)
            if p < pub_threshold:
                to_delete.append(key)

        if not to_delete:
            return 0

        # Compact: remove deleted rows by swapping with tail
        for key in to_delete:
            row = idx.pop(key, -1)
            if row == -1:
                continue
            last = self._n - 1
            if row != last:
                # Move last row into the deleted slot
                last_key              = int(self.keys_arr[last])
                self.keys_arr[row]    = self.keys_arr[last]
                self.data_arr[row]    = self.data_arr[last]
                idx[last_key]         = row
            self._n -= 1

        return len(to_delete)

    # ── spatial prune ─────────────────────────────────────────────────────────

    def prune_outside_radius(self, cx: float, cy: float,
                              radius: float, vs: float) -> int:
        n = self._n
        if n == 0:
            return 0
        keys = self.keys_arr[:n]
        ix_u, iy_u, _ = _unpack(keys)
        vx   = (ix_u + 0.5) * vs - cx
        vy   = (iy_u + 0.5) * vs - cy
        keep = vx * vx + vy * vy <= radius * radius
        n_keep = int(keep.sum())
        if n_keep == n:
            return 0
        new_keys = np.empty(len(self.keys_arr), dtype=np.int64)
        new_data = np.zeros_like(self.data_arr)
        new_keys[:n_keep] = keys[keep]
        new_data[:n_keep] = self.data_arr[:n][keep]
        self.keys_arr = new_keys
        self.data_arr = new_data
        self._index   = {int(new_keys[r]): r for r in range(n_keep)}
        self._n       = n_keep
        return n - n_keep

    # ── zero-copy views ───────────────────────────────────────────────────────

    def live_keys(self) -> np.ndarray:
        return self.keys_arr[:self._n]

    def live_data(self) -> np.ndarray:
        return self.data_arr[:self._n]

    # ── persistence ──────────────────────────────────────────────────────────

    def save(self, path: str, voxel_size: float) -> int:
        n = self._n
        if n == 0:
            return 0
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        tmp = path + '.tmp'
        with open(tmp, 'wb') as f:
            f.write(struct.pack('<IBfQ', _MAGIC, _VERSION, voxel_size, n))
            rec          = np.empty(n, dtype=_RECORD_DTYPE)
            rec['key']   = self.keys_arr[:n]
            rec['alpha'] = self.data_arr[:n, 0]
            rec['beta']  = self.data_arr[:n, 1]
            rec['sumI']  = self.data_arr[:n, 2]
            rec['hits']  = self.data_arr[:n, 3]
            rec.tofile(f)
        os.replace(tmp, path)
        return n

    def load(self, path: str, voxel_size: float) -> int:
        if not os.path.isfile(path):
            raise FileNotFoundError(f'Map file not found: {path}')
        with open(path, 'rb') as f:
            header = f.read(17)
        if len(header) < 17:
            raise ValueError(f'File too short: {path}')
        magic, version, file_vs, n_voxels = struct.unpack_from('<IBfQ', header, 0)
        if magic != _MAGIC:
            raise ValueError(f'Bad magic 0x{magic:08X}')
        if version not in (2, 3, 4, 5, 6, 7, 8):
            raise ValueError(f'Unsupported version {version}')
        if abs(file_vs - voxel_size) > 1e-5:
            raise ValueError(
                f'Voxel size mismatch: file={file_vs:.4f}m map={voxel_size:.4f}m')
        with open(path, 'rb') as f:
            f.read(17)
            records = np.frombuffer(
                f.read(n_voxels * _RECORD_DTYPE.itemsize), dtype=_RECORD_DTYPE)
        if len(records) != n_voxels:
            raise ValueError(f'Expected {n_voxels} records, got {len(records)}')

        needed = self._n + int(n_voxels)
        while needed > len(self.keys_arr):
            self._grow()

        idx = self._index
        ka  = self.keys_arr
        da  = self.data_arr
        n   = self._n

        for rec in records:
            key = int(rec['key'])
            row = idx.get(key, -1)
            if row == -1:
                ka[n]    = key
                da[n, 0] = float(rec['alpha'])
                da[n, 1] = float(rec['beta'])
                da[n, 2] = float(rec['sumI'])
                da[n, 3] = float(rec['hits'])
                idx[key] = n
                n += 1
            else:
                da[row, 0] += float(rec['alpha'])
                da[row, 1] += float(rec['beta'])
                da[row, 2] += float(rec['sumI'])
                da[row, 3] += float(rec['hits'])

        self._n = n
        return int(n_voxels)


# ── Probabilistic voxel map ───────────────────────────────────────────────────

class ProbabilisticVoxelMap:
    """
    Two-sided Beta-Bernoulli occupancy map (v8).

    Depth prior: UNIFORM over [seabed_depth - tallest_object, seabed_depth]
    Hit  update: Δα = lambda_hit  * w_I   (intensity-weighted)
    Miss update: Δβ = lambda_miss          (constant inside band)
    Deletion:    voxel removed if p < pub_threshold after a miss update

    Only beams that the sonar actually fired (present in the point cloud,
    regardless of intensity) carry information about a voxel's column.
    Voxels whose (x,y) column was not insonified this scan are untouched.
    """

    def __init__(
        self,
        voxel_size:      float,
        seabed_depth:    float,
        tallest_object:  float,
        lambda_hit:      float,
        lambda_miss:     float,
        alpha_min:       float,
        beta_min:        float,
        min_intensity:   float,
        intensity_scale: float,
        pub_threshold:   float,
    ):
        self.vs          = float(voxel_size)
        self.inv_vs      = 1.0 / self.vs

        # Uniform depth band
        self.d_hi        = float(seabed_depth)               # bare seabed
        self.d_lo        = float(seabed_depth - tallest_object)  # top of tallest object
        if self.d_lo >= self.d_hi:
            raise ValueError(
                f'tallest_object ({tallest_object}) must be > 0 and < seabed_depth ({seabed_depth})')

        self.lambda_hit      = float(lambda_hit)
        self.lambda_miss     = float(lambda_miss)
        self.min_intensity_  = float(min_intensity)
        self.intensity_scale_= max(float(intensity_scale), 1.0)
        self.pub_threshold_  = float(pub_threshold)

        self._store = VoxelStore(alpha_min, beta_min)

    def __len__(self) -> int:
        return len(self._store)

    # ── insert one scan ───────────────────────────────────────────────────────

    def insert_scan(self, origin_xyz: np.ndarray, pts_xyzi: np.ndarray) -> dict:
        """
        origin_xyz : (3,)  float64  USV/sensor position in map frame (Z-up / ENU)
        pts_xyzi   : (N,4) float32  ALL sonar returns [x,y,z,intensity]
                                    including weak ones — they are needed for
                                    miss detection.

        Returns a dict with scan statistics for logging.
        """
        if pts_xyzi.size == 0:
            return {'hits': 0, 'misses': 0, 'deleted': 0}

        sz      = float(origin_xyz[2])
        inv_vs  = self.inv_vs
        d_lo    = self.d_lo
        d_hi    = self.d_hi

        # Depth of each beam endpoint below the USV
        depths  = sz - pts_xyzi[:, 2].astype(np.float64)   # (N,)

        # ── Depth-band filter (applies to ALL beams) ──────────────────────────
        in_band = (depths >= d_lo) & (depths <= d_hi)       # (N,) bool
        if not np.any(in_band):
            return {'hits': 0, 'misses': 0, 'deleted': 0}

        band_pts    = pts_xyzi[in_band]                     # (B, 4)
        band_inten  = band_pts[:, 3].astype(np.float64)    # (B,)

        # ── Split into hits and misses by intensity gate ──────────────────────
        is_hit  = band_inten >= self.min_intensity_         # (B,) bool
        is_miss = ~is_hit                                   # (B,) bool

        # ── Voxelise ALL band points (hits and misses) ────────────────────────
        ix = np.floor(band_pts[:, 0] * inv_vs).astype(np.int64)
        iy = np.floor(band_pts[:, 1] * inv_vs).astype(np.int64)
        iz = np.floor(band_pts[:, 2] * inv_vs).astype(np.int64)
        pk = _pack(ix, iy, iz)

        stats = {'hits': 0, 'misses': 0, 'deleted': 0}

        # ════════════════════════════════════════════════════════════════════
        # HIT UPDATE — Δα = lambda_hit * w_I
        # w_I = 0.5 + 0.5 * clip(I / intensity_scale, 0, 1)
        # Multiple hits in the same voxel within one scan are averaged.
        # ════════════════════════════════════════════════════════════════════
        if np.any(is_hit):
            hit_pk    = pk[is_hit]
            hit_inten = band_inten[is_hit]

            # Average points landing in the same voxel this scan
            uniq_pk, inv_idx, counts = np.unique(
                hit_pk, return_inverse=True, return_counts=True)
            mean_inten = np.zeros(len(uniq_pk), dtype=np.float64)
            np.add.at(mean_inten, inv_idx, hit_inten)
            mean_inten /= counts

            I_norm = np.clip(mean_inten / self.intensity_scale_, 0.0, 1.0)
            w_I    = 0.5 + 0.5 * I_norm
            delta_alpha = self.lambda_hit * w_I

            self._store.update_hits(uniq_pk, delta_alpha, mean_inten)
            stats['hits'] = len(uniq_pk)

        # ════════════════════════════════════════════════════════════════════
        # MISS UPDATE — Δβ = lambda_miss  (constant, uniform prior)
        #
        # Only existing map voxels whose (x,y,z) column was insonified
        # this scan AND produced a weak/no return get a β increment.
        # New voxels are NOT created for misses.
        # ════════════════════════════════════════════════════════════════════
        if np.any(is_miss):
            miss_pk = np.unique(pk[is_miss])    # deduplicate

            # Keep only keys that already exist in the map
            existing_mask = np.array(
                [self._store._index.get(int(k), -1) != -1 for k in miss_pk],
                dtype=bool)
            existing_miss_keys = miss_pk[existing_mask]

            if len(existing_miss_keys) > 0:
                deleted = self._store.update_misses(
                    existing_miss_keys,
                    self.lambda_miss,
                    self.pub_threshold_)
                stats['misses']  = len(existing_miss_keys)
                stats['deleted'] = deleted

        return stats

    # ── spatial pruning ───────────────────────────────────────────────────────

    def prune_outside_radius(self, cx: float, cy: float, radius: float) -> int:
        return self._store.prune_outside_radius(cx, cy, radius, self.vs)

    # ── query (zero-copy) ─────────────────────────────────────────────────────

    def to_numpy(self, min_hits: int) -> np.ndarray:
        """
        Return (N,5) float32: [x, y, z, occupancy_prob, intensity_mean].
        Only voxels with p >= pub_threshold AND hits >= min_hits are returned.
        (pub_threshold is already enforced on deletion; this just applies min_hits.)
        """
        n = len(self._store)
        if n == 0:
            return np.empty((0, 5), dtype=np.float32)

        keys = self._store.live_keys()
        data = self._store.live_data()

        alpha = data[:, 0]
        beta  = data[:, 1]
        prob  = alpha / np.maximum(alpha + beta, 1e-9)
        hits  = data[:, 3]

        # pub_threshold already guaranteed by deletion, but recheck for safety
        mask = (prob >= self.pub_threshold_) & (hits >= float(min_hits))
        if not np.any(mask):
            return np.empty((0, 5), dtype=np.float32)

        k  = keys[mask]
        d  = data[mask]
        p  = prob[mask]
        vs = self.vs
        ix_u, iy_u, iz_u = _unpack(k)
        cnt = np.maximum(d[:, 3], 1.0)

        return np.column_stack([
            (ix_u + 0.5) * vs,
            (iy_u + 0.5) * vs,
            (iz_u + 0.5) * vs,
            p,
            d[:, 2] / cnt,
        ]).astype(np.float32)

    def depth_stats(self, vehicle_z: float) -> np.ndarray:
        n = len(self._store)
        if n == 0:
            return np.empty(0)
        _, _, iz_u = _unpack(self._store.live_keys())
        return vehicle_z - (iz_u + 0.5) * self.vs

    # ── persistence ──────────────────────────────────────────────────────────

    def save(self, path: str) -> int:
        return self._store.save(path, self.vs)

    def load(self, path: str) -> int:
        return self._store.load(path, self.vs)


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class ProbSonarMapNode(Node):

    def __init__(self):
        super().__init__('prob_sonar_map')

        # ── Declare parameters (defaults come from constants at top of file) ──
        self.declare_parameter('input_topic',          INPUT_TOPIC)
        self.declare_parameter('output_topic',         OUTPUT_TOPIC)
        self.declare_parameter('odom_topic',           ODOM_TOPIC)
        self.declare_parameter('map_frame',            MAP_FRAME)
        self.declare_parameter('voxel_size',           VOXEL_SIZE)
        self.declare_parameter('seabed_depth',         SEABED_DEPTH)
        self.declare_parameter('tallest_object',       TALLEST_OBJECT)
        self.declare_parameter('min_intensity',        MIN_INTENSITY)
        self.declare_parameter('intensity_scale',      INTENSITY_SCALE)
        self.declare_parameter('lambda_hit',           LAMBDA_HIT)
        self.declare_parameter('lambda_miss',          LAMBDA_MISS)
        self.declare_parameter('alpha_min',            ALPHA_MIN)
        self.declare_parameter('beta_min',             BETA_MIN)
        self.declare_parameter('pub_threshold',        PUB_THRESHOLD)
        self.declare_parameter('min_hits',             MIN_HITS)
        self.declare_parameter('window_radius',        WINDOW_RADIUS)
        self.declare_parameter('prune_every_n_scans',  PRUNE_EVERY_N_SCANS)
        self.declare_parameter('publish_rate',         PUBLISH_RATE)
        self.declare_parameter('source_id',            SOURCE_ID)
        self.declare_parameter('save_path',            SAVE_PATH)
        self.declare_parameter('load_path',            LOAD_PATH)
        self.declare_parameter('autosave_on_shutdown', AUTOSAVE_ON_SHUTDOWN)
        self.declare_parameter('autosave_path',        AUTOSAVE_PATH)

        def gp(n): return self.get_parameter(n).value

        voxel_size    = float(gp('voxel_size'))
        seabed_depth  = float(gp('seabed_depth'))
        tallest_obj   = float(gp('tallest_object'))
        publish_rate  = float(gp('publish_rate'))

        self.map_frame_            = gp('map_frame')
        self.window_radius_        = float(gp('window_radius'))
        self.min_hits_             = int(gp('min_hits'))
        self.source_id_            = float(gp('source_id'))
        self.save_path_            = str(gp('save_path'))
        self.autosave_path_        = str(gp('autosave_path'))
        self.autosave_on_shutdown_ = bool(gp('autosave_on_shutdown'))
        self.prune_every_n_scans_  = max(1, int(gp('prune_every_n_scans')))

        self.voxel_map_ = ProbabilisticVoxelMap(
            voxel_size      = voxel_size,
            seabed_depth    = seabed_depth,
            tallest_object  = tallest_obj,
            lambda_hit      = float(gp('lambda_hit')),
            lambda_miss     = float(gp('lambda_miss')),
            alpha_min       = float(gp('alpha_min')),
            beta_min        = float(gp('beta_min')),
            min_intensity   = float(gp('min_intensity')),
            intensity_scale = float(gp('intensity_scale')),
            pub_threshold   = float(gp('pub_threshold')),
        )

        self.vehicle_pose_ = None
        self._scan_count_  = 0

        # ── PointCloud2 field descriptors ──────────────────────────────────
        self._fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='prob',      offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='source',    offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        self._kdtree_fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # ── Publishers ─────────────────────────────────────────────────────
        self.map_pub_ = self.create_publisher(
            PointCloud2, gp('output_topic'), 10)
        latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.kdtree_pub_ = self.create_publisher(
            PointCloud2, 'sonar_map_for_kdtree', latched_qos)

        # ── Subscribers ────────────────────────────────────────────────────
        self.cloud_sub_ = self.create_subscription(
            PointCloud2, gp('input_topic'), self.cloud_callback, 10)
        self.odom_sub_  = self.create_subscription(
            Odometry, gp('odom_topic'), self.odom_callback, 10)

        # ── Services ───────────────────────────────────────────────────────
        self.save_srv_ = self.create_service(
            Trigger, 'sonar_map/save', self._save_cb)
        self.load_srv_ = self.create_service(
            Trigger, 'sonar_map/load', self._load_cb)

        # ── Publish timer ──────────────────────────────────────────────────
        self.timer_ = self.create_timer(1.0 / publish_rate, self.publish_map)

        # ── Load prior map if requested ────────────────────────────────────
        load_path = str(gp('load_path'))
        if load_path:
            self._do_load(load_path)

        vm = self.voxel_map_
        self.get_logger().info(
            f'ProbSonarMap v8 | '
            f'voxel={voxel_size}m | '
            f'depth_band=[{vm.d_lo:.2f}, {vm.d_hi:.2f}]m '
            f'(seabed={seabed_depth}m, tallest_object={tallest_obj}m) | '
            f'lambda_hit={float(gp("lambda_hit"))} '
            f'lambda_miss={float(gp("lambda_miss"))} | '
            f'hit/miss ratio={float(gp("lambda_hit"))/float(gp("lambda_miss")):.0f} | '
            f'min_intensity={float(gp("min_intensity"))} | '
            f'pub_threshold={float(gp("pub_threshold"))} | '
            f'rate={publish_rate}Hz'
        )

    # ── shutdown ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self.autosave_on_shutdown_:
            path = self.autosave_path_ or self.save_path_
            if path:
                self.get_logger().info(f'Autosaving map to {path} ...')
                try:
                    n = self.voxel_map_.save(path)
                    self.get_logger().info(f'Autosave: {n} voxels saved.')
                except Exception as e:
                    self.get_logger().error(f'Autosave failed: {e}')
        super().destroy_node()

    # ── services ──────────────────────────────────────────────────────────────

    def _save_cb(self, _req, resp):
        if not self.save_path_:
            resp.success = False
            resp.message = 'save_path parameter is empty'
            return resp
        resp.success, resp.message = self._do_save(self.save_path_)
        return resp

    def _load_cb(self, _req, resp):
        path = self.save_path_ or self.get_parameter('load_path').value
        if not path:
            resp.success = False
            resp.message = 'Neither save_path nor load_path is set'
            return resp
        resp.success, resp.message = self._do_load(path)
        return resp

    def _do_save(self, path: str) -> Tuple[bool, str]:
        try:
            t0 = time.monotonic()
            n  = self.voxel_map_.save(path)
            dt = time.monotonic() - t0
            msg = f'Saved {n} voxels to {path} in {dt*1000:.0f} ms'
            self.get_logger().info(msg)
            return True, msg
        except Exception as e:
            msg = f'Save failed: {e}'
            self.get_logger().error(msg)
            return False, msg

    def _do_load(self, path: str) -> Tuple[bool, str]:
        try:
            t0 = time.monotonic()
            n  = self.voxel_map_.load(path)
            dt = time.monotonic() - t0
            msg = (f'Loaded {n} voxels from {path} in {dt*1000:.0f} ms | '
                   f'map size: {len(self.voxel_map_)} voxels')
            self.get_logger().info(msg)
            return True, msg
        except Exception as e:
            msg = f'Load failed: {e}'
            self.get_logger().error(msg)
            return False, msg

    # ── callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.vehicle_pose_ = (p.x, p.y, p.z)

    def cloud_callback(self, msg: PointCloud2):
        if self.vehicle_pose_ is None:
            self.get_logger().warn(
                'No odometry yet — skipping scan.',
                throttle_duration_sec=5.0)
            return
        try:
            # IMPORTANT: read ALL points including weak returns
            # Weak returns (I < min_intensity) are needed for miss detection
            pts = _read_xyz_intensity(msg)
        except Exception as exc:
            self.get_logger().error(
                f'Failed to read PointCloud2: {exc}',
                throttle_duration_sec=5.0)
            return
        if pts.size == 0:
            return
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.size == 0:
            return

        t0     = time.monotonic()
        origin = np.array(self.vehicle_pose_, dtype=np.float64)
        stats  = self.voxel_map_.insert_scan(origin, pts)
        dt     = time.monotonic() - t0

        self._scan_count_ += 1

        if self._scan_count_ <= 3 or self._scan_count_ % 50 == 0:
            vm = self.voxel_map_
            self.get_logger().info(
                f'Scan #{self._scan_count_}: {len(pts)} pts | '
                f'hits={stats["hits"]} miss_updates={stats["misses"]} '
                f'deleted={stats["deleted"]} | '
                f'insert={dt*1000:.1f}ms | '
                f'usv_z={origin[2]:.3f}m | '
                f'band=[{vm.d_lo:.2f}, {vm.d_hi:.2f}]m | '
                f'map={len(vm)} voxels'
            )

        if self._scan_count_ % self.prune_every_n_scans_ == 0:
            cx, cy, _ = self.vehicle_pose_
            removed = self.voxel_map_.prune_outside_radius(
                cx, cy, self.window_radius_)
            if removed:
                self.get_logger().info(
                    f'Pruned {removed} voxels | map={len(self.voxel_map_)}')

    # ── publish ───────────────────────────────────────────────────────────────

    def publish_map(self):
        n_vox = len(self.voxel_map_)
        if n_vox == 0:
            self.get_logger().info(
                f'Map empty — scans={self._scan_count_}, '
                f'odom={"ok" if self.vehicle_pose_ else "waiting"}',
                throttle_duration_sec=5.0)
            return

        t0   = time.monotonic()
        pts5 = self.voxel_map_.to_numpy(min_hits=self.min_hits_)
        dt   = time.monotonic() - t0

        if pts5.size == 0:
            if self.vehicle_pose_ is not None:
                depths = self.voxel_map_.depth_stats(self.vehicle_pose_[2])
                if depths.size:
                    self.get_logger().warn(
                        f'{n_vox} voxels but 0 above threshold. '
                        f'depth_from_usv=[{depths.min():.2f}, {depths.max():.2f}]m | '
                        f'band=[{self.voxel_map_.d_lo:.1f}, {self.voxel_map_.d_hi:.1f}]m',
                        throttle_duration_sec=5.0)
            return

        n      = pts5.shape[0]
        header = Header()
        header.frame_id = self.map_frame_
        header.stamp    = self.get_clock().now().to_msg()

        out        = np.empty((n, 6), dtype=np.float32)
        out[:, :5] = pts5
        out[:, 5]  = self.source_id_
        self.map_pub_.publish(pc2.create_cloud(header, self._fields, out))

        self.kdtree_pub_.publish(
            pc2.create_cloud(header, self._kdtree_fields, pts5[:, :4].copy()))

        self.get_logger().info(
            f'Published {n}/{n_vox} voxels | to_numpy={dt*1000:.1f}ms',
            throttle_duration_sec=2.0)


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ProbSonarMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()