#!/usr/bin/env python3
"""
prob_sonar_map_ros2.py  (v13 — ENU frame confirmed, zero-intensity fix, adaptive estimator)
-----------------------------------------------------------------
Beta-Bernoulli probabilistic occupancy map for imaging sonar.

═══════════════════════════════════════════════════════════════════════════════
PARAMETERS  (edit the block below or override via ROS2 launch arguments)
═══════════════════════════════════════════════════════════════════════════════

  GEOMETRY
  ────────────────────────────────────────────────────────────────────────
  voxel_size          float   Side length of each cubic voxel [m].
                              Default: 0.10

  SEABED PRIOR  (uniform depth band)
  ────────────────────────────────────────────────────────────────────────
  seabed_depth        float   Depth of the bare seabed below the USV [m].
                              Defines the BOTTOM of the valid band.
                              Default: 4.0

  tallest_object      float   Height of the tallest object on the seabed [m].
                              Defines the TOP of the valid band.
                              band = [seabed_depth - tallest_object,
                                      seabed_depth]
                              Default: 2.0
                              → with defaults: band = [2.0, 4.0] m

  The prior p(d) is UNIFORM inside this band and zero outside.
  Every voxel in the column [z0 - tallest_object, z0] is updated on
  each beam, where z0 is the φ=0 (nadir) projected depth of the return.

  INTENSITY GATE & COLUMN WEIGHT — THREE REGIMES
  ────────────────────────────────────────────────────────────────────────
  Three intensity regimes determine how each beam updates the map:

  REGIME 1 — MISS       I < min_intensity
    No acoustic return. Beta update (existing voxels only).
    Default: min_intensity = 1100

  REGIME 2 — SEABED HIT  min_intensity <= I < seabed_threshold
    Weak return. Hard decision: it is the seabed.
    Gaussian centred at t=0 (seabed level) with width sigma_seabed.
    Default: seabed_threshold = 3000, sigma_seabed = 0.05

  REGIME 3 — OBJECT HIT  I >= seabed_threshold
    Strong return. Object on the seabed.
    Gaussian centred at t = I_norm = clip(I / intensity_scale, 0, 1)
    with width sigma_t. Attribution follows intensity.
    Default: seabed_threshold = 3000, sigma_t = 0.05,
             intensity_scale = 10000  (max sonar intensity)

  Parameters:
  min_intensity        float  Hit/miss gate.          Default: 1100
  seabed_threshold     float  Seabed/object boundary. Default: 3000
  intensity_scale      float  Max intensity for I_norm normalisation.
                               Default: 10000
  sigma_seabed         float  Gaussian width for seabed regime [t units].
                               Default: 0.05
  sigma_t              float  Gaussian width for object regime [t units].
                               Default: 0.05

  BAYESIAN UPDATE
  ────────────────────────────────────────────────────────────────────────
  lambda_hit          float   Total α increment distributed across the
                              column per hit beam.
                              Default: 1.0

  lambda_miss         float   Total β increment distributed across the
                              column per miss beam (constant, no intensity).
                              Default: 0.1
                              Rule of thumb: lambda_hit / lambda_miss gives
                              the number of consecutive misses to erase
                              one hit.

  alpha_min           float   Initial α prior at voxel creation.
                              Default: 1e-3

  beta_min            float   Initial β prior at voxel creation.
                              Default: 1e-3

  PUBLICATION FILTER
  ────────────────────────────────────────────────────────────────────────
  pub_threshold       float   Minimum p = α/(α+β) to publish.
                              Voxels dropping below this are DELETED.
                              Default: 0.5

  min_hits            int     Minimum hit count to publish a voxel.
                              Default: 2

  SPATIAL WINDOW
  ────────────────────────────────────────────────────────────────────────
  window_radius       float   XY pruning radius [m].   Default: 200.0
  prune_every_n_scans int     Prune cadence [scans].   Default: 50

  PERSISTENCE
  ────────────────────────────────────────────────────────────────────────
  save_path           str     Path for save service and autosave.
  load_path           str     Path to load a prior map on startup.
  autosave_on_shutdown bool   Autosave on node shutdown.  Default: True

═══════════════════════════════════════════════════════════════════════════════
WHAT CHANGED IN v11 vs v10
═══════════════════════════════════════════════════════════════════════════════

CORE CHANGE: Two-regime intensity attribution.

v10 used a single Gaussian centred at I_norm for all hits, which was
noisy for mid-range intensities.

v11 introduces a hard intensity boundary (seabed_threshold):

  I < min_intensity        -> MISS  (beta update, no voxel creation)
  min_intensity <= I < seabed_threshold -> SEABED HIT
      Gaussian centred at t=0 (seabed), width sigma_seabed
      Hard decision: regardless of exact I value, it is the seabed.
  I >= seabed_threshold    -> OBJECT HIT
      Gaussian centred at t = I_norm = I / intensity_scale
      Attribution follows intensity continuously.

Default values tuned to BlueView M900 data:
  min_intensity     = 1100   (average return level)
  seabed_threshold  = 3000   (boundary between seabed and objects)
  intensity_scale   = 10000  (max sonar intensity)
  sigma_seabed      = 0.05   (tight seabed attribution, noise-robust)
  sigma_t           = 0.05   (tight object attribution, noise-robust)

New parameters: seabed_threshold, sigma_seabed
intensity_scale updated default: 2000 -> 10000
FILE version bumped to 11.
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
VOXEL_SIZE           = 0.05     # [m] voxel side length

# ── ADAPTIVE SEABED ESTIMATOR (Kalman filter) ─────────────────────────────────
# The seabed depth h and seabed intensity I_seabed are estimated online.
# You only need to set the prior means, uncertainties, and process noise.

TALLEST_OBJECT       = 0.5      # [m] ONLY fixed parameter — height of tallest object

# Prior on seabed depth h  (depth below USV, positive downward)
H_PRIOR_MEAN         = 2.5      # [m]   initial guess — observed mean depth ~1.84m, max ~4.8m
H_PRIOR_STD          = 0.5      # [m]   wide prior: let the 85th percentile find the real seabed
H_PROCESS_NOISE      = 0.05     # [m]   how much h can change between scans
H_MEAS_NOISE         = 0.02     # [m]   measurement noise on depth readings
H_MIN_CANDIDATES     = 10       # min seabed candidates per scan to trigger update

# Prior on seabed intensity I_seabed
# BlueView M900 observed: mean~530, max~2200, ~half of points have I>0
I_PRIOR_MEAN         = 530.0    # [uint16] initial guess (observed mean of non-zero returns)
I_PRIOR_STD          = 300.0    # [uint16] initial uncertainty
I_PROCESS_NOISE      = 30.0     # [uint16] intensity is fairly stable
I_MEAS_NOISE         = 150.0    # [uint16] per-scan measurement noise

# Object threshold: I > I_seabed_est * OBJECT_INTENSITY_RATIO → object hit
OBJECT_INTENSITY_RATIO = 2.0    # objects ~2x brighter than seabed (~1060 threshold)

# Column attribution widths
SIGMA_SEABED         = 0.05     # Gaussian width for seabed regime [normalised t]
SIGMA_T              = 0.05     # Gaussian width for object regime  [normalised t]
W_MIN                = 0.01     # min column weight to create a voxel (cuts Gaussian tails)
                                 # with sigma=0.05, ~2-3 voxels per column instead of 20
INTENSITY_SCALE      = 2500.0   # [uint16] max expected intensity (observed max ~2200)

# ── BAYESIAN MAP UPDATE ───────────────────────────────────────────────────────
LAMBDA_HIT           = 0.1      # total α increment per hit beam
LAMBDA_MISS          = 0.1      # total β increment per miss beam

ALPHA_MIN            = 1e-3
BETA_MIN             = 1e-3

PUB_THRESHOLD        = 0.70      # min p to publish; voxels below this are deleted
MIN_HITS             = 1        # min hit count to publish

# ── SPATIAL WINDOW ────────────────────────────────────────────────────────────
WINDOW_RADIUS        = 200.0    # [m]
PRUNE_EVERY_N_SCANS  = 50

# ── TOPICS / FRAMES ───────────────────────────────────────────────────────────
INPUT_TOPIC          = 'sonar_scan'
OUTPUT_TOPIC         = 'sonar_map'
ODOM_TOPIC           = 'odometry'
MAP_FRAME            = 'odom'

PUBLISH_RATE         = 10.0     # [Hz]
SOURCE_ID            = 0.0

# ── PERSISTENCE ───────────────────────────────────────────────────────────────
SAVE_PATH            = '/home/rosdev/ros2_ws/saved_maps/voxels/sonar_map.sonarmap'
LOAD_PATH            = ''
AUTOSAVE_ON_SHUTDOWN = True
AUTOSAVE_PATH        = ''


# ══════════════════════════════════════════════════════════════════════════════
# INTERNALS
# ══════════════════════════════════════════════════════════════════════════════

_BITS    = 21
_MASK    = np.int64((1 << _BITS) - 1)
_SIGN    = 1 << (_BITS - 1)
_MAGIC   = 0x534F4E52
_VERSION = 13

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


def _read_xyz_intensity(msg: PointCloud2) -> np.ndarray:
    """Return (N,4) float32 [x,y,z,intensity]."""
    raw = pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                          skip_nans=True)
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


# ── Adaptive seabed estimator ─────────────────────────────────────────────────

class SeabedEstimator:
    """
    Dual scalar Kalman filter that estimates seabed depth h and seabed
    intensity I_seabed online from sonar returns.

    State:  mu_h,  var_h   — seabed depth below USV [m]
            mu_I,  var_I   — typical seabed intensity [uint16]

    Each scan:
      1. Prediction step: add process noise (allows h and I to drift)
      2. Identify seabed candidates: returns inside depth band AND
         intensity below current I estimate
      3. If enough candidates: Kalman update on median depth and median I

    The band used to gate the map update is:
      d_lo = mu_h - tallest_object
      d_hi = mu_h + 2*std_h        (wider when uncertain, tightens over time)

    The seabed/object threshold used for column attribution:
      I_seabed = mu_I               (seabed regime if I < mu_I * object_ratio)
      I_object  = mu_I * object_ratio
    """

    def __init__(
        self,
        h_mean:            float,
        h_std:             float,
        h_process_noise:   float,
        h_meas_noise:      float,
        h_min_candidates:  int,
        I_mean:            float,
        I_std:             float,
        I_process_noise:   float,
        I_meas_noise:      float,
        tallest_object:    float,
        object_ratio:      float,
    ):
        # depth state
        self.mu_h   = float(h_mean)
        self.var_h  = float(h_std) ** 2
        self.q_h    = float(h_process_noise) ** 2
        self.r_h    = float(h_meas_noise) ** 2
        self.n_min  = int(h_min_candidates)

        # intensity state
        self.mu_I   = float(I_mean)
        self.var_I  = float(I_std) ** 2
        self.q_I    = float(I_process_noise) ** 2
        self.r_I    = float(I_meas_noise) ** 2

        self.hh           = float(tallest_object)
        self.object_ratio = float(object_ratio)

        # scan counter for logging
        self._scan_count  = 0

    # ── properties exposed to the map ─────────────────────────────────────────

    @property
    def d_lo(self) -> float:
        """Bottom of valid band. Use a generous lower bound until h converges."""
        return max(0.0, self.mu_h - self.hh - self.std_h)

    @property
    def d_hi(self) -> float:
        """Top of valid band — seabed + uncertainty margin."""
        return self.mu_h + 2.0 * self.std_h

    @property
    def std_h(self) -> float:
        return float(np.sqrt(self.var_h))

    @property
    def std_I(self) -> float:
        return float(np.sqrt(self.var_I))

    @property
    def seabed_threshold(self) -> float:
        """I < this → seabed hit. Floor at 1.0 to avoid zero threshold."""
        return max(self.mu_I * self.object_ratio, 1.0)

    # ── update ────────────────────────────────────────────────────────────────

    def update(self, depths: np.ndarray, intensities: np.ndarray) -> dict:
        """
        Update seabed depth and intensity estimates from one scan.

        Parameters
        ----------
        depths      : (N,) positive-downward depths of ALL returns
        intensities : (N,) corresponding raw intensities

        Zero-intensity points are excluded from the intensity estimator
        (they carry no intensity information) but their depths are still
        used for the depth estimator if they fall in the expected band.
        """
        self._scan_count += 1

        # ── Prediction: add process noise ─────────────────────────────────
        self.var_h += self.q_h
        self.var_I += self.q_I

        # ── Identify seabed candidates ────────────────────────────────────
        nonzero = intensities > 0.0

        # Depth: use ALL returns — 85th percentile finds the seabed regardless
        # of band convergence. Band gate was self-referential and kept h stuck.
        n_depth = len(depths)

        # Intensity: all non-zero returns not brighter than seabed + 2*std
        inten_candidates = nonzero & (intensities <= (self.mu_I + 2.0 * self.std_I))
        n_inten = int(inten_candidates.sum())

        updated_h = False
        updated_I = False

        if n_depth >= self.n_min:
            # 85th percentile of ALL depths = robust seabed estimate
            z_meas = float(np.percentile(depths, 85))
            K_h        = self.var_h / (self.var_h + self.r_h)
            self.mu_h  = self.mu_h + K_h * (z_meas - self.mu_h)
            self.var_h = (1.0 - K_h) * self.var_h
            updated_h  = True

        if n_inten >= self.n_min:
            I_meas = float(np.median(intensities[inten_candidates]))
            K_I        = self.var_I / (self.var_I + self.r_I)
            self.mu_I  = self.mu_I + K_I * (I_meas - self.mu_I)
            self.var_I = (1.0 - K_I) * self.var_I
            updated_I  = True

        return {
            'n_depth_cand': n_depth,
            'n_inten_cand': n_inten,
            'updated':      updated_h or updated_I,
            'mu_h':         self.mu_h,
            'std_h':        self.std_h,
            'mu_I':         self.mu_I,
            'std_I':        self.std_I,
            'd_lo':         self.d_lo,
            'd_hi':         self.d_hi,
            'seabed_thr':   self.seabed_threshold,
        }


# ── Column weight function ─────────────────────────────────────────────────────

def _column_weights(t: np.ndarray, I_norm: float, is_seabed: bool,
                    sigma_seabed: float, sigma_t: float) -> np.ndarray:
    """
    Column weight function.

    SEABED regime (is_seabed=True):
        Symmetric Gaussian centred at t=0 (seabed level).
        w(t) = exp( -t^2 / (2*sigma_seabed^2) )

    OBJECT regime (is_seabed=False):
        One-sided distribution: objects rest on the seabed, so all voxels
        between the seabed and the detected object height are occupied.
        - Below peak (t < t_peak): weight = 1.0  (column is filled)
        - Above peak (t >= t_peak): Gaussian decay  (uncertainty above object)
        Then normalised so sum = 1.

    Both normalised so sum(w) = 1.
    """
    if is_seabed:
        w = np.exp(-0.5 * (t / sigma_seabed) ** 2)
    else:
        t_peak = I_norm
        w = np.where(
            t < t_peak,
            np.ones_like(t),                                          # filled below
            np.exp(-0.5 * ((t - t_peak) / sigma_t) ** 2)             # Gaussian above
        )
    s = w.sum()
    if s < 1e-12:
        return np.ones_like(t) / len(t)
    return w / s


# ── Preallocated voxel store ───────────────────────────────────────────────────

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

    # ── hit update (column-spread) ────────────────────────────────────────────

    def update_hits(self,
                    keys:   np.ndarray,   # (K,) int64  — one entry per voxel in column
                    dalpha: np.ndarray,   # (K,) float64 — weighted α increments
                    inten:  np.ndarray):  # (K,) float64 — mean intensity (same for column)
        """
        Apply weighted α increments to K voxels (may span multiple columns).
        New voxels are created as needed.
        """
        am  = self.alpha_min
        bm  = self.beta_min
        idx = self._index
        ka  = self.keys_arr
        da  = self.data_arr
        n   = self._n

        for i in range(len(keys)):
            key = int(keys[i])
            inc = float(dalpha[i])
            it  = float(inten[i])
            row = idx.get(key, -1)
            if row == -1:
                if n >= len(ka):
                    self._grow()
                    ka = self.keys_arr
                    da = self.data_arr
                ka[n]    = key
                da[n, 0] = am + inc   # alpha
                da[n, 1] = bm         # beta  (fixed at creation)
                da[n, 2] = it         # sumI
                da[n, 3] = 1.0        # hits
                idx[key] = n
                n += 1
            else:
                da[row, 0] += inc
                da[row, 2] += it
                da[row, 3] += 1.0

        self._n = n

    # ── miss update (column-spread) ───────────────────────────────────────────

    def update_misses(self,
                      keys:          np.ndarray,  # (M,) int64 — existing voxel keys
                      dbeta:         np.ndarray,  # (M,) float64 — weighted β increments
                      pub_threshold: float) -> int:
        """
        Apply weighted β increments to existing voxels.
        Voxels whose p = α/(α+β) drops below pub_threshold are deleted.
        Returns the number of voxels deleted.
        """
        idx       = self._index
        da        = self.data_arr
        to_delete = []

        for i in range(len(keys)):
            key = int(keys[i])
            row = idx.get(key, -1)
            if row == -1:
                continue
            da[row, 1] += float(dbeta[i])
            alpha = da[row, 0]
            beta  = da[row, 1]
            p     = alpha / max(alpha + beta, 1e-9)
            if p < pub_threshold:
                to_delete.append(key)

        for key in to_delete:
            row = idx.pop(key, -1)
            if row == -1:
                continue
            last = self._n - 1
            if row != last:
                last_key           = int(self.keys_arr[last])
                self.keys_arr[row] = self.keys_arr[last]
                self.data_arr[row] = self.data_arr[last]
                idx[last_key]      = row
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
        if version not in (2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13):
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
    Two-sided Beta-Bernoulli map with adaptive seabed estimator (v12).

    Seabed depth h and seabed intensity I_seabed are estimated online via
    dual scalar Kalman filters. Only TALLEST_OBJECT is fixed by the user.

    For each sonar beam at projected depth z0:
      - SeabedEstimator updates its estimate of h and I_seabed
      - Band [d_lo, d_hi] is computed from the current h estimate
      - All voxels in the column are updated with Gaussian weights
      - Two regimes: SEABED (I < I_seabed_est) and OBJECT (I >= I_seabed_est)
    """

    def __init__(
        self,
        voxel_size:             float,
        tallest_object:         float,
        lambda_hit:             float,
        lambda_miss:            float,
        alpha_min:              float,
        beta_min:               float,
        intensity_scale:        float,
        pub_threshold:          float,
        sigma_seabed:           float,
        sigma_t:                float,
        h_prior_mean:           float,
        h_prior_std:            float,
        h_process_noise:        float,
        h_meas_noise:           float,
        h_min_candidates:       int,
        I_prior_mean:           float,
        I_prior_std:            float,
        I_process_noise:        float,
        I_meas_noise:           float,
        object_intensity_ratio: float,
        w_min:                  float = 0.01,
    ):
        self.vs     = float(voxel_size)
        self.inv_vs = 1.0 / self.vs
        self.hh     = float(tallest_object)

        self.lambda_hit    = float(lambda_hit)
        self.lambda_miss   = float(lambda_miss)
        self.I_scale_      = max(float(intensity_scale), 1.0)
        self.pub_threshold_= float(pub_threshold)
        self.sigma_seabed_ = max(float(sigma_seabed), 1e-6)
        self.sigma_t_      = max(float(sigma_t), 1e-6)
        self.w_min_        = float(w_min)

        # Adaptive seabed estimator
        self.estimator = SeabedEstimator(
            h_mean           = h_prior_mean,
            h_std            = h_prior_std,
            h_process_noise  = h_process_noise,
            h_meas_noise     = h_meas_noise,
            h_min_candidates = h_min_candidates,
            I_mean           = I_prior_mean,
            I_std            = I_prior_std,
            I_process_noise  = I_process_noise,
            I_meas_noise     = I_meas_noise,
            tallest_object   = tallest_object,
            object_ratio     = object_intensity_ratio,
        )

        # Column precomputation — rebuilt whenever band changes
        self._n_col   = 0
        self._col_dz  = np.empty(0)
        self._col_t   = np.empty(0)
        self._last_hh = -1.0
        self._rebuild_column()

        self._store = VoxelStore(alpha_min, beta_min)

    def _rebuild_column(self):
        n = max(1, int(round(self.hh / self.vs)))
        if n == self._n_col:
            return
        self._n_col  = n
        self._col_dz = np.arange(n, dtype=np.float64) * self.vs
        self._col_t  = self._col_dz / self.hh

    def __len__(self) -> int:
        return len(self._store)

        self.lambda_hit        = float(lambda_hit)
        self.lambda_miss       = float(lambda_miss)
        self.min_intensity_    = float(min_intensity)
        self.seabed_threshold_ = float(seabed_threshold)
    # ── insert one scan ───────────────────────────────────────────────────────

    def insert_scan(self, origin_xyz: np.ndarray, pts_xyzi: np.ndarray) -> dict:
        """
        origin_xyz : (3,)  float64  USV position in ENU odom frame (z ≈ 0)
        pts_xyzi   : (N,4) float32  ALL sonar returns [x, y, z, intensity]
        """
        if pts_xyzi.size == 0:
            return {'hits': 0, 'misses': 0, 'deleted': 0,
                    'col_voxels': 0, 'estimator': {}}

        # ── Depth: ENU frame — z is positive upward ──────────────────────
        # depth = usv_z - point_z  (positive downward)
        sz     = float(origin_xyz[2])
        depths = sz - pts_xyzi[:, 2].astype(np.float64)   # positive downward
        intens = pts_xyzi[:, 3].astype(np.float64)

        # ── Zero-intensity points: real returns but no intensity data ──────
        # Treat as misses — they confirm something is there but can't
        # tell us what. Exclude from estimator intensity update.
        has_intensity = intens > 0.0

        # ── Step 1: update seabed estimator ───────────────────────────────
        est_stats = self.estimator.update(depths, intens)

        # ── Step 2: get current band and thresholds ────────────────────────
        d_lo       = self.estimator.d_lo
        d_hi       = self.estimator.d_hi
        seabed_thr = self.estimator.seabed_threshold
        mu_I       = self.estimator.mu_I

        # ── Step 3: gate to band ───────────────────────────────────────────
        in_band = (depths >= d_lo) & (depths <= d_hi)
        if not np.any(in_band):
            return {'hits': 0, 'misses': 0, 'deleted': 0,
                    'col_voxels': 0, 'estimator': est_stats}

        band_pts   = pts_xyzi[in_band]
        band_inten = intens[in_band]
        band_has_I = has_intensity[in_band]

        # ── Step 4: split hits / misses ────────────────────────────────────
        # Hit:  has intensity AND I >= mu_I
        # Miss: no intensity (I=0) OR I < mu_I
        is_hit  = band_has_I & (band_inten >= mu_I)
        is_miss = ~is_hit

        stats = {'hits': 0, 'misses': 0, 'deleted': 0,
                 'col_voxels': 0, 'estimator': est_stats}

        inv_vs = self.inv_vs
        col_dz = self._col_dz
        col_t  = self._col_t

        # ══════════════════════════════════════════════════════════════════
        # HIT UPDATE — spread α across column with two-regime weights
        # ══════════════════════════════════════════════════════════════════
        if np.any(is_hit):
            hit_pts   = band_pts[is_hit]
            hit_inten = band_inten[is_hit]

            hit_ix0 = np.floor(hit_pts[:, 0] * inv_vs).astype(np.int64)
            hit_iy0 = np.floor(hit_pts[:, 1] * inv_vs).astype(np.int64)
            hit_iz0 = np.floor(hit_pts[:, 2] * inv_vs).astype(np.int64)
            col_key = hit_ix0 * (2**42) + hit_iy0 * (2**21) + hit_iz0

            uniq_col, inv_idx, counts = np.unique(
                col_key, return_inverse=True, return_counts=True)
            n_uniq = len(uniq_col)

            mean_I  = np.zeros(n_uniq); mean_x = np.zeros(n_uniq)
            mean_y  = np.zeros(n_uniq); mean_z0= np.zeros(n_uniq)
            np.add.at(mean_I,  inv_idx, hit_inten)
            np.add.at(mean_x,  inv_idx, hit_pts[:, 0].astype(np.float64))
            np.add.at(mean_y,  inv_idx, hit_pts[:, 1].astype(np.float64))
            np.add.at(mean_z0, inv_idx, hit_pts[:, 2].astype(np.float64))
            mean_I /= counts; mean_x /= counts
            mean_y /= counts; mean_z0 /= counts

            total_col = n_uniq * self._n_col   # upper bound
            all_keys   = np.empty(total_col, dtype=np.int64)
            all_dalpha = np.empty(total_col, dtype=np.float64)
            all_inten  = np.empty(total_col, dtype=np.float64)

            idx_out = 0
            for b in range(n_uniq):
                I_b   = mean_I[b]
                x0    = mean_x[b]; y0 = mean_y[b]; z0 = mean_z0[b]
                col_z = z0 + col_dz

                ix_col = np.floor(x0    * inv_vs).astype(np.int64)
                iy_col = np.floor(y0    * inv_vs).astype(np.int64)
                iz_col = np.floor(col_z * inv_vs).astype(np.int64)
                pk_col = _pack(
                    np.full(self._n_col, ix_col, dtype=np.int64),
                    np.full(self._n_col, iy_col, dtype=np.int64),
                    iz_col)

                is_seabed = I_b < seabed_thr
                I_norm    = float(np.clip(I_b / self.I_scale_, 0.0, 1.0))
                w = _column_weights(col_t, I_norm, is_seabed,
                                    self.sigma_seabed_, self.sigma_t_)

                # Only update voxels that receive meaningful weight
                # Prevents creating ghost voxels at Gaussian tails
                significant = w >= self.w_min_
                w_sig = w[significant]
                w_sig /= w_sig.sum()   # renormalise so total evidence = lambda_hit

                n_sig  = int(significant.sum())
                pk_sig = pk_col[significant]

                all_keys  [idx_out:idx_out + n_sig] = pk_sig
                all_dalpha[idx_out:idx_out + n_sig] = self.lambda_hit * w_sig
                all_inten [idx_out:idx_out + n_sig] = I_b
                idx_out += n_sig

            self._store.update_hits(all_keys[:idx_out], all_dalpha[:idx_out], all_inten[:idx_out])
            stats['hits']       = n_uniq
            stats['col_voxels'] = idx_out

        # ══════════════════════════════════════════════════════════════════
        # MISS UPDATE — uniform β across existing voxels in column
        # ══════════════════════════════════════════════════════════════════
        if np.any(is_miss):
            miss_pts = band_pts[is_miss]
            miss_ix0 = np.floor(miss_pts[:, 0] * inv_vs).astype(np.int64)
            miss_iy0 = np.floor(miss_pts[:, 1] * inv_vs).astype(np.int64)
            miss_iz0 = np.floor(miss_pts[:, 2] * inv_vs).astype(np.int64)
            col_key  = miss_ix0 * (2**42) + miss_iy0 * (2**21) + miss_iz0

            miss_keys_list  = []
            miss_dbeta_list = []
            for col_k in np.unique(col_key):
                mask   = col_key == col_k
                x0     = float(miss_pts[mask, 0].mean())
                y0     = float(miss_pts[mask, 1].mean())
                z0     = float(miss_pts[mask, 2].mean())
                col_z  = z0 + col_dz
                ix_col = np.floor(x0    * inv_vs).astype(np.int64)
                iy_col = np.floor(y0    * inv_vs).astype(np.int64)
                iz_col = np.floor(col_z * inv_vs).astype(np.int64)
                pk_col = _pack(
                    np.full(self._n_col, ix_col, dtype=np.int64),
                    np.full(self._n_col, iy_col, dtype=np.int64),
                    iz_col)
                w_u = np.ones(self._n_col) / self._n_col
                for i in range(self._n_col):
                    k = int(pk_col[i])
                    if self._store._index.get(k, -1) != -1:
                        miss_keys_list.append(k)
                        miss_dbeta_list.append(self.lambda_miss * w_u[i])

            if miss_keys_list:
                mk = np.array(miss_keys_list,  dtype=np.int64)
                mb = np.array(miss_dbeta_list, dtype=np.float64)
                deleted = self._store.update_misses(mk, mb, self.pub_threshold_)
                stats['misses']  = len(np.unique(col_key))
                stats['deleted'] = deleted

        return stats

    # ── spatial prune ─────────────────────────────────────────────────────────

    def prune_outside_radius(self, cx: float, cy: float, radius: float) -> int:
        return self._store.prune_outside_radius(cx, cy, radius, self.vs)

    # ── query ─────────────────────────────────────────────────────────────────

    def to_numpy(self, min_hits: int) -> np.ndarray:
        """Return (N,5) float32: [x, y, z, occupancy_prob, intensity_mean]."""
        n = len(self._store)
        if n == 0:
            return np.empty((0, 5), dtype=np.float32)
        keys  = self._store.live_keys()
        data  = self._store.live_data()
        alpha = data[:, 0]
        beta  = data[:, 1]
        prob  = alpha / np.maximum(alpha + beta, 1e-9)
        hits  = data[:, 3]
        mask  = (prob >= self.pub_threshold_) & (hits >= float(min_hits))
        if not np.any(mask):
            return np.empty((0, 5), dtype=np.float32)
        k   = keys[mask]
        d   = data[mask]
        p   = prob[mask]
        vs  = self.vs
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

        self.declare_parameter('input_topic',            INPUT_TOPIC)
        self.declare_parameter('output_topic',           OUTPUT_TOPIC)
        self.declare_parameter('odom_topic',             ODOM_TOPIC)
        self.declare_parameter('map_frame',              MAP_FRAME)
        self.declare_parameter('voxel_size',             VOXEL_SIZE)
        self.declare_parameter('tallest_object',         TALLEST_OBJECT)
        # Seabed estimator
        self.declare_parameter('h_prior_mean',           H_PRIOR_MEAN)
        self.declare_parameter('h_prior_std',            H_PRIOR_STD)
        self.declare_parameter('h_process_noise',        H_PROCESS_NOISE)
        self.declare_parameter('h_meas_noise',           H_MEAS_NOISE)
        self.declare_parameter('h_min_candidates',       H_MIN_CANDIDATES)
        self.declare_parameter('I_prior_mean',           I_PRIOR_MEAN)
        self.declare_parameter('I_prior_std',            I_PRIOR_STD)
        self.declare_parameter('I_process_noise',        I_PROCESS_NOISE)
        self.declare_parameter('I_meas_noise',           I_MEAS_NOISE)
        self.declare_parameter('object_intensity_ratio', OBJECT_INTENSITY_RATIO)
        # Map
        self.declare_parameter('intensity_scale',        INTENSITY_SCALE)
        self.declare_parameter('sigma_seabed',           SIGMA_SEABED)
        self.declare_parameter('sigma_t',                SIGMA_T)
        self.declare_parameter('w_min',                  W_MIN)
        self.declare_parameter('lambda_hit',             LAMBDA_HIT)
        self.declare_parameter('lambda_miss',            LAMBDA_MISS)
        self.declare_parameter('alpha_min',              ALPHA_MIN)
        self.declare_parameter('beta_min',               BETA_MIN)
        self.declare_parameter('pub_threshold',          PUB_THRESHOLD)
        self.declare_parameter('min_hits',               MIN_HITS)
        self.declare_parameter('window_radius',          WINDOW_RADIUS)
        self.declare_parameter('prune_every_n_scans',    PRUNE_EVERY_N_SCANS)
        self.declare_parameter('publish_rate',           PUBLISH_RATE)
        self.declare_parameter('source_id',              SOURCE_ID)
        self.declare_parameter('save_path',              SAVE_PATH)
        self.declare_parameter('load_path',              LOAD_PATH)
        self.declare_parameter('autosave_on_shutdown',   AUTOSAVE_ON_SHUTDOWN)
        self.declare_parameter('autosave_path',          AUTOSAVE_PATH)

        def gp(n): return self.get_parameter(n).value

        voxel_size   = float(gp('voxel_size'))
        tallest_obj  = float(gp('tallest_object'))
        publish_rate = float(gp('publish_rate'))

        self.map_frame_            = gp('map_frame')
        self.window_radius_        = float(gp('window_radius'))
        self.min_hits_             = int(gp('min_hits'))
        self.source_id_            = float(gp('source_id'))
        self.save_path_            = str(gp('save_path'))
        self.autosave_path_        = str(gp('autosave_path'))
        self.autosave_on_shutdown_ = bool(gp('autosave_on_shutdown'))
        self.prune_every_n_scans_  = max(1, int(gp('prune_every_n_scans')))

        self.voxel_map_ = ProbabilisticVoxelMap(
            voxel_size             = voxel_size,
            tallest_object         = tallest_obj,
            lambda_hit             = float(gp('lambda_hit')),
            lambda_miss            = float(gp('lambda_miss')),
            alpha_min              = float(gp('alpha_min')),
            beta_min               = float(gp('beta_min')),
            intensity_scale        = float(gp('intensity_scale')),
            pub_threshold          = float(gp('pub_threshold')),
            sigma_seabed           = float(gp('sigma_seabed')),
            sigma_t                = float(gp('sigma_t')),
            w_min                  = float(gp('w_min')),
            h_prior_mean           = float(gp('h_prior_mean')),
            h_prior_std            = float(gp('h_prior_std')),
            h_process_noise        = float(gp('h_process_noise')),
            h_meas_noise           = float(gp('h_meas_noise')),
            h_min_candidates       = int(gp('h_min_candidates')),
            I_prior_mean           = float(gp('I_prior_mean')),
            I_prior_std            = float(gp('I_prior_std')),
            I_process_noise        = float(gp('I_process_noise')),
            I_meas_noise           = float(gp('I_meas_noise')),
            object_intensity_ratio = float(gp('object_intensity_ratio')),
        )

        self.vehicle_pose_ = None
        self._scan_count_  = 0

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

        self.map_pub_    = self.create_publisher(PointCloud2, gp('output_topic'), 10)
        latched_qos      = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.kdtree_pub_ = self.create_publisher(PointCloud2, 'sonar_map_for_kdtree', latched_qos)
        self.cloud_sub_  = self.create_subscription(PointCloud2, gp('input_topic'), self.cloud_callback, 10)
        self.odom_sub_   = self.create_subscription(Odometry, gp('odom_topic'), self.odom_callback, 10)
        self.save_srv_   = self.create_service(Trigger, 'sonar_map/save', self._save_cb)
        self.load_srv_   = self.create_service(Trigger, 'sonar_map/load', self._load_cb)
        self.timer_      = self.create_timer(1.0 / publish_rate, self.publish_map)

        load_path = str(gp('load_path'))
        if load_path:
            self._do_load(load_path)

        est = self.voxel_map_.estimator
        self.get_logger().info(
            f'ProbSonarMap v13 — ENU frame, adaptive estimator | '
            f'voxel={voxel_size}m tallest={tallest_obj}m | '
            f'h_prior={est.mu_h:.2f}m ±{est.std_h:.2f}m | '
            f'I_prior={est.mu_I:.0f} ±{est.std_I:.0f} | '
            f'object_ratio={float(gp("object_intensity_ratio"))} | '
            f'scale={float(gp("intensity_scale")):.0f} | '
            f'lambda_hit={float(gp("lambda_hit"))} lambda_miss={float(gp("lambda_miss"))} | '
            f'pub_threshold={float(gp("pub_threshold"))} rate={publish_rate}Hz'
        )


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

    def _save_cb(self, _req, resp):
        if not self.save_path_:
            resp.success = False; resp.message = 'save_path is empty'; return resp
        resp.success, resp.message = self._do_save(self.save_path_)
        return resp

    def _load_cb(self, _req, resp):
        path = self.save_path_ or self.get_parameter('load_path').value
        if not path:
            resp.success = False; resp.message = 'No path set'; return resp
        resp.success, resp.message = self._do_load(path)
        return resp

    def _do_save(self, path):
        try:
            t0 = time.monotonic()
            n  = self.voxel_map_.save(path)
            dt = time.monotonic() - t0
            msg = f'Saved {n} voxels to {path} in {dt*1000:.0f} ms'
            self.get_logger().info(msg); return True, msg
        except Exception as e:
            msg = f'Save failed: {e}'; self.get_logger().error(msg); return False, msg

    def _do_load(self, path):
        try:
            t0 = time.monotonic()
            n  = self.voxel_map_.load(path)
            dt = time.monotonic() - t0
            msg = (f'Loaded {n} voxels from {path} in {dt*1000:.0f} ms | '
                   f'map={len(self.voxel_map_)} voxels')
            self.get_logger().info(msg); return True, msg
        except Exception as e:
            msg = f'Load failed: {e}'; self.get_logger().error(msg); return False, msg

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.vehicle_pose_ = (p.x, p.y, p.z)

    def cloud_callback(self, msg: PointCloud2):
        if self.vehicle_pose_ is None:
            self.get_logger().warn('No odometry yet — skipping scan.',
                                   throttle_duration_sec=5.0)
            return
        try:
            pts = _read_xyz_intensity(msg)
        except Exception as exc:
            self.get_logger().error(f'Failed to read PointCloud2: {exc}',
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
        est = self.voxel_map_.estimator
        if self._scan_count_ <= 5 or self._scan_count_ % 50 == 0:
            # ── Raw depth diagnostic ───────────────────────────────────────
            sz = float(self.vehicle_pose_[2])
            raw_depths = sz - pts[:, 2].astype(np.float64)
            self.get_logger().info(
                f'  [DEPTH_DIAG] usv_z={sz:.3f}m | '
                f'raw_depth: min={raw_depths.min():.2f} max={raw_depths.max():.2f} '
                f'mean={raw_depths.mean():.2f}m'
            )
            e = stats.get('estimator', {})
            self.get_logger().info(
                f'Scan #{self._scan_count_}: {len(pts)} pts | '
                f'hit_beams={stats["hits"]} col_voxels={stats["col_voxels"]} '
                f'miss_beams={stats["misses"]} deleted={stats["deleted"]} | '
                f'insert={dt*1000:.1f}ms | map={len(self.voxel_map_)} voxels'
            )
            # ── Estimator log ──────────────────────────────────────────────
            n_dcand = e.get('n_depth_cand', 0)
            n_icand = e.get('n_inten_cand', 0)
            updated_str = 'UPDATED' if e.get('updated') else f'no update'
            self.get_logger().info(
                f'  [ESTIMATOR] {updated_str} | '
                f'h={est.mu_h:.3f}m ±{est.std_h:.3f}m | '
                f'band=[{est.d_lo:.2f}, {est.d_hi:.2f}]m | '
                f'I_seabed={est.mu_I:.0f} ±{est.std_I:.0f} | '
                f'seabed_thr={est.seabed_threshold:.0f} | '
                f'd_cand={n_dcand} I_cand={n_icand}'
            )

        if self._scan_count_ % self.prune_every_n_scans_ == 0:
            cx, cy, _ = self.vehicle_pose_
            removed = self.voxel_map_.prune_outside_radius(cx, cy, self.window_radius_)
            if removed:
                self.get_logger().info(
                    f'Pruned {removed} voxels | map={len(self.voxel_map_)}')

    def publish_map(self):
        n_vox = len(self.voxel_map_)
        if n_vox == 0:
            self.get_logger().info(
                f'Map empty — scans={self._scan_count_} '
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
                        f'depth=[{depths.min():.2f}, {depths.max():.2f}]m | '
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