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
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy.lib.recfunctions as rfn
from scipy.special import ndtr   # standard normal CDF for Gaussian occupancy probability


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
H_MIN_CANDIDATES     = 10       # min seabed-confirmed points per scan to trigger update
H_MAX_CANDIDATES     = 300      # max expected seabed-confirmed points (static, for noise scaling)

# Prior on seabed intensity I_seabed
# BlueView M900 observed: mean~530, max~2200, ~half of points have I>0
I_PRIOR_MEAN         = 530.0    # [uint16] initial guess (observed mean of non-zero returns)
I_PRIOR_STD          = 300.0    # [uint16] initial uncertainty
I_PROCESS_NOISE      = 30.0     # [uint16] intensity is fairly stable
I_MEAS_NOISE         = 150.0    # [uint16] per-scan measurement noise
I_PERCENTILE         = 60       # [%] percentile of shallow non-zero returns used as I_meas
                                 # low value (25-35) targets the dominant low-intensity seabed cluster

# Object threshold: I > I_seabed_est * OBJECT_INTENSITY_RATIO → object hit
OBJECT_INTENSITY_RATIO = 1.2    # objects ~2x brighter than seabed (~1060 threshold)

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
PCD_SAVE_DIR         = '/home/rosdev/ros2_ws/saved_maps/PCD'

# ── BATHYMETRY MAP ─────────────────────────────────────────────────────────────
# Sparse 2-D Kalman-filter map that estimates seabed depth per (x,y) cell.
# Resolution is coarser than the voxel grid; each cell aggregates multiple returns.
# All noise values are standard deviations in metres (squared internally).
BATHY_CELL_SIZE    = 0.5    # [m]   horizontal cell size
BATHY_Z_PRIOR_MEAN = -2.5   # [m]   initial depth guess (ENU frame, negative = below vehicle)
BATHY_Z_PRIOR_STD  = 0.5    # [m]   initial depth uncertainty
BATHY_Q_Z          = 0.01   # [m]   process noise std  (seabed is static → keep small)
BATHY_R_Z          = 0.05   # [m]   measurement noise std per sonar return (~5 cm)

# ── STRUCTURE UPDATE ───────────────────────────────────────────────────────────
# A return is classified as STRUCTURE if:
#   (a) I >= lambda_structure * mu_I_seabed  (above adaptive seabed intensity baseline)
#   (b) has LiDAR support within epsilon_struct XY radius
# Update: single voxel alpha increment at the return point zi only.
LIDAR_TOPIC        = '/map_visualization'
EPSILON_STRUCT     = 0.5    # [m]   XY radius for LiDAR neighbourhood query
ELLIPSOID_A        = 5.0    # [m]   USV exclusion ellipsoid semi-axis in X
ELLIPSOID_B        = 5.0    # [m]   USV exclusion ellipsoid semi-axis in Y
ELLIPSOID_C        = 2.0    # [m]   USV exclusion ellipsoid semi-axis in Z
SEABED_VOXEL_SIZE  = 0.5    # [m] coarse voxel size for seabed (larger = fewer voxels)
LAMBDA_STRUCTURE   = 1.5    # structure threshold multiplier on estimated seabed intensity

# ── OBJECT UPDATE ──────────────────────────────────────────────────────────────
# A return is classified as OBJECT if:
#   (a) I >= lambda_object * mu_I_seabed  (above adaptive object intensity threshold)
#   (b) NOT classified as structure (no LiDAR support)
# The update fills two regions (same (xi, yi) column):
#   1. BODY   — flat α increment from seabed_z up to zi  (the object occupies this space)
#   2. ABOVE  — Gaussian-weighted α from zi upward       (uncertain object height)
# α weight = λ_hit · min(I / (lambda_object * mu_I), 3)  — intensity-normalised, capped at 3×.
LAMBDA_OBJECT      = 3.0     # object threshold multiplier on estimated seabed intensity
SIGMA_OBJECT       = 0.15    # [m]      Gaussian sigma for uncertainty above return height zi


# ══════════════════════════════════════════════════════════════════════════════
# INTERNALS
# ══════════════════════════════════════════════════════════════════════════════

_BITS    = 21
_MASK    = np.int64((1 << _BITS) - 1)
_SIGN    = 1 << (_BITS - 1)
_MAGIC   = 0x534F4E52
_VERSION = 14

# v14 adds 'semantic' column; v13 and earlier do not have it.
_RECORD_DTYPE = np.dtype([
    ('key',      np.int64),
    ('alpha',    np.float32),
    ('beta',     np.float32),
    ('sumI',     np.float32),
    ('hits',     np.float32),
    ('semantic', np.float32),
])
_RECORD_DTYPE_V13 = np.dtype([
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
        h_max_candidates:  int,
        I_mean:            float,
        I_std:             float,
        I_process_noise:   float,
        I_meas_noise:      float,
        I_percentile:      int,
        tallest_object:    float,
        object_ratio:      float,
    ):
        # depth state
        self.mu_h   = float(h_mean)
        self.var_h  = float(h_std) ** 2
        self.q_h    = float(h_process_noise) ** 2
        self.r_h    = float(h_meas_noise) ** 2
        self.n_min  = int(h_min_candidates)
        self.m_max  = max(1, int(h_max_candidates))

        # intensity state
        self.mu_I        = float(I_mean)
        self.var_I       = float(I_std) ** 2
        self.q_I         = float(I_process_noise) ** 2
        self.r_I         = float(I_meas_noise) ** 2
        self.I_percentile = int(np.clip(I_percentile, 1, 99))

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

        # ── Shallow non-zero candidates (used for both updates) ──────────────
        nonzero      = intensities > 0.0
        z_85         = float(np.percentile(depths, 85))
        shallow_mask = depths <= z_85
        i_candidates = shallow_mask & nonzero          # all shallow non-zero returns
        n_i          = int(i_candidates.sum())

        updated_h = False
        updated_I = False

        # ── Step 1: intensity update (runs first, no intensity gate needed) ──
        # I_meas = low percentile of shallow non-zero returns.
        # Targets the dominant low-intensity cluster (seabed) without needing
        # to know the threshold in advance.
        if n_i >= self.n_min:
            I_meas       = float(np.percentile(intensities[i_candidates], self.I_percentile))
            r_I_adaptive = self.r_I * (self.m_max / n_i)
            K_I          = self.var_I / (self.var_I + r_I_adaptive)
            self.mu_I    = self.mu_I + K_I * (I_meas - self.mu_I)
            self.var_I   = (1.0 - K_I) * self.var_I
            updated_I    = True

        # ── Step 2: depth update (uses the freshly updated seabed_threshold) ─
        # seabed_mask now uses the current-scan intensity estimate.
        seabed_mask  = shallow_mask & nonzero & (intensities < self.seabed_threshold)
        m            = int(seabed_mask.sum())

        if m >= self.n_min:
            z_meas       = float(np.mean(depths[seabed_mask]))
            r_h_adaptive = self.r_h * (self.m_max / m)
            K_h          = self.var_h / (self.var_h + r_h_adaptive)
            self.mu_h    = self.mu_h + K_h * (z_meas - self.mu_h)
            self.var_h   = (1.0 - K_h) * self.var_h
            updated_h    = True

        return {
            'n_depth_cand': m,
            'n_inten_cand': n_i,
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
      data_arr : (capacity, 5) float32  [alpha, beta, sumI, hits, semantic]
                                         semantic: 0.0 = SEABED, 1.0 = STRUCTURE
      _index   : dict[int -> int]       key -> row
      _n       : int                    number of live voxels
    """

    _INIT_CAP = 8_192

    def __init__(self, alpha_min: float, beta_min: float):
        self.alpha_min = np.float32(alpha_min)
        self.beta_min  = np.float32(beta_min)
        cap = self._INIT_CAP
        self.keys_arr = np.empty(cap, dtype=np.int64)
        self.data_arr = np.zeros((cap, 5), dtype=np.float32)
        self._index: Dict[int, int] = {}
        self._n = 0

    def _grow(self):
        old_cap = len(self.keys_arr)
        new_cap = old_cap * 2
        new_keys = np.empty(new_cap, dtype=np.int64)
        new_data = np.zeros((new_cap, 5), dtype=np.float32)
        new_keys[:old_cap] = self.keys_arr
        new_data[:old_cap] = self.data_arr
        self.keys_arr = new_keys
        self.data_arr = new_data

    def __len__(self) -> int:
        return self._n

    # ── hit update ────────────────────────────────────────────────────────────

    def update_hits(self,
                    keys:       np.ndarray,         # (K,) int64
                    dalpha:     np.ndarray,          # (K,) float64  weighted α increments
                    inten:      np.ndarray,          # (K,) float64  intensity values (sum or mean)
                    semantic:   np.float32 = np.float32(0.0),
                    raw_counts: np.ndarray = None):  # (K,) int — raw point count per key
        """
        Vectorised α update.  Duplicate keys are aggregated before lookup.
        Structure label (1.0) takes priority: once marked structure, stays structure.
        New voxels are created as needed.

        raw_counts: if provided, used as the hits increment instead of 1-per-call.
        Pass the raw number of sonar returns per voxel so that sumI/hits gives
        the true mean intensity over all raw returns.
        """
        if len(keys) == 0:
            return
        am = self.alpha_min
        bm = self.beta_min

        # ── Aggregate duplicate keys ──────────────────────────────────────────
        unique_keys, inv = np.unique(keys, return_inverse=True)
        n_u = len(unique_keys)
        agg_da = np.zeros(n_u, dtype=np.float32)
        agg_I  = np.zeros(n_u, dtype=np.float32)
        agg_ct = np.zeros(n_u, dtype=np.float32)
        np.add.at(agg_da, inv, dalpha.astype(np.float32))
        np.add.at(agg_I,  inv, inten.astype(np.float32))
        if raw_counts is not None:
            np.add.at(agg_ct, inv, raw_counts.astype(np.float32))
        else:
            np.add.at(agg_ct, inv, np.float32(1.0))

        # ── Look up existing rows ─────────────────────────────────────────────
        rows = np.fromiter(
            (self._index.get(int(k), -1) for k in unique_keys),
            dtype=np.int64, count=n_u)
        existing = rows >= 0

        # ── Update existing voxels (vectorised) ───────────────────────────────
        if np.any(existing):
            er = rows[existing]
            self.data_arr[er, 0] += agg_da[existing]
            self.data_arr[er, 2] += agg_I[existing]
            self.data_arr[er, 3] += agg_ct[existing]
            # Structure (1.0) beats seabed (0.0) — never downgrade a structure voxel
            self.data_arr[er, 4] = np.maximum(self.data_arr[er, 4], semantic)

        # ── Insert new voxels (sequential — must update index dict) ───────────
        if np.any(~existing):
            new_uk = unique_keys[~existing]
            new_da = agg_da[~existing]
            new_I  = agg_I[~existing]
            new_ct = agg_ct[~existing]
            n   = self._n
            idx = self._index
            ka  = self.keys_arr
            da  = self.data_arr
            for i in range(len(new_uk)):
                if n >= len(ka):
                    self._grow()
                    ka = self.keys_arr
                    da = self.data_arr
                key      = int(new_uk[i])
                ka[n]    = key
                da[n, 0] = am + float(new_da[i])
                da[n, 1] = bm
                da[n, 2] = float(new_I[i])
                da[n, 3] = float(new_ct[i])
                da[n, 4] = semantic
                idx[key] = n
                n += 1
            self._n = n

    # ── miss update (column-spread) ───────────────────────────────────────────

    def update_misses(self,
                      keys:          np.ndarray,  # (M,) int64
                      dbeta:         np.ndarray,  # (M,) float64
                      pub_threshold: float) -> int:
        """
        Vectorised β update on existing voxels.
        Duplicate keys are aggregated.  Voxels whose p = α/(α+β) drops below
        pub_threshold are deleted.  Returns the number of voxels deleted.
        """
        if len(keys) == 0:
            return 0

        # ── Aggregate duplicate keys ──────────────────────────────────────────
        unique_keys, inv = np.unique(keys, return_inverse=True)
        n_u = len(unique_keys)
        agg_db = np.zeros(n_u, dtype=np.float32)
        np.add.at(agg_db, inv, dbeta.astype(np.float32))

        # ── Look up existing rows ─────────────────────────────────────────────
        rows = np.fromiter(
            (self._index.get(int(k), -1) for k in unique_keys),
            dtype=np.int64, count=n_u)
        valid = rows >= 0
        if not np.any(valid):
            return 0

        # ── Apply β increments (vectorised) ───────────────────────────────────
        vr = rows[valid]
        self.data_arr[vr, 1] += agg_db[valid]

        alpha = self.data_arr[vr, 0]
        beta  = self.data_arr[vr, 1]
        p     = alpha / np.maximum(alpha + beta, 1e-9)
        to_delete = unique_keys[valid][p < pub_threshold]

        for key in to_delete:
            row = self._index.pop(int(key), -1)
            if row == -1:
                continue
            last = self._n - 1
            if row != last:
                lk = int(self.keys_arr[last])
                self.keys_arr[row] = self.keys_arr[last]
                self.data_arr[row] = self.data_arr[last]
                self._index[lk]    = row
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
            rec             = np.empty(n, dtype=_RECORD_DTYPE)
            rec['key']      = self.keys_arr[:n]
            rec['alpha']    = self.data_arr[:n, 0]
            rec['beta']     = self.data_arr[:n, 1]
            rec['sumI']     = self.data_arr[:n, 2]
            rec['hits']     = self.data_arr[:n, 3]
            rec['semantic'] = self.data_arr[:n, 4]
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
        if version not in (2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14):
            raise ValueError(f'Unsupported version {version}')
        if abs(file_vs - voxel_size) > 1e-5:
            raise ValueError(
                f'Voxel size mismatch: file={file_vs:.4f}m map={voxel_size:.4f}m')
        rec_dtype = _RECORD_DTYPE if version >= 14 else _RECORD_DTYPE_V13
        with open(path, 'rb') as f:
            f.read(17)
            records = np.frombuffer(
                f.read(n_voxels * rec_dtype.itemsize), dtype=rec_dtype)
        if len(records) != n_voxels:
            raise ValueError(f'Expected {n_voxels} records, got {len(records)}')
        needed = self._n + int(n_voxels)
        while needed > len(self.keys_arr):
            self._grow()
        idx = self._index
        ka  = self.keys_arr
        da  = self.data_arr
        n   = self._n
        has_semantic = version >= 14
        for rec in records:
            key = int(rec['key'])
            row = idx.get(key, -1)
            if row == -1:
                ka[n]    = key
                da[n, 0] = float(rec['alpha'])
                da[n, 1] = float(rec['beta'])
                da[n, 2] = float(rec['sumI'])
                da[n, 3] = float(rec['hits'])
                da[n, 4] = float(rec['semantic']) if has_semantic else 0.0
                idx[key] = n
                n += 1
            else:
                da[row, 0] += float(rec['alpha'])
                da[row, 1] += float(rec['beta'])
                da[row, 2] += float(rec['sumI'])
                da[row, 3] += float(rec['hits'])
        self._n = n
        return int(n_voxels)


# ── Bathymetry map ────────────────────────────────────────────────────────────

class BathymetryMap:
    """
    Sparse 2-D map that estimates seabed depth per (x, y) cell using a
    scalar Kalman filter.

    Each cell (indexed at ``cell_size`` resolution, coarser than sonar voxels)
    stores:
        mu_z   — estimated seabed depth [m, ENU frame]
        var_z  — depth estimation variance [m²]
        hits   — number of sonar returns used in updates

    The map is updated only from sonar returns classified as SEABED.
    It drives the occupancy probability of 3-D seabed voxels via the Gaussian
    CDF formula:

        p(voxel at z) = Φ((z_hi - mu_z) / σ_z) − Φ((z_lo - mu_z) / σ_z)

    where [z_lo, z_hi] is the voxel's z range and σ_z = sqrt(var_z).
    This probability integrates to 1 over all voxels in a column — the seabed
    must be somewhere — and concentrates near mu_z as the estimate converges.
    """

    def __init__(
        self,
        cell_size:    float,
        z_prior_mean: float,
        z_prior_std:  float,   # [m] — squared internally to get prior variance
        q_z:          float,   # [m] process noise std — squared internally
        r_z:          float,   # [m] measurement noise std — squared internally
    ):
        self.cs          = float(cell_size)
        self._z0         = float(z_prior_mean)
        self._var0       = float(z_prior_std)  ** 2
        self._q          = float(q_z)          ** 2
        self._r          = float(r_z)          ** 2

        # Sparse cell storage — all dicts share the same key set
        self._mu:   Dict[int, float] = {}
        self._var:  Dict[int, float] = {}
        self._hits: Dict[int, int]   = {}

    # ── helpers ───────────────────────────────────────────────────────────────

    def _cell_keys(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        inv_cs = 1.0 / self.cs
        bix = np.floor(xs * inv_cs).astype(np.int64)
        biy = np.floor(ys * inv_cs).astype(np.int64)
        # Reuse the existing 21-bit packing with iz=0
        return _pack(bix, biy, np.zeros(len(xs), dtype=np.int64))

    # ── Kalman update ─────────────────────────────────────────────────────────

    def update(self, xs: np.ndarray, ys: np.ndarray, zs: np.ndarray):
        """
        Kalman update for a batch of seabed returns.
        Returns in the same cell are aggregated (median z used as measurement).
        """
        if len(xs) == 0:
            return

        ckeys             = self._cell_keys(xs, ys)
        unique_ck, inv_idx = np.unique(ckeys, return_inverse=True)

        for i, ck in enumerate(unique_ck):
            k      = int(ck)
            mask   = inv_idx == i
            z_meas = float(np.median(zs[mask]))

            if k not in self._mu:
                self._mu[k]   = self._z0
                self._var[k]  = self._var0
                self._hits[k] = 0

            # Prediction step: add process noise
            self._var[k] += self._q

            # Kalman correction
            v           = self._var[k]
            K           = v / (v + self._r)
            self._mu[k]   += K * (z_meas - self._mu[k])
            self._var[k]   = (1.0 - K) * v
            self._hits[k] += int(mask.sum())

    # ── Gaussian CDF occupancy probability ───────────────────────────────────

    def gaussian_cdf_prob(self, voxel_keys: np.ndarray,
                          voxel_size: float) -> np.ndarray:
        """
        For a batch of 3-D voxel keys, return the probability that the seabed
        falls within each voxel's z-range given the local depth estimate:

            p = Φ((z_hi − mu_z) / σ_z) − Φ((z_lo − mu_z) / σ_z)

        Voxels whose bathymetry cell has no data yet get probability 0.
        """
        ix_v, iy_v, iz_v = _unpack(voxel_keys)
        x_v  = (ix_v.astype(np.float64) + 0.5) * voxel_size
        y_v  = (iy_v.astype(np.float64) + 0.5) * voxel_size
        z_lo =  iz_v.astype(np.float64)         * voxel_size   # bottom of voxel
        z_hi = z_lo + voxel_size                               # top of voxel

        ckeys   = self._cell_keys(x_v, y_v)
        mu_arr  = np.array([self._mu.get( int(k), np.nan) for k in ckeys])
        var_arr = np.array([self._var.get(int(k), np.nan) for k in ckeys])

        has_data = np.isfinite(mu_arr)
        probs    = np.zeros(len(voxel_keys), dtype=np.float32)

        if np.any(has_data):
            sigma = np.sqrt(np.maximum(var_arr[has_data], 1e-9))
            lo    = (z_lo[has_data] - mu_arr[has_data]) / sigma
            hi    = (z_hi[has_data] - mu_arr[has_data]) / sigma
            probs[has_data] = (ndtr(hi) - ndtr(lo)).astype(np.float32)

        return probs

    # ── export ────────────────────────────────────────────────────────────────

    def to_numpy(self) -> np.ndarray:
        """Return (N, 5) float32: [x, y, mu_z, std_z, hits]."""
        if not self._mu:
            return np.empty((0, 5), dtype=np.float32)

        keys    = np.array(list(self._mu.keys()),    dtype=np.int64)
        mu_arr  = np.array(list(self._mu.values()),  dtype=np.float32)
        var_arr = np.array(list(self._var.values()),  dtype=np.float32)
        hit_arr = np.array(list(self._hits.values()), dtype=np.float32)

        ix, iy, _ = _unpack(keys)
        x = (ix.astype(np.float32) + 0.5) * np.float32(self.cs)
        y = (iy.astype(np.float32) + 0.5) * np.float32(self.cs)

        return np.column_stack(
            [x, y, mu_arr, np.sqrt(var_arr), hit_arr]
        ).astype(np.float32)


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
        h_max_candidates:       int,
        I_prior_mean:           float,
        I_prior_std:            float,
        I_process_noise:        float,
        I_meas_noise:           float,
        I_percentile:           int,
        object_intensity_ratio: float,
        w_min:                  float = 0.01,
        bathy_cell_size:        float = BATHY_CELL_SIZE,
        bathy_z_prior_mean:     float = BATHY_Z_PRIOR_MEAN,
        bathy_z_prior_std:      float = BATHY_Z_PRIOR_STD,
        bathy_q_z:              float = BATHY_Q_Z,
        bathy_r_z:              float = BATHY_R_Z,
        seabed_voxel_size: float = SEABED_VOXEL_SIZE,
        lambda_structure: float = LAMBDA_STRUCTURE,
        lambda_object:    float = LAMBDA_OBJECT,
        sigma_object:     float = SIGMA_OBJECT,
    ):
        self.vs     = float(voxel_size)
        self.inv_vs = 1.0 / self.vs
        self.svs    = float(seabed_voxel_size)   # coarse seabed voxel size
        self.inv_svs= 1.0 / self.svs
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
            h_max_candidates = h_max_candidates,
            I_mean           = I_prior_mean,
            I_std            = I_prior_std,
            I_process_noise  = I_process_noise,
            I_meas_noise     = I_meas_noise,
            I_percentile     = I_percentile,
            tallest_object   = tallest_object,
            object_ratio     = object_intensity_ratio,
        )

        # Column precomputation — rebuilt whenever band changes
        self._n_col   = 0
        self._col_dz  = np.empty(0)
        self._col_t   = np.empty(0)
        self._last_hh = -1.0
        self._rebuild_column()

        # Fine-resolution store: structure + object voxels
        self._store = VoxelStore(alpha_min, beta_min)
        # Coarse-resolution store: seabed voxels only (svs >> vs → far fewer voxels)
        self._seabed_store = VoxelStore(alpha_min, beta_min)

        # Semantic labels:
        #   _seabed_store — always 0.0 = SEABED    (prob from Gaussian CDF of bathy map)
        #   _store        — 1.0 = STRUCTURE (Beta-Bernoulli; single-voxel update)
        #                   2.0 = OBJECT    (Beta-Bernoulli; column + Gaussian update)
        self.lambda_structure_ = max(float(lambda_structure), 1e-6)
        self.lambda_object_    = max(float(lambda_object),    1e-6)
        self.sigma_object_     = max(float(sigma_object),     1e-6)

        # Bathymetry map — per-(x,y)-cell Kalman depth estimator
        self._bathy_map = BathymetryMap(
            cell_size    = bathy_cell_size,
            z_prior_mean = bathy_z_prior_mean,
            z_prior_std  = bathy_z_prior_std,
            q_z          = bathy_q_z,
            r_z          = bathy_r_z,
        )

    def _rebuild_column(self):
        n = max(1, int(round(self.hh / self.vs)))
        if n == self._n_col:
            return
        self._n_col  = n
        self._col_dz = np.arange(n, dtype=np.float64) * self.vs
        self._col_t  = self._col_dz / self.hh

    def __len__(self) -> int:
        return len(self._store) + len(self._seabed_store)

    # ── insert one scan ───────────────────────────────────────────────────────

    def insert_scan(self, origin_xyz: np.ndarray, pts_xyzi: np.ndarray,
                    lidar_tree=None, epsilon: float = 0.5,
                    ellipsoid_abc: tuple = None) -> dict:
        """
        origin_xyz : (3,)  float64  USV position in ENU odom frame (z ≈ 0)
        pts_xyzi   : (N,4) float32  ALL sonar returns [x, y, z, intensity]
        lidar_tree : cKDTree | None  2-D KD-tree of LiDAR XY points (odom frame)
        epsilon    : float           XY radius [m] for LiDAR neighbourhood query
        """
        if pts_xyzi.size == 0:
            return {'hits': 0, 'misses': 0, 'deleted': 0,
                    'col_voxels': 0, 'struct_hits': 0, 'estimator': {}}



        # ── Depth: ENU frame — z is positive upward ──────────────────────
        # depth = usv_z - point_z  (positive downward)
        sz     = float(origin_xyz[2])
        depths = sz - pts_xyzi[:, 2].astype(np.float64)   # positive downward
        intens = pts_xyzi[:, 3].astype(np.float64)

        # ── Step 1: update seabed estimator (depth tracking for bathy CDF) ──
        est_stats = self.estimator.update(depths, intens)

        # ── Step 2: classify ALL points ────────────────────────────────────
        # STRUCTURE : I ≥ λ_struct * μ_I_seabed  AND  LiDAR support within ε
        #             → single-voxel α update at return point zi
        # OBJECT    : I ≥ I_object  AND  NOT structure
        #             → column body (seabed→zi) + Gaussian above zi
        # SEABED    : 0 < I < I_object  AND  NOT structure
        #             → Gaussian column spread + bathy KF update
        # MISS      : I == 0  → beta decrement on existing voxels only
        struct_thr    = self.lambda_structure_ * self.estimator.mu_I
        object_thr    = self.lambda_object_    * self.estimator.mu_I

        # LiDAR support check (vectorised over all high-intensity candidates)
        is_struct_candidate = intens >= struct_thr
        has_lidar       = np.zeros(len(pts_xyzi), dtype=bool)
        n_lidar_counts  = np.zeros(len(pts_xyzi), dtype=np.float64)
        if lidar_tree is not None and np.any(is_struct_candidate):
            cand_xy  = pts_xyzi[is_struct_candidate, :2].astype(np.float64)
            hits_idx = lidar_tree.query_ball_point(cand_xy, r=epsilon, return_sorted=False)
            cand_counts = np.array([len(h) for h in hits_idx], dtype=np.float64)
            has_lidar[is_struct_candidate]      = cand_counts > 0
            n_lidar_counts[is_struct_candidate] = cand_counts

        is_struct_hit = is_struct_candidate & has_lidar

        # ── Ellipsoid exclusion: reject structures inside or below the USV ellipsoid ──
        # Inside:  ((px-cx)/a)² + ((py-cy)/b)² + ((pz-cz)/c)² ≤ 1
        # Under:   XY footprint of ellipsoid AND pz < cz (below USV plane)
        if ellipsoid_abc is not None and np.any(is_struct_hit):
            a, b, c = ellipsoid_abc
            cx, cy, cz = float(origin_xyz[0]), float(origin_xyz[1]), float(origin_xyz[2])
            px = pts_xyzi[:, 0].astype(np.float64)
            py = pts_xyzi[:, 1].astype(np.float64)
            pz = pts_xyzi[:, 2].astype(np.float64)
            dx = (px - cx) / a;  dy = (py - cy) / b;  dz = (pz - cz) / c
            inside    = (dx*dx + dy*dy + dz*dz) <= 1.0
            in_xy     = (dx*dx + dy*dy)          <= 1.0
            under     = in_xy & (pz < cz)
            is_struct_hit = is_struct_hit & ~(inside | under)

        is_object_hit = (intens >= object_thr) & ~is_struct_hit
        is_seabed_hit = (intens > 0) & ~is_struct_hit & ~is_object_hit
        is_miss       = intens == 0

        stats = {'hits': 0, 'misses': 0, 'deleted': 0,
                 'col_voxels': 0, 'struct_hits': 0, 'estimator': est_stats}

        inv_vs = self.inv_vs
        col_dz = self._col_dz
        col_t  = self._col_t

        # ══════════════════════════════════════════════════════════════════
        # HIT UPDATE — two regimes with different update strategies
        # ══════════════════════════════════════════════════════════════════

        # ── SEABED HITS: single voxel per coarse cell → _seabed_store ───────
        # Published probability comes from Gaussian CDF (bathy map), so no
        # column spread is needed. Alpha is used only as a presence filter.
        if np.any(is_seabed_hit):
            sb_pts   = pts_xyzi[is_seabed_hit]
            sb_inten = intens[is_seabed_hit]

            inv_svs = self.inv_svs
            sb_ix = np.floor(sb_pts[:, 0] * inv_svs).astype(np.int64)
            sb_iy = np.floor(sb_pts[:, 1] * inv_svs).astype(np.int64)
            sb_iz = np.floor(sb_pts[:, 2] * inv_svs).astype(np.int64)
            col_key = sb_ix * (2**42) + sb_iy * (2**21) + sb_iz

            uniq_col, inv_idx, counts = np.unique(
                col_key, return_inverse=True, return_counts=True)
            n_uniq = len(uniq_col)

            mean_x  = np.zeros(n_uniq)
            mean_y  = np.zeros(n_uniq)
            mean_z0 = np.zeros(n_uniq)
            np.add.at(mean_x,  inv_idx, sb_pts[:, 0].astype(np.float64))
            np.add.at(mean_y,  inv_idx, sb_pts[:, 1].astype(np.float64))
            np.add.at(mean_z0, inv_idx, sb_pts[:, 2].astype(np.float64))
            mean_x /= counts; mean_y /= counts; mean_z0 /= counts

            # Intensity: per voxel group use only the bottom 80th percentile
            # of returns (same philosophy as the 85th-percentile depth estimator —
            # discard the brightest 20% which may be misclassified object returns).
            sum_I      = np.zeros(n_uniq, dtype=np.float64)
            raw_counts = np.zeros(n_uniq, dtype=np.float32)
            for i in range(n_uniq):
                grp = sb_inten[inv_idx == i]
                thr = np.percentile(grp, 80)
                filt = grp[grp <= thr]
                if filt.size == 0:
                    filt = grp          # fallback: all points if filter removes all
                sum_I[i]      = filt.sum()
                raw_counts[i] = float(filt.size)

            ix_u = np.floor(mean_x * inv_svs).astype(np.int64)
            iy_u = np.floor(mean_y * inv_svs).astype(np.int64)
            iz_u = np.floor(mean_z0 * inv_svs).astype(np.int64)
            pk_sb = _pack(ix_u, iy_u, iz_u)
            da_sb = np.full(n_uniq, self.lambda_hit, dtype=np.float64)
            self._seabed_store.update_hits(pk_sb, da_sb, sum_I,
                                           semantic=np.float32(0.0),
                                           raw_counts=raw_counts)

            # Bathymetry KF update — pass all raw seabed points
            self._bathy_map.update(
                sb_pts[:, 0].astype(np.float64),
                sb_pts[:, 1].astype(np.float64),
                sb_pts[:, 2].astype(np.float64),
            )

            stats['hits']       = n_uniq
            stats['col_voxels'] = n_uniq

        # ── STRUCTURE HITS: single-voxel α update at return point zi ────────
        # Δα = λ_hit · (I / I_struct) · n_lidar
        #   intensity weight : I / I_struct  (≥ 1 by construction, no cap)
        #   LiDAR weight     : number of LiDAR points within ε of the return
        if np.any(is_struct_hit):
            st_pts   = pts_xyzi[is_struct_hit]
            st_inten = intens[is_struct_hit]
            w_I   = st_inten / max(struct_thr, 1e-9)          # intensity weight
            w_L   = n_lidar_counts[is_struct_hit]              # LiDAR count weight
            st_ix = np.floor(st_pts[:, 0] * inv_vs).astype(np.int64)
            st_iy = np.floor(st_pts[:, 1] * inv_vs).astype(np.int64)
            st_iz = np.floor(st_pts[:, 2] * inv_vs).astype(np.int64)
            pk_st = _pack(st_ix, st_iy, st_iz)
            da_st = self.lambda_hit * w_I * w_L
            self._store.update_hits(pk_st, da_st, st_inten, semantic=np.float32(1.0))
            stats['struct_hits'] = int(is_struct_hit.sum())

        # ── OBJECT HITS: seabed-to-zi body column + Gaussian above zi ──────
        #
        # Physics assumption: the object rests on the seabed.
        # The sonar return at zi tells us the object surface is at zi.
        # We do not know exactly how tall the object is, so we model two regions:
        #
        #  Region 1 — BODY (certain occupation):
        #    Every voxel from the estimated seabed_z UP TO zi receives a flat
        #    α increment:
        #      Δα = λ_hit · min(I / I_object, 3)
        #    The normalisation by I_object means a return at exactly the threshold
        #    gives Δα = λ_hit; brighter returns give proportionally more evidence,
        #    capped at 3× to prevent runaway from saturated returns.
        #
        #  Region 2 — ABOVE (uncertain height):
        #    Voxels from zi upward (up to tallest_object above zi) receive a
        #    Gaussian-weighted α increment with the same I-normalised scale.
        #      w(dz) = exp(−0.5 · (dz / σ_object)²), Σw = 1
        #    σ_object controls how much we trust the object extends above zi.
        #    Small σ → concentrated at zi; large σ → spread over tallest_object.
        #
        #  Semantic label: 1.0 (OBJECT) → occupancy uses Beta-Bernoulli, not
        #  Gaussian CDF (which is reserved for seabed voxels).
        if np.any(is_object_hit):
            obj_pts    = pts_xyzi[is_object_hit]
            obj_intens = intens[is_object_hit]

            # Estimated seabed z in ENU: z_usv − μ_h  (global estimator)
            seabed_z  = float(origin_xyz[2]) - self.estimator.mu_h
            iz_seabed = int(np.floor(seabed_z * inv_vs))

            # Intensity-normalised alpha weight, capped at 3×
            alpha_wt = np.minimum(obj_intens / object_thr, 3.0)

            # Group returns into (ix, iy, iz_return) columns
            obj_ix0 = np.floor(obj_pts[:, 0] * inv_vs).astype(np.int64)
            obj_iy0 = np.floor(obj_pts[:, 1] * inv_vs).astype(np.int64)
            obj_iz0 = np.floor(obj_pts[:, 2] * inv_vs).astype(np.int64)
            col_key = obj_ix0 * (2**42) + obj_iy0 * (2**21) + obj_iz0

            uniq_col, inv_idx, counts = np.unique(
                col_key, return_inverse=True, return_counts=True)
            n_uniq = len(uniq_col)

            # Per-column aggregation
            mean_x   = np.zeros(n_uniq); mean_y  = np.zeros(n_uniq)
            mean_z0  = np.zeros(n_uniq); mean_I  = np.zeros(n_uniq)
            mean_awt = np.zeros(n_uniq)
            np.add.at(mean_x,   inv_idx, obj_pts[:, 0].astype(np.float64))
            np.add.at(mean_y,   inv_idx, obj_pts[:, 1].astype(np.float64))
            np.add.at(mean_z0,  inv_idx, obj_pts[:, 2].astype(np.float64))
            np.add.at(mean_I,   inv_idx, obj_intens)
            np.add.at(mean_awt, inv_idx, alpha_wt)
            mean_x   /= counts; mean_y  /= counts
            mean_z0  /= counts; mean_I  /= counts; mean_awt /= counts

            ix_obj = np.floor(mean_x * inv_vs).astype(np.int64)
            iy_obj = np.floor(mean_y * inv_vs).astype(np.int64)
            iz_obj = np.floor(mean_z0 * inv_vs).astype(np.int64)

            # ── Region 1: flat α from seabed_z up to zi ──────────────────────
            # Iterate over unique columns (objects are rare, loop cost is small).
            all_keys_b: list = []; all_da_b: list = []; all_I_b: list = []
            for i in range(n_uniq):
                iz_top = int(iz_obj[i])
                iz_bot = min(iz_seabed, iz_top)  # defensive: seabed above return?
                n_vox  = iz_top - iz_bot + 1      # inclusive [iz_bot, iz_top]
                iz_rng = np.arange(iz_bot, iz_bot + n_vox, dtype=np.int64)
                pk_i   = _pack(
                    np.full(n_vox, int(ix_obj[i]), dtype=np.int64),
                    np.full(n_vox, int(iy_obj[i]), dtype=np.int64),
                    iz_rng,
                )
                all_keys_b.append(pk_i)
                all_da_b.append(np.full(n_vox,
                                        float(self.lambda_hit * mean_awt[i]),
                                        dtype=np.float64))
                all_I_b.append(np.full(n_vox, float(mean_I[i]), dtype=np.float64))

            if all_keys_b:
                self._store.update_hits(
                    np.concatenate(all_keys_b),
                    np.concatenate(all_da_b),
                    np.concatenate(all_I_b),
                    semantic=np.float32(2.0))   # OBJECT label

            # ── Region 2: Gaussian above zi (uncertain object height) ─────────
            # w(dz) = exp(−0.5·(dz/σ_object)²), normalised, cut at w_min.
            n_above  = max(1, int(round(self.hh / self.vs)))
            dz_above = np.arange(n_above, dtype=np.float64) * self.vs
            w_above  = np.exp(-0.5 * (dz_above / self.sigma_object_) ** 2)
            s_above  = w_above.sum()
            w_above  = w_above / s_above if s_above > 1e-12 else np.ones_like(dz_above) / n_above
            sig_mask = w_above >= self.w_min_
            w_sig    = w_above[sig_mask]; w_sig /= w_sig.sum()
            n_sig    = int(sig_mask.sum())
            dz_sig   = dz_above[sig_mask]

            if n_sig > 0:
                col_z_ab = mean_z0[:, np.newaxis] + dz_sig[np.newaxis, :]  # (n_uniq, n_sig)
                iz_ab    = np.floor(col_z_ab * inv_vs).astype(np.int64)
                pk_ab    = _pack(
                    np.repeat(ix_obj, n_sig),
                    np.repeat(iy_obj, n_sig),
                    iz_ab.ravel(),
                )
                da_ab = (np.repeat(self.lambda_hit * mean_awt, n_sig)
                         * np.tile(w_sig, n_uniq))
                I_ab  = np.repeat(mean_I, n_sig).astype(np.float64)
                self._store.update_hits(pk_ab, da_ab, I_ab, semantic=np.float32(2.0))

            stats['struct_hits'] = n_uniq  # object column groups detected

        # ══════════════════════════════════════════════════════════════════
        # MISS UPDATE — uniform β across column (fully vectorised)
        # ══════════════════════════════════════════════════════════════════
        if np.any(is_miss):
            miss_pts = pts_xyzi[is_miss]
            miss_ix0 = np.floor(miss_pts[:, 0] * inv_vs).astype(np.int64)
            miss_iy0 = np.floor(miss_pts[:, 1] * inv_vs).astype(np.int64)
            col_key  = miss_ix0 * (2**42) + miss_iy0 * (2**21)

            uniq_miss, inv_miss = np.unique(col_key, return_inverse=True)
            n_uniq_miss  = len(uniq_miss)
            cnt_miss     = np.bincount(inv_miss, minlength=n_uniq_miss).astype(np.float64)
            mean_mx = np.zeros(n_uniq_miss)
            mean_my = np.zeros(n_uniq_miss)
            mean_mz = np.zeros(n_uniq_miss)
            np.add.at(mean_mx, inv_miss, miss_pts[:, 0].astype(np.float64))
            np.add.at(mean_my, inv_miss, miss_pts[:, 1].astype(np.float64))
            np.add.at(mean_mz, inv_miss, miss_pts[:, 2].astype(np.float64))
            mean_mx /= cnt_miss; mean_my /= cnt_miss; mean_mz /= cnt_miss

            # All miss-column voxel keys at once — no Python loop
            col_z_miss = mean_mz[:, np.newaxis] + col_dz[np.newaxis, :]  # (n_uniq_miss, n_col)
            mix_all = np.floor(mean_mx * inv_vs).astype(np.int64)
            miy_all = np.floor(mean_my * inv_vs).astype(np.int64)
            miz_all = np.floor(col_z_miss * inv_vs).astype(np.int64)

            pk_miss = _pack(
                np.repeat(mix_all, self._n_col),
                np.repeat(miy_all, self._n_col),
                miz_all.ravel(),
            )
            db_miss = np.full(len(pk_miss),
                              self.lambda_miss / self._n_col,
                              dtype=np.float64)
            deleted = self._store.update_misses(pk_miss, db_miss, self.pub_threshold_)

            # Also decrement seabed store at coarse resolution (single voxel per miss beam)
            smix = np.floor(mean_mx * self.inv_svs).astype(np.int64)
            smiy = np.floor(mean_my * self.inv_svs).astype(np.int64)
            smiz = np.floor(mean_mz * self.inv_svs).astype(np.int64)
            pk_miss_sb = _pack(smix, smiy, smiz)
            db_miss_sb = np.full(len(pk_miss_sb), self.lambda_miss, dtype=np.float64)
            deleted += self._seabed_store.update_misses(
                pk_miss_sb, db_miss_sb, self.pub_threshold_)

            stats['misses']  = n_uniq_miss
            stats['deleted'] = deleted

        return stats

    # ── spatial prune ─────────────────────────────────────────────────────────

    def prune_outside_radius(self, cx: float, cy: float, radius: float) -> int:
        n  = self._store.prune_outside_radius(cx, cy, radius, self.vs)
        n += self._seabed_store.prune_outside_radius(cx, cy, radius, self.svs)
        return n

    # ── retroactive structure invalidation ────────────────────────────────────

    def purge_structures_in_ellipsoid(self, cx: float, cy: float, cz: float,
                                       a: float, b: float, c: float) -> int:
        """
        Delete any structure voxels (semantic == 1.0) whose centre falls inside
        the USV exclusion ellipsoid.  Called each scan so that structures the
        USV physically passes through are retroactively removed — if the USV
        can be there, it was not a real structure.
        """
        store = self._store
        n = store._n
        if n == 0:
            return 0

        keys = store.live_keys()
        data = store.live_data()
        struct_mask = data[:, 4] == 1.0
        if not np.any(struct_mask):
            return 0

        ix, iy, iz = _unpack(keys[struct_mask])
        px = (ix.astype(np.float64) + 0.5) * self.vs
        py = (iy.astype(np.float64) + 0.5) * self.vs
        pz = (iz.astype(np.float64) + 0.5) * self.vs

        dx = (px - cx) / a
        dy = (py - cy) / b
        dz = (pz - cz) / c
        inside = (dx*dx + dy*dy + dz*dz) <= 1.0
        in_xy  = (dx*dx + dy*dy)          <= 1.0
        under  = in_xy & (pz < cz)
        to_purge = inside | under
        if not np.any(to_purge):
            return 0

        struct_indices = np.where(struct_mask)[0]
        keys_to_delete = keys[struct_indices[to_purge]].copy()

        deleted = 0
        for key in keys_to_delete:
            row = store._index.pop(int(key), -1)
            if row == -1:
                continue
            last = store._n - 1
            if row != last:
                lk = int(store.keys_arr[last])
                store.keys_arr[row] = store.keys_arr[last]
                store.data_arr[row] = store.data_arr[last]
                store._index[lk]    = row
            store._n -= 1
            deleted += 1
        return deleted

    # ── query ─────────────────────────────────────────────────────────────────

    def _store_to_numpy(self, store: 'VoxelStore', vs: float, min_hits: int,
                        cx: float, cy: float, radius: float,
                        use_bathy_cdf: bool) -> np.ndarray:
        """Helper: convert one VoxelStore to (N,6) float32."""
        n = len(store)
        if n == 0:
            return np.empty((0, 6), dtype=np.float32)
        keys  = store.live_keys()
        data  = store.live_data()
        alpha = data[:, 0]
        beta  = data[:, 1]
        prob  = alpha / np.maximum(alpha + beta, 1e-9)
        hits  = data[:, 3]
        mask  = (prob >= self.pub_threshold_) & (hits >= float(min_hits))

        if cx is not None and cy is not None and radius is not None:
            ix_all, iy_all, _ = _unpack(keys)
            vx = (ix_all.astype(np.float64) + 0.5) * vs - cx
            vy = (iy_all.astype(np.float64) + 0.5) * vs - cy
            mask &= (vx * vx + vy * vy) <= radius * radius

        if not np.any(mask):
            return np.empty((0, 6), dtype=np.float32)

        k   = keys[mask]
        d   = data[mask]
        p   = prob[mask].copy()
        ix_u, iy_u, iz_u = _unpack(k)
        cnt = np.maximum(d[:, 3], 1.0)

        if use_bathy_cdf:
            p = self._bathy_map.gaussian_cdf_prob(k, vs)

        return np.column_stack([
            (ix_u + 0.5) * vs,
            (iy_u + 0.5) * vs,
            (iz_u + 0.5) * vs,
            p,
            d[:, 2] / cnt,    # intensity mean
            d[:, 4],          # semantic
            d[:, 0],          # alpha  (Beta-Bernoulli hit count)
            d[:, 1],          # beta   (Beta-Bernoulli miss count)
        ]).astype(np.float32)

    def to_numpy(self, min_hits: int,
                 cx: float = None, cy: float = None,
                 radius: float = None) -> np.ndarray:
        """
        Return (N,8) float32: [x, y, z, occupancy_prob, intensity_mean, semantic, alpha, beta].

        occupancy_prob semantics:
          SEABED    (0.0) — Gaussian CDF: p = Φ((z_hi-mu_z)/σ_z) - Φ((z_lo-mu_z)/σ_z)
          STRUCTURE (1.0) — Beta-Bernoulli: α/(α+β)
          OBJECT    (2.0) — Beta-Bernoulli: α/(α+β)

        Seabed voxels are stored at coarse resolution (svs); structure/object at fine (vs).
        Optional spatial window: if cx, cy, radius are given, only voxels within
        that XY circle are returned (for fast local-map publishing).
        """
        pts_fine   = self._store_to_numpy(self._store, self.vs, min_hits,
                                          cx, cy, radius, use_bathy_cdf=False)
        pts_seabed = self._store_to_numpy(self._seabed_store, self.svs, min_hits,
                                          cx, cy, radius, use_bathy_cdf=True)
        if pts_fine.shape[0] == 0 and pts_seabed.shape[0] == 0:
            return np.empty((0, 6), dtype=np.float32)
        if pts_fine.shape[0] == 0:
            return pts_seabed
        if pts_seabed.shape[0] == 0:
            return pts_fine
        return np.concatenate([pts_fine, pts_seabed], axis=0)

    def bathy_to_numpy(self) -> np.ndarray:
        """Return (N, 5) float32: [x, y, mu_z, std_z, hits] for the bathymetry map."""
        return self._bathy_map.to_numpy()

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
        self.declare_parameter('seabed_voxel_size',      SEABED_VOXEL_SIZE)
        self.declare_parameter('tallest_object',         TALLEST_OBJECT)
        # Seabed estimator
        self.declare_parameter('h_prior_mean',           H_PRIOR_MEAN)
        self.declare_parameter('h_prior_std',            H_PRIOR_STD)
        self.declare_parameter('h_process_noise',        H_PROCESS_NOISE)
        self.declare_parameter('h_meas_noise',           H_MEAS_NOISE)
        self.declare_parameter('h_min_candidates',       H_MIN_CANDIDATES)
        self.declare_parameter('h_max_candidates',       H_MAX_CANDIDATES)
        self.declare_parameter('I_percentile',           I_PERCENTILE)
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

        self.declare_parameter('global_publish_rate',    0.5)

        self.declare_parameter('source_id',              SOURCE_ID)
        self.declare_parameter('save_path',              SAVE_PATH)
        self.declare_parameter('load_path',              LOAD_PATH)
        self.declare_parameter('autosave_on_shutdown',   AUTOSAVE_ON_SHUTDOWN)
        self.declare_parameter('autosave_path',          AUTOSAVE_PATH)
        self.declare_parameter('mission_log_path',       '')
        self.declare_parameter('pcd_save_dir',           PCD_SAVE_DIR)
        # Bathymetry map
        self.declare_parameter('bathy_cell_size',        BATHY_CELL_SIZE)
        self.declare_parameter('bathy_z_prior_mean',     BATHY_Z_PRIOR_MEAN)
        self.declare_parameter('bathy_z_prior_std',      BATHY_Z_PRIOR_STD)
        self.declare_parameter('bathy_q_z',              BATHY_Q_Z)
        self.declare_parameter('bathy_r_z',              BATHY_R_Z)
        # Structure update
        self.declare_parameter('lidar_topic',       LIDAR_TOPIC)
        self.declare_parameter('epsilon_struct',    EPSILON_STRUCT)
        self.declare_parameter('ellipsoid_a',       ELLIPSOID_A)
        self.declare_parameter('ellipsoid_b',       ELLIPSOID_B)
        self.declare_parameter('ellipsoid_c',       ELLIPSOID_C)
        self.declare_parameter('lambda_structure',  LAMBDA_STRUCTURE)
        self.declare_parameter('lambda_object',     LAMBDA_OBJECT)
        self.declare_parameter('sigma_object',      SIGMA_OBJECT)
        def gp(n): return self.get_parameter(n).value

        voxel_size        = float(gp('voxel_size'))
        tallest_obj       = float(gp('tallest_object'))
        global_pub_rate   = float(gp('global_publish_rate'))

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
            seabed_voxel_size      = float(gp('seabed_voxel_size')),
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
            h_max_candidates       = int(gp('h_max_candidates')),
            I_percentile           = int(gp('I_percentile')),
            I_prior_mean           = float(gp('I_prior_mean')),
            I_prior_std            = float(gp('I_prior_std')),
            I_process_noise        = float(gp('I_process_noise')),
            I_meas_noise           = float(gp('I_meas_noise')),
            object_intensity_ratio = float(gp('object_intensity_ratio')),
            bathy_cell_size        = float(gp('bathy_cell_size')),
            bathy_z_prior_mean     = float(gp('bathy_z_prior_mean')),
            bathy_z_prior_std      = float(gp('bathy_z_prior_std')),
            bathy_q_z              = float(gp('bathy_q_z')),
            bathy_r_z              = float(gp('bathy_r_z')),
            lambda_structure        = float(gp('lambda_structure')),
            lambda_object          = float(gp('lambda_object')),
            sigma_object           = float(gp('sigma_object')),
        )

        self._epsilon_struct_  = float(gp('epsilon_struct'))
        self._ellipsoid_abc_   = (float(gp('ellipsoid_a')),
                                   float(gp('ellipsoid_b')),
                                   float(gp('ellipsoid_c')))
        self._lidar_tree_      = None   # built from /cloud_registered

        self.vehicle_pose_       = None
        self._scan_count_        = 0
        self.mission_log_path_   = str(gp('mission_log_path'))
        self.pcd_save_dir_       = str(gp('pcd_save_dir'))
        self._mission_log_: list = []          # rows: (t, mu_I, std_I, mu_h, std_h)
        self._mission_t0_        = time.monotonic()

        self._fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='prob',      offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='source',    offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='semantic',  offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name='alpha',     offset=28, datatype=PointField.FLOAT32, count=1),
            PointField(name='beta',      offset=32, datatype=PointField.FLOAT32, count=1),
        ]

        self.map_pub_       = self.create_publisher(PointCloud2, gp('output_topic'), 10)
        self.ellipsoid_pub_ = self.create_publisher(Marker, 'sonar_map/usv_ellipsoid', 10)
        self.bathy_pub_     = self.create_publisher(PointCloud2, 'sonar_map_bathy', 10)
        self._bathy_fields = [
            PointField(name='x',     offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',     offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',     offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='std_z', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='hits',  offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        self.cloud_sub_  = self.create_subscription(PointCloud2, gp('input_topic'), self.cloud_callback, 1)
        self.odom_sub_   = self.create_subscription(Odometry, gp('odom_topic'), self.odom_callback, 1)
        self.lidar_sub_  = self.create_subscription(PointCloud2, str(gp('lidar_topic')), self.lidar_callback, 1)
        self.save_srv_   = self.create_service(Trigger, 'sonar_map/save', self._save_cb)
        self.load_srv_   = self.create_service(Trigger, 'sonar_map/load', self._load_cb)
        self.global_timer_ = self.create_timer(1.0 / global_pub_rate, self._publish_global)

        load_path = str(gp('load_path'))
        if load_path:
            self._do_load(load_path)

        est = self.voxel_map_.estimator
        self.get_logger().info(
            f'ProbSonarMap v14 — ENU frame, adaptive estimator | '
            f'voxel={voxel_size}m tallest={tallest_obj}m | '
            f'h_prior={est.mu_h:.2f}m ±{est.std_h:.2f}m | '
            f'I_prior={est.mu_I:.0f} ±{est.std_I:.0f} | '
            f'lambda_struct={float(gp("lambda_structure"))} lambda_obj={float(gp("lambda_object"))} sigma_object={float(gp("sigma_object"))}m | '
            f'scale={float(gp("intensity_scale")):.0f} | '
            f'lambda_hit={float(gp("lambda_hit"))} lambda_miss={float(gp("lambda_miss"))} | '
            f'pub_threshold={float(gp("pub_threshold"))} | '
            f'global={global_pub_rate}Hz'
        )


    @staticmethod
    def _write_pcd(path: str, fields: list, data: np.ndarray):
        """Write a binary-little-endian PCD file.

        fields  — list of (name, type_char, size)  e.g. ('x','F',4)
        data    — (N, len(fields)) float32 array
        """
        n = data.shape[0]
        field_names  = ' '.join(f[0] for f in fields)
        field_sizes  = ' '.join(str(f[2]) for f in fields)
        field_types  = ' '.join(f[1] for f in fields)
        field_counts = ' '.join('1' for _ in fields)
        header = (
            f'# .PCD v0.7\n'
            f'VERSION 0.7\n'
            f'FIELDS {field_names}\n'
            f'SIZE {field_sizes}\n'
            f'TYPE {field_types}\n'
            f'COUNT {field_counts}\n'
            f'WIDTH {n}\n'
            f'HEIGHT 1\n'
            f'VIEWPOINT 0 0 0 1 0 0 0\n'
            f'POINTS {n}\n'
            f'DATA binary\n'
        )
        with open(path, 'wb') as f:
            f.write(header.encode('ascii'))
            f.write(data.astype(np.float32).tobytes())

    def _save_mission_log(self):
        """Write mission_log.csv to the mission log directory."""
        base = self.mission_log_path_
        if not base or not self._mission_log_:
            return
        try:
            os.makedirs(base, exist_ok=True)
            log_path = os.path.join(base, 'mission_log.csv')
            arr = np.array(self._mission_log_, dtype=np.float64)
            header = 'time_s,mu_I,std_I,mu_h,std_h,n_fine_voxels,n_seabed_voxels'
            np.savetxt(log_path, arr, delimiter=',', header=header, comments='')
            self.get_logger().info(f'Mission log saved: {len(arr)} rows → {log_path}')
        except Exception as exc:
            self.get_logger().error(f'Mission log export failed: {exc}')

    def _save_pcd(self):
        """Write sonar_map.pcd and bathy_map.pcd to the PCD directory."""
        pcd_dir = self.pcd_save_dir_
        if not pcd_dir:
            return
        try:
            os.makedirs(pcd_dir, exist_ok=True)

            # ── sonar_map.pcd: x y z probability intensity semantic ───────────
            pts6 = self.voxel_map_.to_numpy(min_hits=0)
            sonar_path = os.path.join(pcd_dir, 'sonar_map.pcd')
            fields = [('x','F',4), ('y','F',4), ('z','F',4),
                      ('probability','F',4), ('intensity','F',4), ('semantic','F',4)]
            self._write_pcd(sonar_path, fields, pts6)
            self.get_logger().info(
                f'Sonar map PCD saved: {pts6.shape[0]} voxels → {sonar_path}')

            # ── bathy_map.pcd: x y z(=mu_z) z_std hits ───────────────────────
            bathy = self.voxel_map_.bathy_to_numpy()   # (N,5) x y mu_z std_z hits
            bathy_path = os.path.join(pcd_dir, 'bathy_map.pcd')
            bfields = [('x','F',4), ('y','F',4), ('z','F',4),
                       ('z_std','F',4), ('hits','F',4)]
            self._write_pcd(bathy_path, bfields, bathy)
            self.get_logger().info(
                f'Bathy map PCD saved: {bathy.shape[0]} cells → {bathy_path}')

        except Exception as exc:
            self.get_logger().error(f'PCD export failed: {exc}')

    def destroy_node(self):
        self._save_mission_log()
        self._save_pcd()
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
        if resp.success:
            self._save_mission_log()
            self._save_pcd()
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
        self._publish_ellipsoid(msg.header, p)

    def _publish_ellipsoid(self, odom_header, position):
        a, b, c = self._ellipsoid_abc_
        marker = Marker()
        marker.header.frame_id = self.map_frame_
        marker.header.stamp    = odom_header.stamp
        marker.ns              = 'usv_exclusion'
        marker.id              = 0
        marker.type            = Marker.SPHERE
        marker.action          = Marker.ADD
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0 * a   # full diameter per axis
        marker.scale.y = 2.0 * b
        marker.scale.z = 2.0 * c
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.0
        marker.color.a = 0.25       # semi-transparent
        self.ellipsoid_pub_.publish(marker)

    def lidar_callback(self, msg: PointCloud2):
        """Build a 2-D KD-tree from LiDAR XY, filtered to window_radius around USV."""
        try:
            pts = _read_xyz_intensity(msg)
        except Exception as exc:
            self.get_logger().error(f'lidar_callback read error: {exc}',
                                    throttle_duration_sec=5.0)
            return
        if pts.shape[0] == 0:
            return
        xy = pts[:, :2].astype(np.float64)
        # Keep only points within window_radius of USV to limit tree size
        if self.vehicle_pose_ is not None:
            cx, cy = self.vehicle_pose_[0], self.vehicle_pose_[1]
            r2 = self.window_radius_ ** 2
            d2 = (xy[:, 0] - cx) ** 2 + (xy[:, 1] - cy) ** 2
            xy = xy[d2 <= r2]
        if xy.shape[0] > 0:
            from scipy.spatial import cKDTree
            self._lidar_tree_ = cKDTree(xy)

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
        stats  = self.voxel_map_.insert_scan(
            origin, pts,
            lidar_tree    = self._lidar_tree_,
            epsilon       = self._epsilon_struct_,
            ellipsoid_abc = self._ellipsoid_abc_,
        )

        # Retroactively remove structure voxels the USV has just passed through
        a, b, c = self._ellipsoid_abc_
        self.voxel_map_.purge_structures_in_ellipsoid(
            float(origin[0]), float(origin[1]), float(origin[2]), a, b, c)

        dt     = time.monotonic() - t0

        self._scan_count_ += 1
        est = self.voxel_map_.estimator

        # ── Mission log: record estimator state each scan ──────────────────────
        if self.mission_log_path_:
            self._mission_log_.append((
                time.monotonic() - self._mission_t0_,
                est.mu_I, est.std_I,
                est.mu_h, est.std_h,
                len(self.voxel_map_._store),
                len(self.voxel_map_._seabed_store),
            ))
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
                f'I_seabed={est.mu_I:.0f} ±{est.std_I:.0f} | '
                f'd_cand={n_dcand} I_cand={n_icand}'
            )

        if self._scan_count_ % self.prune_every_n_scans_ == 0:
            cx, cy, _ = self.vehicle_pose_
            removed = self.voxel_map_.prune_outside_radius(cx, cy, self.window_radius_)
            if removed:
                self.get_logger().info(
                    f'Pruned {removed} voxels | map={len(self.voxel_map_)}')

    # ── publishing helpers ─────────────────────────────────────────────────────

    def _make_header(self) -> Header:
        h = Header()
        h.frame_id = self.map_frame_
        h.stamp    = self.get_clock().now().to_msg()
        return h

    def _pts6_to_cloud(self, pts6: np.ndarray, header: Header) -> PointCloud2:
        """Convert (N,8) [x,y,z,prob,intensity,semantic,alpha,beta] to PointCloud2."""
        n   = pts6.shape[0]
        out = np.empty((n, 9), dtype=np.float32)
        out[:, :5] = pts6[:, :5]      # x, y, z, prob, intensity
        out[:, 5]  = self.source_id_  # source
        out[:, 6]  = pts6[:, 5]       # semantic
        out[:, 7]  = pts6[:, 6]       # alpha
        out[:, 8]  = pts6[:, 7]       # beta
        return pc2.create_cloud(header, self._fields, out)

    def _publish_global(self):
        """Slow publisher: full map at global_publish_rate."""
        n_vox = len(self.voxel_map_)
        if n_vox == 0:
            self.get_logger().info(
                f'Map empty — scans={self._scan_count_} '
                f'odom={"ok" if self.vehicle_pose_ else "waiting"}',
                throttle_duration_sec=5.0)
            return

        header = self._make_header()
        t0   = time.monotonic()
        pts6 = self.voxel_map_.to_numpy(min_hits=self.min_hits_)
        dt   = time.monotonic() - t0

        if pts6.shape[0] > 0:
            self.map_pub_.publish(self._pts6_to_cloud(pts6, header))

        bathy = self.voxel_map_.bathy_to_numpy()
        if bathy.shape[0] > 0:
            self.bathy_pub_.publish(
                pc2.create_cloud(header, self._bathy_fields, bathy))

        self.get_logger().info(
            f'Global map: {pts6.shape[0]}/{n_vox} voxels | '
            f'bathy={bathy.shape[0]} | to_numpy={dt*1000:.1f}ms',
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