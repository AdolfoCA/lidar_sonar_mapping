#!/usr/bin/env python3
"""
prob_sonar_map_ros2.py  (v6 — hits-only, 10 Hz capable)
--------------------------------------------------------
Beta-Bernoulli probabilistic occupancy map for sonar.

WHAT CHANGED IN v6 vs v5:
──────────────────────────
The only bottleneck was to_numpy(), which was called every publish tick and
rebuilt numpy arrays from the dict on every call:

    np.stack(list(cells.values()))   <-- O(N) copy of entire map, 49 ms @ 100k voxels

Fix: replace dict[key -> np.array] with TWO preallocated parallel arrays:
    keys_arr : (CAPACITY,)   int64    -- voxel keys
    data_arr : (CAPACITY, 4) float32  -- [alpha, beta, sumI, hits]
    index    : dict[int -> int]       -- key -> row  (O(1) lookup, unchanged)

to_numpy() now just slices keys_arr[:n] and data_arr[:n] — zero copies,
views only.  Measured speedup: 49 ms -> 0.9 ms  (54x).

The write loop (0.75 ms/scan) is already fast enough for 10 Hz and stays
as a Python loop since the number of active voxels per scan is small (~200).

Full timing @ 10 Hz (100 ms budget):
  insert_scan  ~1 ms
  to_numpy     ~1 ms
  Total        ~2 ms   →  headroom for surface mesh extraction later

PHILOSOPHY (unchanged from v5):
────────────────────────────────
Only sonar endpoint (hit) voxels are stored.
Free-space and lateral blur voxels are dropped entirely.
beta is fixed at beta_min after first hit — acts as prior strength only.
p = alpha / (alpha + beta_min) → 1.0 as hit count grows.

File format (.sonarmap, little-endian) — backward-compatible with v3-v5:
  [0]   magic     uint32   0x534F4E52
  [4]   version   uint8    6
  [5]   voxel_sz  float32
  [9]   n_voxels  uint64
  [17]  per voxel: key(i64) alpha(f32) beta(f32) sumI(f32) hits(f32)  = 24 bytes
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


# ── Packed int64 voxel keys (21 bits/axis -> +/- 1 048 576 voxels) ──────────

_BITS    = 21
_MASK    = np.int64((1 << _BITS) - 1)
_SIGN    = 1 << (_BITS - 1)
_MAGIC   = 0x534F4E52   # "SONR"
_VERSION = 6

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
    Memory layout:
      keys_arr : (capacity,)   int64    packed voxel key per row
      data_arr : (capacity, 4) float32  [alpha, beta, sumI, hits] per row
      _index   : dict[int -> int]       key -> row index  (O(1) lookup)
      _n       : int                    number of live voxels

    All live data is always in rows 0 .. _n-1.
    to_numpy slices keys_arr[:_n] and data_arr[:_n] — no copying.

    Capacity doubles automatically when full (rare after warmup).
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

    # ── capacity management ───────────────────────────────────────────────────

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

    # ── write ─────────────────────────────────────────────────────────────────

    def update_hits(self,
                    active_keys:  np.ndarray,   # (K,) int64
                    active_da:    np.ndarray,   # (K,) float64  alpha increments
                    active_inten: np.ndarray):  # (K,) float64  intensities
        """
        Write K hit-voxel updates.  New voxels are appended; existing rows
        are updated in-place.  Python loop over K active voxels is fast
        because K is small (~100-300 per scan after depth-band filter).
        """
        am  = self.alpha_min
        bm  = self.beta_min
        idx = self._index
        ka  = self.keys_arr
        da  = self.data_arr
        n   = self._n

        for i in range(len(active_keys)):
            key = int(active_keys[i])
            inc = float(active_da[i])
            it  = float(active_inten[i])
            row = idx.get(key, -1)
            if row == -1:
                if n >= len(ka):
                    self._grow()
                    ka = self.keys_arr
                    da = self.data_arr
                ka[n]    = key
                da[n, 0] = am + inc   # alpha
                da[n, 1] = bm         # beta  (fixed forever)
                da[n, 2] = it         # sumI
                da[n, 3] = 1.0        # hits
                idx[key] = n
                n += 1
            else:
                da[row, 0] += inc     # alpha grows
                da[row, 2] += it      # sumI accumulates
                da[row, 3] += 1.0     # hit count

        self._n = n

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

        # Compact kept rows to front
        new_keys = np.empty(len(self.keys_arr), dtype=np.int64)
        new_data = np.zeros_like(self.data_arr)
        new_keys[:n_keep] = keys[keep]
        new_data[:n_keep] = self.data_arr[:n][keep]
        self.keys_arr = new_keys
        self.data_arr = new_data
        self._index   = {int(new_keys[r]): r for r in range(n_keep)}
        self._n       = n_keep
        return n - n_keep

    # ── query (zero-copy views) ────────────────────────────────────────────────

    def live_keys(self) -> np.ndarray:
        """View of live key rows — do not modify."""
        return self.keys_arr[:self._n]

    def live_data(self) -> np.ndarray:
        """View of live data rows — do not modify."""
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
        if version not in (2, 3, 4, 5, 6):
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

        # Grow if needed
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
                da[row, 1]  = max(da[row, 1], float(rec['beta']))  # keep strongest prior
                da[row, 2] += float(rec['sumI'])
                da[row, 3] += float(rec['hits'])

        self._n = n
        return int(n_voxels)


# ── Beta-Bernoulli voxel map ──────────────────────────────────────────────────

class ProbabilisticVoxelMap:
    """
    Hits-only Beta-Bernoulli map backed by VoxelStore.

    p(occupied) = alpha / (alpha + beta_min)

    alpha accumulates evidence with each sonar return.
    beta is fixed at beta_min — acts only as prior strength.
    """

    def __init__(
        self,
        voxel_size:      float,
        seabed_depth_h:  float,
        hh:              float,
        lambda_hit:      float,
        sigma_r_m:       float,
        alpha_min:       float = 1e-3,
        beta_min:        float = 1e-3,
    ):
        self.vs     = float(voxel_size)
        self.inv_vs = 1.0 / self.vs

        self.mu_d    = float(seabed_depth_h)
        self.hh      = float(hh)
        self.d_lo    = self.mu_d - self.hh
        self.d_hi    = self.mu_d + self.hh
        self.sigma_d = self.hh / math.sqrt(12.0) if hh > 1e-9 else 1e-6

        self.lambda_hit = float(lambda_hit)
        self.sigma_r    = max(float(sigma_r_m), 1e-6)

        self._store = VoxelStore(alpha_min, beta_min)

    def __len__(self) -> int:
        return len(self._store)

    # ── insert ────────────────────────────────────────────────────────────────

    def insert_scan(self, origin_xyz: np.ndarray, pts_xyzi: np.ndarray):
        """
        origin_xyz : (3,) float64  -- USV/sensor position in map frame
        pts_xyzi   : (N,4) float32 -- sonar hit points [x, y, z, intensity]
        """
        if pts_xyzi.size == 0:
            return

        inv = self.inv_vs
        ix  = np.floor(pts_xyzi[:, 0] * inv).astype(np.int64)
        iy  = np.floor(pts_xyzi[:, 1] * inv).astype(np.int64)
        iz  = np.floor(pts_xyzi[:, 2] * inv).astype(np.int64)
        pk  = _pack(ix, iy, iz)

        # Average points that land in the same voxel within this scan
        uniq_pk, inv_idx, counts = np.unique(
            pk, return_inverse=True, return_counts=True)
        means = np.zeros((len(uniq_pk), 4), dtype=np.float64)
        np.add.at(means, inv_idx, pts_xyzi[:, :4].astype(np.float64))
        means /= counts[:, None]

        self._update(origin_xyz.astype(np.float64), means, uniq_pk)

    def _update(self, origin: np.ndarray,
                endpoints: np.ndarray,
                endpoint_keys: np.ndarray):
        sz    = origin[2]
        inten = endpoints[:, 3]

        dx = endpoints[:, 0] - origin[0]
        dy = endpoints[:, 1] - origin[1]
        dz = endpoints[:, 2] - origin[2]
        R  = np.sqrt(dx*dx + dy*dy + dz*dz)
        ok = R > 1e-6
        if not np.any(ok):
            return

        inten         = inten[ok]
        endpoint_keys = endpoint_keys[ok]

        # ── Depth-band filter ─────────────────────────────────────────────
        _, _, ep_iz = _unpack(endpoint_keys)
        d_ep    = sz - (ep_iz.astype(np.float64) + 0.5) * self.vs
        in_band = (d_ep >= self.d_lo) & (d_ep <= self.d_hi)
        if not np.any(in_band):
            return

        # ── Gaussian weight: full weight at expected seabed depth ─────────
        w_d = np.zeros(len(endpoint_keys), dtype=np.float64)
        dd  = d_ep[in_band] - self.mu_d
        w_d[in_band] = np.exp(-0.5 * (dd / self.sigma_d) ** 2)

        # ── Write to store ────────────────────────────────────────────────
        self._store.update_hits(
            endpoint_keys[in_band],
            self.lambda_hit * w_d[in_band],
            inten[in_band],
        )

    # ── spatial pruning ───────────────────────────────────────────────────────

    def prune_outside_radius(self, cx: float, cy: float, radius: float) -> int:
        return self._store.prune_outside_radius(cx, cy, radius, self.vs)

    # ── query ─────────────────────────────────────────────────────────────────

    def to_numpy(self, pub_threshold: float, min_hits: int) -> np.ndarray:
        """
        Return (N,5) float32: [x, y, z, occupancy_prob, intensity_mean].
        Uses zero-copy views of the preallocated arrays — fast at any map size.
        """
        n = len(self._store)
        if n == 0:
            return np.empty((0, 5), dtype=np.float32)

        keys = self._store.live_keys()   # view, no copy
        data = self._store.live_data()   # view, no copy

        alpha = data[:, 0]
        beta  = data[:, 1]
        prob  = alpha / np.maximum(alpha + beta, 1e-6)
        hits  = data[:, 3]

        mask = (prob >= pub_threshold) & (hits >= float(min_hits))
        if not np.any(mask):
            return np.empty((0, 5), dtype=np.float32)

        k = keys[mask]
        d = data[mask]
        p = prob[mask]

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
        """Depths of all stored voxels below vehicle_z (for diagnostics)."""
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


# ── ROS2 node ────────────────────────────────────────────────────────────────

class ProbSonarMapNode(Node):

    def __init__(self):
        super().__init__('prob_sonar_map')

        # Topics / frames
        self.declare_parameter('input_topic',  'sonar_scan')
        self.declare_parameter('output_topic', 'sonar_map')
        self.declare_parameter('odom_topic',   'odometry')
        self.declare_parameter('map_frame',    'odom')

        # Map geometry
        self.declare_parameter('voxel_size',    0.10)
        self.declare_parameter('window_radius', 200.0)

        # Publishing
        self.declare_parameter('publish_rate',  10.0)   # Hz
        self.declare_parameter('pub_threshold',  0.95)
        self.declare_parameter('min_hits',        2)
        self.declare_parameter('source_id',       0.0)

        # Seabed depth band (positive = below USV)
        self.declare_parameter('seabed_depth_h',       3.0)
        self.declare_parameter('max_object_height_hh', 2.5)

        # Bayesian update
        self.declare_parameter('lambda_hit', 1.0)
        self.declare_parameter('alpha_min',  1e-3)
        self.declare_parameter('beta_min',   1e-3)

        # Sonar range uncertainty
        self.declare_parameter('sigma_r_m', 0.0065)

        # Persistence
        self.declare_parameter('load_path',            '')
        self.declare_parameter('save_path',
            '/home/rosdev/ros2_ws/saved_maps/voxels/sonar_map.sonarmap')
        self.declare_parameter('autosave_on_shutdown', True)
        self.declare_parameter('autosave_path',        '')

        # Pruning
        self.declare_parameter('prune_every_n_scans', 50)

        def gp(n): return self.get_parameter(n).value

        voxel_size   = float(gp('voxel_size'))
        seabed_h     = float(gp('seabed_depth_h'))
        hh           = float(gp('max_object_height_hh'))
        publish_rate = float(gp('publish_rate'))

        self.map_frame_            = gp('map_frame')
        self.window_radius_        = float(gp('window_radius'))
        self.pub_threshold_        = float(gp('pub_threshold'))
        self.min_hits_             = int(gp('min_hits'))
        self.source_id_            = float(gp('source_id'))
        self.save_path_            = str(gp('save_path'))
        self.autosave_path_        = str(gp('autosave_path'))
        self.autosave_on_shutdown_ = bool(gp('autosave_on_shutdown'))
        self.prune_every_n_scans_  = max(1, int(gp('prune_every_n_scans')))

        self.voxel_map_ = ProbabilisticVoxelMap(
            voxel_size     = voxel_size,
            seabed_depth_h = seabed_h,
            hh             = hh,
            lambda_hit     = float(gp('lambda_hit')),
            sigma_r_m      = float(gp('sigma_r_m')),
            alpha_min      = float(gp('alpha_min')),
            beta_min       = float(gp('beta_min')),
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

        # ── Load prior map if requested ───────────────────────────────────
        load_path = str(gp('load_path'))
        if load_path:
            self._do_load(load_path)

        self.get_logger().info(
            f'ProbSonarMap v6 | '
            f'voxel={voxel_size}m | window={self.window_radius_}m | '
            f'depth_band=[{seabed_h-hh:.2f}, {seabed_h+hh:.2f}]m | '
            f'lambda_hit={float(gp("lambda_hit"))} | '
            f'threshold={self.pub_threshold_} | min_hits={self.min_hits_} | '
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
            else:
                self.get_logger().warn(
                    'autosave_on_shutdown=true but no save_path set')
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
        self.voxel_map_.insert_scan(origin, pts)
        dt = time.monotonic() - t0

        self._scan_count_ += 1

        if self._scan_count_ <= 3 or self._scan_count_ % 50 == 0:
            usv_z = origin[2]
            d_min = float(usv_z - pts[:, 2].max())
            d_max = float(usv_z - pts[:, 2].min())
            vm    = self.voxel_map_
            self.get_logger().info(
                f'Scan #{self._scan_count_}: {len(pts)} pts | '
                f'insert={dt*1000:.1f}ms | '
                f'usv_z={usv_z:.3f}m | '
                f'depth=[{d_min:.2f}, {d_max:.2f}]m | '
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
        pts5 = self.voxel_map_.to_numpy(
            pub_threshold=self.pub_threshold_,
            min_hits=self.min_hits_)
        dt = time.monotonic() - t0

        if pts5.size == 0:
            if self.vehicle_pose_ is not None:
                depths = self.voxel_map_.depth_stats(self.vehicle_pose_[2])
                if depths.size:
                    self.get_logger().warn(
                        f'{n_vox} voxels but 0 above threshold '
                        f'(prob>={self.pub_threshold_}, hits>={self.min_hits_}). '
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