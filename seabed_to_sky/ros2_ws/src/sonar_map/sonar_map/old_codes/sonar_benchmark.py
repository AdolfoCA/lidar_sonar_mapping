#!/usr/bin/env python3
"""
sonar_benchmark.py
------------------
Standalone benchmark for sonar_map_ned.py hot paths.
No ROS2 required.

Run inside the container:
  python3 /ros2_ws/src/spark-fast-lio/sonar_map/src/sonar_benchmark.py
"""

import sys
import time
import numpy as np
from scipy.spatial import cKDTree

# ── Import the module under test ───────────────────────────────────────────────
import importlib.util

spec = importlib.util.spec_from_file_location(
    'sonar_map_ned',
    '/home/rosdev/ros2_ws/src/spark-fast-lio/sonar_map/src/sonar_map_ned.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

VoxelStore           = mod.VoxelStore
BathymetryMap        = mod.BathymetryMap
ProbabilisticVoxelMap= mod.ProbabilisticVoxelMap
_pack                = mod._pack
_unpack              = mod._unpack

# ── Benchmark helpers ──────────────────────────────────────────────────────────

REPEATS = 10

def bench(label, fn, repeats=REPEATS):
    # warmup
    fn()
    times = []
    for _ in range(repeats):
        t0 = time.perf_counter()
        fn()
        times.append((time.perf_counter() - t0) * 1000)
    arr = np.array(times)
    print(f'  {label:<50s}  {arr.mean():7.2f} ms  ±{arr.std():5.2f}  '
          f'(min={arr.min():.2f}  max={arr.max():.2f})')
    return arr.mean()

print('=' * 80)
print('sonar_map_ned.py  —  bottleneck benchmark')
print('=' * 80)

# ── Scenario sizes ─────────────────────────────────────────────────────────────
N_SONAR       = 5000    # sonar points per scan
N_LIDAR       = 100_000 # LiDAR points in accumulated cloud (after radius filter)
N_LIDAR_LARGE = 1_000_000  # before radius filter (full accumulated map)
N_VOXELS      = 300_000 # voxels already in the map
VOXEL_SIZE    = 0.05
EPSILON       = 0.5

print(f'\nScenario: {N_SONAR} sonar pts/scan | {N_VOXELS} voxels in map | '
      f'{N_LIDAR} LiDAR pts (filtered) | {N_LIDAR_LARGE} (full cloud)')

# ── Build a realistic VoxelStore ───────────────────────────────────────────────
print('\n[1] Building test fixtures ...')
rng = np.random.default_rng(42)
store = VoxelStore(1e-3, 1e-3)

# Pre-populate with N_VOXELS
keys_init = rng.integers(0, 2**60, size=N_VOXELS, dtype=np.int64)
keys_init = np.unique(keys_init)
da_init   = rng.random(len(keys_init)).astype(np.float64) * 0.5 + 0.5
I_init    = rng.random(len(keys_init)).astype(np.float64) * 2000
store.update_hits(keys_init, da_init, I_init, semantic=np.float32(0.0))
print(f'   VoxelStore populated: {len(store)} voxels')

# ── Realistic sonar scan points ────────────────────────────────────────────────
# Seabed hits: 70%, structure hits: 10%, miss: 20%
n_sb  = int(N_SONAR * 0.70)
n_st  = int(N_SONAR * 0.10)
n_ms  = N_SONAR - n_sb - n_st

sb_pts = np.column_stack([
    rng.uniform(-10, 10, n_sb),
    rng.uniform(-10, 10, n_sb),
    rng.uniform(-3.5, -2.5, n_sb),
    rng.uniform(100, 1500, n_sb),
]).astype(np.float32)

st_pts = np.column_stack([
    rng.uniform(-5, 5, n_st),
    rng.uniform(-5, 5, n_st),
    rng.uniform(-3.0, 0.0, n_st),
    rng.uniform(2000, 5000, n_st),
]).astype(np.float32)

ms_pts = np.column_stack([
    rng.uniform(-10, 10, n_ms),
    rng.uniform(-10, 10, n_ms),
    rng.uniform(-4.0, -1.0, n_ms),
    np.zeros(n_ms),
]).astype(np.float32)

all_pts = np.vstack([sb_pts, st_pts, ms_pts])

# LiDAR trees
xy_lidar_small = rng.uniform(-10, 10, (N_LIDAR, 2))
xy_lidar_large = rng.uniform(-50, 50, (N_LIDAR_LARGE, 2))
tree_small = cKDTree(xy_lidar_small)

# ── [A] LiDAR KD-tree build ────────────────────────────────────────────────────
print('\n[A] LiDAR KD-tree build')
bench(f'cKDTree build  {N_LIDAR:>7,} pts (radius-filtered)',
      lambda: cKDTree(xy_lidar_small))
bench(f'cKDTree build  {N_LIDAR_LARGE:>7,} pts (full cloud, no filter)',
      lambda: cKDTree(xy_lidar_large), repeats=5)

# ── [B] query_ball_point ───────────────────────────────────────────────────────
print('\n[B] query_ball_point  (structure detection)')
st_xy = st_pts[:, :2].astype(np.float64)
bench(f'query_ball_point  {n_st} pts  ε={EPSILON}m  (small tree)',
      lambda: tree_small.query_ball_point(st_xy, r=EPSILON, return_length=True))
# worst case: all points are structure
all_xy = all_pts[:, :2].astype(np.float64)
bench(f'query_ball_point  {N_SONAR} pts  ε={EPSILON}m  (small tree)',
      lambda: tree_small.query_ball_point(all_xy, r=EPSILON, return_length=True))

# ── [C] VoxelStore.update_hits ────────────────────────────────────────────────
print('\n[C] VoxelStore.update_hits')
inv_vs = 1.0 / VOXEL_SIZE

# Seabed column spread output (typical: 3500 pts × 2 sig voxels = 7000 keys)
n_sb_cols = n_sb // 3   # ~unique columns
n_sig_vox = 2
n_hit_keys = n_sb_cols * n_sig_vox
hit_keys  = rng.integers(0, 2**60, size=n_hit_keys, dtype=np.int64)
hit_da    = rng.random(n_hit_keys).astype(np.float64) * 0.1
hit_I     = rng.random(n_hit_keys).astype(np.float64) * 1000

# Fresh store for each call
def _make_store():
    s = VoxelStore(1e-3, 1e-3)
    s.update_hits(keys_init[:50000], da_init[:50000], I_init[:50000])
    return s

s_test = _make_store()
bench(f'update_hits  {n_hit_keys} keys  (seabed columns)',
      lambda: s_test.update_hits(hit_keys, hit_da, hit_I, np.float32(0.0)))

# structure: smaller batch
st_keys = rng.integers(0, 2**60, size=n_st, dtype=np.int64)
st_da   = rng.random(n_st).astype(np.float64) * 1.0
st_I    = rng.random(n_st).astype(np.float64) * 3000
bench(f'update_hits  {n_st} keys  (structure)',
      lambda: s_test.update_hits(st_keys, st_da, st_I, np.float32(1.0)))

# ── [D] VoxelStore.update_misses ──────────────────────────────────────────────
print('\n[D] VoxelStore.update_misses')
# Miss: unique miss cols × n_col voxels
n_miss_cols = n_ms // 5
n_col = 2
n_miss_keys = n_miss_cols * n_col
miss_keys = store.live_keys()[:min(n_miss_keys, len(store))].copy()
miss_db   = np.full(len(miss_keys), 0.05)

bench(f'update_misses  {len(miss_keys)} keys  (existing voxels)',
      lambda: store.update_misses(miss_keys, miss_db, 0.70))

# Worst case: all miss keys are unknown (all dict misses)
miss_keys_unknown = rng.integers(0, 2**63, size=n_miss_keys, dtype=np.int64)
bench(f'update_misses  {n_miss_keys} keys  (all unknown — dict miss path)',
      lambda: store.update_misses(miss_keys_unknown, miss_db[:n_miss_keys], 0.70))

# ── [E] insert_scan (seabed column spread) — vectorised ──────────────────────
print('\n[E] insert_scan seabed column-spread  (vectorised broadcast)')
# Simulate the vectorised seabed block directly
col_t   = np.linspace(0, 1, 2)
col_dz  = np.array([0.0, VOXEL_SIZE])
sigma_s = 0.05
w_min   = 0.01
lambda_hit = 0.1

def _seabed_block_vectorised():
    inv_vs_ = 1.0 / VOXEL_SIZE
    sb_ix0  = np.floor(sb_pts[:, 0] * inv_vs_).astype(np.int64)
    sb_iy0  = np.floor(sb_pts[:, 1] * inv_vs_).astype(np.int64)
    sb_iz0  = np.floor(sb_pts[:, 2] * inv_vs_).astype(np.int64)
    col_key = sb_ix0 * (2**42) + sb_iy0 * (2**21) + sb_iz0
    uniq_col, inv_idx, counts = np.unique(col_key, return_inverse=True, return_counts=True)
    n_uniq = len(uniq_col)
    mean_I  = np.zeros(n_uniq); mean_x = np.zeros(n_uniq)
    mean_y  = np.zeros(n_uniq); mean_z0 = np.zeros(n_uniq)
    sb_inten = sb_pts[:, 3].astype(np.float64)
    np.add.at(mean_I,  inv_idx, sb_inten)
    np.add.at(mean_x,  inv_idx, sb_pts[:, 0].astype(np.float64))
    np.add.at(mean_y,  inv_idx, sb_pts[:, 1].astype(np.float64))
    np.add.at(mean_z0, inv_idx, sb_pts[:, 2].astype(np.float64))
    mean_I /= counts; mean_x /= counts; mean_y /= counts; mean_z0 /= counts
    w = np.exp(-0.5 * (col_t / sigma_s) ** 2)
    w /= w.sum()
    sig_mask = w >= w_min
    w_sig = w[sig_mask]; w_sig /= w_sig.sum()
    n_sig = int(sig_mask.sum())
    col_dz_sig = col_dz[sig_mask]
    col_z_all = mean_z0[:, np.newaxis] + col_dz_sig[np.newaxis, :]
    ix_all = np.floor(mean_x * inv_vs_).astype(np.int64)
    iy_all = np.floor(mean_y * inv_vs_).astype(np.int64)
    iz_all = np.floor(col_z_all * inv_vs_).astype(np.int64)
    pk_all = _pack(np.repeat(ix_all, n_sig), np.repeat(iy_all, n_sig), iz_all.ravel())
    da_all = np.tile(lambda_hit * w_sig, n_uniq)
    I_all  = np.repeat(mean_I, n_sig).astype(np.float64)
    return pk_all, da_all, I_all, n_uniq

bench(f'seabed vectorised block  {n_sb} pts',
      _seabed_block_vectorised)

# ── [F] insert_scan miss — vectorised ─────────────────────────────────────────
print('\n[F] insert_scan miss update  (vectorised broadcast)')

def _miss_block_vectorised():
    inv_vs_ = 1.0 / VOXEL_SIZE
    miss_ix0 = np.floor(ms_pts[:, 0] * inv_vs_).astype(np.int64)
    miss_iy0 = np.floor(ms_pts[:, 1] * inv_vs_).astype(np.int64)
    col_key  = miss_ix0 * (2**42) + miss_iy0 * (2**21)
    uniq_miss, inv_miss = np.unique(col_key, return_inverse=True)
    n_uniq_miss = len(uniq_miss)
    cnt_miss = np.bincount(inv_miss, minlength=n_uniq_miss).astype(np.float64)
    mean_mx = np.zeros(n_uniq_miss); mean_my = np.zeros(n_uniq_miss); mean_mz = np.zeros(n_uniq_miss)
    np.add.at(mean_mx, inv_miss, ms_pts[:, 0].astype(np.float64))
    np.add.at(mean_my, inv_miss, ms_pts[:, 1].astype(np.float64))
    np.add.at(mean_mz, inv_miss, ms_pts[:, 2].astype(np.float64))
    mean_mx /= cnt_miss; mean_my /= cnt_miss; mean_mz /= cnt_miss
    n_col_ = 2
    col_dz_ = np.array([0.0, VOXEL_SIZE])
    col_z_miss = mean_mz[:, np.newaxis] + col_dz_[np.newaxis, :]
    mix_all = np.floor(mean_mx * inv_vs_).astype(np.int64)
    miy_all = np.floor(mean_my * inv_vs_).astype(np.int64)
    miz_all = np.floor(col_z_miss * inv_vs_).astype(np.int64)
    pk_miss = _pack(np.repeat(mix_all, n_col_), np.repeat(miy_all, n_col_), miz_all.ravel())
    db_miss = np.full(len(pk_miss), 0.1 / n_col_)
    return pk_miss, db_miss, n_uniq_miss

bench(f'miss vectorised block  {n_ms} pts',
      _miss_block_vectorised)

# ── [G] to_numpy ──────────────────────────────────────────────────────────────
print('\n[G] to_numpy  (global + local)')

# Build a realistic ProbabilisticVoxelMap
pvm = ProbabilisticVoxelMap(
    voxel_size=VOXEL_SIZE, tallest_object=0.5,
    lambda_hit=0.1, lambda_miss=0.1,
    alpha_min=1e-3, beta_min=1e-3,
    intensity_scale=2500.0, pub_threshold=0.70,
    sigma_seabed=0.05, sigma_t=0.05,
    h_prior_mean=2.5, h_prior_std=0.5,
    h_process_noise=0.05, h_meas_noise=0.02,
    h_min_candidates=10,
    I_prior_mean=530.0, I_prior_std=300.0,
    I_process_noise=30.0, I_meas_noise=150.0,
    object_intensity_ratio=2.0,
)
# Populate via insert_scan
origin = np.array([0.0, 0.0, 0.0])
for _ in range(20):
    pts_scan = np.column_stack([
        rng.uniform(-10, 10, N_SONAR),
        rng.uniform(-10, 10, N_SONAR),
        rng.uniform(-3.5, -2.0, N_SONAR),
        rng.uniform(0, 3000, N_SONAR),
    ]).astype(np.float32)
    pvm.insert_scan(origin, pts_scan, lidar_tree=tree_small, epsilon=EPSILON)

print(f'   PVM populated: {len(pvm)} voxels')

bench(f'to_numpy  global  ({len(pvm)} voxels)',
      lambda: pvm.to_numpy(min_hits=1))
bench(f'to_numpy  local   r=20m  ({len(pvm)} voxels)',
      lambda: pvm.to_numpy(min_hits=1, cx=0.0, cy=0.0, radius=20.0))

# ── [H] BathymetryMap.update ──────────────────────────────────────────────────
print('\n[H] BathymetryMap.update')
bm = BathymetryMap(cell_size=0.5, z_prior_mean=-2.5,
                   z_prior_std=0.5, q_z=0.01, r_z=0.05)
xs = rng.uniform(-10, 10, n_sb).astype(np.float64)
ys = rng.uniform(-10, 10, n_sb).astype(np.float64)
zs = rng.uniform(-3.5, -2.5, n_sb).astype(np.float64)
bench(f'BathymetryMap.update  {n_sb} pts',
      lambda: bm.update(xs, ys, zs))

# BathymetryMap after many updates (large cell count)
for _ in range(50):
    bm.update(xs + rng.uniform(-5, 5), ys + rng.uniform(-5, 5), zs)
bench(f'BathymetryMap.gaussian_cdf_prob  ({len(bm._mu)} cells)',
      lambda: bm.gaussian_cdf_prob(store.live_keys()[:5000], VOXEL_SIZE))

# ── [I] np.fromiter dict lookup (in update_hits/update_misses) ───────────────
print('\n[I] np.fromiter dict lookup  (bottleneck inside update_hits/update_misses)')
idx_dict = store._index
all_keys_live = store.live_keys()[:50000]
unknown_keys  = rng.integers(0, 2**63, size=50000, dtype=np.int64)

bench(f'np.fromiter dict lookup  50k  (all hits — best case)',
      lambda: np.fromiter(
          (idx_dict.get(int(k), -1) for k in all_keys_live),
          dtype=np.int64, count=len(all_keys_live)))

bench(f'np.fromiter dict lookup  50k  (all miss — worst case)',
      lambda: np.fromiter(
          (idx_dict.get(int(k), -1) for k in unknown_keys),
          dtype=np.int64, count=len(unknown_keys)))

# ── [J] Full insert_scan end-to-end ──────────────────────────────────────────
print('\n[J] Full insert_scan  end-to-end  (most important!)')
pvm2 = ProbabilisticVoxelMap(
    voxel_size=VOXEL_SIZE, tallest_object=0.5,
    lambda_hit=0.1, lambda_miss=0.1,
    alpha_min=1e-3, beta_min=1e-3,
    intensity_scale=2500.0, pub_threshold=0.70,
    sigma_seabed=0.05, sigma_t=0.05,
    h_prior_mean=2.5, h_prior_std=0.5,
    h_process_noise=0.05, h_meas_noise=0.02,
    h_min_candidates=10,
    I_prior_mean=530.0, I_prior_std=300.0,
    I_process_noise=30.0, I_meas_noise=150.0,
    object_intensity_ratio=2.0,
)
# Warm up with some voxels
for _ in range(5):
    pvm2.insert_scan(origin, all_pts, lidar_tree=tree_small, epsilon=EPSILON)

bench(f'insert_scan  {N_SONAR} pts  map={len(pvm2)} voxels  with LiDAR tree',
      lambda: pvm2.insert_scan(origin, all_pts, lidar_tree=tree_small, epsilon=EPSILON))
bench(f'insert_scan  {N_SONAR} pts  map={len(pvm2)} voxels  no LiDAR tree',
      lambda: pvm2.insert_scan(origin, all_pts, lidar_tree=None, epsilon=EPSILON))

print('\n' + '=' * 80)
print('Done.')
