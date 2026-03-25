# Probabilistic Acoustic Seabed Mapping — Algorithm Description

## 1. Overview

The `sonar_map_ned` node builds a probabilistic 3-D voxel map of the underwater
environment from imaging sonar data in real time.  Four semantic classes are
maintained: **Seabed** (0), **Structure** (1), **Object** (2), and **Miss**.

Two preallocated voxel stores are used:

| Store | Resolution | Contents | Occupancy model |
|-------|-----------|----------|-----------------|
| Fine | δ_v = 0.05 m | Structure + Object voxels | Beta–Bernoulli |
| Coarse | δ_s = 0.5 m | Seabed voxels | Gaussian CDF of bathymetry KF |

---

## 2. Coordinate Frame and Voxel Grid

All positions are in the **ENU odometry frame** (z positive upward).  The depth
of a return below the USV is:

```
depth(p) = o_z - p_z    [positive downward]
```

Voxel indices are computed as:

```
i_x = floor(p_x / δ),  i_y = floor(p_y / δ),  i_z = floor(p_z / δ)
```

They are packed into a single int64 key using 21 bits per axis:

```
key = (i_x & M) << 42 | (i_y & M) << 21 | (i_z & M)    where M = 2^21 - 1
```

This gives a spatial range of ±2^20 · δ ≈ ±1048 m from the origin.

---

## 3. Adaptive Seabed Estimator

Two independent scalar Kalman filters run online to track:

- **(μ_h, σ²_h)** — seabed depth below USV [m, positive downward]
- **(μ_I, σ²_I)** — typical intensity of seabed returns [counts]

### 3.1 Prediction Step (every scan)

```
σ²_h ← σ²_h + q_h²
σ²_I ← σ²_I + q_I²
```

### 3.2 Depth Measurement

The measurement is the **85th percentile** of all return depths in the scan:

```
z_meas = percentile_85( depth(p_i) for all i )
```

Using the 85th percentile robustly tracks the seabed even when objects are present.

### 3.3 Intensity Measurement

Only non-zero returns below the current estimate are used:

```
C_I = { i : I_i > 0  and  I_i ≤ μ_I + 2·σ_I }
I_meas = median( I_i for i in C_I )
```

Clipping at μ_I + 2σ_I prevents bright objects from biasing the estimate.

### 3.4 Kalman Correction (if enough candidates)

```
K_h = σ²_h / (σ²_h + r_h²)
μ_h ← μ_h + K_h · (z_meas - μ_h)
σ²_h ← (1 - K_h) · σ²_h

K_I = σ²_I / (σ²_I + r_I²)
μ_I ← μ_I + K_I · (I_meas - μ_I)
σ²_I ← (1 - K_I) · σ²_I
```

### 3.5 Derived Thresholds

```
d_lo = max(0, μ_h - h_max - σ_h)    [lower band limit]
d_hi = μ_h + 2·σ_h                   [upper band limit]
I_struct = λ_struct · μ_I             [structure intensity threshold]
I_obj    = λ_obj    · μ_I             [object intensity threshold]
```

---

## 4. Point Classification

Each return (p_i, I_i) is assigned to exactly one class in this priority order:

### 4.1 Intensity Thresholds

```
Structure candidate:  I_i ≥ λ_struct · μ_I    (default λ_struct = 1.5)
Object candidate:     I_i ≥ λ_obj    · μ_I    (default λ_obj    = 3.0)
```

### 4.2 LiDAR Support Check (for Structure)

A structure must also appear in the LiDAR scan.  A 2-D KD-tree of LiDAR XY
points is queried:

```
hasLiDAR(p_i) = ( min_j || p_i^XY - q_j^XY || ≤ ε )     ε = 0.5 m
```

### 4.3 USV Exclusion Ellipsoid

Returns inside or directly below the USV are rejected as structural candidates:

```
inEllipsoid(p) = [ ((p_x-o_x)/a)² + ((p_y-o_y)/b)² + ((p_z-o_z)/c)² ≤ 1 ]
              OR [ ((p_x-o_x)/a)² + ((p_y-o_y)/b)² ≤ 1  AND  p_z < o_z ]
```

Default semi-axes: a = b = 5.0 m, c = 2.0 m.

### 4.4 Classification Rules (mutually exclusive, in priority order)

| Class | Condition |
|-------|-----------|
| **MISS** | I_i = 0 |
| **STRUCTURE** | I_i ≥ I_struct  **AND**  hasLiDAR(p_i)  **AND NOT** inEllipsoid(p_i) |
| **OBJECT** | I_i ≥ I_obj  **AND NOT** STRUCTURE |
| **SEABED** | I_i > 0  AND NOT STRUCTURE  AND NOT OBJECT |

---

## 5. Bathymetric Map

A sparse 2-D collection of scalar Kalman filters, one per horizontal cell of
size c_s (default 0.5 m).  Each cell stores (μ_z, σ²_z, n_hits).

### 5.1 Kalman Update (from SEABED returns)

Returns within a cell are grouped; the measurement is the median depth:

```
z_meas = median( p_{z,i} for returns in cell (u,v) )

σ²_z ← σ²_z + q_z²                    [predict]
K_z   = σ²_z / (σ²_z + r_z²)          [gain]
μ_z  ← μ_z  + K_z · (z_meas - μ_z)   [correct]
σ²_z ← (1 - K_z) · σ²_z
```

### 5.2 Gaussian CDF Occupancy Probability

The probability that the seabed passes through a 3-D voxel with z-range
[z_lo, z_hi]:

```
p_bathy(z_lo, z_hi) = Φ( (z_hi - μ_z) / σ_z ) - Φ( (z_lo - μ_z) / σ_z )
```

where Φ is the standard normal CDF.  As σ_z → 0, the probability concentrates
at the voxel closest to μ_z.  This formula is used **only** for seabed voxels.

---

## 6. Beta–Bernoulli Occupancy Model

Structure and object voxels each store pseudo-counts (α, β).  The occupancy
probability is:

```
p = α / (α + β)
```

Initial prior: α₀ = β₀ = 1e-3.

**Hit update:** α ← α + Δα  (increment depends on class, see below)

**Miss update:** β ← β + Δβ  (only on existing voxels, no new voxels created)

**Deletion:** voxels with p < τ_pub are deleted.

---

## 7. Map Update Rules

### 7.1 Seabed Hit

Returns in the same coarse voxel are averaged to one representative point.
The coarse-resolution store receives:

```
Δα = λ_hit
```

The BathymetryMap is simultaneously updated from all raw seabed returns
(preserving spatial resolution in the depth estimate).

Semantic label: **s = 0**.

### 7.2 Structure Hit

A single fine-resolution voxel at the exact return position receives:

```
Δα = λ_hit
```

Semantic label: **s = 1** (permanent — never downgraded once set).

### 7.3 Object Hit

Objects rest on the seabed.  The estimated seabed height is:

```
z_seabed = o_z - μ_h
```

An intensity-normalised weight prevents runaway from saturated returns:

```
w_I = min( I_i / I_obj, 3 )
```

**Region 1 — Body** (certain occupation, from z_seabed up to z_i):

```
Δα(z) = λ_hit · w_I    for z in [z_seabed, z_i]
```

**Region 2 — Above** (uncertain height, from z_i upward):

```
w(dz) = exp( -0.5 · (dz / σ_obj)² ) / Σ_k exp( -0.5 · (k·δ / σ_obj)² )
Δα(z) = λ_hit · w_I · w(z - z_i)
```

Only weights above w_min = 0.01 contribute (cuts Gaussian tails to ≈3 voxels
with default σ_obj = 0.15 m).

Semantic label: **s = 2**.

### 7.4 Miss Update

The fine-resolution column at the return position receives a uniform β increment:

```
Δβ_fine = λ_miss / N_col    for N_col = ceil(h_max / δ_v) voxels
```

The coarse-resolution seabed store receives a single-voxel decrement:

```
Δβ_coarse = λ_miss
```

Voxels with p = α/(α+β) < τ_pub are deleted after the update.

---

## 8. Publishing and Spatial Window

### 8.1 Publication Probability

| Semantic | Formula |
|----------|---------|
| SEABED (s=0) | Gaussian CDF: Φ((z_hi − μ_z)/σ_z) − Φ((z_lo − μ_z)/σ_z) |
| STRUCTURE (s=1) | Beta–Bernoulli: α / (α + β) |
| OBJECT (s=2) | Beta–Bernoulli: α / (α + β) |

Voxels are published only if p ≥ τ_pub and hit count n ≥ n_min.

### 8.2 Spatial Pruning

Voxels beyond R_max from the USV are deleted every n_prune scans:

```
delete if || (i_x + 0.5)·δ − o_x,  (i_y + 0.5)·δ − o_y || > R_max
```

---

## 9. Parameter Reference

| Parameter | Symbol | Default | Unit |
|-----------|--------|---------|------|
| `voxel_size` | δ_v | 0.05 | m |
| `seabed_voxel_size` | δ_s | 0.50 | m |
| `bathy_cell_size` | c_s | 0.50 | m |
| `tallest_object` | h_max | 0.50 | m |
| `bathy_z_prior_mean` | μ_z^(0) | −2.5 | m |
| `bathy_z_prior_std` | σ_z^(0) | 0.50 | m |
| `bathy_q_z` | q_z | 0.01 | m |
| `bathy_r_z` | r_z | 0.05 | m |
| `I_prior_mean` | μ_I^(0) | 530 | counts |
| `I_prior_std` | σ_I^(0) | 300 | counts |
| `I_process_noise` | q_I | 30 | counts |
| `I_meas_noise` | r_I | 150 | counts |
| `h_min_candidates` | N_min | 10 | — |
| `lambda_structure` | λ_struct | 1.5 | — |
| `lambda_object` | λ_obj | 3.0 | — |
| `epsilon_struct` | ε | 0.5 | m |
| `ellipsoid_a/b` | a, b | 5.0 | m |
| `ellipsoid_c` | c | 2.0 | m |
| `lambda_hit` | λ_hit | 0.10 | — |
| `lambda_miss` | λ_miss | 0.10 | — |
| `sigma_object` | σ_obj | 0.15 | m |
| `pub_threshold` | τ_pub | 0.70 | — |
| `min_hits` | n_min | 1 | — |
| `window_radius` | R_max | 200 | m |

---

## 10. Data Products (saved on shutdown)

| File | Format | Columns |
|------|--------|---------|
| `mission_log.csv` | CSV | t, μ_I, σ_I, μ_h, σ_h |
| `voxel_map.npy` | (N×6) float32 | x, y, z, p, I_sum, semantic |
| `bathy_map.npy` | (M×5) float32 | x, y, μ_z, σ_z, n_hits |
| `acoustic_surface_map.mat` | MATLAB | X, Y, Z grids for `surf()` |
