# sonar_map

Sonar mapping for the seabed-to-sky maritime stack. Two pipelines live in this
package:

* the **Beta-Bernoulli occupancy** pipeline (`sonar_mapping*.launch.py`), and
* the **Dirichlet-Categorical voxel** pipeline (`dirichlet_voxel.launch.py`),
  documented below.

---

## Dirichlet voxel mapping pipeline

An evidence-accumulation pipeline that classifies sonar returns into four
semantic classes — **FREE, SEABED, OBJECT, STRUCTURE** — accumulates them in a
transient Dirichlet-Categorical voxel map, and hands confidently-classified
SEABED voxels off to a coarse seabed-surface layer via a periodic *promotion
sweep*.

Run it with:

```bash
ros2 launch sonar_map dirichlet_voxel.launch.py
```

All nodes are configured from a single file:
[`config/dirichlet_voxel.yaml`](config/dirichlet_voxel.yaml). Custom messages
are defined in the sibling package **`sonar_map_msgs`** (an `ament_cmake`
package — ROS 2 message generation cannot live in this `ament_python` one).

### Topic graph

```
  (raw sonar driver)      (LiDAR / SLAM)         (odometry)
        │                       │                    │
 /blueview/point2/leading  /cloud_registered      odometry
        │                       │                    │
        ▼                       │                    │
 ┌──────────────┐               │                    │
 │ sonar_scan_  │               │                    │
 │ ned          │               │                    │
 └──────────────┘               │                    │
        │ sonar_scan            │                    │
        ├───────────────┐       │                    │
        ▼               ▼       ▼                    ▼
 ┌──────────────┐   ┌─────────────────────────────────────┐
 │ seabed_      │   │        return_classifier            │
 │ estimator    │──▶│  (also subscribes estimator state)  │
 └──────────────┘   └─────────────────────────────────────┘
        seabed_estimator_state         │ classified_returns
                                       ▼
                            ┌────────────────────┐   dirichlet_voxels
                            │    voxel_mapper     │──────────────▶ rviz
                            │  Dirichlet voxels   │   (PointCloud2)
                            └────────────────────┘
                  dirichlet_voxel_map │   ▲ promoted_voxel_keys
                  (full snapshot)     ▼   │ (retire these voxels)
                            ┌────────────────────┐
                            │   seabed_surface    │  ← owns the promotion
                            │  sweep + patches    │    sweep (every 5 s)
                            └────────────────────┘
                  seabed_patches │      │ seabed_mesh
                  (PointCloud2)  ▼      ▼ (Marker, TRIANGLE_LIST)
                               rviz   rviz
```

| Topic                     | Type                              | Publisher          |
|---------------------------|-----------------------------------|--------------------|
| `sonar_scan`              | `sensor_msgs/PointCloud2`         | `sonar_scan_ned` (started by this launch) |
| `/blueview/point2/leading`| `sensor_msgs/PointCloud2`         | *external* (raw sonar driver) |
| `/cloud_registered`       | `sensor_msgs/PointCloud2`         | *external* (LiDAR/SLAM) |
| `odometry`                | `nav_msgs/Odometry`               | *external* |
| `seabed_estimator_state`  | `sonar_map_msgs/SeabedEstimatorState` | `seabed_estimator` |
| `seabed_mu_intensity`     | `std_msgs/Float64`                | `seabed_estimator` (plot-friendly mirror of `mu_i`) |
| `classified_returns`      | `sonar_map_msgs/ClassifiedReturns`    | `return_classifier` |
| `dirichlet_voxels`        | `sensor_msgs/PointCloud2`         | `voxel_mapper` (visualisation) |
| `dirichlet_voxel_map`     | `sonar_map_msgs/DirichletVoxelMap`| `voxel_mapper` (snapshot for the sweep) |
| `promoted_voxel_keys`     | `sonar_map_msgs/PromotedVoxelKeys`| `seabed_surface` (feedback → `voxel_mapper`) |
<!-- seabed_patches / seabed_mesh / /debug/promoted have been stripped from
     seabed_surface for performance debugging — see the seabed_surface section
     below. The patch / Kalman / smoothing code is in git history if needed. -->

This launch file starts the **`sonar_scan_ned` front-end plus the four
pipeline nodes**. The remaining inputs — the raw sonar cloud
`/blueview/point2/leading`, the registered LiDAR cloud `/cloud_registered` and
`odometry` — come from the rest of the seabed-to-sky stack and must already be
running (or replayed from a bag).

### Nodes

0. **`sonar_scan_ned`** — transforms the raw leading-edge sonar cloud into the
   odom frame via TF and republishes it (throttled) on `sonar_scan`. Swap sonar
   at launch time with `sonar_frame:=oculus`.
1. **`seabed_estimator`** — a single scalar Kalman filter tracks the typical
   seabed backscatter intensity `mu_I` (with variance). The depth half was
   removed once `voxel_mapper` started spreading evidence over the elevation
   band — the global `mu_h` is no longer used downstream. Process noise is
   derived from a mission-scale prior; measurement noise is adaptive (sensor
   floor + MAD dispersion). Publishes once per scan.
2. **`return_classifier`** — derives adaptive thresholds `tau_struct`,
   `tau_obj` from `mu_I`, checks LiDAR support within a horizontal radius
   `epsilon`, and applies the ordered rule FREE → STRUCTURE → OBJECT → SEABED.
   Emits one `ClassifiedReturns` batch per scan (parallel arrays).
3. **`voxel_mapper`** — the central Dirichlet voxel map (see below). It only
   *accumulates* evidence; it does not promote.
4. **`seabed_surface`** — owns the promotion sweep. Every 5 s it marks every
   SEABED-ready voxel as promoted and sends the keys back to `voxel_mapper`.
   Currently stripped to this bare minimum: patch aggregation, the smoothed
   Delaunay mesh and the `/debug/promoted` cumulative cloud have all been
   removed for performance debugging. The patch / Kalman / smoothing code
   lives in git history if it's wanted back.

### The promotion protocol

The Dirichlet map in `voxel_mapper` is a **transient working layer**, not a
persistent map. Each voxel holds four Dirichlet evidence weights
`(w_free, w_sb, w_obj, w_str)`, their total `W`, and a ring buffer of recent
return depths. The posterior class probabilities are

```
pi_c = (alpha_0 + w_c) / (4*alpha_0 + W)
```

Promotion is a **periodic sweep owned by `seabed_surface`** (rate `sweep_rate`,
default 0.5 Hz — every 5 s). `voxel_mapper` publishes its whole active store as
a `DirichletVoxelMap` snapshot; each sweep `seabed_surface`:

1. computes `pi_c` for every voxel and selects those that are **promotion-ready**:

   ```
   argmax_c pi_c == SEABED   AND   pi_SEABED >= pi_conf   AND   W >= W_conf
   ```

2. routes each promoted voxel into the patch that contains it (create-or-append);
3. sends the promoted voxel **keys back** to `voxel_mapper` on
   `promoted_voxel_keys`, which then **removes** those voxels from the active
   store and retires their locations.

> **Note — this departs from the spec's original "Architecture B".** The
> promotion criterion was moved out of `voxel_mapper` into `seabed_surface` at
> the user's request, and promotion is now a 5 s batch rather than a
> per-update check. Because the two nodes are separate processes, this requires
> the `promoted_voxel_keys` feedback topic so `voxel_mapper`'s active store
> still shrinks and stays bounded.

Key invariants — useful when reasoning about what you see:

* **`seabed_surface` is the sole promoter.** `voxel_mapper` never promotes; it
  accumulates evidence and applies the retirements it is told about.
* **Promotion is irreversible.** A voxel is promoted at most once
  (`seabed_surface` tracks promoted keys). Once retired, returns that later
  fall in that location are dropped by `voxel_mapper`.
* **Only SEABED voxels are promoted.** OBJECT / STRUCTURE / FREE voxels are
  never swept out — they stay in the active Dirichlet map (no consumer yet),
  which is why criterion #5 below expects them to remain in the active cloud.
* **The stores are disjoint.** A voxel is in the Dirichlet active store, *or*
  it is a seed/measurement in a seabed patch — never both.

### Debugging the visualisation

Expected behaviour on a representative bag (this *is* the acceptance test):

1. The `dirichlet_voxels` cloud starts **dim and dense** — evidence is thin.
2. Individual voxels **brighten** as evidence accumulates.
3. Voxels **disappear** from `dirichlet_voxels` and **reappear** aggregated in
   `seabed_patches` as they are promoted.
4. Once patches activate, the `seabed_mesh` **fills in** progressively.
5. After a sustained interval the active cloud is **sparse** (boundaries,
   transients, and OBJECT/STRUCTURE voxels) while the mesh covers the
   surveyed footprint.

Field encodings (colour the clouds by these in rviz):

* `dirichlet_voxels`: `confidence = max_c pi_c * tanh(W / W_ref)` — brightens
  with both probability *and* evidence. `weight` carries raw `W` for
  debugging. `semantic` is the dominant class index
  (`0=FREE, 1=SEABED, 2=OBJECT, 3=STRUCTURE`). The four per-class posteriors
  `pi_free`, `pi_sb`, `pi_obj`, `pi_str` are also carried as float32 in [0, 1]
  (e.g. 0.87 = 87 %) — colour by any of them in rviz to read each voxel's
  per-class probability directly. A display-only floor `W >= W_display_min`
  keeps near-empty voxels out of the cloud.
* `seabed_patches`: `inv_sigma_z = 1/sigma_z` — **brighter = more certain**.
* `seabed_mesh`: a `TRIANGLE_LIST` `Marker`; triangles whose longest
  horizontal edge exceeds `eta * r_p` are dropped so the mesh never bridges
  un-surveyed gaps.

Note: with the 5 s batch sweep, voxels leave the active cloud in **bursts every
5 s** rather than vanishing continuously — that is expected, not a bug.

**Debugging promotion speed (`/debug/promoted`).** `seabed_surface` publishes a
cumulative cloud of every voxel it has ever promoted, with three per-point
fields:

* `sweep` — the sweep index at which the voxel was promoted. Colour by this in
  rviz to watch promotion progress and judge *how fast* it happens.
* `weight` — `W` at the moment of promotion. If every promoted voxel hugs
  `w_conf`, voxels are promoting the instant they qualify — the criterion may
  be **too loose**. A wide spread means voxels accumulate evidence first.
* `probability` — `pi_SEABED` at promotion (always ≥ `pi_conf`).

The `seabed_surface` log also prints `W@promotion=[lo..hi]` each sweep — a
quick read on the same question without opening rviz.

Common "nothing shows up" causes:

* No `dirichlet_voxels` points → `voxel_mapper` has no input. Check that
  `classified_returns` is publishing, which needs both `seabed_estimator_state`
  *and* `odometry` (the classifier skips scans until odometry arrives).
* Voxels accumulate but never leave → check `seabed_surface` is receiving
  `dirichlet_voxel_map`; `w_conf` / `pi_conf` too high, or `r_v` too small
  (returns scatter and no voxel ever reaches `w_conf`).
* `seabed_patches` empty but voxels are promoted → not enough SEABED seeds per
  patch yet; a patch needs `rho_patch * n_p^2` seeds to activate.
* `seabed_mesh` empty with active patches → fewer than 3 patches, or all
  triangles culled by the `eta * r_p` edge limit.

### Custom messages (`sonar_map_msgs`)

* **`SeabedEstimatorState`** — `mu_h, var_h, mu_i, var_i`.
* **`ClassifiedReturns`** — per-scan batch: parallel arrays
  `x, y, z, intensity, slant_range, label, lidar_support`, plus the scan-level
  scalars `seabed_depth, tau_struct, tau_obj` so a consumer needs no other
  input. Label constants `FREE/SEABED/OBJECT/STRUCTURE`.
* **`DirichletVoxelMap`** — a full snapshot of the active voxel store as
  parallel arrays (`voxel_key`, `center_*`, the four `w_*`, `weight`,
  `mean_slant_range`) plus a flattened ragged ring buffer (`ring_count` +
  `ring_depths_flat`). `mean_slant_range` lets `seabed_surface` form the
  sensor-intrinsic measurement noise `sigma_sens = r * phi_max / 6`.
* **`PromotedVoxelKeys`** — the feedback message: the `int64[] keys` of voxels
  `seabed_surface` has promoted, sent back so `voxel_mapper` retires them.
