# Ablation study — Dirichlet voxel mapping

These are **parameter overlays** for the ablation study. Each file changes only
the few parameters that disable one component of the pipeline. You pass the base
config first and the overlay second; ROS 2 merges them and the *later* file wins
per parameter:

```bash
ros2 launch sonar_map dirichlet_voxel.launch.py \
    extra_config:=$(ros2 pkg prefix sonar_map)/share/sonar_map/config/ablation/A1_no_lidar.yaml
```

> NOTE: the launch file does not yet accept a second `extra_config` argument. Until
> the bag is available, treat these as the *definition* of each run. The simplest
> way to run one is to copy the base `dirichlet_voxel.yaml`, paste the overlay's
> values over it, and launch with that single file. (When you have a bag, ask me
> to add an `extra_config:=` argument to the launch file + a driver script.)

---

## The two claims and which ablations defend them

**Claim 1 — LiDAR-aided classification.** Surface LiDAR support is what lets us
label underwater sonar returns as STRUCTURE where intensity alone cannot.
Defended by: **A1 (no LiDAR support)**, **A4 (no structure prior)**.

**Claim 2 — Probabilistic Dirichlet accumulation.** Soft, accumulated evidence
beats a single hard label and grows confident with the number of hits.
Defended by: **A5 (hard labels)**, the **semantic-evolution figure** (confidence
vs. #hits), and the **A6 voxel-size sweep** (more revisits → sharper posteriors).

The recipe in every run: **same bag**, then `ros2 service call /save_map
std_srvs/srv/Trigger {}` → one `dirichlet_voxels_<ts>.pcd` per variant. Score
each PCD against the reference (below).

---

## The variants

| ID | File | Disables | Mechanism (why it works) |
|----|------|----------|--------------------------|
| A0 | `A0_full_baseline.yaml` | nothing | The full method. All other rows compare to this. |
| A1 | `A1_no_lidar.yaml` | LiDAR support | `r_lidar→1e-6` ⇒ `count_factor≈0` ⇒ `f_L_geom≈0` ⇒ `r_str≈0`. No voxel can become STRUCTURE. |
| A2 | `A2_no_similarity.yaml` | neighbour-similarity gate | `n_min_neighbors→100000` ⇒ the neighbour query never has enough support ⇒ `similarity=1` always. |
| A3 | `A3_no_intensity_band.yaml` | absolute intensity band | `w_str→1e6` ⇒ `m_str≈1` for all `u` ⇒ the band stops gating STRUCTURE. |
| A4 | `A4_no_struct_prior.yaml` | LiDAR spatial prior | `beta_struct_prior=0` ⇒ no per-voxel STRUCTURE prior `alpha_str_prior`. (This is the current default, so A4≈A0 unless you raise the prior in A0.) |
| A5 | `A5_hard_labels.yaml` | soft responsibilities (approx.) | `alpha_i→large`, `w_str→small`, `sigma_sim→small` ⇒ the soft split collapses toward near-binary assignments. The closest config-only proxy for "hard labels"; a true hard-label variant needs a code flag (ask me). |
| A6 | `A6_voxelsize_*.yaml` | — (sweep) | `r_v ∈ {0.10, 0.20, 0.40}` ⇒ trade resolution vs. #revisits per voxel (evidence accumulation). |

A4 is listed for completeness — note it equals the baseline at the current
`beta_struct_prior=0`. If you want a meaningful prior ablation, set
`beta_struct_prior` > 0 in A0 and keep A4 at 0.

---

## Metrics (what makes a number out of "the map got worse")

Reference, in order of strength:

1. **Multibeam scan — real ground truth (headline).** Register the multibeam to
   `odom`, voxelize at the same `r_v`, and call a cell "structure" where the
   multibeam returns sit above the seabed. Then per run report, over STRUCTURE
   voxels:
   - **Precision / Recall / F1** vs. the multibeam structure cells.
   - **Surface error**: mean & RMS nearest-neighbour distance from predicted
     STRUCTURE voxel centres to the multibeam surface.
2. **LiDAR map — corroborating, above-water only.** Free (already exported). Use
   to validate coverage/geometry. Do NOT use it to score A1/A4 — LiDAR is also an
   *input* there, so it would be circular.
3. **Symmetry — qualitative figure only.** "Predicted underwater structure mirrors
   the above-water LiDAR about the waterline." A nice illustration, not a metric —
   too many real structures aren't symmetric.

Intrinsic measures (no ground truth needed), useful as secondary columns:
- **# confident voxels**: count with `max_c pi_c ≥ 0.7` (and `W ≥ w_conf`-ish).
- **mean STRUCTURE confidence** over voxels the method calls STRUCTURE.
- **map entropy**: mean per-voxel categorical entropy (lower = sharper).

### Output table shape
Rows = {A0, A1, A2, A3, A4, A5, A6.10, A6.20, A6.40}; columns =
Precision, Recall, F1, RMS surface err [m], #STRUCTURE voxels, #confident voxels,
mean STRUCTURE conf. One row per run, plus a grouped bar chart of F1 vs. ablation.

The story to land: A0 (full) is best; removing **any** component drops F1 /
raises surface error — each component earns its place.
