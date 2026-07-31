# sonar_classification

Online probabilistic **semantic** seabed mapping: every voxel holds a
Dirichlet–Categorical belief over a fixed set of a-priori classes, and each
sonar leading-edge return contributes soft evidence to it.

This package owns **everything to do with semantic classification** — the
measurement channels, the four class-conditional likelihood factors, the
Dirichlet voxel map, the class definitions, and the debug tooling. It does not
own detection (leading-edge extraction and region growing, which live in
`sonar3d_reconstruction`) or localisation.

Per ping ("sweep") the system takes one surface point per beam, scores each
point against every class on four independent physical channels, and
accumulates the resulting responsibilities into a persistent voxel map. The MAP
label is reported.

**It is** a fixed-parameter Bayesian classifier over a voxel grid.
**It is not** a learning system: class parameters are assigned a priori and are
never updated. There is no online parameter estimation and no new-class birth.

---

## Quick start

```bash
colcon build --packages-select sonar_classification
source install/setup.bash

# the pipeline (needs sonar_scan, /cloud_registered and odometry — see below)
ros2 launch sonar_classification sonar_classification.launch.py

# afterwards: one plot, one curve per class
ros2 run sonar_classification plot_class_census ~/ros2_ws/class_census.csv
```

> **Everything is configured in `config/sonar_classification.yaml`.** The launch
> file declares **no arguments** and overrides nothing — there is exactly one
> place to change a setting, and what you read in the YAML is what runs. Class
> parameters live in `config/classes.yaml`.
>
> The nodes read the **installed copies** under
> `install/sonar_classification/share/`, so an edit under `src/` needs a
> `colcon build` (~1 s).
>
> **For a tuning session**, point both nodes at the source `classes.yaml`
> instead — then class edits are live on the next node restart, no rebuild:
>
> ```yaml
> measurement_node:
>   ros__parameters:
>     classes_file: "/abs/path/to/src/sonar_classification/config/classes.yaml"
> classification_node:
>   ros__parameters:
>     classes_file: "/abs/path/to/src/sonar_classification/config/classes.yaml"
> ```
>
> **Do not build this package with `--symlink-install`.** It symlinks the config
> correctly, but setuptools' develop mode leaves an `egg-link` with no
> `easy-install.pth`, so nothing puts the package metadata on `sys.path` and
> every node dies at startup with
> `importlib.metadata.PackageNotFoundError: sonar-classification`. If you have
> already done it, recover with:
>
> ```bash
> rm -rf build/sonar_classification install/sonar_classification
> colcon build --packages-select sonar_classification
> ```
>
> To run a variant, copy the YAML, edit it, and pass
> `--ros-args --params-file /abs/path/my_config.yaml`.

### Inputs (not started by this package)

| topic | type | source |
|---|---|---|
| `sonar_scan` | `PointCloud2` | leading-edge returns **in the map frame**, `intensity` = the raw count. In this workspace: `sonar_map/sonar_scan_ned` fed by `sonar3d_reconstruction/leading_edge_node`. |
| `/cloud_registered` | `PointCloud2` | registered surface-LiDAR cloud |
| `odometry` | `Odometry` | vehicle pose — fixes the slant range and the waterline |
| `/blueview_message_polar` | `ProjectedSonarImage` | the **raw** sonar — needed only by the dB debug image, which wants the whole image rather than the leading-edge returns. Same topic `leading_edge_node` reads. |

### Outputs

| topic | type | contents |
|---|---|---|
| `semantic_returns` | `PointCloud2` | the per-return measurement tuple (intermediate; directly inspectable) |
| `semantic_voxels` | `PointCloud2` | the map: centre, MAP label id, display confidence, evidence `M_v`, and one `theta_<class>` field per class |
| `class_census` | `Float32MultiArray` | live per-class voxel counts (debug census only) |

---

## Configuring the classes

**`config/classes.yaml` is the model.** Adding, removing, splitting or retuning
a class is an edit to that file plus a restart — never a code change. Each class
gives four blocks:

```yaml
- name: pillar
  position:  {l: 0.0, u: 8.0, tau: 0.30}      # height occupancy interval [m]
  intensity: {mu: -5.0, sigma: 5.0}           # compensated backscatter [dB]
  lidar:
    pi: 0.75                                  # P(validated overhead support | class)
    clearance: {kind: exp, gamma: 1.0}        # exp = surface-piercing
  shape:
    rho: 0.90                                 # P(forms a bright region | class)
    extent:    {median_m: 0.50, spread_factor: 2.0}
    thickness: {median_m: 0.10, spread_factor: 2.0}
```

The file's own header carries the three editing rules and a worked sanity
check. The one worth repeating here:

> **The responsibility is a ratio, so shared parameters cancel.** Two classes
> are separated *only* by the channels in which their parameters differ. To
> split a class, copy the shared channels verbatim and put the distinction in
> the one channel whose physics actually differs — `wall` and `pillar` below
> are identical everywhere except `extent`, because that is the only thing
> that physically differs. Do not invent a difference in a channel that is
> physically identical.

> **Editing this file needs a `colcon build`** by default — the nodes read the
> installed copy. Set `classes_file` to the source path (see Quick start) to
> make edits live during a tuning session, or to an entirely separate file to
> A/B a different inventory.

Node wiring, voxel size, the spatial-prediction radius and the debug switches
live separately in `config/sonar_classification.yaml`.

---

## Debug: the per-class voxel census

The headline debug feature. With `classification_node.debug_census: true` in
the config, every `census_period_sec` seconds the classification node counts how
many voxels each class currently owns and appends a row to a CSV — one column
per class — then publishes the same counts on `class_census`.

```
t_sec, stamp_iso, sweeps, n_voxels, seabed, vegetation, object, wall, pillar,
                                    mass_seabed, mass_vegetation, …
```

`plot_class_census` turns that into **one plot with one curve per class**:

```bash
ros2 run sonar_classification plot_class_census ~/ros2_ws/class_census.csv
ros2 run sonar_classification plot_class_census census.csv --metric both --dark
```

Two metrics are recorded, and reading them together is the point:

* **count** — how many voxels each class *owns* (its MAP label). The
  convergence history of the map, and the default plot.
* **mass** — how much soft evidence each class has *attracted*, `sum_v m_{v,c}`.
  It moves earlier and more smoothly. A flat count curve **with** a rising mass
  curve means "detected but always losing the argmax", not "not detected".

`census_min_evidence` sets the floor on `M_v` below which a voxel is not
counted, so a single grazing return cannot register as a confidently labelled
voxel. The CSV is truncated at start — one file per run, so two runs never
splice into one curve.

> **The plot is a separate step.** The node writes the CSV; nothing draws it for
> you. Run `plot_class_census` during or after the run.

---

## Debug: the sweep in compensated dB

`db_image_node` — a pure observer, with two modes.

**`mode: viewer` (default)** — an interactive window refreshed at the sonar's
own rate. **Hover the cursor anywhere on the fan** and it reports that pixel's
range, bearing, raw count, compensated dB, and which class `mu` is nearest.
That last line is the tuning tool: hover bare bed, hover a quay wall, and the
*difference* between the two dB readings is the `mu` offset you are trying to
assign. Keys: `s` snapshot, `q`/Esc quit.

**`mode: png`** — the headless fallback. Every `every_n` scans it writes
`sonar_db_image.png`: the same fan with a polar section grid and, in each
section, the **mean and median compensated dB** (plus the raw count when
`show_counts` is on, which is what makes the conversion hand-checkable).

Same fan layout as the detection stack's `sonar_image.png`, so the two sit
side by side — with two deliberate differences:

* **The units are dB, not raw counts.** The classifier never sees a count: the
  intensity factor is a Gaussian over the compensated level, and the point of
  the compensation is that the same material reads the *same* number at every
  range. A picture in raw uint16 can't be compared with anything in
  `classes.yaml` — the same seabed is ~5000 counts close in and ~900 far out. A
  picture in dB can: read a section as −21 dB, look up seabed's `mu: -22.0`, and
  you know how the intensity channel will vote. If the numbers march with range
  instead of holding steady, the compensation is wrong — run `fit_compensation`.
* **No leading edge is drawn.** This is the measurement field, not the
  detection. Overlaying the two invites reading a detection failure as an
  intensity problem, or the reverse.

Sections that are mostly below the noise floor are dimmed and labelled with the
fraction of cells that carried a return, so a number backed by 3% of its cells
never looks like one backed by all of them. The greyscale is stretched over a
percentile window, so one saturated specular hit can't wash out the picture; the
printed numbers are always exact regardless.

It writes in place, so you can keep the file open in a viewer and watch it
refresh. `flip_ranges` **must match** the detection stack's `leading_edge.yaml`
setting, or this image and the detector's disagree about which end is near.

---

## How it works

### The per-sweep algorithm

| stage | where | what |
|---|---|---|
| 1. Detection | *upstream* | condition a copy of the image, extract the leading edge per beam, robustify, grow bright regions |
| 2. Measurement | `measurement_node` | read the raw count from the **unconditioned** image, compensate it to dB; the slant range gives `rho_var_i`; the local bed fit gives `h_i` |
| 3. LiDAR query | `measurement_node` | the overhead column: coverage `o_i`, validated presence `s_i`, waterline-referenced clearance `g̃_i` |
| 4. Scoring | `classification_node` | the four factors → `log L_{i,c}`; the spatial prediction from the frozen `(t-1)` snapshot; responsibilities by log-sum-exp |
| 5. Accumulation | `classification_node` | add `r_i` to the hit voxel's concentrations; the morphology descriptor enters each voxel once per region per sweep |
| 6. Commit | `classification_node` | refresh the snapshot, report MAP labels |

### The four factors

```
L_{i,c} = p(p_i|c) · p(I_i|c) · p(l_i|c) · p(m_i|c)
          \___ every return ___/  \_ unity  _/  \_ hurdle over _/
                                    outside       bright regions
                                    coverage
```

* **Position** — a **soft box** over the class's height interval, not a
  Gaussian. A class owns an *interval*, not a typical height: a return anywhere
  on a wall between bed and surface is fully consistent with the wall. The
  class softness `tau_c` and the per-return elevation blur `rho_var_i` combine
  in quadrature at evaluation and are never folded into each other. `u == l`
  degenerates exactly to a Gaussian, so the seabed is the zero-extent member of
  the same family, not a special case.
* **Intensity** — Gaussian in **compensated dB**. Range is removed by
  construction, so the level is a material property rather than a trajectory
  statistic. Only *differences* matter; the uncalibrated offset cancels.
* **LiDAR support** — a coverage-gated hurdle. The overhead point count
  *validates* but never *discriminates* (it scales as 1/R², so it belongs to
  the trajectory), and the clearance is referenced to the **waterline**, not to
  the return — referencing the return would couple this channel to the position
  factor and break the factorisation.
* **Morphology** — a hurdle over log-Gaussians in the bearing extent `e_R` and
  the range thickness `w_R`. `e_R` splits wall from pillar; `w_R` splits a hard
  face (thin) from a volume scatterer such as vegetation (thick). The
  descriptor enters each voxel **once per region per sweep** — the region is one
  observation about a voxel, not `N_R` observations.

### Layout

```
sonar_classification/
  argus/                      pure NumPy — no ROS, unit-testable on synthetic data
    classes.py                the inventory + validator (classes.yaml)
    soft_interval.py          B(x; l, u, sigma) — shared by position and clearance
    compensation.py           radiometric compensation to range-free dB
    geometry.py               s_i, rho_var_i, h_i
    bottom_grid.py            the rolling local bed fit
    region_descriptor.py      e_R, w_R and region identity
    factor_{position,intensity,lidar,shape}.py
    likelihood.py             assembly + responsibilities
    voxel_map.py              Dirichlet map, snapshot, spatial prediction, census
    census.py                 the debug CSV
    db_render.py              the compensated-dB fan render
  measurement_node.py         stages 2–3
  classification_node.py      stages 4–6 (+ the census)
  db_image_node.py            debug observer: the dB fan image
  plot_class_census.py        the census plot
  fit_compensation.py         the flat-patch check / calibration
```

---

## Ablation switches

In `classes.yaml` under `ablation:`. Each changes exactly one behaviour and
leaves the rest identical, so the study isolates the effect.

| flag | off (default) | on |
|---|---|---|
| `position_gaussian` | soft box | mid-extent Gaussian |
| `lidar_legacy_gap_ref` | clearance from the waterline | clearance from the return |
| `shape_dedup_mode` | `per_voxel` — once per region per voxel per sweep | `inv_nr` — the legacy `1/N_R` weighting |
| `spatial_prior` | neighbour-evidence prediction | uniform `pi` (also via `eps: 0`) |

`factors: {position, intensity, lidar, shape}` disables a whole channel; a
disabled factor abstains for every return, so the rest still combine correctly.

---

## `sonar_tilt_deg` vs. the static TF — no clash

A reasonable worry, so to be explicit: **nothing in this package rotates the
point cloud.** Returns arrive already georeferenced — the `os_imu →
blueview_sonar` static transform did that upstream, in `sonar_scan_ned` — and
`measurement_node` reads `x/y/z` as given.

`global.sonar_tilt_deg` is used at exactly one place, the height sensitivity

```
s_i       = r_i · cos(theta)                  (eq 21)
rho_var_i = (s_i · Delta_phi)² / 12           (eq 22)
```

which is *how much of the unresolved elevation angle turns into height
uncertainty*. It scales the width of a blur; it is not a coordinate transform,
so it cannot double-apply the mounting rotation.

It does have to **agree** with the TF, though, since both describe the same
mounting angle — and two sources of truth drift. `measurement_node` therefore
looks up the mounting transform once at startup and warns if they disagree by
more than `tilt_check_tolerance_deg`:

```
TILT MISMATCH: classes.yaml global.sonar_tilt_deg = 15.0 deg, but the
os_imu -> blueview_sonar static transform carries 30.0 deg. … Set
global.sonar_tilt_deg: 30.0.
```

The check never overrides the config — `theta` is a declared mounting
constant, and inferring it from whatever TF happened to be available at
startup would make the model depend on TF timing. Set
`tilt_check_parent: ""` to disable it.

---

## Before trusting a map: calibrate the intensity

**Run this first**, over a homogeneous patch of known sediment:

```bash
ros2 run sonar_classification fit_compensation --duration 60
```

It bins the returns by range, takes the median compensated level per bin, fits
the residual trend, and prints the exact YAML to paste back. Three outcomes:

* **FLAT** → the compensation curve is correct; it reports the `C` that anchors
  the scale on the reference class. Proceed.
* **TRENDING** → residual TVG is still in the chain; it reports corrected `u`/`w`.
  Paste, re-run, repeat. **Do not proceed to semantics until it is flat** — a
  trending median means the intensity channel is reporting *range*, and any
  `mu_c` assigned against it is a trajectory statistic, not a material property.
* **floor closing on the median** → expected, not a bug: the reapplied
  transmission loss lifts a fixed noise floor, so the softest classes get
  sampled from their upper tail at far range. A selection effect of *detection*.
  Don't retune `mu_c` to chase it.

The trend is measured on the **compensated** signal, not the raw count, which
makes the procedure a fixed point: whatever `TVG_hw` is in force, the residual
is what's left, and folding it back in converges. You never need the SDK's
analytic TVG curve.

> **Read the two coefficients together, not separately.** Over a 2.5–9 m gate,
> `log10(rho)` and `rho` are nearly collinear, so their *sum* — the correction
> actually applied — is well determined while the split between `u` and `w` is
> not. The tool reports the conditioning and warns when this bites. Don't read a
> physical law out of the individual numbers, and don't extrapolate the
> correction outside the gate you fitted on.

### What `Gain = 35` and `TVG = 6` mean here

* **`Gain = 35` needs no entry anywhere.** It is range-flat, so it collapses
  entirely into `calibration_offset_db` and cancels from every responsibility.
  Subtracting it explicitly would change nothing.
* **`TVG = 6` is the range-varying half** and is exactly what `global.tvg`
  removes. The shipped `u: 0.0, w: 6.0` encodes the *linear* reading —
  `TVG_hw(rho) = 6 dB/m · rho` — which is the convention already assumed in
  [train_intensity_correction.py:42](../sonar3d_reconstruction/tools/train_intensity_correction.py#L42)
  (`gain + slope*R`, defaults `--tvg-slope 6 --gain 35`). That file's own
  docstring flags the form as swappable, and the BlueView SDK curve has not been
  verified against this unit — so **treat `w: 6.0` as a starting point, not as
  truth.** Across the 2.5–9 m gate it removes 15–54 dB; getting the *form* wrong
  is a tens-of-dB error, which dominates the ~15–20 dB spread of the entire
  material ladder. `fit_compensation` is what settles it.

Until `C` is set, the `mu_c` values in `classes.yaml` are placeholders on an
arbitrary scale. They still classify correctly relative to one another — the
offset cancels — but they will not match literature levels.

---

## Tuning the class levels: measure them, don't guess

Three parameters are not assignable from a datasheet and should be **counted at
a known target**: `mu_c`/`sigma_c` (the level and its spread), `pi_lidar_c`, and
`rho_c`. `sample_class` measures all three — park in front of a thing you can
name, gate the returns down to it, and paste what it prints.

```bash
# 1. the reference sediment first — bare bed, so gate on height near zero
ros2 run sonar_classification sample_class --label seabed \
    --height-max 0.25 --duration 45

# 2. everything else, as an OFFSET from the reference's measured mode
ros2 run sonar_classification sample_class --label wall \
    --height-min 0.8 --reference-db <mode from step 1> --duration 45
```

Gates combine (`--box`, `--height-min/max`, `--range-min/max`). For bare
sediment a height gate near zero usually suffices; for a wall, a height gate
well above the bed excludes the seabed returns.

**You don't have to fix `C` before doing this.** Only level *differences* enter
the responsibility, so sampling the reference class first and passing its mode
back as `--reference-db` gives offsets that are C-independent and stay correct
when the calibration constant is corrected later. The tool converts the offset
into the `mu_c` to paste, using the reference class's assigned `mu`.

**You do need the compensation flat first.** If the level still trends with
range, what you measure is a statistic of how far away you parked and it won't
transfer. `sample_class` reports the level drift across its own sample's range
spread and warns above 3 dB.

Two reporting choices worth knowing: the level is anchored on the **mode**, not
the mean, because the detection threshold censors the lower tail and pulls the
mean up; and `sigma` is `1.4826 × MAD`, so a few specular spikes don't inflate
the spread the class is supposed to describe. Each run appends to
`~/ros2_ws/class_samples.csv`, so a session builds a table.

Once you have the levels, remember rule 1 in `classes.yaml`: if two classes
measure within a couple of dB of each other, **that channel cannot separate
them** — put the distinction where the physics differs instead of widening the
gap by hand.

---

## Known limitations

* **`e_R` / `w_R` are derived, not measured.** The current extractor publishes
  the older descriptor set, so `region_descriptor` reconstructs the two lengths
  from the region's beam count and area. The extent derivation is exact; the
  thickness is an equivalent-rectangle approximation, exact for a rectangular
  highlight and biased low for a ragged one. Set `shape_source: native` once
  the extractor publishes `region_extent` / `region_thickness` directly.
* **The coverage mask is a radius test**, not a true sensor-coverage polygon. It
  errs toward abstaining, which is the safe direction, but a better mask lets
  you honestly raise `pi` for the piercing classes.
* **A bright, bearing-elongated, range-thin bedform (a sand ripple) shares the
  wall signature** in the shape channel. Left to intensity, height and the
  spatial prediction. The principled fix, if it becomes a problem, is another
  channel with genuinely distinct physics — a shadow-presence bit — not
  tightening `extent` until the ripple falls out.

---

## Tests

```bash
cd src/sonar_classification && python3 -m pytest test -q
```

They encode the invariants the design depends on, not just line coverage: the
Gaussian and uniform limits of the soft box, responsibility invariance under a
global dB offset, that an unobserved LiDAR column yields factor 1 for every
class, that `pi = 1` is refused by the config validator, that a large region's
shape evidence is not diluted by its own size, and that a sweep's result does
not depend on the order returns are processed in.
