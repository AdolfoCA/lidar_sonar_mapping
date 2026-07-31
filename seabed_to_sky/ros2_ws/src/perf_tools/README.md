# perf_tools — seabed-to-sky real-time performance suite

Measures, during a 1.0× rosbag replay, the per-stage processing latency, system
& per-process resource usage, and map growth of the seabed-to-sky mapping stack,
and produces publication-quality figures + a `summary.csv` for an IEEE paper
demonstrating live embedded performance on the Otter's Jetson.

Everything runs on a PC first (GPU/power telemetry is gracefully skipped) and
transfers unchanged to the Jetson, where `jtop`/`tegrastats` light up automatically.

## What is measured

Two chains, four stages, all matched **offline** by `header.stamp` (exact after
the three measurement patches below):

| Stage | From → To | Topics |
|---|---|---|
| A FastLIO odometry | LiDAR scan → odom | `/ouster/points` → `/odometry` |
| B FastLIO sky map  | LiDAR scan → registered cloud | `/ouster/points` → `/cloud_registered` |
| C Leading edge     | raw sonar → leading-edge cloud | `/blueview_message_polar` → `/blueview/point2/leading` |
| D Seabed insert    | leading-edge scan → integrated into map | `/blueview/point2/leading` → `/sonar_map/last_insert_stamp` |

- **Sky chain** = stage B. **Seabed chain** = C + D.
- Sonar source: **BlueView**.

### Measurement patches (minimal, non-semantic, measurement-only)

1. **FastLIO** (`spark_fast_lio/src/spark_fast_lio.cpp`): the scan-rate `/odometry`
   and `/cloud_registered` were stamped with `this->now()`; they are now stamped
   with the LiDAR scan time (`Measures.lidar_beg_time`, which equals the input
   `/ouster/points` header stamp). Pure `header.stamp` change — no algorithmic effect.
2. **Non-semantic seabed node** (`sonar_map/sonar_map/sonar_map_ned_no_semantic.py`):
   the map is published on a 1 Hz timer, so a map message has no single input stamp.
   We added a tiny debug publisher `/sonar_map/last_insert_stamp`
   (`geometry_msgs/PointStamped`) emitted the instant each leading-edge scan is folded
   into the map, carrying that scan's input stamp → **exact** per-scan insert latency.
   The Dirichlet **semantic** pipeline (`voxel_mapper.py` etc.) is **not** touched.

## Capture procedure (replay at 1.0×, sim time)

Build & source:
```bash
cd ~/ros2_ws
colcon build --packages-select spark_fast_lio sonar_map perf_tools
source install/setup.bash
```

Terminal 1 — the pipeline (note `use_sim_time:=true` everywhere):
```bash
ros2 launch spark_fast_lio os2.launch.yaml use_sim_time:=true
```
Terminal 2 — sonar leading edge + non-semantic seabed map:
```bash
ros2 run sonar3d_reconstruction acoustic3d_edge --ros-args \
    --params-file $(ros2 pkg prefix sonar3d_reconstruction)/share/sonar3d_reconstruction/config/params_edge_blueview.yaml \
    -p use_sim_time:=true
ros2 launch sonar_map sonar_mapping_no_semantic.launch.py use_sim_time:=true
```
Terminal 3 — start capture (launches all three loggers, dumps reproducibility info):
```bash
ros2 run perf_tools ... # or simply:
src/perf_tools/run_perf_capture.sh --bag ~/bags/mission_1h
```
Terminal 4 — when prompted, start the replay:
```bash
ros2 bag play ~/bags/mission_1h --clock --rate 1.0
```
When the bag finishes, press **Ctrl-C** in Terminal 3. Output lands in
`ros2_ws/perf_data/<run_id>/`:
```
messages.csv      # one row per message (latency_sniffer)
resources.csv     # 1 Hz system + per-process (resource_logger)
map_growth.csv    # per-update map size (map_growth_logger)
bag_info.txt nvpmodel.txt jetson_release.txt config.used.yaml
```

> **Warm-up:** the first 60 s of mission time are discarded by the analysis
> (`run.warmup_s`, override with `--warmup`). Let the bag run a little before
> reading too much into early numbers.

## Analysis (laptop — pandas + matplotlib only)

```bash
python3 src/perf_tools/perf_tools/analyze_perf.py \
    --run-dir perf_data/<run_id> --config src/perf_tools/config.yaml
```
Writes to `ros2_ws/plot/<run_id>/` — each figure as **PDF + PNG**:

`latency_cdf`, `stage_latency_box`, `stage_breakdown`, `degradation_over_time`
(hero figure), `rtf_over_time`, `resources_timeseries`, `cpu_per_component`,
`insertion_vs_mapsize`, `throughput_drops`, and **`summary.csv`** (the paper table:
median/p95/max per stage & chain, drop rates, achieved Hz, per-component CPU share
and effective ms/msg, mean power, peak temperature, total energy).

## Standalone usage / help

Every script is standalone and has `--help`:
```bash
python3 src/perf_tools/perf_tools/latency_sniffer.py  --help
python3 src/perf_tools/perf_tools/resource_logger.py  --help
python3 src/perf_tools/perf_tools/map_growth_logger.py --help
python3 src/perf_tools/perf_tools/analyze_perf.py     --help
src/perf_tools/run_perf_capture.sh --help
```

## PC vs Jetson notes

- **PC:** no `jtop`/`tegrastats`/`nvpmodel`/`jetson_release` — GPU%, power, SoC
  temps and those metadata dumps are written empty/SKIPPED with a warning. Per-core
  CPU, RAM, and per-process CPU/RSS work everywhere via `psutil`.
- **Jetson:** install `jetson-stats` (`sudo pip3 install jetson-stats`) for the
  richest telemetry; the suite auto-detects it.

## Configuration

All topic names, message types, nominal sensor rates, per-process match strings,
throttle temperature, warm-up and plot sizes live in [`config.yaml`](config.yaml).
Verify the BlueView nominal rate (marked `GUESS`) against `bag_info.txt` before the
final run. Requirements: Python 3.10; `rclpy, pandas, matplotlib, numpy, pyyaml,
psutil` (`jtop` optional). No seaborn.
