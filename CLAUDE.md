# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PhD research project building seabed-to-sky maps fusing LiDAR and sonar data in ROS2 Humble. The system transforms sonar scans into a probabilistic 3D voxel map and merges them with LiDAR point clouds.

## Docker Environment

The primary development environment is Docker. The workspace is in `seabed_to_sky/`:

```bash
# Build and start the container
cd seabed_to_sky/
docker compose up --build

# Or at the repo root (separate docker-compose.yml)
docker compose up --build
```

Inside the container, source ROS2 and the workspace:
```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

## Building the ROS2 Workspace

From inside the container or with ROS2 sourced:
```bash
cd seabed_to_sky/ros2_ws/
colcon build
# Or build a specific package
colcon build --packages-select sonar_map
colcon build --packages-select spark_fast_lio  # C++ — takes longer
```

## Running the System

```bash
# Main sonar mapping pipeline (3-node pipeline)
ros2 launch sonar_map sonar_mapping.launch.py

# Swap sonar type at runtime (default: blueview)
ros2 launch sonar_map sonar_mapping.launch.py sonar_frame:=oculus

# 3D reconstruction from sonar images
ros2 launch sonar3d_reconstruction acoustic3d_launch.py

# Static TF transforms
ros2 launch tf_static_publisher tf_static_launch.py

# Save sonar map to PCD file
ros2 service call /save_map std_srvs/srv/Trigger
```

## Running Tests

```bash
cd seabed_to_sky/ros2_ws/
colcon test --packages-select pcl_merger
colcon test-result --verbose
# Or run directly with pytest
pytest src/pcl_merger/test/
```

## Architecture

### Data Flow

```
LiDAR → spark_fast_lio (Fast-LIO2) → /odometry topic
Sonar raw → sonar_scan_ned → sonar_map_ned → 3D voxel map → save_map → .pcd file
Sonar + LiDAR → pcl_merger → merged point cloud
Sonar image → sonar3d_reconstruction (edge detection) → 3D reconstruction
```

### Key Packages (`seabed_to_sky/ros2_ws/src/`)

| Package | Language | Role |
|---------|----------|------|
| `spark-fast-lio/spark_fast_lio` | C++ | Modified Fast-LIO2 LiDAR odometry — provides pose estimates |
| `spark-fast-lio/sonar_map` | Python | Core sonar mapping: transforms scans + builds probabilistic voxel map |
| `sonar3d_reconstruction` | Python | 3D seabed reconstruction from orthogonal sonar images via edge detection |
| `pcl_merger` | Python | Merges sonar and LiDAR point clouds (assigns virtual ring numbers to sonar points) |
| `tf_static_publisher` | C++ | Publishes static transforms (sensor→robot frames) from YAML at 1 Hz |
| `cfar` | C++/pybind11 | CFAR detection for sonar returns |
| `marine_acoustic_msgs` | ROS2 msgs | DVL, multibeam sonar, imaging sonar message definitions |
| `apl_msgs` | ROS2 msgs | Custom `RawData.msg` definition |

### Sonar Map Pipeline (core logic)

The `sonar_map` package has three nodes:

1. **`sonar_scan_ned`** (`sonar_scan_ned.py`) — Subscribes to sonar PointCloud2, transforms points from sensor frame to odometry frame via TF chain. Publishes transformed cloud.

2. **`sonar_map_ned`** (`sonar_map_ned.py`) — Accumulates transformed scans into a 3D voxel map using a **Beta-Bernoulli probabilistic model** with 3 intensity regimes:
   - MISS: no return
   - SEABED HIT: return below intensity threshold
   - OBJECT HIT: return above intensity threshold
   Each voxel stores alpha/beta parameters updated via Bayesian updates.

3. **`save_map`** (`save_map.py`) — Service that serializes the voxel map to a `.pcd` file on request.

### Coordinate Frames

The system uses NED (North-East-Down) convention. Static transforms between robot body links and sensor frames are configured in `tf_static_publisher/config/transforms.yaml`. The TF chain is: `world → odom → base_link → sensor_frame`.

### Configuration

- Sonar parameters (topics, voxel size, intensity thresholds, TF frames): `sonar_map/config/sonar.yaml`
- Edge detection params per sonar type: `sonar3d_reconstruction/config/params_edge_blueview.yaml` / `params_edge_oculus.yaml`
- Static transforms: `tf_static_publisher/config/transforms.yaml`
- DDS middleware: `rtps_udp_profile.xml`

## Code Style

C++ files use `.clang-format` (in `seabed_to_sky/ros2_ws/src/`). Python follows PEP 257 + Flake8 enforced via pre-commit hooks (`.pre-commit-config.yaml`). C++ packages target C++20.
