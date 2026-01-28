# PCL Post-Processing

## Overview

The PCL post-processing module is responsible for concatenating the PointCloud2 topics of an Ouster LiDAR and Blueview imaging sonar.
The Ouster LiDAR is flattened, the Blueview pointclouds are encoded to match the Ouster LiDAR, and is then concatenated at the end of the flattened LiDAR point cloud.

The SONAR points will be given their own `ring`-id, corresponding to the max-ring in Ouster + 1.

## Contents

The directory structure is organized as follows:

- `configs`: Containing config yaml files to handle how to transform the SONAR pointcloud to match the LiDAR frame, and some data about the topics to use
- `merge_pcl.py`: Main script to call (see usage below)
- `pcl2.py`: Modified version of `sensor_msgs_py.point_cloud2` library to handle the padding at the end of the Ouster LiDAR point encoding.

## Configuration

```yaml
transform: # transform from sonar to lidar frame
  translation:
    x: 0.513
    y: 0.000
    z: -1.549
  rotation:
    x: 0.000
    y: 0.966
    z: 0.000
    w: 0.259
ros:
  lidar_cloud_topic: "/ouster/points"       # name of topic for the LiDAR pointcloud
  sonar_cloud_topic: "/blueview/points2"    # name of topic for the SONAR pointcloud
  output_cloud_topic: "/points/fusion"      # name of the topic for the resulting, concatenated pointcloud
  target_frame: "os_lidar"                  # frame given to the concatenated pointcloud messages
```

**TIP:** The transform can be obtained using `ros2 run tf2_ros tf2_echo <lidar_frame> <sonar_frame>` 

## Usage

To use the PCL post-processing module, follow these steps:

1. Install the required dependencies (see [`requirements.txt`](../../../../requirements.txt))
3. Navigate to the `pcl` (this) directory.
4. Concatenate the LiDAR and SONAR PointCloud2 topics in a desired `mcap` rosbag. 
```bash
python3 merge_pcl.py <path/to/input_bag.mcap> <path/to/output> <path/to/config.yaml>
```

This will generate an `mcap` rosbag containing the topic of the concatenated PointCloud2 topics.
This rosbag can then be merged with the desired rosbag.
