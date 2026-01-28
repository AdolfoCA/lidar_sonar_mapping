# LIO-SAM SLAM Docker Container

## Contents

### ROS2 Packages

| Package | Description |
|----------|----------|
|   [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/tree/ros2)  | Package implementation for LIO-SAM SLAM algorithm.  |
|   [robot_localization](https://github.com/cra-ros-pkg/robot_localization)  | Package implementing the GNSS + IMU localization fusion.  |
|   [nmea_converter](https://github.com/MapIV/nmea_converter/tree/ros2)  | Package for converting NMEA Sentences to `nmea_msg/msgs/NavSatFix`, required by `robot_localization`.  |
|   tf_rebroadcaster  | Custom package responsible for rebroadcasting the IMU data in the LiDAR- or base link frame, required for sensor fusion with both `robot_localization` & `LIO-SAM` packages. |

## Building the LIO-SAM container

The container for LIO-SAM contains a `ros2_ws` workspace with all relevant packages and dependencies to play `mcap`-rosbags, run LIO-SAM and auxiliary ROS2 packages.

To build the LIO-SAM container, navigate to the `lio-sam` directory, where the `Dockerfile` is located, and use the following command:

```bash
docker build -t lio-sam-thesis .
```

This will fetch dependencies and build all relevant packages for the LIO-SAM container.

## Running the LIO-SAM container

To run the LIO-SAM container, navigate to the `lio-sam` directory, where the `docker-compose.yaml`-file is located, and use the following command:

```bash
docker compose up -d
```

This will start the LIO-SAM container in the background as a daemon.
**NOTE:** Be sure to have no build-folders on the `ros2_ws`-directory, as this will cause the container to fail to start.

 To enter the container, use the following command:

```bash
docker exec -it lio-sam-container bash
```
