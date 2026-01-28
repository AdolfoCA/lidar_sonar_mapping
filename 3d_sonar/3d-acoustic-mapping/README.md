# 3D Acoustic Mapping
The purpose of this project is to create a 3D acoustic map of the underwater environment using two multibeam sonars. The main ros2 node is called `acoustic3d` and is responsible for the 3D reconstruction of the environment. The node subscribes to the sonar data, where the message has to be of type "ProjectedSonarImages" and publishes a 3D point cloud of the environment. The node is implemented in C++ and uses the PCL library for the 3D reconstruction and its transformation.

## Installation
Use the dockercompose or development environment to install the necessary dependencies.

## TODO
- [x] Fix issue with installation of Jupyter notebook and that it doesn't recognize ros python packages
- [ ] Investigate the z-dimension of the data. It seemed to be wrong when visualizing the point cloud.
- [ ] Implement leading edge detection for the sonar data
- [ ] Enhance the way transformation is done in the node
- [ ] Expand the 3D orthogonal detection to use abritrary angles
- [ ] Fix the issue with eigen3 so we can use the C++ implementations of CFAR and Match


## Start node
```bash
ros2 run sonar3d_reconstruction acoustic3d --ros-args --params-file src/sonar3d_reconstruction/config/params.yaml 
```

### Manage mcap file using kappe
https://github.com/sensmore/kappe
```bash
kappe convert --config src/sonar3d_reconstruction/config/mcap_config.yaml
```

### Play ros2 bag file
```bash
ros2 bag play /home/rosdev/data/interim/sydhavn/rosbag_0.mcap --storage mcap
```

