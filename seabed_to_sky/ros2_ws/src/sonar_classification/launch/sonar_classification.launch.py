#!/usr/bin/env python3
"""
sonar_classification.launch.py
------------------------------
Launch the ARGUS semantic classification pipeline end to end.

    measurement_node      stages 2-3: build y_i = (p_i, I_i, l_i, m_i) per
                          return — radiometric compensation (eq 1), the local
                          bed fit and h_i (eq 18), the elevation variance
                          rho_var_i (eq 22), the overhead LiDAR column
                          (eqs 26-27), and the region descriptor (e_R, w_R)

    classification_node   stages 4-6: the four factor likelihoods (eq 34), the
                          spatial prediction from the frozen (t-1) snapshot
                          (eq 16), the responsibilities (eq 9), the Dirichlet
                          accumulation (eq 13) and the MAP report (eq 15)

EVERY PARAMETER COMES FROM config/sonar_classification.yaml. This file declares
no launch arguments and overrides nothing: there is exactly one place to change
a setting, and what you read in the YAML is what runs. (Class parameters live
in config/classes.yaml, which those nodes load by path.)

    REMEMBER: the launch reads the INSTALLED copy under
    install/sonar_classification/share/sonar_classification/config/. Editing
    either YAML under src/ needs a `colcon build` to take effect.

To run a variant, copy the YAML, edit it, and point the nodes at it:

    ros2 launch sonar_classification sonar_classification.launch.py \\
        --ros-args --params-file /abs/path/to/my_config.yaml

EXTERNAL PREREQUISITES (not started here — they belong to the detection and
localisation stacks):
  * the leading-edge cloud, TRANSFORMED INTO THE MAP FRAME, on `sonar_scan`
    (sonar_map's `sonar_scan_ned`, fed from sonar3d_reconstruction's
    `leading_edge_node`);
  * the registered LiDAR cloud on `/cloud_registered` and the vehicle pose on
    `odometry`.

DEBUG OUTPUTS, both switched on in the YAML:
  * `debug_census` -> a CSV of per-class voxel counts every n seconds. THE PLOT
    IS A SEPARATE STEP:
        ros2 run sonar_classification plot_class_census ~/ros2_ws/class_census.csv
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sonar_classification'),
        'config', 'sonar_classification.yaml')

    return LaunchDescription([
        # ── stages 2-3 ───────────────────────────────────────────────────────
        Node(
            package='sonar_classification',
            executable='measurement_node',
            name='measurement_node',
            output='screen',
            parameters=[config],
        ),
        # ── stages 4-6 (+ the debug class census) ────────────────────────────
        Node(
            package='sonar_classification',
            executable='classification_node',
            name='classification_node',
            output='screen',
            parameters=[config],
        ),
    ])
