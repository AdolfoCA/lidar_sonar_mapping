#!/usr/bin/env python3
"""
dirichlet_voxel.launch.py
-------------------------
Launches the eq-24 FIXED-CLASS Dirichlet-Categorical voxel mapping pipeline
end to end:

  0. sonar_scan_ned     — transforms raw sonar returns into the odom frame
  1. seabed_estimator   — scalar Kalman filter on seabed intensity μ_I
                          (telemetry in this pipeline: the class intensity
                          means μ_c are ABSOLUTE values from classes.yaml)
  2. return_classifier  — assembles the eq-9 measurement channels per return:
                          bottom reference ẑ_bed → h_i (eq 25), overhead
                          LiDAR column k_i/g_i (eqs 35–36); the SHAPE channels
                          m_i (eqs 39–40) are extracted UPSTREAM in
                          leading_edge_node (sonar3d_reconstruction) and ride
                          the sonar cloud through — no hard label is assigned
  3. voxel_mapper       — per-voxel Dirichlet posterior over the PREDEFINED
                          class set of config/classes.yaml + catch-all
                          "other": eq-24 factor likelihoods, eq-16
                          responsibilities, eq-20 count update, optional
                          eq-23 spatial prior; per-voxel label = argmax of
                          the posterior mean (eqs 21–22)

  (seabed_surface / promotion protocol not launched — the map is a persistent
   probabilistic representation. The node itself survives, generalized to
   promote any classes.yaml class by name via `seabed_class_name`, for stacks
   that still want surface promotion.)

The class-conditional likelihood is the paper's eq-24 factorization
    L_{i,c} = p(p_i|c) · p(I_i|p_i,c) · p(ℓ_i|c) · p(m_i|c)
with EVERY class parameter (Table I) read from config/classes.yaml. Node
wiring and the class-independent knobs live in config/dirichlet_voxel.yaml.

To swap sonar at launch time:
  ros2 launch sonar_map dirichlet_voxel.launch.py sonar_frame:=oculus

External inputs (not started here): the registered LiDAR cloud on
`/cloud_registered` and `odometry`, both from the rest of the seabed_to_sky
stack (or replayed from a bag); the leading-edge cloud (with the per-return
shape channels) from leading_edge_node of sonar3d_reconstruction.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('sonar_map')
    config = os.path.join(pkg, 'config', 'dirichlet_voxel.yaml')

    # ------------------------------------------------------------------ #
    # Arguments                                                           #
    # ------------------------------------------------------------------ #
    arg_sonar_frame = DeclareLaunchArgument(
        'sonar_frame',
        default_value='blueview_sonar',
        description='Active sonar TF frame: "blueview_sonar" or "oculus"',
    )
    arg_sonar_topic = DeclareLaunchArgument(
        'sonar_cloud_topic',
        default_value='/blueview/leading_edge',
        description='Raw sonar point cloud input topic (leading_edge_node '
                    'output; x/y/z/intensity float32 + the per-return shape '
                    'channels of eqs 39-40, extracted in image domain there)',
    )
    arg_debug_lidar_count = DeclareLaunchArgument(
        'debug_lidar_count',
        default_value='true',
        description='Start the lidar_count_debug observer node '
                    '(publishes /debug/lidar_count for Foxglove).',
    )
    arg_debug_csv = DeclareLaunchArgument(
        'debug_csv',
        default_value='false',
        description='Start the lidar_support_csv_logger node '
                    '(dumps per-return LiDAR-support rows to a CSV). The node '
                    'also self-gates on its enable_csv config param.',
    )
    arg_learned_mapper = DeclareLaunchArgument(
        'learned_mapper',
        default_value='false',
        description='Also start learned_voxel_mapper (online-learned, DYNAMIC '
                    'classes, intensity-only) IN PARALLEL with voxel_mapper, on '
                    'its own topics (learned_voxels / learned_voxel_map) for an '
                    'A/B comparison. voxel_mapper is unaffected.',
    )

    # ------------------------------------------------------------------ #
    # 0. sonar_scan_ned  — raw sonar returns -> odom-frame `sonar_scan`   #
    # ------------------------------------------------------------------ #
    node_sonar_scan = Node(
        package='sonar_map',
        executable='sonar_scan_ned',
        name='sonar_scan_ned',
        output='screen',
        parameters=[
            config,
            {'sonar_frame':       LaunchConfiguration('sonar_frame')},
            {'sonar_cloud_topic': LaunchConfiguration('sonar_cloud_topic')},
        ],
    )

    # ------------------------------------------------------------------ #
    # 1. seabed_estimator                                                 #
    # ------------------------------------------------------------------ #
    node_seabed_estimator = Node(
        package='sonar_map',
        executable='seabed_estimator',
        name='seabed_estimator',
        output='screen',
        parameters=[config],
    )

    # ------------------------------------------------------------------ #
    # 2. return_classifier — eq-9 measurement channels (h_i via the       #
    # rolling bed grid, overhead LiDAR column k_i/g_i, shape pass-through)#
    # ------------------------------------------------------------------ #
    node_return_classifier = Node(
        package='sonar_map',
        executable='return_classifier',
        name='return_classifier',
        output='screen',
        parameters=[config],
    )

    # ------------------------------------------------------------------ #
    # 3. voxel_mapper — eq-24 likelihoods over the classes.yaml inventory #
    # (class tuning lives THERE; wiring/knobs in dirichlet_voxel.yaml)    #
    # ------------------------------------------------------------------ #
    node_voxel_mapper = Node(
        package='sonar_map',
        executable='voxel_mapper',
        name='voxel_mapper',
        output='screen',
        parameters=[config],
    )

    # ------------------------------------------------------------------ #
    # 3b. learned_voxel_mapper (ALT, optional) — online-learned dynamic   #
    # classes, intensity-only. Runs in PARALLEL with voxel_mapper on its  #
    # own topics. Enable with:  learned_mapper:=true                      #
    # ------------------------------------------------------------------ #
    node_learned_voxel_mapper = Node(
        package='sonar_map',
        executable='learned_voxel_mapper',
        name='learned_voxel_mapper',
        output='screen',
        parameters=[config],
        condition=IfCondition(LaunchConfiguration('learned_mapper')),
    )

    # ------------------------------------------------------------------ #
    # (4. seabed_surface — NOT LAUNCHED)                                  #
    # The promotion / retirement protocol stays out of this launch: the   #
    # voxel map is a persistent probabilistic representation. Voxels      #
    # accumulate evidence indefinitely; the per-voxel label is the argmax #
    # of the posterior over the classes.yaml inventory + "other" (eq 22). #
    # The node itself remains available (generalized: promotes the class  #
    # named by `seabed_class_name`) for stacks that want promotion back.  #
    # ------------------------------------------------------------------ #

    # ------------------------------------------------------------------ #
    # 5. save_map — /save_map Trigger service (pure observer)            #
    # Saves map_topic's cloud as ASCII PCD with EVERY field it carries   #
    # (nothing hardcoded) + the FAST-LIO LiDAR map + pose metadata to    #
    # ~/ros2_ws/saved_maps/PCD/. Defaults to /sonar_map; set map_topic   #
    # to /dirichlet_voxels for the eq-24 posterior cloud (x/y/z + one    #
    # theta_<class> column per classes.yaml entry + display fields).     #
    # Call with:  ros2 service call /save_map std_srvs/srv/Trigger {}    #
    # ------------------------------------------------------------------ #
    node_save_map = Node(
        package='sonar_map',
        executable='save_map',
        name='save_map',
        output='screen',
        parameters=[{'map_topic': '/sonar_map'}],
    )

    # ------------------------------------------------------------------ #
    # Debug — lidar_count_debug  (pure observer, optional)               #
    # Enable with:  ros2 launch ... debug_lidar_count:=true              #
    # ------------------------------------------------------------------ #
    node_lidar_count_debug = Node(
        package='sonar_map',
        executable='lidar_count_debug',
        name='lidar_count_debug',
        output='screen',
        parameters=[config],
        condition=IfCondition(LaunchConfiguration('debug_lidar_count')),
    )

    # ------------------------------------------------------------------ #
    # Debug — lidar_support_csv_logger  (pure observer, optional)        #
    # Enable with:  ros2 launch ... debug_csv:=true                      #
    # (the node also self-gates on its enable_csv config param).         #
    # ------------------------------------------------------------------ #
    node_csv_logger = Node(
        package='sonar_map',
        executable='lidar_support_csv_logger',
        name='lidar_support_csv_logger',
        output='screen',
        parameters=[config],
        condition=IfCondition(LaunchConfiguration('debug_csv')),
    )

    return LaunchDescription([
        arg_sonar_frame,
        arg_sonar_topic,
        arg_debug_lidar_count,
        arg_debug_csv,
        arg_learned_mapper,
        node_sonar_scan,
        node_seabed_estimator,
        node_return_classifier,
        node_voxel_mapper,
        node_learned_voxel_mapper,
        node_save_map,
        node_lidar_count_debug,
        node_csv_logger,
    ])
