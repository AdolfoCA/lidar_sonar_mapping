import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ekf_launch = os.path.join(get_package_share_directory('robot_localization'), 'launch', 'ekf.launch.py')
    # ekf_launch_description = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(ekf_launch)
    # )

    gps_odom_node = ExecuteProcess(
        cmd=['ros2', 'run', 'gps_odometry', 'gps_odometry'],
        output='screen'
    )

    imu_tf_node = ExecuteProcess(
        cmd=['ros2', 'run', 'tf_rebroadcaster', 'imu_rep_transform'],
        output='screen'
    )

    ekf_node = ExecuteProcess(
        cmd=['ros2', 'run', 'kalman_filter', 'gnss_imu_filter'],
        output='screen'
    )
    
    # gps_path_node = ExecuteProcess(
    #     cmd=['ros2', 'run', 'gps_path', 'gps_path'],
    #     output='screen'
    # )

    # ekf_path_node = ExecuteProcess(
    #     cmd=['ros2', 'run', 'gps_path', 'ekf_path'],
    #     output='screen'
    # )


    ld = LaunchDescription()
    ld.add_action(gps_odom_node)
    ld.add_action(imu_tf_node)
    ld.add_action(ekf_node)
    # ld.add_action(ekf_launch_description)
    # ld.add_action(gps_path_node)
    # ld.add_action(ekf_path_node)

    return ld