import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    lio_sam_launch = os.path.join(get_package_share_directory('lio_sam'), 'launch', 'run.launch.py')

    lio_sam_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lio_sam_launch)
    )

    imu_tf_node = ExecuteProcess(
        cmd=['ros2', 'run', 'tf_rebroadcaster', 'imu_lidar_transform'],
        output='screen'
    )

    gps_odom_node = ExecuteProcess(
        cmd=['ros2', 'run', 'gps_odometry', 'gps_odometry'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(lio_sam_launch_description)
    ld.add_action(imu_tf_node)
    ld.add_action(gps_odom_node)

    return ld