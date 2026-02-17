from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Sonar Scan Node - transforms sonar points to odom frame
        Node(
            package='sonar_map',
            executable='norbit_scan_NED',
            name='norbit_scan_NED',
            output='screen',
            remappings=[
                ('sonar_cloud', '/norbit/bathymetry/points'),
                ('odometry', '/odometry'),
            ]
        ),

        # Sonar Map Node - accumulates sonar points into map
        Node(
            package='sonar_map',
            executable='norbit_map_NED',
            name='norbit_map_NED',
            output='screen',
            remappings=[
                ('sonar_scan_ned', '/sonar_scan_ned'),
                ('sonar_map', '/sonar_map'),
            ]
        ),

        # Save Map Server - saves maps to PCD files
        Node(
            package='sonar_map',
            executable='save_map',
            name='save_map',
            output='screen',
            remappings=[
                ('sonar_map', '/sonar_map'),
            ]
        ),
    ])