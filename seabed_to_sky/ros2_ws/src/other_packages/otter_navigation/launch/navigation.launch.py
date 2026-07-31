from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the odometry_to_nmea node.
    
    The node reads configuration from ~/nmea_config.yaml
    
    Usage:
      ros2 launch your_package odometry_to_nmea.launch.py
    """
    
    return LaunchDescription([
        Node(
            package='otter_navigation',
            executable='odometry_to_nmea',
            name='odometry_to_nmea',
            output='screen',
            emulate_tty=True,
        ),
    ])