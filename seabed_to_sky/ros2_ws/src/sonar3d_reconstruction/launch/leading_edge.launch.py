import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("sonar3d_reconstruction"),
        "config",
        "leading_edge.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="sonar3d_reconstruction",
                executable="leading_edge_node",
                name="leading_edge_node",
                output="screen",
                parameters=[config],
            ),
            # Interactive raw-amplitude viewer. Pure observer — self-gates on
            # its own `enable` flag, so leaving it here costs nothing when off.
            Node(
                package="sonar3d_reconstruction",
                executable="sonar_viewer",
                name="sonar_viewer",
                output="screen",
                parameters=[config],
            ),
        ]
    )
