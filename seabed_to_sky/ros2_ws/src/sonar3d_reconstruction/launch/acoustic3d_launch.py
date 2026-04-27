import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory("sonar3d_reconstruction")

    # Define the parameter file paths
    params_edge_horizontal = os.path.join(
        package_dir, "config", "params_edge_blueview.yaml"
    )
    
    return LaunchDescription(
        [
            Node(
                package="sonar3d_reconstruction",
                executable="acoustic3d_edge",
                name="acoustic3d_edge_horizontal",
                parameters=[params_edge_horizontal],
                output="screen",
            )
        ]
    )
