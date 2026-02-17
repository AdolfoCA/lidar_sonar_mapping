import numpy as np
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from builtin_interfaces.msg import Time
from sonar3d_reconstruction.profile import Profile


def populate_profile(matches: np.ndarray, uncertaintyMax: float) -> Profile:
    match_status = matches.shape[0] > 0

    if match_status:
        x = matches[:, 5]
        y = matches[:, 6]
        z = matches[:, 7]
        delay = np.zeros_like(z)
        intensity = matches[:, 8].astype(int)
        valid = matches[:, 4] > uncertaintyMax
        bearing_angle = np.arctan2(y, x)
        r = np.sqrt(x**2 + y**2 + z**2)

        return Profile(r, bearing_angle, x, y, z, delay, intensity, valid)
    else:
        return Profile(
            np.array([]),
            np.array([]),
            np.array([]),
            np.array([]),
            np.array([]),
            np.array([]),
            np.array([]),
            np.array([]),
        )


def profile_to_laserfield(
    profile: Profile, timestamp: Time, frame_id: str, return_only_valid: bool = True
) -> PointField:
    """
    Processes laser scan data and publishes a point cloud for use in a ROS environment.

    Parameters:
    - Profile object containing the Cartesian coordinates of the points.
    - The timestamp of the message.
    - A boolean indicating whether to return only valid points.

    Returns:
    - A tuple containing the ROS point cloud message and the Cartesian coordinates of the points.
    """

    laser_fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    # assemble the point cloud for ROS, the order may be different based on your coordinate frame
    profile = profile.filter_valid() if return_only_valid else profile
    points = np.column_stack(
        (profile.x, profile.y, -profile.z, profile.intensity)
    )  # Note: the negative signs as the coordinate frame is different
    
    # ISSUE: the extracted points are flipped when concatenated with lidar
    # points further down if -y. Ideally, the points should be flipped

    # SOMAR coordinate frame
    # x: forward
    # y: right
    # z: down

    # ROS coordinate frame
    # x: forward
    # y: left
    # z: up

    # package the point cloud
    header = Header()
    header.frame_id = frame_id
    header.stamp = timestamp  # use input msg timestamp for better sync downstream
    laser_cloud_out = pc2.create_cloud(header, laser_fields, points)

    return laser_cloud_out
