import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

"""This module provides tools for working with ROS transforms.
The function are made for this project and are not necessarily general purpose.
GitHub Copilot was used in the development of this script
"""

def compute_global_transform(parent_transform, child_transform):
    """
    Combines a parent transform with a child transform to compute the child's global transform.

    Args:
        parent_transform (tuple): (translation, rotation) of the parent in global coordinates.
        child_transform (tuple): (translation, rotation) of the child relative to the parent.

    Returns:
        tuple: (translation, rotation) of the child in global coordinates.
    """
    parent_translation, parent_rotation = parent_transform
    child_translation, child_rotation = child_transform

    # Apply parent rotation to child translation
    global_translation = parent_translation + parent_rotation.apply(
        [child_translation.x, child_translation.y, child_translation.z]
    )

    # Combine rotations
    global_rotation = parent_rotation * R.from_quat([
        child_rotation.x, child_rotation.y, child_rotation.z, child_rotation.w
    ])

    return global_translation, global_rotation


def create_global_transforms(tf_message, exclude_frames = []):
    transforms = {}

    # Collect transforms
    for transform in tf_message.transforms:
        if transform.header.frame_id in exclude_frames or transform.child_frame_id in exclude_frames:
            continue
        parent_frame = transform.header.frame_id
        child_frame = transform.child_frame_id
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        transforms[child_frame] = (parent_frame, translation, rotation)

    global_transforms = {}

    # Compute global transforms
    for frame, (parent, translation, rotation) in transforms.items():
        if parent not in global_transforms:
            # Assume parent is at the origin if not specified
            global_transforms[parent] = (np.array([0, 0, 0]), R.from_quat([0, 0, 0, 1]))

        parent_global_transform = global_transforms[parent]
        global_transforms[frame] = compute_global_transform(
            parent_global_transform,
            (translation, rotation)
        )
        
    return global_transforms

def compute_relative_transform(global_transform_1, global_transform_2):
    """
    Computes the relative transform between two frames given their global transforms.

    Args:
        global_transform_1 (tuple): (translation, rotation) of the first frame in global coordinates.
        global_transform_2 (tuple): (translation, rotation) of the second frame in global coordinates.

    Returns:
        tuple: (translation, rotation) of the second frame relative to the first frame.
    """
    translation_1, rotation_1 = global_transform_1
    translation_2, rotation_2 = global_transform_2

    # Compute relative translation
    relative_translation = rotation_1.inv().apply(translation_2 - translation_1)

    # Compute relative rotation
    relative_rotation = rotation_1.inv() * rotation_2

    return relative_translation, relative_rotation

def compute_relative_transform_matrix(global_transform_1, global_transform_2):
    relative_translation, relative_rotation = compute_relative_transform(global_transform_1, global_transform_2)
    transorm_matrix = np.eye(4)
    transorm_matrix[:3, :3] = relative_rotation.as_matrix()
    transorm_matrix[:3, 3] = relative_translation
    return transorm_matrix

def rotate_vector_with_quarternion(vector, quaternion):
    r = R.from_quat(quaternion)

    rotated_vector = r.apply(vector)

    return rotated_vector

def rotate_point_by_quaternion(point, quaternion):
    """
    Rotates a 3D point using a quaternion via the Hamilton product.
    
    :param point: Tuple (x, y, z) representing the 3D point.
    :param quaternion: Tuple (qx, qy, qz, qw) representing the quaternion.
    :return: Tuple (x', y', z') representing the rotated 3D point.
    """
    px, py, pz = point
    qx, qy, qz, qw = quaternion
    
    # Convert point into a quaternion (px, py, pz, 0)
    point_quat = (px, py, pz, 0)
    
    # Quaternion conjugate
    q_conj = (-qx, -qy, -qz, qw)
    
    # Hamilton product function
    def hamilton_product(q1, q2):
        a1, b1, c1, d1 = q1
        a2, b2, c2, d2 = q2
        return (
            d1*a2 + a1*d2 + b1*c2 - c1*b2,
            d1*b2 - a1*c2 + b1*d2 + c1*a2,
            d1*c2 + a1*b2 - b1*a2 + c1*d2,
            d1*d2 - a1*a2 - b1*b2 - c1*c2
        )
    
    # Rotate point using quaternion multiplication: q * p * q_conj
    temp = hamilton_product(quaternion, point_quat)
    rotated_quat = hamilton_product(temp, q_conj)
    
    # Extract rotated coordinates
    return rotated_quat[0], rotated_quat[1], rotated_quat[2]