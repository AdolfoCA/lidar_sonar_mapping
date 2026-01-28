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
    Transform from frame 1 to frame 2.

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

def plot_tf_message(tf_message, exclude_frames=[]):
    """
    Plots the 2D transforms on the X-Z plane from a TFMessage, including rotation.

    Args:
        tf_message (tf2_msgs.msg.TFMessage): The TFMessage containing transforms to plot.
    """
    fig, ax = plt.subplots(figsize=(3, 5))

    ax.set_xlabel('X')
    ax.set_ylabel('Z')
    ax.set_title('TF Transforms on X-Z Plane')

    global_transforms = create_global_transforms(tf_message, exclude_frames)

    # Plot transforms
    for frame, (translation, rotation) in global_transforms.items():
        x, z = translation[0], translation[2]

        # Define orientation axes
        axes_length = 0.1
        x_axis = rotation.apply([axes_length, 0, 0])
        z_axis = rotation.apply([0, 0, axes_length])

        # Plot position
        ax.scatter(x, z, label=frame)
        ax.text(x + 0.03, z + 0.03, frame)

        # Plot orientation
        ax.quiver(x, z, x_axis[0], x_axis[2], color='r')
        ax.quiver(x, z, z_axis[0], z_axis[2], color='b')

    # ax.legend(loc='upper left', bbox_to_anchor=(1, 1), frameon=False)
    ax.axis('equal')
    ax.grid(True)
    plt.tight_layout()
    plt.show()

    return global_transforms
