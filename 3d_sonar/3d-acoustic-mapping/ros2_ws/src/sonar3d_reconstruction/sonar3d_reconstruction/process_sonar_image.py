import numpy as np
import typing
from marine_acoustic_msgs.msg import ProjectedSonarImage
from geometry_msgs.msg import Vector3
import yaml
import cv2

dtype_mapping = {
    0: np.uint8,  # DTYPE_UINT8
    2: np.uint16,  # DTYPE_UINT16
    # Add other mappings as needed
}


def ProjectedSonarImage2Image(msg: ProjectedSonarImage, normalize=False) -> np.ndarray:
    """Converts a ProjectedSonarImage message to a numpy array.
    Also returns the ranges and beams that corresponds to the image.

    Parameters:
    msg -- The ProjectedSonarImage messag
    normalize -- Whether to normalize the image to the range [0, 255]

    Returns:
    image -- The sonar image
    ranges -- The range data in meters
    beams -- The beam directions in radians
    """

    ranges = np.array(msg.ranges)
    nbeams = msg.image.beam_count
    nrange = len(msg.ranges)

    # Convert beam directions from Vector3 to list of beam directions given in radians
    beams = np.array([np.arctan2(beam.y, beam.z) for beam in msg.beam_directions])

    # Get the image data and reshape it according to the dtype
    dtype = dtype_mapping[msg.image.dtype]
    image = np.frombuffer(msg.image.data, dtype=dtype).reshape((nrange, nbeams))

    # Check if beams are in the correct order (left to right) to ensure correct image orientation
    if beams[0] > beams[-1]:
        beams = np.flip(beams)
        image = np.flip(image, axis=1)

    image = image.astype(dtype=dtype)

    # normalize the image to the range [0, 255]
    if normalize or dtype == np.uint16:
        image = cv2.convertScaleAbs(image, alpha=(255.0 / image.max()))

    return image, ranges, beams


def filter_horizontal_image(imageHorizontal, method: list = ["otsu"], kernel_size=(3, 1)):
    """Filter the horizontal sonar image by removing the mean of the image over each beam and normalizing the image to the range [0, 255].
    
    Parameters:
    imageHorizontal -- The horizontal sonar image
    method -- The method to use for thresholding. Default is "otsu". Choises are "otsu" and "open"
    
    Returns:
    imageHorizontal -- The filtered horizontal sonar image
    """

    imageHorizontal = np.maximum(
        0, imageHorizontal - np.quantile(imageHorizontal, 0.1, axis=1)[:, None]
    )  # Remove the mean of the image over each beam.
    imageHorizontal = np.maximum(0, imageHorizontal - np.quantile(imageHorizontal, 0.1))
    imageHorizontal = (imageHorizontal / np.max(imageHorizontal) * 255).astype(np.uint8)
    if "otsu" in method:
        imageHorizontal *= cv2.threshold(imageHorizontal, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    if "open" in method:
        if kernel_size != 0:
            kernel = np.ones(kernel_size, np.uint8)
            imageHorizontal = cv2.morphologyEx(imageHorizontal, cv2.MORPH_ERODE, kernel, iterations=1)

    return imageHorizontal


def filter_vertical_image(imageVertical):
    # First remove the mean of the image over each beam.
    imageVertical = np.maximum(0, np.subtract(imageVertical, np.mean(imageVertical, axis=1, keepdims=True)))
    # We normalize the image to the range [0, 255] and convert it to uint8.
    imageVertical = (imageVertical / np.max(imageVertical) * 255).astype(np.uint8)
    # We remove additional noise introduced by high gain values in the center of the image.
    n_bearings_right = 40
    n_bearings_left = 25
    mask = imageVertical[:, 128 - n_bearings_left : 127 + n_bearings_right] < np.percentile(imageVertical[:, 127 - n_bearings_left : 127 + n_bearings_right], 98)
    imageVertical[:, 128 - n_bearings_left : 127 + n_bearings_right][mask] = 0
    # We apply a median filter for additional noise removal.
    imageVertical = cv2.medianBlur(imageVertical, 3)
    return imageVertical


def cut_image(
    image: np.ndarray, desired_FOV: float, beams: np.ndarray, ranges: np.ndarray, min_range: float
) -> np.ndarray:
    """
    Cut the sonar image based on the desired FOV.

    Parameters:
    image -- The sonar image
    desired_FOV -- The desired field of view to cut off
    beams -- The beams of the sonar image given in radians

    Returns:
    cut_image -- The cut sonar image
    beams_cut -- The beams that are left in the sonar image
    """

    # Cut range first
    start_range_idx = np.where(ranges >= min_range)[0][0]
    ranges_cut = ranges[start_range_idx:]
    image = image[start_range_idx:, :]

    # Given the desired FOV, find the indexes that cut higher FOV than the desired FOV
    start_FOV_idx = np.where(beams >= -desired_FOV / 2)[0][0]
    end_FOV_idx = np.where(beams <= desired_FOV / 2)[0][-1]

    cut_image = image[:, start_FOV_idx : end_FOV_idx + 1]
    beams_cut = beams[start_FOV_idx : end_FOV_idx + 1]

    return cut_image, ranges_cut, beams_cut


def polar2cartesian(
    ranges: np.ndarray, beams: np.ndarray, offset_x: float = 0.0, offset_y: float = 0.0
) -> typing.Tuple[np.ndarray, np.ndarray]:
    # 2D array of x coordinate in sonar fame
    x = np.outer(ranges, np.cos(beams)) + offset_x
    y = np.outer(ranges, np.sin(beams)) + offset_y

    return x, y


def filter_image_rows(image, ranges, x_coords, x_coords_other, y_coords):
    """remove the rows of the sonar image based on the x coordinates of the other sonar image.
    The filtering is done by comparing the minimum and maximum x coordinates of the sonar images.
    We are not interested in the rows that are outside the x coordinates of the other sonar image.

    Parameters:
    image -- The sonar image containing intesities
    ranges -- The range data in meters
    x_coords -- The x coordinates of the sonar image
    x_coords_other -- The x coordinates of the other sonar image
    y_coords -- The y coordinates of the sonar image

    """
    rows_keep = np.where((x_coords.min(axis=1) > x_coords_other.min()) & (x_coords.max(axis=1) < x_coords_other.max()))[
        0
    ]
    x_coords = x_coords[rows_keep, :]
    y_coords = y_coords[rows_keep, :]
    return image[rows_keep, :], ranges[rows_keep], x_coords, y_coords


def sound_speed_correction(
    rangesHorizontal,
    rangesVertical,
    msgHorizontal,
    msgVertical,
    sound_speed_horizontal,
    sound_speed_vertical,
    actual_sound_speed,
):
    """Correct difference in speed of sound by aligning the ranges. We assume that the horizontal sonar is the reference."""

    sound_speed_vertical = (
        # self.get_parameter("sonars.vertical.sound_speed").value
        sound_speed_vertical
        or msgVertical.ping_info.sound_speed
    )
    sound_speed_horizontal = sound_speed_horizontal or msgHorizontal.ping_info.sound_speed
    if sound_speed_vertical == 0 or sound_speed_horizontal == 0:
        raise ValueError(
            "Sound speed cannot be zero. The message does not contain the sound speed information and the parameter is not set."
        )
    if actual_sound_speed != 0:
        # We wish to adjust the current ranges to the actual sound speed
        rangesHorizontal *= actual_sound_speed / sound_speed_horizontal
        rangesVertical *= actual_sound_speed / sound_speed_vertical
    else:
        # We wish to adjust the current ranges to the horizontal sound speed by default
        rangesVertical *= sound_speed_horizontal / sound_speed_vertical

    return rangesHorizontal, rangesVertical
