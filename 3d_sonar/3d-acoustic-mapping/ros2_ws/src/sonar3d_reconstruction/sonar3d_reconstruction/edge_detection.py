import numpy as np
from sonar3d_reconstruction.profile import Profile
from sklearn import linear_model
from typing import Tuple


def _get_falling_edges(occupancy: np.ndarray) -> np.ndarray:
    """
    Get the indices of the falling edges in the occupancy array.
    :param occupancy: The occupancy array.
    :return: An array of indices of the falling edges.
    """

    indices = _get_leading_edges(occupancy)
    for i in range(len(indices)):
        j = indices[i]
        while j < len(occupancy) - 1 and occupancy[j, i] != 0:
            j += 1
        indices[i] = j

    return indices


def _get_leading_edges(occupancy: np.ndarray) -> np.ndarray:
    """
    Get the indices of the leading edges in the occupancy array.
    :param occupancy: The occupancy array.
    :return: An array of indices of the leading edges.
    """

    indices = np.argmax(occupancy, axis=0)
    return indices


def image2Profile(
    image: np.ndarray,
    ranges: np.ndarray,
    beams: np.ndarray,
    threshold: float,
    method: str = "leading",
    pitch: float = 0.0,
) -> Profile:
    """Converts sonar image to profile given method and threshold.
    :param image: The image array for getting the intensity values.
    :param ranges: The ranges of the image given in meters.
    :param beams: The beams of the image given as angles in radians.
    :param threshold: The threshold value.
    :param method: The method to use for edge detection ('leading' or 'falling').
    :param pitch: The incline of the sonar in radians. This can be used to alter the assumption

    # Profiles are assumed to be in sonar frame, i.e
    # x is forward, y is right, z is down
    # The pitch is the angle. If the sonar is tilted downward,
    # the pitch is positive as it is in the sonar frame.

    of the inclination of the plane that the points are on.
    :return: A Profile object with the real world coordinates.
    """

    occupancy = image >= threshold
    if method == "leading":
        edges = _get_leading_edges(occupancy)
    elif method == "falling":
        edges = _get_falling_edges(occupancy)
    else:
        raise ValueError("Method must be either leading or falling.")

    r = ranges[edges]
    if pitch != 0.0:
        x = r * np.cos(beams) * np.cos(pitch)
        y = r * np.sin(beams) * np.cos(pitch)
        z = r * np.sin(pitch)
    else:
        x = r * np.cos(beams)
        y = r * np.sin(beams)
        z = np.zeros_like(x)
    delay = np.zeros_like(x)  # Assume no delay for now
    intensity = image[edges, np.arange(len(edges))]
    valid = occupancy[edges, np.arange(len(edges))].astype(bool)

    return Profile(r, beams, x, y, z, delay, intensity, valid)


def ransac_on_profile(
    profile: Profile, max_trials: int = 200, residual_threshold: float = 1.0
) -> Tuple[Profile, linear_model.RANSACRegressor]:
    """
    Apply RANSAC to the profile.
    :param profile: The profile to apply RANSAC to. It does't remove points, but changes the valid attribute.
    :return: The profile with RANSAC applied.
    """
    x = profile.x
    y = profile.y

    ransac = linear_model.RANSACRegressor(max_trials=max_trials, residual_threshold=residual_threshold)
    ransac.fit(y.reshape(-1, 1), x)
    inlier_mask = ransac.inlier_mask_
    profile.valid = inlier_mask

    return profile, ransac


def mad(x: np.ndarray) -> float:
    """
    Calculate the median absolute deviation (MAD) of the array.
    :param x: The array to calculate the MAD of.
    :return: The MAD of the array.
    """
    return np.median(np.fabs(x - np.median(x)))


def rolling_MAD_loop(
    x: np.ndarray, window_size: int, threshold: float = 2.5, b: float = 1.4826
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Apply rolling median absolute deviation (MAD) to the profile.
    :param x: A 1D numpy array of the profile to apply rolling MAD to.
    :param window_size: The size of the window for the rolling MAD.
    :param threshold: The threshold for the MAD.
    :param b: The scaling factor for the MAD.
    :return: valid indices, upper bound, lower bound
    """
    n = len(x)
    half_window = window_size // 2

    # Pad x to handle the edges
    x_padded = np.pad(x, (half_window, half_window), mode="edge")

    rolling_median = np.zeros(n)
    rolling_mad = np.zeros(n)

    for i in range(n):
        window = x_padded[i : i + window_size]
        window_median = np.median(window)
        rolling_mad[i] = b * np.median(np.abs(window - window_median))
        rolling_median[i] = window_median

    upper_bound = rolling_median + (rolling_mad * threshold)
    lower_bound = rolling_median - (rolling_mad * threshold)

    valid = (x >= lower_bound) & (x <= upper_bound)

    return valid, upper_bound, lower_bound


def rolling_MAD(
    x: np.ndarray, window_size: int, threshold: float = 2.5, b: float = 1.4826
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Apply rolling median absolute deviation (MAD) to the profile.
    :param x: A 1D numpy array of the profile to apply rolling MAD to.
    :param window_size: The size of the window for the rolling MAD.
    :param threshold: The threshold for the MAD.
    :return: valid indices, upper bound, lower bound
    """
    n = len(x)
    half_window = window_size // 2

    # Pad x to handle the edges
    x_padded = np.pad(x, (half_window, half_window), mode="edge")

    # Create a rolling window view of the padded array
    shape = (n, window_size)
    strides = (x_padded.strides[0], x_padded.strides[0])
    windows = np.lib.stride_tricks.as_strided(x_padded, shape=shape, strides=strides)

    # Calculate rolling median and MAD
    rolling_median = np.median(windows, axis=1)
    rolling_mad = b * np.median(np.abs(windows - rolling_median[:, None]), axis=1)

    upper_bound = rolling_median + (rolling_mad * threshold)
    lower_bound = rolling_median - (rolling_mad * threshold)

    valid = (x >= lower_bound) & (x <= upper_bound)

    return valid, upper_bound, lower_bound
