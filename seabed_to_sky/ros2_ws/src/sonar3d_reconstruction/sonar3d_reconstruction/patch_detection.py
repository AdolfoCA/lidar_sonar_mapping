import numpy as np
import cv2
from cfar.CFAR import CFAR
from typing import Tuple
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation as R


def features_to_XYI(
    features: np.ndarray, image: np.ndarray, x: np.ndarray, y: np.ndarray
) -> np.ndarray:
    """Convert features to XYI coordinates.

    Keyword Arguments:
    :param features: The features to convert. Dimensions is features x 2. (u,v)
    :param image: The image array for getting the intensity values.
    :param x: The x coordinates of the sonar image
    :param y: The y coordinates of the sonar image

    Returns:
    XYI: The XYI coordinates of the features. Dimensions is features x 5. (x, y, intensity, u, v)
    """

    x = x[features[:, 0].astype(int), features[:, 1].astype(int)]
    y = y[features[:, 0].astype(int), features[:, 1].astype(int)]
    intensity = image[features[:, 0].astype(int), features[:, 1].astype(int)]
    return np.vstack((x, y, intensity, features[:, 0], features[:, 1])).T


def cluster_features(
    XYI: np.ndarray, eps: float = 0.2, min_samples: int = 8
) -> np.ndarray:
    """Cluster the XYI features using DBSCAN.

    Keyword Arguments:
    :param XYI: The XYI features to cluster.
    :param eps: The maximum distance between two samples for one to be considered as in the neighborhood of the other.
    :param min_samples: The number of samples in a neighborhood for a point to be considered as a core point.

    Returns:
    labels: The cluster labels of the
    XYI: The XYI features without the noise points
    """
    XY = XYI[:, :2]
    XY = StandardScaler().fit_transform(XY)
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(XY)
    labels = db.labels_

    # Remove the noise points
    XYI = XYI[labels != -1]
    labels = labels[labels != -1]

    return labels, XYI


def cluster_descriptors(XYI: np.ndarray, labels: np.ndarray) -> np.ndarray:
    """Make descriptors from the XYI features to the cluster labels.

    Keyword Arguments:
    :param XYI: The XYI features to associate.
    :param labels: The cluster labels of the XYI features.

    Returns:
    cluster_descriptors: [mean, var, x_min, x_max]
    """

    clusters = np.unique(labels)
    cluster_descriptors = np.zeros((len(clusters), 4))

    for i, cluster in enumerate(clusters):
        cluster_data = XYI[labels == cluster, 0]
        cluster_descriptors[i] = [
            np.mean(cluster_data),
            np.var(cluster_data),
            np.min(cluster_data),
            np.max(cluster_data),
        ]

    return cluster_descriptors


def cluster_descriptors_vector(x: np.ndarray, labels: np.ndarray) -> np.ndarray:
    """Make descriptors from the x features to the cluster labels.

    Keyword Arguments:
    :param x: The x features to associate.
    :param labels: The cluster labels of the XYI features.

    Returns:
    cluster_descriptors: [mean, var, x_min, x_max]
    """

    clusters = np.unique(labels)
    cluster_descriptors = np.zeros((len(clusters), 4))

    for i, cluster in enumerate(clusters):
        cluster_data = x[labels == cluster]
        cluster_descriptors[i] = [
            np.mean(cluster_data),
            np.var(cluster_data),
            np.min(cluster_data),
            np.max(cluster_data),
        ]

    return cluster_descriptors


def match_clusters(CH: np.ndarray, CV: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Match the clusters from two sonar images.
    Loss function is ||cluster_1 - cluster_2||_2
    We use the Hungarian algorithm to find the optimal matching

    Keyword Arguments:
    :param CH: The cluster descriptors of the first sonar image.
    :param CV: The cluster descriptors of the second sonar image.

    Returns:
    matches: The matches between the clusters. Shape is (n_matches, 2)
    """

    cost_matrix = np.linalg.norm(CH[:, np.newaxis] - CV, axis=2)
    CH_Matches, CV_Matches = linear_sum_assignment(cost_matrix)

    return CH_Matches, CV_Matches


def extract_patches2(
    v: np.ndarray, u: np.ndarray, img: np.ndarray, size: int, rot: float = 0.0
):
    """A function to get the patch around all features.

    Keyword Arguments:
    v -- The vertical features
    u -- The horizontal features
    img -- The sonar image in intensity values
    size -- The size of the patch
    rot -- The rotation around the x axis (roll) of the sonar image in degrees
    """

    # Make sure v and u are integers
    v = v.astype(int)
    u = u.astype(int)

    # container for the image patches
    patches = np.empty((v.shape[0], 2))
    rotation = None

    # create a padded image to make the kernel always inside the image
    paddedImg = cv2.copyMakeBorder(
        img, size, size, size, size, cv2.BORDER_CONSTANT, value=0
    )

    if rot != 0:
        rotation = R.from_euler("z", rot, degrees=True)

    # Adjust coordinates for padding
    v = v + size
    u = u + size

    # Extract patches
    patches = np.array(
        [
            paddedImg[i - size : i + size + 1, j - size : j + size + 1]
            for i, j in zip(u, v)
        ]
    )

    if rotation is not None:
        patches = np.array([rotation.apply(patch) for patch in patches])

    # Calculate mu_x and mu_y
    mu_x = patches[:, 1, :].mean(axis=1)
    mu_y = patches[:, :, 1].mean(axis=1)

    return np.vstack((mu_x, mu_y))


def extract_patches(
    v: np.ndarray, u: np.ndarray, img: np.ndarray, size: int, rot: float = 0.0
):
    """A function to get the patch around all features.

    Keyword Arguments:
    v -- The vertical features
    u -- The horizontal features
    img -- The sonar image in intensity values
    size -- The size of the patch
    rot -- The rotation around the x axis (roll) of the sonar image in degrees
    """

    # Make sure v and u are integers
    v = v.astype(int)
    u = u.astype(int)

    # container for the image patches
    patches = np.empty((v.shape[0], 2))
    rotation = None

    # create a padded image to make the kernel always inside the image
    paddedImg = cv2.copyMakeBorder(
        img, size, size, size, size, cv2.BORDER_CONSTANT, value=0
    )

    if rot != 0.0:
        rotation = R.from_euler("z", rot, degrees=True)
    # loop over the set features
    iter = 0
    for j, i in zip(v, u):

        j = j + size
        i = i + size

        # add the kernel to a patch list, if vertical sonar, rotate the patch along the way
        patch = paddedImg[i - size : 1 + i + size, j - size : 1 + j + size]
        if rotation is not None:
            patch = rotation.apply(patch)

        # Get mu_x and mu_y
        patches[iter, 0] = patch[1, :].mean()
        patches[iter, 1] = patch[:, 1].mean()
        iter += 1

    return patches


def extract_features(
    img: np.ndarray, detector: CFAR, alg: str, threshold: float
) -> np.ndarray:
    """Function to take a raw greyscale sonar image and apply CFAR.

    Keyword Parameters:
    img -- raw greyscale sonar image
    detector -- detector object
    alg -- CFAR version to be used
    threshold -- CFAR thershold

    Returns:
    CFAR points as ndarray
    """

    # get raw detections
    peaks = detector.detect(img, alg)

    # check against threhold
    peaks &= img > threshold

    # compile points
    points = np.c_[np.nonzero(peaks)]

    # return a numpy array
    return np.array(points)


def match_within_clusters(
    CH_Matches: np.ndarray,
    CV_Matches: np.ndarray,
    horizontalXYI: np.ndarray,
    verticalXYI: np.ndarray,
    horizontalLabels: np.ndarray,
    verticalLabels: np.ndarray,
    horizontalImage: np.ndarray,
    verticalImage: np.ndarray,
    rot_vertical: float,
) -> np.ndarray:
    """Match the horizontal and vertical clusters within themselves.

    Keyword Arguments:
    CH_Matches -- The matches of the horizontal clusters
    CV_Matches -- The matches of the vertical clusters
    horizontalXYI -- The horizontal XYI features
    verticalXYI -- The vertical XYI features
    horizontalLabels -- The horizontal cluster labels
    verticalLabels -- The vertical cluster labels
    horizontalImage -- The horizontal sonar image
    verticalImage -- The vertical sonar image
    rot_vertical -- The rotation of the vertical sonar image in degrees

    Returns:
    matches -- The matches between the horizontal and vertical clusters. Shape is (n_matches, 9)
               The array has the following structure:
               [uh, vh, uv, vv, confidence, x_mean, y_horizontal, y_vertical, mean_intensity]

    """

    iter = 0
    matches = np.zeros(
        (
            np.min(
                [
                    len(horizontalXYI[np.isin(horizontalLabels, CH_Matches)]),
                    len(verticalXYI[np.isin(verticalLabels, CV_Matches)]),
                ]
            ),
            9,
        )
    )
    for i, j in zip(CH_Matches, CV_Matches):
        horizontalCluster = horizontalXYI[horizontalLabels == i]
        verticalCluster = verticalXYI[verticalLabels == j]

        # Extract the patches
        horizontalPatches = extract_patches(
            horizontalCluster[:, 4], horizontalCluster[:, 3], horizontalImage, 1
        )
        verticalPatches = extract_patches(
            verticalCluster[:, 4],
            verticalCluster[:, 3],
            verticalImage,
            1,
            rot=rot_vertical,
        )

        # z = [x, intensity, mu_x, mu_y]
        zHorizontal = np.vstack(
            (
                horizontalCluster[:, 0],
                horizontalCluster[:, 2],
                horizontalPatches[:, 0],
                horizontalPatches[:, 1],
            )
        ).T
        zVertical = np.vstack(
            (
                verticalCluster[:, 0],
                verticalCluster[:, 2],
                verticalPatches[:, 0],
                verticalPatches[:, 1],
            )
        ).T

        # Calculate the cost matrix
        costMatrix = np.linalg.norm(zHorizontal[:, np.newaxis] - zVertical, axis=2)
        horizontalMatches, verticalMatches = linear_sum_assignment(costMatrix)

        # Calculate the confidence
        # confidence = np.min(costMatrix[horizontalMatches, verticalMatches] / np.max(costMatrix, axis=1))

        # Calculate confidence of matches
        confidence = costMatrix[horizontalMatches, verticalMatches]

        # Standardize the confidence
        confidence = 1 - (confidence - confidence.min()) / (
            confidence.max() - confidence.min()
        )  # TODO: Fix error occuring here (division by zero)

        match = np.vstack(
            (
                horizontalCluster[horizontalMatches, 3],
                horizontalCluster[horizontalMatches, 4],
                verticalCluster[verticalMatches, 3],
                verticalCluster[verticalMatches, 4],
                confidence,
                (
                    horizontalCluster[horizontalMatches, 0]
                    + verticalCluster[verticalMatches, 0]
                )
                / 2,  # x_mean
                horizontalCluster[horizontalMatches, 1],  # y
                -verticalCluster[verticalMatches, 1],  # z
                (
                    horizontalCluster[horizontalMatches, 2]
                    + verticalCluster[verticalMatches, 2]
                )
                / 2,  # intensity
            )
        ).T

        matches[iter : iter + match.shape[0]] = match
        iter += match.shape[0]

    # Remove zero rows
    matches = matches[~np.all(matches == 0, axis=1)]

    return matches
