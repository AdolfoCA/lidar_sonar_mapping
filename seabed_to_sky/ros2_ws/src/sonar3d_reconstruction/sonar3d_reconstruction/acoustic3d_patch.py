import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch
from matplotlib import cm
import os

# from match_cpp import matchFeatures as match_features_cpp
from cfar.CFAR import CFAR
from .transform_tools import create_global_transforms, compute_relative_transform
from sonar3d_reconstruction.process_sonar_image import (
    ProjectedSonarImage2Image,
    cut_image,
    filter_image_rows,
    polar2cartesian,
    filter_horizontal_image,
    filter_vertical_image,
)
from sonar3d_reconstruction.sonar_to_pointcloud import populate_profile, profile_to_laserfield
from sonar3d_reconstruction.patch_detection import (
    features_to_XYI,
    cluster_features,
    cluster_descriptors,
    match_clusters,
    match_within_clusters,
    extract_features,
)
from sonar3d_reconstruction.plot_tools import plot_sonar_image
from marine_acoustic_msgs.msg import ProjectedSonarImage


class Acoustic3dPatch(Node):
    """Class for stereo sonar images"""

    def __init__(self):
        super().__init__("acoustic3d_patch")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("sonars.horizontal.topic", Parameter.Type.STRING),
                ("sonars.horizontal.aperture", Parameter.Type.DOUBLE),
                ("sonars.horizontal.frame_id", Parameter.Type.STRING),
                ("sonars.horizontal.sound_speed", Parameter.Type.DOUBLE),
                ("sonars.vertical.topic", Parameter.Type.STRING),
                ("sonars.vertical.aperture", Parameter.Type.DOUBLE),
                ("sonars.vertical.frame_id", Parameter.Type.STRING),
                ("sonars.vertical.sound_speed", Parameter.Type.DOUBLE),
                ("sonars.min_range", Parameter.Type.DOUBLE),
                ("sonars.sound_speed_actual", Parameter.Type.DOUBLE),
                ("visualize.enable", Parameter.Type.BOOL),
                ("visualize.output_folder", Parameter.Type.STRING),
                ("uncertaintyMax", Parameter.Type.DOUBLE),
                ("patchSize", Parameter.Type.INTEGER),
                ("cfar.horizontal.tc", Parameter.Type.INTEGER),
                ("cfar.horizontal.gc", Parameter.Type.INTEGER),
                ("cfar.horizontal.pfa", Parameter.Type.DOUBLE),
                ("cfar.horizontal.threshold", Parameter.Type.DOUBLE),
                ("cfar.vertical.tc", Parameter.Type.INTEGER),
                ("cfar.vertical.gc", Parameter.Type.INTEGER),
                ("cfar.vertical.pfa", Parameter.Type.DOUBLE),
                ("cfar.vertical.threshold", Parameter.Type.DOUBLE),
                ("cluster.threshold", Parameter.Type.DOUBLE),
                ("cluster.min_samples", Parameter.Type.INTEGER),
                ("denoise.method", Parameter.Type.STRING),
                ("denoise.horizontal.enable", Parameter.Type.BOOL),
                ("denoise.horizontal.h", Parameter.Type.INTEGER),
                ("denoise.horizontal.templateWindowSize", Parameter.Type.INTEGER),
                ("denoise.horizontal.searchWindowSize", Parameter.Type.INTEGER),
                ("denoise.vertical.enable", Parameter.Type.BOOL),
                ("denoise.vertical.h", Parameter.Type.INTEGER),
                ("denoise.vertical.templateWindowSize", Parameter.Type.INTEGER),
                ("denoise.vertical.searchWindowSize", Parameter.Type.INTEGER),
            ],
        )

        self.bridge = CvBridge()
        self.denoise_method = self.get_parameter("denoise.method").value

        # Define the CFAR detectors
        self.detector_horizontal = CFAR(
            self.get_parameter("cfar.horizontal.tc").value,
            self.get_parameter("cfar.horizontal.gc").value,
            self.get_parameter("cfar.horizontal.pfa").value,
            None,
        )
        self.detector_vertical = CFAR(
            self.get_parameter("cfar.vertical.tc").value,
            self.get_parameter("cfar.vertical.gc").value,
            self.get_parameter("cfar.vertical.pfa").value,
            None,
        )

        # Sonar Subscribers
        self.sonar_horizontal_sub = Subscriber(
            self,
            ProjectedSonarImage,
            self.get_parameter("sonars.horizontal.topic").value,
        )
        self.sonar_vertical_sub = Subscriber(
            self, ProjectedSonarImage, self.get_parameter("sonars.vertical.topic").value
        )

        # TF Subscriber
        self.tf_sub = Subscriber(self, TFMessage, "/tf_static")
        self.tf_sub.registerCallback(self.tf_callback)
        self.rel_transform_linear = None
        self.rel_transform_rotation = None

        # Point Cloud Publisher
        self.cloud_publisher = self.create_publisher(PointCloud2, "sonar_cloud_patch", 10)

        # define time sync object for both sonar images
        self.timeSync = ApproximateTimeSynchronizer([self.sonar_horizontal_sub, self.sonar_vertical_sub], 1, 10)
        self.timeSync.registerCallback(self.fusion_callback)

        # Visualize
        if self.get_parameter("visualize.enable").value:
            self.get_logger().info("Visualization is enabled for patch matching.")
            self.timer = self.create_timer(2.0, self.save_image)

        self.match_status = False
        self.profile = None
        self.point_cloud_msg = None
        self.timestamp = None
        self.get_logger().info("Acoustic 3D Patch Node Started")

    def tf_callback(self, msg):
        """Callback for the TF message"""
        global_transform = create_global_transforms(msg, [])
        try:
            self.rel_transform_linear, self.rel_transform_rotation = compute_relative_transform(
                global_transform[self.get_parameter("sonars.horizontal.frame_id").value],
                global_transform[self.get_parameter("sonars.vertical.frame_id").value],
            )
        except KeyError as e:
            self.get_logger().error(
                f"Could not compute relative transform due to missing frames: {e}. Keeping the previous transform observed."
            )

    def sound_speed_correction(self, rangesHorizontal, rangesVertical, msgHorizontal, msgVertical):
        """Correct difference in speed of sound by aligning the ranges. We assume that the horizontal sonar is the reference."""

        sound_speed_vertical = (
            self.get_parameter("sonars.vertical.sound_speed").value or msgVertical.ping_info.sound_speed
        )
        sound_speed_horizontal = (
            self.get_parameter("sonars.horizontal.sound_speed").value or msgHorizontal.ping_info.sound_speed
        )
        if sound_speed_vertical == 0 or sound_speed_horizontal == 0:
            raise ValueError(
                "Sound speed cannot be zero. The message does not contain the sound speed information and the parameter is not set."
            )
        actual_sound_speed = self.get_parameter("sonars.sound_speed_actual").value
        if actual_sound_speed != 0:
            # We wish to adjust the current ranges to the actual sound speed
            rangesHorizontal *= actual_sound_speed / sound_speed_horizontal
            rangesVertical *= actual_sound_speed / sound_speed_vertical
        else:
            # We wish to adjust the current ranges to the horizontal sound speed by default
            rangesVertical *= sound_speed_horizontal / sound_speed_vertical

        return rangesHorizontal, rangesVertical

    def fusion_callback(self, msgHorizontal, msgVertical):
        """Ros callback for dual sonar system.

        Keyword Parameters:
        msgHorizontal -- horizontal sonar msg
        msgVertical -- vertical sonar msg
        """
        if self.rel_transform_linear is None:
            self.get_logger().debug("No relative transform available")
            return

        self.get_logger().debug("Received Sonar Images")

        self.timestamp = msgHorizontal.header.stamp
        imageHorizontal, rangesHorizontal, beamsHorizontal = ProjectedSonarImage2Image(msgHorizontal)
        imageVertical, rangesVertical, beamsVertical = ProjectedSonarImage2Image(msgVertical)

        # Correct difference in speed of sound by aligning the ranges
        rangesHorizontal, rangesVertical = self.sound_speed_correction(
            rangesHorizontal, rangesVertical, msgHorizontal, msgVertical
        )
        # Get the desired aperture. This corresponds to the horizontal elevation angle
        desired_aperture = np.deg2rad(self.get_parameter("sonars.horizontal.aperture").value)

        # Cut the image based on the desired aperture and adjust min range
        imageHorizontal, rangesHorizontal, beamsHorizontal = cut_image(
            imageHorizontal,
            desired_aperture,
            beamsHorizontal,
            rangesHorizontal,
            min_range=self.get_parameter("sonars.min_range").value,
        )
        imageVertical, rangesVertical, beamsVertical = cut_image(
            imageVertical,
            desired_aperture,
            beamsVertical,
            rangesVertical,
            min_range=self.get_parameter("sonars.min_range").value,
        )

        # 2D array of x coordinate in sonar fame
        xHorizontal, yHorizontal = polar2cartesian(rangesHorizontal, beamsHorizontal)
        xVertical, yVertical = polar2cartesian(
            rangesVertical, beamsVertical, self.rel_transform_linear[0], self.rel_transform_linear[2]
        )

        # Remove rows based on x coordinates. Don't look at areas that are not visible by the other sonar
        imageHorizontal, rangesHorizontal, xHorizontal, yHorizontal = filter_image_rows(
            imageHorizontal, rangesHorizontal, xHorizontal, xVertical, yHorizontal
        )
        imageVertical, rangesVertical, xVertical, yVertical = filter_image_rows(
            imageVertical, rangesVertical, xVertical, xHorizontal, yVertical
        )

        # Denoise the images
        if self.get_parameter("denoise.horizontal.enable").value:
            if self.get_parameter("denoise.method").value == "nlmeans":
                imageHorizontal = cv2.fastNlMeansDenoising(
                    imageHorizontal,
                    None,
                    self.get_parameter("denoise.horizontal.h").value,
                    self.get_parameter("denoise.horizontal.templateWindowSize").value,
                    self.get_parameter("denoise.horizontal.searchWindowSize").value,
                )
            elif self.get_parameter("denoise.method").value == "standard":
                imageHorizontal = filter_horizontal_image(imageHorizontal)
            else:
                self.get_logger().error("Unknown denoise method for horizontal sonar")
        if self.get_parameter("denoise.vertical.enable").value:
            if self.get_parameter("denoise.method").value == "nlmeans":
                imageVertical = cv2.fastNlMeansDenoising(
                    imageVertical,
                    None,
                    self.get_parameter("denoise.vertical.h").value,
                    self.get_parameter("denoise.vertical.templateWindowSize").value,
                    self.get_parameter("denoise.vertical.searchWindowSize").value,
                )
            elif self.get_parameter("denoise.method").value == "standard":
                imageVertical = filter_vertical_image(imageVertical)
            else:
                self.get_logger().error("Unknown denoise method for vertical sonar")

        # get some features using CFAR
        horizontalFeatures = extract_features(
            imageHorizontal, self.detector_horizontal, "SOCA", self.get_parameter("cfar.horizontal.threshold").value
        )
        verticalFeatures = extract_features(
            imageVertical, self.detector_vertical, "SOCA", self.get_parameter("cfar.vertical.threshold").value
        )

        # convert the features to XYI (x, y, intensity np.array)
        horizontalXYI = features_to_XYI(horizontalFeatures, imageHorizontal, xHorizontal, yHorizontal)
        verticalXYI = features_to_XYI(verticalFeatures, imageVertical, xVertical, yVertical)

        # Check if there are features to match
        if horizontalXYI.shape[0] == 0 or verticalXYI.shape[0] == 0:
            self.get_logger().info("No features to match")
            return

        # Cluster the features.
        horizontalLabels, horizontalXYI = cluster_features(
            horizontalXYI,
            self.get_parameter("cluster.threshold").value,
            self.get_parameter("cluster.min_samples").value,
        )
        verticalLabels, verticalXYI = cluster_features(
            verticalXYI, self.get_parameter("cluster.threshold").value, self.get_parameter("cluster.min_samples").value
        )

        # match the clusters within themselves
        horizontalDescriptors = cluster_descriptors(horizontalXYI, horizontalLabels)
        verticalDescriptors = cluster_descriptors(verticalXYI, verticalLabels)
        CH_Matches, CV_Matches = match_clusters(horizontalDescriptors, verticalDescriptors)

        roll_vertical_deg = np.rad2deg(self.rel_transform_rotation.as_euler("xyz")[0])
        matches = match_within_clusters(
            CH_Matches,
            CV_Matches,
            horizontalXYI,
            verticalXYI,
            horizontalLabels,
            verticalLabels,
            imageHorizontal,
            imageVertical,
            rot_vertical=roll_vertical_deg,
        )
        match_status = matches.shape[0] > 0

        if match_status:
            uncertaintyMax = self.get_parameter("uncertaintyMax").value
            self.profile = populate_profile(matches, uncertaintyMax)
            self.point_cloud_msg = profile_to_laserfield(
                self.profile,
                self.timestamp,
                frame_id=self.get_parameter("sonars.horizontal.frame_id").value,
            )

            self.cloud_publisher.publish(self.point_cloud_msg)

        self.get_logger().debug(f"Number of features: {horizontalFeatures.shape[0]}")
        self.get_logger().debug(f"Number of matches: {matches.shape[0]}")
        self.imgHorizontal = imageHorizontal
        self.imgVertical = imageVertical
        self.rangesHorizontal = rangesHorizontal
        self.rangesVertical = rangesVertical
        self.beamsHorizontal = beamsHorizontal
        self.beamsVertical = beamsVertical
        self.horizontalFeatures = horizontalFeatures
        self.verticalFeatures = verticalFeatures
        self.match_status = match_status
        self.matches = matches

    def save_image(self):
        """Save the image"""
        if self.match_status:
            timestamp = self.timestamp.sec + self.timestamp.nanosec * 1e-9
            readable_timestamp = datetime.fromtimestamp(timestamp).strftime("%Y%m%d_%H%M%S")
            if not os.path.exists(self.get_parameter("visualize.output_folder").value):
                os.makedirs(self.get_parameter("visualize.output_folder").value)
            fig, ax = plt.subplots(1, 2, figsize=(5, 5))
            plot_sonar_image(ax[0], self.imgHorizontal, self.rangesHorizontal, self.beamsHorizontal, "Oculus Sonar")
            plot_sonar_image(ax[1], self.imgVertical, self.rangesVertical, self.beamsVertical, "BlueView Sonar")
            cmap = cm.get_cmap("viridis", self.matches.shape[0])
            for i in range(self.matches.shape[0]):
                x1, y1 = self.matches[i, 1], self.matches[i, 0]
                x2, y2 = self.matches[i, 3], self.matches[i, 2]
                con = ConnectionPatch(
                    xyA=(x1, y1), xyB=(x2, y2), coordsA="data", coordsB="data", axesA=ax[0], axesB=ax[1], color=cmap(i)
                )
                ax[1].add_artist(con)

            plt.suptitle(f"Time: {readable_timestamp}")
            plt.tight_layout()
            plt.savefig(f"{self.get_parameter('visualize.output_folder').value}/sonar_image_{readable_timestamp}.png")
            plt.close()


def main(args=None):
    rclpy.init(args=args)
    node = Acoustic3dPatch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
