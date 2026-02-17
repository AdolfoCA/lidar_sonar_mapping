import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from message_filters import Subscriber
from marine_acoustic_msgs.msg import ProjectedSonarImage
from cv_bridge import CvBridge
import cv2
from sonar3d_reconstruction.process_sonar_image import (
    ProjectedSonarImage2Image,
    filter_horizontal_image,
    filter_vertical_image,
)
from sonar3d_reconstruction.edge_detection import (
    image2Profile,
    ransac_on_profile,
    rolling_MAD,
)
from sonar3d_reconstruction.process_sonar_image import polar2cartesian
from sonar3d_reconstruction.sonar_to_pointcloud import profile_to_laserfield
import numpy as np
import os
import matplotlib.pyplot as plt
from datetime import datetime
from sonar3d_reconstruction.plot_tools import (
    set_size,
    set_style,
    get_dtu_color_palette,
    plot_sonar_image_cartesian,
    plot_RANSAC,
    plot_MAD,
)


class Acoustic3dEdge(Node):
    """Class for stereo sonar images"""

    def __init__(self):
        super().__init__("acoustic3d_edge")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("sonar.topic", Parameter.Type.STRING),
                ("sonar.threshold", Parameter.Type.DOUBLE),
                ("sonar.min_range", Parameter.Type.DOUBLE),
                ("sonar.sound_speed", Parameter.Type.DOUBLE),
                ("sonar.sound_speed_actual", Parameter.Type.DOUBLE),
                ("sonar.pitch", Parameter.Type.DOUBLE),
                ("denoise.standard", Parameter.Type.BOOL),
                ("denoise.open", Parameter.Type.INTEGER),
                ("denoise.RANSAC", Parameter.Type.BOOL),
                ("denoise.MAD", Parameter.Type.BOOL),
                ("method.leading_edge", Parameter.Type.BOOL),
                ("method.falling_edge", Parameter.Type.BOOL),
                ("output.topic", Parameter.Type.STRING),
                ("output.frame_id", Parameter.Type.STRING),
                ("output.visualize", Parameter.Type.BOOL),
                ("output.folder", Parameter.Type.STRING),
            ],
        )

        # Create subscriber and publisher
        self.sub = Subscriber(self, ProjectedSonarImage, self.get_parameter("sonar.topic").value)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("output.topic").value, 10)

        # Create CvBridge
        self.bridge = CvBridge()

        # Create output folder if it doesn't exist
        if self.get_parameter("output.visualize").value:
            self.get_logger().info("Visualization is enabeled for edge detection.")
            self.output_folder = self.get_parameter("output.folder").value
            if not os.path.exists(self.output_folder):
                os.makedirs(self.output_folder)
            # Timer to save image every second
            self.timer = self.create_timer(10.0, self.save_image)

        # Callback function
        self.sub.registerCallback(self.callback)

        # Initialize image storage
        self.image = None
        self.image_timestamp = None
        self.profile = None
        self.ranges = None
        self.beams = None
        self.pitch = np.deg2rad(self.get_parameter("sonar.pitch").value)
        self.lower_bound = None
        self.upper_bound = None
        self.regressor = None

        self.get_logger().info("Acoustic 3D Edge Node Started")

    def adjust_range_based_on_sound_speed(self, msg: ProjectedSonarImage):
        """Adjust the range based on the sound speed.

        The function adjusts the range based on the sound speed and the actual sound speed.
        """

        sound_speed = self.get_parameter("sonar.sound_speed").value
        actual_sound_speed = self.get_parameter("sonar.sound_speed_actual").value
        if actual_sound_speed == 0:
            return
        if sound_speed == 0:
            sound_speed = msg.info.sound_speed

        self.ranges = self.ranges * actual_sound_speed / sound_speed

    def callback(self, msg: ProjectedSonarImage):
        """Callback function for the subscriber.

        Parameters:
        - msg: The message received from the subscriber.

        The function converts the message to a numpy array, cuts the image range, gets the profile, creates the PointCloud2 message based on the leading edge, and publishes the message.
        """
        # Convert the message to a numpy array
        image, self.ranges, self.beams = ProjectedSonarImage2Image(msg)

        # Denoise the image
        if self.get_parameter("denoise.standard").value:
            if self.get_parameter("output.frame_id").value == "oculus":
                image = filter_horizontal_image(image)
            elif self.get_parameter("output.frame_id").value == "blueview":
                image = filter_vertical_image(image)
            else:
                self.get_logger().error("Invalid sonar type.")
                return

        if self.get_parameter("denoise.open").value:
            kernel = np.ones((3, 1), np.uint8)
            image = cv2.morphologyEx(
                image,
                cv2.MORPH_OPEN,
                kernel,
                iterations=self.get_parameter("denoise.open").value,
            )

        # Adjust the range based on the sound speed
        self.adjust_range_based_on_sound_speed(msg)

        # Cut the image range
        start_range_idx = np.where(self.ranges >= self.get_parameter("sonar.min_range").value)[0][0]
        self.ranges = self.ranges[start_range_idx:]
        self.image = image[start_range_idx:, :]
        self.image_timestamp = msg.header.stamp

        # Get the profile
        self.profile = image2Profile(
            self.image,
            self.ranges,
            self.beams,
            self.get_parameter("sonar.threshold").value,
            "leading",
            self.pitch,
        )

        if self.get_parameter("denoise.RANSAC").value:
            self.profile = self.profile.filter_valid()
            self.profile, self.regressor = ransac_on_profile(self.profile)

        if self.get_parameter("denoise.MAD").value:
            self.profile = self.profile.filter_valid()
            self.profile.valid, self.upper_bound, self.lower_bound = rolling_MAD(self.profile.x, 39, 3.0)

        # Create the PointCloud2 message
        msg = profile_to_laserfield(
            self.profile.filter_valid(),
            msg.header.stamp,
            self.get_parameter("output.frame_id").value,
        )

        # Publish the message
        self.pub.publish(msg)

    def save_image(self):
        if self.image is not None and self.image_timestamp is not None and self.profile is not None:
            set_style()
            if self.get_parameter("output.frame_id").value == "oculus":
                fig_size = set_size("thesis", fraction=1.0, height_ratio=1.0)
            elif self.get_parameter("output.frame_id").value == "blueview":
                fig_size = set_size("thesis", fraction=0.6, height_ratio=1.8)
            else:
                set_size("thesis")
            fig, ax1 = plt.subplots(1, 1, figsize=fig_size)
            # Plot the sonar image
            x, y = polar2cartesian(self.ranges, self.beams)
            plot_sonar_image_cartesian(ax1, self.image, y, x)
            ax1.scatter(
                self.profile.y[self.profile.valid],
                self.profile.x[self.profile.valid],
                color=get_dtu_color_palette("green"),
                label="Valid",
                s=1,
            )
            ax1.scatter(
                self.profile.y[~self.profile.valid],
                self.profile.x[~self.profile.valid],
                color=get_dtu_color_palette("dtured"),
                label="Invalid",
                s=1,
            )
            ax1.legend(loc="lower right")

            # Convert ROS timestamp to a human-readable format
            timestamp = self.image_timestamp.sec + self.image_timestamp.nanosec * 1e-9
            readable_timestamp = datetime.fromtimestamp(timestamp).strftime("%Y%m%d_%H%M%S")
            filepath = os.path.join(self.output_folder, f"sonar_image_{readable_timestamp}.png")

            # Adjust layout to minimize whitespace
            plt.tight_layout()
            plt.savefig(filepath)
            self.get_logger().debug(f"Saved image to {filepath}")

            # Close the figure
            plt.close(fig)
            if (
                self.lower_bound is not None
                and self.upper_bound is not None
                and self.get_parameter("denoise.MAD").value
            ):
                fig, ax = plt.subplots(1, 1, figsize=set_size("thesis", fraction=1, height_ratio=1.2))
                plot_MAD(self.profile, self.upper_bound, self.lower_bound, ax)
                plt.tight_layout()
                # Make time stamp for the plot
                plt.savefig(filepath[:-4] + "_MAD.pdf")
                plt.close()

                self.lower_bound = None
                self.upper_bound = None
            if self.regressor is not None and self.get_parameter("denoise.RANSAC").value:
                fig, ax = plt.subplots(1, 1, figsize=set_size("thesis", fraction=0.6, height_ratio=1.8))
                plot_RANSAC(self.profile, self.regressor, ax)

                plt.tight_layout()

                # Save the plot
                plt.savefig(filepath[:-4] + "_RANSAC.pdf")
                self.regressor = None
                plt.close()


def main(args=None):
    rclpy.init(args=args)
    node = Acoustic3dEdge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
