import cv2
import numpy as np
import rclpy
from marine_acoustic_msgs.msg import ProjectedSonarImage
from message_filters import Subscriber
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2

from sonar3d_reconstruction.edge_detection import image2Profile, ransac_on_profile, rolling_MAD
from sonar3d_reconstruction.process_sonar_image import ProjectedSonarImage2Image, filter_horizontal_image
from sonar3d_reconstruction.sonar_to_pointcloud import profile_to_laserfield


class Acoustic3dEdge(Node):
    """Extracts the leading edge from a ProjectedSonarImage and publishes it as PointCloud2."""

    def __init__(self):
        super().__init__("acoustic3d_edge")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("sonar.topic",              Parameter.Type.STRING),
                ("sonar.threshold",          Parameter.Type.DOUBLE),
                ("sonar.min_range",          Parameter.Type.DOUBLE),
                ("sonar.max_range",          Parameter.Type.DOUBLE),
                ("sonar.sound_speed",        Parameter.Type.DOUBLE),
                ("sonar.sound_speed_actual", Parameter.Type.DOUBLE),
                ("denoise.standard",         Parameter.Type.BOOL),
                ("denoise.open",             Parameter.Type.INTEGER),
                ("denoise.RANSAC",           Parameter.Type.BOOL),
                ("denoise.MAD",              Parameter.Type.BOOL),
                ("method.leading_edge",      Parameter.Type.BOOL),
                ("method.falling_edge",      Parameter.Type.BOOL),
                ("output.topic",             Parameter.Type.STRING),
                ("output.frame_id",          Parameter.Type.STRING),
            ],
        )

        # Pitch is NOT applied here — it is already encoded in the TF chain
        # (blueview_joint1 carries the -30° y rotation). Applying it here too
        # would double-count the tilt. sonar_scan_ned handles the full transform.

        self.sub = Subscriber(self, ProjectedSonarImage, self.get_parameter("sonar.topic").value)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("output.topic").value, 10)
        self.sub.registerCallback(self.callback)

        self.get_logger().info("Acoustic3dEdge started")

    def _adjust_ranges_for_sound_speed(self, ranges: np.ndarray, msg: ProjectedSonarImage) -> np.ndarray:
        actual = self.get_parameter("sonar.sound_speed_actual").value
        if actual == 0:
            return ranges
        configured = self.get_parameter("sonar.sound_speed").value or msg.info.sound_speed
        return ranges * actual / configured

    def callback(self, msg: ProjectedSonarImage):
        # 1. Convert message to image + polar coordinates
        image, ranges, beams = ProjectedSonarImage2Image(msg)

        # 2. Denoise
        if self.get_parameter("denoise.standard").value:
            image = filter_horizontal_image(image)

        open_iters = self.get_parameter("denoise.open").value
        if open_iters:
            image = cv2.morphologyEx(
                image, cv2.MORPH_OPEN, np.ones((3, 1), np.uint8), iterations=open_iters
            )

        # 3. Sound-speed correction
        ranges = self._adjust_ranges_for_sound_speed(ranges, msg)

        # 4. Crop to [min_range, max_range]
        i0 = np.searchsorted(ranges, self.get_parameter("sonar.min_range").value)
        i1 = np.searchsorted(ranges, self.get_parameter("sonar.max_range").value, side="right")
        ranges = ranges[i0:i1]
        image  = image[i0:i1, :]

        # 5. Extract leading edge in the sonar's local frame (pitch=0).
        #    x = r·cos(beam)   along bore
        #    y = -r·sin(beam)  lateral (sign corrected for blueview convention)
        #    z = 0             1D sonar — no elevation in its own frame
        profile = image2Profile(
            image, ranges, beams,
            self.get_parameter("sonar.threshold").value,
            "leading",
            pitch=0.0,
        )

        # 6. Optional outlier rejection
        if self.get_parameter("denoise.RANSAC").value:
            profile = profile.filter_valid()
            profile, _ = ransac_on_profile(profile)

        if self.get_parameter("denoise.MAD").value:
            profile = profile.filter_valid()
            profile.valid, _, _ = rolling_MAD(profile.x, 39, 3.0)

        # 7. Publish
        cloud = profile_to_laserfield(
            profile.filter_valid(),
            msg.header.stamp,
            self.get_parameter("output.frame_id").value,
        )
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Acoustic3dEdge())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
