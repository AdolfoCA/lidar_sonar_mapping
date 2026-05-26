import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from message_filters import Subscriber
from marine_acoustic_msgs.msg import ProjectedSonarImage
from cv_bridge import CvBridge
import cv2
from sonar3d_reconstruction.process_sonar_image import (
    ProjectedSonarImage2Image,
    filter_horizontal_image,
    filter_vertical_image,
    remove_bscan_stripes,
)
from sonar3d_reconstruction.edge_detection import (
    image2Profile,
    ransac_on_profile,
    rolling_MAD,
    reject_azimuth_bands,
)
from sonar3d_reconstruction.sonar_to_pointcloud import profile_to_laserfield
from sonar3d_reconstruction.profile import Profile
import numpy as np
import os


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
                ("sonar.max_range", Parameter.Type.DOUBLE),
                ("sonar.sound_speed", Parameter.Type.DOUBLE),
                ("sonar.sound_speed_actual", Parameter.Type.DOUBLE),
                ("sonar.pitch", Parameter.Type.DOUBLE),
                ("denoise.standard", Parameter.Type.BOOL),
                ("denoise.destripe", Parameter.Type.BOOL),
                ("denoise.open", Parameter.Type.INTEGER),
                ("denoise.RANSAC", Parameter.Type.BOOL),
                ("denoise.MAD", Parameter.Type.BOOL),
                # reject_azimuths_deg: dynamic_typing lets an empty YAML list
                # ([]) be accepted — a bare DOUBLE_ARRAY declaration rejects
                # the empty list because its element type can't be inferred,
                # leaving the parameter uninitialized. Default [] keeps it
                # initialized when the key is omitted entirely.
                (
                    "denoise.reject_azimuths_deg",
                    [],
                    ParameterDescriptor(dynamic_typing=True),
                ),
                ("denoise.reject_width_deg", 1.0),
                ("method.leading_edge", Parameter.Type.BOOL),
                ("method.falling_edge", Parameter.Type.BOOL),
                ("output.topic", Parameter.Type.STRING),
                ("output.frame_id", Parameter.Type.STRING),
                ("output.flipped", Parameter.Type.BOOL),
                ("output.publish_rate", Parameter.Type.DOUBLE),
                ("output.visualize", Parameter.Type.BOOL),
                ("output.folder", Parameter.Type.STRING),
            ],
        )

        # Create subscriber and publisher.
        # The sonar source (driver / rosbag) publishes RELIABLE — a BEST_EFFORT
        # subscriber is QoS-incompatible with a RELIABLE publisher and the link
        # silently fails to (re)connect, which stops the leading-edge output.
        # Match RELIABLE here so the subscription stays connected.
        _sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = Subscriber(
            self, ProjectedSonarImage, self.get_parameter("sonar.topic").value, qos_profile=_sensor_qos
        )
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("output.topic").value, 10)

        # Create CvBridge
        self.bridge = CvBridge()

        # When visualization is enabled, sonar_image.png is overwritten every
        # frame inside _process_frame — no timer needed.
        if self.get_parameter("output.visualize").value:
            self.get_logger().info("Visualization is enabeled for edge detection.")
            self.output_folder = self.get_parameter("output.folder").value
            if not os.path.exists(self.output_folder):
                os.makedirs(self.output_folder)

        # Callback function
        self.sub.registerCallback(self.callback)

        # Rate limiting for leading edge publish
        publish_rate = self.get_parameter("output.publish_rate").value
        self._min_interval = Duration(seconds=1.0 / publish_rate)
        self._last_publish_time = None

        # Per-frame state (reassigned in _process_frame before use)
        self.image = None
        self.profile = None
        self.ranges = None
        self.beams = None
        self.pitch = np.deg2rad(self.get_parameter("sonar.pitch").value)

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

        Wrapped in a try/except: a single malformed sonar frame must NOT raise
        out of the callback. An uncaught exception makes the rclpy executor
        stop scheduling this subscription — the node stays alive but the topic
        goes permanently silent ("stops after some time"). Logging and
        returning instead keeps the subscription processing later frames.
        """
        try:
            self._process_frame(msg)
        except Exception as exc:  # noqa: BLE001 — deliberately broad: keep node alive
            self.get_logger().warn(
                f"Dropped a sonar frame due to an error in the callback: {exc!r}"
            )

    def _process_frame(self, msg: ProjectedSonarImage):
        """Process one sonar frame: convert, denoise, detect leading edge,
        build and publish the PointCloud2. May raise; callback() catches."""
        # Convert the message to a numpy array
        image, self.ranges, self.beams = ProjectedSonarImage2Image(msg)

        # Denoise the image
        if self.get_parameter("denoise.standard").value:
            image = filter_horizontal_image(image)
            
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

        # Cut the image range. np.where(...)[0] can be empty for a malformed or
        # out-of-range frame; indexing [0]/[-1] would then raise IndexError and
        # kill the subscription — guard and skip the frame instead.
        min_range = self.get_parameter("sonar.min_range").value
        max_range = self.get_parameter("sonar.max_range").value
        above_min = np.where(self.ranges >= min_range)[0]
        below_max = np.where(self.ranges <= max_range)[0]
        if above_min.size == 0 or below_max.size == 0:
            self.get_logger().warn(
                "Sonar frame has no range bins within "
                f"[{min_range}, {max_range}] m — skipping frame."
            )
            return
        start_range_idx = above_min[0]
        end_range_idx = below_max[-1] + 1
        if start_range_idx >= end_range_idx:
            self.get_logger().warn("Empty range window after cut — skipping frame.")
            return
        self.ranges = self.ranges[start_range_idx:end_range_idx]
        self.image = image[start_range_idx:end_range_idx, :]

        # Remove vertical "bar line" stripes from the B-scan before edge
        # detection. Operates on the post-cut image the detector consumes, so
        # the debug overlay below shows the destriped result.
        if self.get_parameter("denoise.destripe").value:
            self.image = remove_bscan_stripes(self.image)

        # Get the profile
        threshold = self.get_parameter("sonar.threshold").value
        self.profile = image2Profile(
            self.image,
            self.ranges,
            self.beams,
            threshold,
            "leading",
            self.pitch,
        )

        # Static azimuth-band reject — kill fixed-azimuth artefacts (driver
        # beam-pattern peaks / hardware echoes) before RANSAC/MAD see them.
        bad_az = self.get_parameter("denoise.reject_azimuths_deg").value
        if bad_az:
            self.profile = reject_azimuth_bands(
                self.profile,
                bad_az,
                self.get_parameter("denoise.reject_width_deg").value,
            )

        # Overwrite sonar_image.png with the post-cut filtered B-scan plus the
        # leading-edge overlay. Updated every frame; no history is kept.
        if self.get_parameter("output.visualize").value:
            out_dir = self.get_parameter("output.folder").value
            os.makedirs(out_dir, exist_ok=True)
            self._save_horizontal_with_edges(
                self.image, threshold,
                out_path=os.path.join(out_dir, "sonar_image.png"),
            )

        if self.get_parameter("denoise.RANSAC").value:
            self.profile = self.profile.filter_valid()
            self.profile, self.regressor = ransac_on_profile(self.profile)

        if self.get_parameter("denoise.MAD").value:
            self.profile = self.profile.filter_valid()
            if len(self.profile.x) == 0:
                return
            self.profile.valid, self.upper_bound, self.lower_bound = rolling_MAD(self.profile.x, 39, 3.0)

        # Rate limiting
        now = self.get_clock().now()
        if self._last_publish_time is not None:
            if (now - self._last_publish_time) < self._min_interval:
                return
        self._last_publish_time = now

        # Create the PointCloud2 message
        valid_profile = self.profile.filter_valid()
        if self.get_parameter("output.flipped").value:
            # Mirror y to fix left/right swap (x forward direction is correct)
            valid_profile = Profile(
                valid_profile.range, valid_profile.bearing,
                valid_profile.x, -valid_profile.y, valid_profile.z,
                valid_profile.delay, valid_profile.intensity, valid_profile.valid,
            )
        msg = profile_to_laserfield(
            valid_profile,
            msg.header.stamp,
            self.get_parameter("output.frame_id").value,
            return_only_valid=False,
        )

        # Publish the message
        self.pub.publish(msg)

    def _save_horizontal_with_edges(self, image: np.ndarray, threshold: float,
                                     out_path: str = "sonar_image.png") -> None:
        """Overwrite ``out_path`` with the post-cut filtered horizontal sonar
        image, leading-edge row per beam overlaid in red, plus soft blue
        vertical lines marking the azimuths in ``denoise.reject_azimuths_deg``
        (so the operator can see which beam columns the static azimuth-band
        reject is dropping). 8-bit auto-scaled BGR PNG, viewable in any
        standard image viewer.
        """
        if image is None or image.size == 0:
            return

        # Leading-edge row per column — same formula as image2Profile.
        occupancy = image >= threshold
        edges = np.argmax(occupancy, axis=0)
        valid = occupancy[edges, np.arange(len(edges))]
        cols_valid = np.arange(len(edges))[valid]
        rows_valid = edges[valid]

        # Beam columns that the static azimuth-band reject will drop — drawn
        # as soft blue vertical lines (alpha-blended so the sonar content
        # shows through). One column per azimuth: nearest beam to that angle.
        reject_az_deg = self.get_parameter("denoise.reject_azimuths_deg").value
        reject_cols = []
        if reject_az_deg and self.beams is not None and len(self.beams) > 0:
            beams_rad = np.asarray(self.beams, dtype=np.float64)
            for az_deg in reject_az_deg:
                col = int(np.argmin(np.abs(beams_rad - np.deg2rad(az_deg))))
                if 0 <= col < image.shape[1]:
                    reject_cols.append(col)

        # 8-bit auto-scaled image so the overlay is clearly visible.
        img_max = max(int(image.max()), 1)
        img_u8 = np.clip(image.astype(np.float32) * 255.0 / img_max, 0, 255).astype(np.uint8)
        overlay = cv2.cvtColor(img_u8, cv2.COLOR_GRAY2BGR)
        if reject_cols:
            layer = overlay.copy()
            for col in reject_cols:
                cv2.line(layer, (col, 0), (col, layer.shape[0] - 1),
                         (255, 0, 0), 1)  # BGR blue
            overlay = cv2.addWeighted(overlay, 0.6, layer, 0.4, 0)
        if cols_valid.size:
            overlay[rows_valid, cols_valid] = (0, 0, 255)  # BGR red
        cv2.imwrite(out_path, overlay)


def main(args=None):
    rclpy.init(args=args)
    node = Acoustic3dEdge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
