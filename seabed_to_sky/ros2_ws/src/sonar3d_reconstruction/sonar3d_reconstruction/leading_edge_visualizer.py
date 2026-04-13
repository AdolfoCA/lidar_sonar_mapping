"""Node that saves the first 10 raw blueview sonar images.

Saves both:
- a preview PNG (matplotlib figure for quick inspection)
- a .npz file with the raw arrays (image, ranges, beams) for notebook analysis
"""

import os
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from marine_acoustic_msgs.msg import ProjectedSonarImage
from rclpy.node import Node

from sonar3d_reconstruction.process_sonar_image import ProjectedSonarImage2Image

MAX_IMAGES = 10


class LeadingEdgeVisualizer(Node):
    """Saves the first 10 raw blueview sonar images to disk."""

    def __init__(self):
        super().__init__("leading_edge_visualizer")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("sonar.topic", "/blueview_message_polar"),
                ("output.folder", "/home/rosdev/ros2_ws/src/images"),
            ],
        )

        self.output_folder = self.get_parameter("output.folder").value
        os.makedirs(self.output_folder, exist_ok=True)

        self._count = 0

        self.sub = self.create_subscription(
            ProjectedSonarImage,
            self.get_parameter("sonar.topic").value,
            self._callback,
            10,
        )

        self.get_logger().info(
            f"LeadingEdgeVisualizer started — saving first {MAX_IMAGES} images to {self.output_folder}"
        )

    def _callback(self, msg: ProjectedSonarImage):
        if self._count >= MAX_IMAGES:
            return

        self._count += 1

        image, ranges, beams = ProjectedSonarImage2Image(msg)

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        readable = datetime.fromtimestamp(timestamp).strftime("%Y%m%d_%H%M%S_%f")[:-3]
        base = os.path.join(self.output_folder, f"sonar_{self._count:02d}_{readable}")

        # Save raw arrays for notebook analysis
        np.savez(base + ".npz", image=image, ranges=ranges, beams=beams)

        # Save preview PNG
        fig, ax = plt.subplots(1, 1, figsize=(6, 8))
        ax.imshow(image, cmap="inferno", aspect="auto", vmin=0, vmax=255)
        range_ticks = np.linspace(0, image.shape[0] - 1, num=8, dtype=int)
        bearing_ticks = np.linspace(0, image.shape[1] - 1, num=5, dtype=int)
        ax.set_yticks(range_ticks)
        ax.set_yticklabels([f"{ranges[i]:.1f}" for i in range_ticks])
        ax.set_xticks(bearing_ticks)
        ax.set_xticklabels([f"{np.rad2deg(beams[i]):.1f}" for i in bearing_ticks])
        ax.set_ylabel("Range (m)")
        ax.set_xlabel("Bearing (deg)")
        ax.set_title(f"Blueview sonar image #{self._count}")
        plt.tight_layout()
        plt.savefig(base + ".png", dpi=150)
        plt.close(fig)

        self.get_logger().info(f"[{self._count}/{MAX_IMAGES}] Saved {base}.(npz|png)")

        if self._count >= MAX_IMAGES:
            self.get_logger().info("Done — all images saved.")


def main(args=None):
    rclpy.init(args=args)
    node = LeadingEdgeVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
