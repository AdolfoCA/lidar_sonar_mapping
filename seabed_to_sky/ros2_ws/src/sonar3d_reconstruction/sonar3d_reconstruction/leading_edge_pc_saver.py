"""Node that saves the first 10 leading edge scans from /blueview/point2/leading.

Each scan is plotted as a scatter of points on a white sonar-fan canvas
(bearing angle on x-axis, range on y-axis increasing downward) and saved
as a PNG.  Useful for quickly inspecting leading-edge quality from a bag
without needing the raw sonar image.
"""

import os

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

MAX_IMAGES = 10


class LeadingEdgePCSaver(Node):
    """Saves the first 10 leading-edge scans to PNG."""

    def __init__(self):
        super().__init__("leading_edge_pc_saver")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("topic", "/blueview/point2/leading"),
                ("output.folder", "/home/rosdev/ros2_ws/src/images/leading_edge"),
                ("range.max", 10.0),
            ],
        )

        self.output_folder = self.get_parameter("output.folder").value
        self.range_max = self.get_parameter("range.max").value
        os.makedirs(self.output_folder, exist_ok=True)

        self._count = 0

        self.sub = self.create_subscription(
            PointCloud2,
            self.get_parameter("topic").value,
            self._callback,
            10,
        )

        self.get_logger().info(
            f"LeadingEdgePCSaver started — saving first {MAX_IMAGES} scans to {self.output_folder}"
        )

    def _callback(self, msg: PointCloud2):
        if self._count >= MAX_IMAGES:
            return

        # Read x, y, z, intensity from the PointCloud2
        points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        if len(points) == 0:
            self.get_logger().warn("Received empty point cloud, skipping.")
            return

        # read_points returns a structured array — access fields by name
        pts = np.array(points, dtype=[("x", np.float32), ("y", np.float32),
                                      ("z", np.float32), ("intensity", np.float32)])
        x = pts["x"]
        y = pts["y"]
        intensity = pts["intensity"]

        # Reconstruct sonar fan coordinates
        # x=forward, y=left in ROS frame — bearing is angle in the horizontal plane
        bearing_deg = np.rad2deg(np.arctan2(y, x))
        range_m = np.sqrt(x**2 + y**2)

        self._count += 1

        ts = msg.header.stamp
        stamp_str = f"{ts.sec}_{ts.nanosec:09d}"

        fig, ax = plt.subplots(figsize=(8, 6))
        ax.set_facecolor("white")
        fig.patch.set_facecolor("white")

        sc = ax.scatter(
            bearing_deg,
            range_m,
            c=intensity,
            cmap="plasma",
            s=4,
            vmin=0,
            vmax=255,
        )
        plt.colorbar(sc, ax=ax, label="Intensity")

        ax.set_xlabel("Bearing (deg)")
        ax.set_ylabel("Range (m)")
        ax.set_ylim(self.range_max, 0)  # near range at top, far range at bottom
        ax.set_title(f"Leading edge scan #{self._count}  —  stamp {stamp_str}\n"
                     f"{len(points)} points,  bearing [{bearing_deg.min():.1f}°, {bearing_deg.max():.1f}°],  "
                     f"range [{range_m.min():.2f}, {range_m.max():.2f}] m")
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        out_path = os.path.join(self.output_folder, f"leading_{self._count:02d}_{stamp_str}.png")
        plt.savefig(out_path, dpi=150)
        plt.close(fig)

        self.get_logger().info(f"[{self._count}/{MAX_IMAGES}] Saved {out_path}")

        if self._count >= MAX_IMAGES:
            self.get_logger().info("Done — all scans saved.")


def main(args=None):
    rclpy.init(args=args)
    node = LeadingEdgePCSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
