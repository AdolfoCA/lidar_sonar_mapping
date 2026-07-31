#!/usr/bin/env python3
"""Capture a few raw ProjectedSonarImage frames from the live topic and save them
as .npz (image, ranges, bearings) for offline parameter sweeps. Uses the SAME
flip_ranges the node uses so the offline image matches what the node detects on."""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from marine_acoustic_msgs.msg import ProjectedSonarImage
from sonar3d_reconstruction.leading_edge_node import sonar_image_from_msg

OUT = "/tmp/sonar_frames"
N = int(sys.argv[1]) if len(sys.argv) > 1 else 6
FLIP_RANGES = True   # matches leading_edge.yaml


class Grab(Node):
    def __init__(self):
        super().__init__("frame_grabber")
        import os
        os.makedirs(OUT, exist_ok=True)
        self.count = 0
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(ProjectedSonarImage, "/blueview_message_polar", self.cb, qos)

    def cb(self, msg):
        if self.count >= N:
            return
        image, ranges, bearings = sonar_image_from_msg(msg, flip_ranges=FLIP_RANGES)
        path = f"{OUT}/frame_{self.count:02d}.npz"
        np.savez(path, image=image, ranges=ranges, bearings=bearings)
        self.get_logger().info(
            f"saved {path}  image={image.shape} range[{ranges[0]:.2f},{ranges[-1]:.2f}]m "
            f"bear[{np.rad2deg(bearings[0]):.1f},{np.rad2deg(bearings[-1]):.1f}]deg max={image.max():.0f}")
        self.count += 1


def main():
    rclpy.init()
    node = Grab()
    while rclpy.ok() and node.count < N:
        rclpy.spin_once(node, timeout_sec=2.0)
    node.get_logger().info(f"done: captured {node.count} frames -> {OUT}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
