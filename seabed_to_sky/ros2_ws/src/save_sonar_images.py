#!/usr/bin/env python3
"""
save_sonar_images.py
--------------------
Subscribes to the Blueview sonar topic, collects 10 raw sonar images,
saves each one as a PNG in the images/blueview folder, then shuts down.

Usage (inside the container, with ROS2 sourced):
    python3 /ros2_ws/src/save_sonar_images.py

Optional arguments:
    --topic   Sonar topic  (default: /blueview_message_polar)
    --output  Output dir   (default: /home/rosdev/ros2_ws/src/images/blueview)
    --count   Number of images to save (default: 10)
"""

import argparse
import os
import sys
from datetime import datetime

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from marine_acoustic_msgs.msg import ProjectedSonarImage


# ── Image extraction ──────────────────────────────────────────────────────────

_DTYPE_MAP = {0: np.uint8, 2: np.uint16}


def _msg_to_image(msg: ProjectedSonarImage):
    """Convert a ProjectedSonarImage to (image uint8, ranges, beams_deg)."""
    ranges = np.array(msg.ranges, dtype=np.float32)
    nbeams = msg.image.beam_count
    nrange = len(ranges)

    beams_rad = np.array(
        [np.arctan2(b.y, b.z) for b in msg.beam_directions], dtype=np.float32
    )

    dtype = _DTYPE_MAP.get(msg.image.dtype, np.uint8)
    image = np.frombuffer(msg.image.data, dtype=dtype).reshape((nrange, nbeams))

    # Ensure beams go left (negative) to right (positive)
    if beams_rad[0] > beams_rad[-1]:
        beams_rad = np.flip(beams_rad)
        image = np.flip(image, axis=1)

    # Normalise to uint8
    if dtype == np.uint16 and image.max() > 0:
        image = cv2.convertScaleAbs(image, alpha=255.0 / image.max())
    else:
        image = image.astype(np.uint8)

    return image, ranges, np.rad2deg(beams_rad)


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class SonarImageSaver(Node):

    def __init__(self, topic: str, output_dir: str, count: int):
        super().__init__('sonar_image_saver')
        self._output_dir = output_dir
        self._target     = count
        self._saved      = 0

        os.makedirs(output_dir, exist_ok=True)

        self._sub = self.create_subscription(
            ProjectedSonarImage, topic, self._callback, 10
        )
        self.get_logger().info(
            f'Listening on {topic} — will save {count} images to {output_dir}'
        )

    def _callback(self, msg: ProjectedSonarImage):
        if self._saved >= self._target:
            return

        image, ranges, beams_deg = _msg_to_image(msg)

        # ── Build figure ──────────────────────────────────────────────────────
        fig, ax = plt.subplots(figsize=(6, 8))

        ax.imshow(
            image,
            cmap='inferno',
            vmin=0,
            vmax=255,
            aspect='auto',
            origin='lower',         # row 0 = min_range (bottom of image)
            extent=[beams_deg[0], beams_deg[-1], ranges[0], ranges[-1]],
        )

        ax.set_xlabel('Bearing (deg)')
        ax.set_ylabel('Range (m)')

        ts = msg.header.stamp
        stamp_sec = ts.sec + ts.nanosec * 1e-9
        readable  = datetime.fromtimestamp(stamp_sec).strftime('%Y%m%d_%H%M%S_%f')
        ax.set_title(f'Blueview sonar — {readable}')

        plt.colorbar(ax.images[0], ax=ax, label='Intensity')
        plt.tight_layout()

        # ── Save ─────────────────────────────────────────────────────────────
        filename = os.path.join(self._output_dir, f'sonar_{self._saved + 1:02d}_{readable}.png')
        fig.savefig(filename, dpi=150)
        plt.close(fig)

        self._saved += 1
        self.get_logger().info(f'[{self._saved}/{self._target}] Saved {filename}')

        if self._saved >= self._target:
            self.get_logger().info('Done — shutting down.')
            raise SystemExit


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Save Blueview sonar images to PNG.')
    parser.add_argument('--topic',  default='/blueview_message_polar',
                        help='ProjectedSonarImage topic')
    parser.add_argument('--output', default='/home/rosdev/ros2_ws/src/images/blueview',
                        help='Output directory for PNG files')
    parser.add_argument('--count',  type=int, default=10,
                        help='Number of images to save before exiting')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = SonarImageSaver(args.topic, args.output, args.count)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
