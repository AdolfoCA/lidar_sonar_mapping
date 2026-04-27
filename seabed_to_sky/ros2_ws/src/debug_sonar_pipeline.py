#!/usr/bin/env python3
"""
debug_sonar_pipeline.py
-----------------------
Subscribes to the Blueview sonar topic and saves a 4-panel debug figure
for each ping, showing every stage of the processing pipeline:

  Panel 1 — Raw image            (straight out of the message, uint8)
  Panel 2 — After filter         (filter_horizontal_image applied)
  Panel 3 — Threshold mask       (pixels >= threshold shown in white)
  Panel 4 — Edge overlay         (leading edge as red dots on filtered image)

Usage (inside the container, with ROS2 sourced):
    python3 /ros2_ws/src/debug_sonar_pipeline.py

Optional arguments:
    --topic      Sonar topic      (default: /blueview_message_polar)
    --output     Output directory (default: /home/rosdev/ros2_ws/src/images/debug)
    --count      Number of pings  (default: 5)
    --threshold  Edge threshold   (default: 130)
    --min-range  Min range in m   (default: 0.7)
    --max-range  Max range in m   (default: 10.0)
"""

import argparse
import os
from datetime import datetime

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

import rclpy
from rclpy.node import Node
from marine_acoustic_msgs.msg import ProjectedSonarImage


# ---------------------------------------------------------------------------
# Image extraction  (mirrors process_sonar_image.py exactly)
# ---------------------------------------------------------------------------

_DTYPE_MAP = {0: np.uint8, 2: np.uint16}


def _extract(msg: ProjectedSonarImage):
    """Return (image_uint8, ranges, beams_rad) — fixed dtype-based normalisation."""
    ranges = np.array(msg.ranges, dtype=np.float32)
    nbeams = msg.image.beam_count
    nrange = len(ranges)

    beams = np.array([np.arctan2(b.y, b.z) for b in msg.beam_directions], dtype=np.float32)

    dtype = _DTYPE_MAP.get(msg.image.dtype, np.uint8)
    image = np.frombuffer(msg.image.data, dtype=dtype).reshape((nrange, nbeams))

    if beams[0] > beams[-1]:
        beams = np.flip(beams)
        image = np.flip(image, axis=1)

    image = image.astype(dtype)

    # Fixed-scale normalisation (P2 fix: use dtype max, not per-frame max)
    if dtype == np.uint16:
        image = cv2.convertScaleAbs(image, alpha=255.0 / np.iinfo(dtype).max)
    else:
        image = image.copy()

    return image, ranges, beams


def _filter(image: np.ndarray) -> np.ndarray:
    """Mirrors filter_horizontal_image (P3 fix: no Otsu)."""
    img = np.maximum(0, image.astype(np.float32) - np.quantile(image, 0.1, axis=1)[:, None])
    img = np.maximum(0, img - np.quantile(img, 0.1))
    if img.max() > 0:
        img = (img / img.max() * 255).astype(np.uint8)
    else:
        img = img.astype(np.uint8)
    return img


def _leading_edge(image: np.ndarray, ranges: np.ndarray, threshold: float):
    """Return (edge_ranges, valid_mask) — one range value per beam column."""
    occupancy = image >= threshold
    indices = np.argmax(occupancy, axis=0)   # first row >= threshold per column
    valid = occupancy[indices, np.arange(image.shape[1])]
    edge_ranges = ranges[indices]
    return edge_ranges, valid


def _crop(image, ranges, min_range, max_range):
    i0 = np.searchsorted(ranges, min_range)
    i1 = np.searchsorted(ranges, max_range, side='right')
    return image[i0:i1, :], ranges[i0:i1]


# ---------------------------------------------------------------------------
# Plotting helpers
# ---------------------------------------------------------------------------

def _imshow(ax, image, ranges, beams_deg, title, cmap='inferno'):
    ax.imshow(
        image,
        cmap=cmap,
        vmin=0, vmax=255,
        aspect='auto',
        origin='lower',   # row 0 = min_range at the bottom
        extent=[beams_deg[0], beams_deg[-1], ranges[0], ranges[-1]],
        interpolation='nearest',
    )
    ax.set_title(title, fontsize=9)
    ax.set_xlabel('Bearing (deg)', fontsize=8)
    ax.set_ylabel('Range (m)', fontsize=8)
    ax.tick_params(labelsize=7)


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class SonarDebugger(Node):

    def __init__(self, topic, output_dir, count, threshold, min_range, max_range):
        super().__init__('sonar_debugger')
        self._output_dir = output_dir
        self._target     = count
        self._saved      = 0
        self._threshold  = threshold
        self._min_range  = min_range
        self._max_range  = max_range

        os.makedirs(output_dir, exist_ok=True)

        self._sub = self.create_subscription(
            ProjectedSonarImage, topic, self._callback, 10
        )
        self.get_logger().info(
            f'Listening on {topic} — will save {count} debug figures to {output_dir}'
        )

    def _callback(self, msg: ProjectedSonarImage):
        if self._saved >= self._target:
            return

        # -- Stage 0: extract ------------------------------------------------
        raw, ranges_full, beams = _extract(msg)
        beams_deg = np.rad2deg(beams)

        # -- Stage 1: crop to [min_range, max_range] -------------------------
        raw_crop, ranges = _crop(raw, ranges_full, self._min_range, self._max_range)

        # -- Stage 2: filter -------------------------------------------------
        filtered = _filter(raw_crop)

        # -- Stage 3: threshold mask -----------------------------------------
        mask = (filtered >= self._threshold).astype(np.uint8) * 255

        # -- Stage 4: leading edge -------------------------------------------
        edge_ranges, valid = _leading_edge(filtered, ranges, self._threshold)

        # -- Build figure ----------------------------------------------------
        fig = plt.figure(figsize=(18, 6))
        gs  = gridspec.GridSpec(1, 4, figure=fig, wspace=0.35)

        ax1 = fig.add_subplot(gs[0])
        ax2 = fig.add_subplot(gs[1])
        ax3 = fig.add_subplot(gs[2])
        ax4 = fig.add_subplot(gs[3])

        _imshow(ax1, raw_crop,  ranges, beams_deg, 'Panel 1 — Raw (cropped)')
        _imshow(ax2, filtered,  ranges, beams_deg, 'Panel 2 — After filter_horizontal_image')
        _imshow(ax3, mask,      ranges, beams_deg, 'Panel 3 — Threshold mask (>= %d)' % self._threshold, cmap='gray')
        _imshow(ax4, filtered,  ranges, beams_deg, 'Panel 4 — Leading edge overlay')

        # Overlay edge on Panel 4
        valid_beams  = beams_deg[valid]
        valid_ranges = edge_ranges[valid]
        ax4.scatter(valid_beams, valid_ranges, s=2, c='red', linewidths=0, zorder=5, label='leading edge')
        ax4.legend(fontsize=7, loc='upper right')

        # Stats annotation on Panel 3
        n_total = len(valid)
        n_valid = int(valid.sum())
        ax3.set_title(
            'Panel 3 — Threshold mask (>= %d)\n%d/%d beams hit' % (self._threshold, n_valid, n_total),
            fontsize=9,
        )

        ts = msg.header.stamp
        stamp_sec = ts.sec + ts.nanosec * 1e-9
        readable  = datetime.fromtimestamp(stamp_sec).strftime('%Y%m%d_%H%M%S_%f')
        fig.suptitle(
            'Blueview debug — %s  |  ranges %.2f–%.2f m  |  beams %.1f°–%.1f°' % (
                readable, ranges[0], ranges[-1], beams_deg[0], beams_deg[-1]
            ),
            fontsize=9,
        )

        filename = os.path.join(
            self._output_dir, 'debug_%02d_%s.png' % (self._saved + 1, readable)
        )
        fig.savefig(filename, dpi=150, bbox_inches='tight')
        plt.close(fig)

        self._saved += 1
        self.get_logger().info('[%d/%d] Saved %s  (%d/%d edge points valid)' % (
            self._saved, self._target, filename, n_valid, n_total
        ))

        if self._saved >= self._target:
            self.get_logger().info('Done.')
            raise SystemExit


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='Save Blueview pipeline debug figures.')
    parser.add_argument('--topic',     default='/blueview_message_polar')
    parser.add_argument('--output',    default='/home/rosdev/ros2_ws/src/images/debug')
    parser.add_argument('--count',     type=int,   default=5)
    parser.add_argument('--threshold', type=float, default=130.0)
    parser.add_argument('--min-range', type=float, default=0.7)
    parser.add_argument('--max-range', type=float, default=10.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = SonarDebugger(
        args.topic, args.output, args.count,
        args.threshold, args.min_range, args.max_range,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
