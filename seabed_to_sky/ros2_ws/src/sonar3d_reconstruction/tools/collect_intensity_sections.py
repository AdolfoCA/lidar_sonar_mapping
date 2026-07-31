#!/usr/bin/env python3
"""
collect_intensity_sections.py
=============================
Log per-section RAW sonar intensity to a CSV for a learning-based intensity-correction
of the noisy beams.

The sonar image is treated as a MATRIX (rows = range bins, cols = beams) and split into
`--ranges` x `--beams` rectangular sections. For every incoming frame one ROW is written;
the COLUMNS are the section IDs `r{ri}_b{bi}` where ri = range-section index (0 = near) and
bi = beam-section index (0 = first beam). So the columns r0_b0, r0_b1, ... reconstruct the
section matrix row-major.

This sonar is 512 beams x 1219 range bins. Defaults: --beams 512 (ONE column per beam,
so the noisy beams stay isolated) x --ranges 40 (~0.25 m/section). That is 512x40 = 20480
section columns; ~1 h at 10 Hz is ~4-5 GB. Lower --ranges (or --beams) for a smaller file.

Run it for ~1 h while the bag plays, Ctrl-C to stop:
  source install/setup.bash
  python3 src/sonar3d_reconstruction/tools/collect_intensity_sections.py \
      --output ~/ros2_ws/intensity_sections.csv          # 512 x 40 (per-beam)
  # smaller/coarser instead:
  #   ... --beams 128 --ranges 20                          # average groups of 4 beams

Also writes `<output>.meta.csv` (once): for each section, its beam/range/bearing extent
and `frac_band` (fraction of its beams inside the known noisy azimuth bands) -> tells you
which columns are the noisy sections vs the clean ones.

Values are RAW intensity (no denoise), mean per section by default (--stat median for the
robust floor). flip_ranges matches the node (near -> far top->bottom).
"""
import argparse
import csv
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from marine_acoustic_msgs.msg import ProjectedSonarImage
from sonar3d_reconstruction.leading_edge_node import sonar_image_from_msg, band_beam_mask


class IntensitySectionCollector(Node):
    def __init__(self, args):
        super().__init__('intensity_section_collector')
        self.nb = int(args.beams)
        self.nr = int(args.ranges)
        self.flip = bool(args.flip_ranges)
        self.stat = args.stat
        self.path = os.path.expanduser(args.output)
        self.band_az = np.deg2rad(np.asarray(args.band_az_deg, dtype=np.float64))
        self.band_half = int(args.band_half_beams)

        os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
        self._f = open(self.path, 'w', newline='')
        self._w = csv.writer(self._f)
        self._ready = False
        self._n = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(ProjectedSonarImage, args.input, self._cb, qos)
        self.get_logger().info(
            f"collecting {self.nr}x{self.nb} sections ({self.stat}) -> {self.path} "
            f"(flip_ranges={self.flip}); Ctrl-C to stop")

    def _setup(self, ranges, bearings):
        """First frame: fix the section edges, write the CSV header + the meta sidecar."""
        R, B = ranges.shape[0], bearings.shape[0]
        self._re = np.linspace(0, R, self.nr + 1).astype(int)
        self._be = np.linspace(0, B, self.nb + 1).astype(int)
        cols = ['stamp_ns', 'wall_ns', 'n_range', 'n_beam']
        cols += [f"r{ri}_b{bi}" for ri in range(self.nr) for bi in range(self.nb)]
        self._w.writerow(cols)

        band = band_beam_mask(bearings, self.band_az, self.band_half)
        meta_path = self.path + '.meta.csv'
        with open(meta_path, 'w', newline='') as mf:
            mw = csv.writer(mf)
            mw.writerow(['section_id', 'ri', 'bi', 'beam_lo', 'beam_hi',
                         'bearing_lo_deg', 'bearing_hi_deg', 'range_lo_m', 'range_hi_m',
                         'frac_band'])
            for ri in range(self.nr):
                r0, r1 = int(self._re[ri]), int(self._re[ri + 1])
                for bi in range(self.nb):
                    b0, b1 = int(self._be[bi]), int(self._be[bi + 1])
                    fb = float(band[b0:b1].mean()) if b1 > b0 else 0.0
                    mw.writerow([f"r{ri}_b{bi}", ri, bi, b0, b1,
                                 f"{np.rad2deg(bearings[b0]):.2f}",
                                 f"{np.rad2deg(bearings[min(b1, B-1)]):.2f}",
                                 f"{ranges[r0]:.3f}", f"{ranges[min(r1, R-1)]:.3f}",
                                 f"{fb:.3f}"])
        self.get_logger().info(f"wrote section map -> {meta_path} "
                               f"(use frac_band>0 to find the noisy-beam sections)")
        self._ready = True

    def _section_values(self, img):
        re, be = self._re, self._be
        if self.stat == 'median':
            v = np.empty((self.nr, self.nb), np.float64)
            for ri in range(self.nr):
                for bi in range(self.nb):
                    v[ri, bi] = np.median(img[re[ri]:re[ri + 1], be[bi]:be[bi + 1]])
            return v
        # mean: two cumulative reduceat passes (fast, vectorised)
        cr = np.diff(re).astype(np.float64)
        cb = np.diff(be).astype(np.float64)
        sums = np.add.reduceat(np.add.reduceat(img, re[:-1], axis=0), be[:-1], axis=1)
        return sums / (cr[:, None] * cb[None, :])

    def _cb(self, msg):
        try:
            img, ranges, bearings = sonar_image_from_msg(msg, flip_ranges=self.flip)
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn(f"dropped frame: {exc!r}")
            return
        if not self._ready:
            self._setup(ranges, bearings)
        vals = self._section_values(img)
        s = msg.header.stamp
        stamp_ns = int(s.sec) * 1_000_000_000 + int(s.nanosec)
        row = [stamp_ns, time.time_ns(), img.shape[0], img.shape[1]]
        row += np.round(vals.reshape(-1), 2).tolist()
        self._w.writerow(row)
        self._n += 1
        if self._n % 50 == 0:
            self._f.flush()
            self.get_logger().info(f"{self._n} frames logged")

    def close(self):
        try:
            self._f.flush()
            self._f.close()
            self.get_logger().info(f"closed {self.path} ({self._n} frames)")
        except Exception:                              # noqa: BLE001
            pass


def main():
    pa = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    pa.add_argument('--input', default='/blueview_message_polar')
    pa.add_argument('--output', default='/home/rosdev/ros2_ws/intensity_sections.csv')
    pa.add_argument('--beams', type=int, default=512,
                    help='# beam (azimuth) sections. Sonar has 512 beams -> 512 = one '
                         'column PER BEAM (keeps the noisy beams isolated). Use fewer to '
                         'average neighbouring beams together.')
    pa.add_argument('--ranges', type=int, default=20,
                    help='# range sections (near->far). Sonar has 1219 range bins; 40 '
                         '~= 0.25 m/section. (Per-bin x 512 beams would be ~624k columns.)')
    pa.add_argument('--stat', choices=['mean', 'median'], default='mean')
    pa.add_argument('--band-az-deg', type=float, nargs='*', default=[-23.0, 0.0, 23.0],
                    help='noisy band center azimuths (for the meta frac_band label)')
    pa.add_argument('--band-half-beams', type=int, default=17)
    pa.add_argument('--flip-ranges', dest='flip_ranges', action='store_true', default=True)
    pa.add_argument('--no-flip-ranges', dest='flip_ranges', action='store_false')
    args, _ = pa.parse_known_args()

    rclpy.init()
    node = IntensitySectionCollector(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        try:
            node.destroy_node()
        except Exception:                              # noqa: BLE001
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:                              # noqa: BLE001 (signal may have shut it down)
            pass


if __name__ == '__main__':
    main()
