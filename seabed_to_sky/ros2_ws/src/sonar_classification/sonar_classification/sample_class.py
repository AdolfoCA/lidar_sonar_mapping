#!/usr/bin/env python3
"""
sample_class — measure a class's parameters by pointing the sonar at one.

    ros2 run sonar_classification sample_class --label wall --height-min 0.8

Three of the parameters in classes.yaml are not assignable from a datasheet and
should be COUNTED at a known target rather than guessed:

    mu_c, sigma_c   the compensated backscatter level and its spread
    pi_lidar_c      the fraction of the class's returns with validated
                    overhead LiDAR support
    rho_c           the fraction that land in a grown bright region

This tool measures all three. Park in front of a thing you can name — a quay
section, a pillar, a patch of bare sediment — gate the returns down to it, and
it reports the numbers with the YAML block to paste.

THE LEVEL IS ONLY MEANINGFUL AS A DIFFERENCE. The sonar is uncalibrated, so
every absolute level carries the unknown offset C, and the responsibility
compares classes by level DIFFERENCES alone. Two consequences, both of which
this tool leans on:

  * You do NOT have to fix C before tuning levels. Sample the reference
    sediment first, then pass its measured level back in as ``--reference-db``
    when sampling everything else; the reported offsets are C-independent and
    stay correct when C is later corrected.
  * What you paste is ``mu_c = mu_reference + offset``, with mu_reference the
    level you assigned the reference class in classes.yaml. The tool computes
    that for you.

DO FIX THE COMPENSATION FIRST, THOUGH. ``fit_compensation`` must report FLAT
before any of this means anything: if the level still trends with range, then
what you measure here is a statistic of how far away you happened to park, and
it will not transfer to a run with a different geometry. This tool refuses to
be quiet about it — it reports the range spread of its own sample so you can
see whether the number is stable across the swath.

WHAT TO GATE ON. Any combination, ANDed; leave out what you do not need:
  --box XMIN XMAX YMIN YMAX ZMIN ZMAX   a region of the map you know
  --height-min / --height-max            height above the fitted bed
  --range-min / --range-max              slant range gate
For bare sediment the height gate alone usually does it (h near 0); for a wall,
a height gate well above the bed is enough to exclude the seabed returns.
"""

import argparse
import csv
import os
import sys

import numpy as np
import numpy.lib.recfunctions as rfn

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

FIELDS = ('x', 'y', 'z', 'slant_range', 'count_raw', 'h',
          'lidar_observed', 'lidar_present', 'shape_flag')

#: 1.4826 * MAD is the normal-consistent robust standard deviation. Used
#: instead of np.std because a handful of specular spikes or a clipped return
#: would otherwise inflate sigma_c, and sigma_c is meant to describe the
#: class's ordinary spread, not its outliers.
_MAD_TO_SIGMA = 1.4826


class ClassSampler(Node):

    def __init__(self, args):
        super().__init__('sample_class')
        self.args = args
        self.rows = []
        self.create_subscription(PointCloud2, args.topic, self._cb, 50)
        self.get_logger().info(
            f'sampling "{args.label}" from {args.topic} for '
            f'{args.duration:.0f} s — keep the sonar on the target')

    def _cb(self, msg: PointCloud2):
        present = {f.name for f in msg.fields}
        if not set(FIELDS) <= present:
            self.get_logger().error(
                f'{self.args.topic} lacks {sorted(set(FIELDS) - present)} — '
                'point --topic at measurement_node output (semantic_returns).',
                throttle_duration_sec=10.0)
            return
        raw = pc2.read_points(msg, field_names=FIELDS, skip_nans=False)
        arr = rfn.structured_to_unstructured(np.asarray(raw), dtype=np.float64)
        if arr.size:
            self.rows.append(arr)

    def collected(self):
        return (np.concatenate(self.rows) if self.rows
                else np.empty((0, len(FIELDS))))


def gate(arr, args):
    """Keep only the returns that belong to the target."""
    x, y, z = arr[:, 0], arr[:, 1], arr[:, 2]
    rng, h = arr[:, 3], arr[:, 5]
    keep = np.ones(arr.shape[0], dtype=bool)

    if args.box:
        xmin, xmax, ymin, ymax, zmin, zmax = args.box
        keep &= (x >= xmin) & (x <= xmax) & (y >= ymin) & (y <= ymax)
        keep &= (z >= zmin) & (z <= zmax)
    if args.height_min is not None:
        keep &= np.isfinite(h) & (h >= args.height_min)
    if args.height_max is not None:
        keep &= np.isfinite(h) & (h <= args.height_max)
    if args.range_min is not None:
        keep &= rng >= args.range_min
    if args.range_max is not None:
        keep &= rng <= args.range_max
    return arr[keep]


def measure(arr, args):
    """The three counted parameters, plus the diagnostics that qualify them."""
    i_db = arr[:, 4]
    rng = arr[:, 3]
    observed, present, shape_flag = arr[:, 6], arr[:, 7], arr[:, 8]

    lvl = i_db[np.isfinite(i_db)]
    if lvl.size < args.min_returns:
        raise SystemExit(
            f'only {lvl.size} returns with a level survived the gate — need at '
            f'least {args.min_returns}. Loosen the gate, dwell longer, or check '
            'that the target is actually in view.')

    median = float(np.median(lvl))
    mad = float(np.median(np.abs(lvl - median)))
    sigma = _MAD_TO_SIGMA * mad

    # The MODE is what anchors a level: the detection threshold censors the
    # lower tail, so the mean is pulled up and the median mildly so, while the
    # peak of the histogram still sits where the material actually scatters.
    counts, edges = np.histogram(lvl, bins=max(int(np.sqrt(lvl.size)), 10))
    mode = float(0.5 * (edges[np.argmax(counts)] + edges[np.argmax(counts) + 1]))

    # pi_lidar: a fraction of RETURNS, counted only where the column was
    # actually observed — an unobserved column is not a miss, and counting it
    # as one would bias the number toward zero exactly where coverage is poor.
    cov = np.isfinite(observed) & (observed > 0.5)
    pi_lidar = (float(np.mean(present[cov] > 0.5)) if cov.any() else float('nan'))

    # rho: fraction landing in a grown region, over the returns where the
    # extractor actually ran.
    known = np.isfinite(shape_flag)
    rho = float(np.mean(shape_flag[known] > 0.5)) if known.any() else float('nan')

    # Is the level stable across the swath? If it is not, the compensation is
    # not flat and this measurement is a statistic of where we parked.
    near, far = np.nanpercentile(rng, [10, 90])
    lo_sel = np.isfinite(i_db) & (rng <= near)
    hi_sel = np.isfinite(i_db) & (rng >= far)
    drift = (float(np.median(i_db[hi_sel]) - np.median(i_db[lo_sel]))
             if lo_sel.sum() >= 20 and hi_sel.sum() >= 20 else float('nan'))

    return dict(n=int(lvl.size), n_gated=int(arr.shape[0]),
                median=median, mode=mode, sigma=sigma,
                p5=float(np.percentile(lvl, 5)),
                p95=float(np.percentile(lvl, 95)),
                pi_lidar=pi_lidar, n_cov=int(cov.sum()),
                rho=rho, n_shape=int(known.sum()),
                r_lo=float(near), r_hi=float(far), drift=drift)


def report(m, cs, args):
    print()
    print('=' * 72)
    print(f'class sample — "{args.label}"   {m["n"]} returns with a level '
          f'(of {m["n_gated"]} gated)')
    print('=' * 72)
    print(f'  level   mode   {m["mode"]:+8.2f} dB   <- anchor on this')
    print(f'          median {m["median"]:+8.2f} dB')
    print(f'          5-95%  {m["p5"]:+8.2f} .. {m["p95"]:+.2f} dB')
    print(f'  spread  sigma  {m["sigma"]:8.2f} dB   (1.4826 x MAD, robust)')
    print(f'  lidar   pi     {m["pi_lidar"]:8.3f}      '
          f'({m["n_cov"]} returns with an observed column)')
    print(f'  shape   rho    {m["rho"]:8.3f}      '
          f'({m["n_shape"]} returns the extractor saw)')
    print(f'  range   {m["r_lo"]:.1f} .. {m["r_hi"]:.1f} m, '
          f'level drift far-near {m["drift"]:+.2f} dB')
    print()

    if np.isfinite(m['drift']) and abs(m['drift']) > 3.0:
        print(f'WARNING: the level drifts {m["drift"]:+.1f} dB across this')
        print('         sample\'s own range spread. The compensation is not flat,')
        print('         so this level is a statistic of where you parked and will')
        print('         NOT transfer. Run fit_compensation until it reports FLAT,')
        print('         then re-sample.')
        print()

    print(f'Paste into classes.yaml for "{args.label}":')
    print()
    print(f'  - name: {args.label}')
    print(f'    amplitude: {{mu: {m["mode"]:.0f}, sigma: {max(m["sigma"], 1.0):.0f}}}')
    print()
    print('  NOTE sigma/mu ~ 0.52 is forced by Rayleigh speckle, so a bright')
    print('  class WILL get a wide spread. That is the model, not a bad sample.')

    print()
    print('  lidar: {pi: %.2f, ...}      shape: {rho: %.2f, ...}'
          % (min(max(m['pi_lidar'], 0.01), 0.99) if np.isfinite(m['pi_lidar']) else 0.5,
             min(max(m['rho'], 0.01), 0.99) if np.isfinite(m['rho']) else 0.5))
    print('  (both clamped away from 0 and 1 — the loader rejects the endpoints,')
    print('   since one contrary observation would be log(0) and veto the class)')
    print('=' * 72)


def append_csv(path, m, args):
    path = os.path.abspath(os.path.expanduser(path))
    new = not os.path.exists(path)
    with open(path, 'a', newline='') as fh:
        w = csv.writer(fh)
        if new:
            w.writerow(['label', 'n', 'mode_db', 'median_db', 'sigma_db',
                        'p5_db', 'p95_db', 'pi_lidar', 'rho',
                        'range_lo_m', 'range_hi_m', 'drift_db'])
        w.writerow([args.label, m['n'], f'{m["mode"]:.3f}', f'{m["median"]:.3f}',
                    f'{m["sigma"]:.3f}', f'{m["p5"]:.3f}', f'{m["p95"]:.3f}',
                    f'{m["pi_lidar"]:.4f}', f'{m["rho"]:.4f}',
                    f'{m["r_lo"]:.2f}', f'{m["r_hi"]:.2f}', f'{m["drift"]:.3f}'])
    print(f'appended to {path}')


def main(argv=None):
    argv = sys.argv if argv is None else argv
    ap = argparse.ArgumentParser(
        description='Measure a class\'s mu/sigma/pi/rho by sampling a known target.')
    ap.add_argument('--label', required=True,
                    help='what you are pointing at (used for the report and CSV)')
    ap.add_argument('--topic', default='semantic_returns')
    ap.add_argument('--classes-file', default='')
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--reference-db', type=float, default=None,
                    help='the mode measured for the REFERENCE class; makes the '
                         'level a C-independent offset from it')
    ap.add_argument('--box', type=float, nargs=6, default=None,
                    metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX', 'ZMIN', 'ZMAX'),
                    help='map-frame bounding box the target sits in')
    ap.add_argument('--height-min', type=float, default=None,
                    help='minimum height above the fitted bed [m]')
    ap.add_argument('--height-max', type=float, default=None)
    ap.add_argument('--range-min', type=float, default=None)
    ap.add_argument('--range-max', type=float, default=None)
    ap.add_argument('--min-returns', type=int, default=300)
    ap.add_argument('--csv', default='~/ros2_ws/class_samples.csv',
                    help='append the result here; "" to skip')
    args = ap.parse_args(remove_ros_args(argv)[1:])

    from ament_index_python.packages import get_package_share_directory
    from sonar_classification.argus.classes import load_classes

    classes_file = args.classes_file.strip() or os.path.join(
        get_package_share_directory('sonar_classification'),
        'config', 'classes.yaml')
    cs = load_classes(classes_file)

    rclpy.init(args=argv)
    node = ClassSampler(args)
    end = node.get_clock().now().nanoseconds + int(args.duration * 1e9)
    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    arr = node.collected()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if arr.shape[0] == 0:
        raise SystemExit(f'no returns on {args.topic}. Is measurement_node running?')

    m = measure(gate(arr, args), args)
    report(m, cs, args)
    if args.csv:
        append_csv(args.csv, m, args)
    return 0


if __name__ == '__main__':
    sys.exit(main())
