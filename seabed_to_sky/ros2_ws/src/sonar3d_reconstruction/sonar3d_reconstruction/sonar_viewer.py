#!/usr/bin/env python3
"""
sonar_viewer — see the sonar sweep in RAW AMPLITUDE, live.

Two modes, one purpose: look at the numbers the intensity factor actually
consumes, in the units `classes.yaml` is written in.

    mode: viewer   an interactive window, refreshed at the sonar's own rate.
                   HOVER THE CURSOR over any point and it reports that pixel's
                   range, bearing, raw count, compensated dB, and which class's
                   mu is nearest. This is the tuning tool: point at bare bed and
                   read the level, point at a quay wall and read the level, and
                   the difference between those two readings IS the parameter
                   you are trying to assign.

    mode: png      the same fan written to a file every `every_n` scans, with
                   per-section numbers printed on it. The headless fallback,
                   and what you want when the machine has no display.

WHY dB AND NOT RAW COUNTS. The classifier never sees a count. The intensity
factor is a Gaussian over the compensated level, and the log domain is not
cosmetic: speckle is MULTIPLICATIVE, so in raw counts a class's spread scales
with its brightness and sigma_c would confound "how variable is this material"
with "how bright is it". The log transform turns that into additive noise of
roughly constant variance, which is the only thing that makes a Gaussian the
right family. A picture in dB is therefore directly comparable with the mu_c in
classes.yaml.

WHAT IS DRAWN IS OPT-IN. By default the window shows the SONAR IMAGE AND
NOTHING ELSE — no leading edge, no status text, no grid. Three switches add
things back:

    uint16_vis        map the greyscale from RAW COUNTS instead of dB. The
                      hover readout still reports both, so this only changes
                      contrast — and flipping between the two is the clearest
                      way to compare the picture with the class levels.
    leading_edge_vis  overlay the detector's leading edge in red. OFF by
                      default and worth leaving off: this is the measurement
                      field, not the detection, and painting one over the other
                      invites reading a detection failure as an intensity
                      problem or the reverse. The detector has its own image.
    overlay_info      the status line in the corner.

PURE OBSERVER — it subscribes and draws. It publishes nothing and touches no
map state, so leaving it enabled cannot change a result.
"""

import os

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from marine_acoustic_msgs.msg import ProjectedSonarImage

from sonar3d_reconstruction.sonar_fan_render import (FanLUT, draw_bands,
                                                     draw_probe, enhance,
                                                     greyscale, render)

#: ProjectedSonarImage.image.dtype codes emitted by the sonar driver.
_DTYPE = {0: np.uint8, 2: np.uint16}


def read_class_levels(path):
    """(names, mu, sigma) of the amplitude levels in a classes.yaml.

    Deliberately a five-line YAML read rather than an import of
    sonar_classification: this viewer is an IMAGE tool and must run with the
    detection stack alone. Pointing it at the classifier's inventory is an
    optional convenience for tuning, not a dependency. Returns None if the file
    is absent or has no amplitude blocks, and the readout simply omits the line.
    """
    try:
        import yaml
        with open(os.path.expanduser(path)) as fh:
            blocks = (yaml.safe_load(fh) or {}).get('classes') or []
        names, mu, sd = [], [], []
        for b in blocks:
            amp = b.get('amplitude') or {}
            if 'mu' in amp and 'sigma' in amp:
                names.append(str(b['name']))
                mu.append(float(amp['mu']))
                sd.append(float(amp['sigma']))
        if not names:
            return None
        return tuple(names), np.array(mu), np.array(sd)
    except Exception:
        return None

_WINDOW = 'sonar amplitude  [hover: read counts | s: save | q: quit]'


def sweep_from_msg(msg: ProjectedSonarImage, flip_ranges: bool = False):
    """(image, ranges, bearings) from a ProjectedSonarImage.

    image    : float64 (n_range, n_beam), row 0 = nearest range bin.
    ranges   : float64 (n_range,) ascending [m].
    bearings : float64 (n_beam,) ascending [rad].

    ``flip_ranges`` reverses the image ROWS (not ``ranges``) for drivers that
    store the buffer far->near while reporting ranges near->far. It must match
    the detection stack's setting, or this picture and the detector's disagree
    about which end of the sweep is near — and a range-flipped sweep still
    looks like a perfectly normal sonar image, so nothing else will tell you.
    """
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    n_range = ranges.size
    n_beam = msg.image.beam_count

    dtype = _DTYPE.get(msg.image.dtype, np.uint8)
    image = np.frombuffer(msg.image.data, dtype=dtype).reshape(
        n_range, n_beam).astype(np.float64)

    bearings = np.arctan2(
        np.fromiter((v.y for v in msg.beam_directions), dtype=np.float64,
                    count=n_beam),
        np.fromiter((v.z for v in msg.beam_directions), dtype=np.float64,
                    count=n_beam),
    )
    if n_beam >= 2 and bearings[0] > bearings[-1]:
        bearings = bearings[::-1]
        image = image[:, ::-1]
    if flip_ranges:
        image = image[::-1, :]
    return image, ranges, bearings


class SonarViewerNode(Node):

    def __init__(self):
        super().__init__('sonar_viewer')

        self.declare_parameter('enable', False)
        self.declare_parameter('mode', 'viewer')
        self.declare_parameter('sonar_image_topic', '/blueview_message_polar')
        self.declare_parameter('classes_file', '')
        self.declare_parameter('every_n', 1)
        self.declare_parameter('output_path',
                               '/home/rosdev/ros2_ws/src/images/blueview/sonar_db_image.png')
        self.declare_parameter('flip_ranges', True)
        self.declare_parameter('grid_ranges', 6)
        self.declare_parameter('grid_beams', 24)
        self.declare_parameter('grey_lo_pct', 5.0)
        self.declare_parameter('grey_hi_pct', 99.0)
        self.declare_parameter('min_valid_frac', 0.05)
        self.declare_parameter('show_counts', True)
        self.declare_parameter('window_scale', 0.6)
        self.declare_parameter('highlight_bands', [30000.0, 40000.0, 50000.0])
        self.declare_parameter('highlight_colours', ['pink', 'yellow', 'red'])
        self.declare_parameter('highlight_alpha', 0.8)
        self.declare_parameter('display_log', False)
        self.declare_parameter('display_stretch', 'max')
        self.declare_parameter('contrast', 'none')
        self.declare_parameter('clahe_clip', 2.0)
        self.declare_parameter('clahe_tiles', 8)
        self.declare_parameter('display_gamma', 1.0)
        self.declare_parameter('leading_edge_vis', False)
        self.declare_parameter('leading_edge_topic', '/blueview/leading_edge')
        self.declare_parameter('overlay_info', False)
        self.declare_parameter('hover_readout', True)

        def gp(n):
            return self.get_parameter(n).value

        self.active = bool(gp('enable'))
        if not self.active:
            self.get_logger().info(
                'sonar_viewer: disabled (set sonar_viewer.enable: true in '
                'leading_edge.yaml).')
            return

        self.mode = str(gp('mode')).lower()
        if self.mode not in ('viewer', 'png'):
            raise ValueError('sonar_viewer.mode must be "viewer" or "png"')

        # Optional: the classifier's amplitude levels, for the readout only.
        classes_file = str(gp('classes_file')).strip()
        self._levels = read_class_levels(classes_file) if classes_file else None

        self._every_n = max(int(gp('every_n')), 1)
        self._path = os.path.abspath(os.path.expanduser(str(gp('output_path'))))
        os.makedirs(os.path.dirname(self._path), exist_ok=True)
        self._flip = bool(gp('flip_ranges'))
        self._grid = (int(gp('grid_ranges')), int(gp('grid_beams')))
        self._lo_pct = float(gp('grey_lo_pct'))
        self._hi_pct = float(gp('grey_hi_pct'))
        self._min_frac = float(gp('min_valid_frac'))
        self._show_counts = bool(gp('show_counts'))
        self._scale = float(np.clip(float(gp('window_scale')), 0.05, 4.0))
        self._bands = [float(v) for v in (gp('highlight_bands') or [])]
        self._band_cols = [str(v) for v in (gp('highlight_colours') or [])]
        self._band_alpha = float(gp('highlight_alpha'))
        if len(self._bands) != len(self._band_cols):
            raise ValueError(
                f'highlight_bands ({len(self._bands)}) and highlight_colours '
                f'({len(self._band_cols)}) must be the same length — band i is '
                'tinted colour i.')
        if any(nxt <= cur for cur, nxt in zip(self._bands, self._bands[1:])):
            raise ValueError('highlight_bands must be strictly ascending; band i '
                             'covers [bands[i], bands[i+1]) and the last is open.')
        self._n_band = [0] * len(self._bands)
        self._display_log = bool(gp('display_log'))
        self._stretch = str(gp('display_stretch')).lower()
        self._contrast = str(gp('contrast')).lower()
        self._clip = float(gp('clahe_clip'))
        self._tiles = int(gp('clahe_tiles'))
        self._gamma = float(gp('display_gamma'))
        self._edge_vis = bool(gp('leading_edge_vis'))
        self._overlay_info = bool(gp('overlay_info'))
        self._hover = bool(gp('hover_readout'))
        self._edge = None            # (range, bearing) of the latest leading edge

        self._lut = FanLUT()
        self._scans = 0
        self._written = 0
        self._probed = False

        # Viewer state: the display frame plus the value planes behind it, so a
        # hovered pixel and the number reported for it come from one table.
        self._frame = None          # (H, W, 3) uint8, already scaled
        self._amp = None            # (H, W) float, raw count per pixel
        self._counts = None         # (H, W) float, raw count per pixel
        self._rng = None            # (H, W) float, slant range per pixel
        self._bear = None           # (H, W) float, bearing per pixel
        self._cursor = None
        self._window_ready = False
        self._stamp = ''

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            ProjectedSonarImage, str(gp('sonar_image_topic')), self._cb, qos)
        if self._edge_vis:
            # The detector's own output, in the SENSOR frame, so it drops
            # straight onto the fan with no TF involved.
            from sensor_msgs.msg import PointCloud2
            self.create_subscription(
                PointCloud2, str(gp('leading_edge_topic')), self._edge_cb, 1)

        self.get_logger().info(
            f'sonar_viewer[{self.mode}]: {gp("sonar_image_topic")} | '
            f'raw amplitude [counts]'
            + (f' | class levels from {classes_file}' if self._levels
               else ' | no class levels (set classes_file for the readout)')
            + (f' | window scale {self._scale:g}, every {self._every_n} scan(s)'
               if self.mode == 'viewer'
               else f' -> {self._path} every {self._every_n} scans'))

    # ── optional leading-edge overlay ────────────────────────────────────────

    def _edge_cb(self, msg):
        """Cache the detector's leading edge as (range, bearing).

        The cloud is in the SENSOR frame with the same convention the fan uses
        — x = r cos(beta), y = -r sin(beta) — so it maps onto the picture with
        no transform. Kept as polar rather than pixels because the fan geometry
        can change between sweeps.
        """
        import numpy.lib.recfunctions as rfn
        import sensor_msgs_py.point_cloud2 as pc2
        try:
            raw = pc2.read_points(msg, field_names=('x', 'y'), skip_nans=True)
            xy = rfn.structured_to_unstructured(np.asarray(raw), dtype=np.float64)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Bad leading-edge cloud: {exc}',
                                   throttle_duration_sec=10.0)
            return
        if xy.size == 0:
            self._edge = None
            return
        x, y = xy[:, 0], xy[:, 1]
        self._edge = (np.hypot(x, y), np.arctan2(-y, x))

    def _draw_edge(self, bgr, scale):
        if not self._edge_vis or self._edge is None:
            return
        import cv2
        rng, bear = self._edge
        for r, b in zip(rng, bear):
            px, py = self._lut.xy(float(r), float(b))
            px, py = int(px * scale), int(py * scale)
            if 0 <= px < bgr.shape[1] and 0 <= py < bgr.shape[0]:
                cv2.circle(bgr, (px, py), 1, (0, 0, 255), -1)   # BGR red

    # ── per-sweep ────────────────────────────────────────────────────────────

    def _cb(self, msg: ProjectedSonarImage):
        self._scans += 1
        if self._scans % self._every_n:
            return
        try:
            counts, ranges, bearings = sweep_from_msg(msg, self._flip)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Bad ProjectedSonarImage: {exc}',
                                   throttle_duration_sec=10.0)
            return
        if counts.size == 0 or ranges.size < 2 or bearings.size < 2:
            return

        # The amplitude the classifier consumes: the raw count, with cells at
        # or below zero marked NaN so they are excluded rather than averaged in.
        amp = np.where(counts > 0.0, counts, np.nan)

        if not self._probed:
            self._probed = True
            step = float(np.mean(np.abs(np.diff(bearings))))
            self.get_logger().info(
                f'sweep geometry: {bearings.size} beams, FOV '
                f'{np.rad2deg(abs(bearings[-1] - bearings[0])):.1f} deg, spacing '
                f'{np.rad2deg(step):.4f} deg, {ranges.size} range bins '
                f'{ranges[0]:.2f}-{ranges[-1]:.2f} m')

        if self.mode == 'png':
            self._write_png(amp, counts, ranges, bearings)
        else:
            self._build_frame(amp, counts, ranges, bearings)

    def _write_png(self, amp, counts, ranges, bearings):
        try:
            import cv2
            bgr = render(amp, ranges, bearings, self._lut,
                         bands=self._bands, band_colours=self._band_cols,
                         band_alpha=self._band_alpha,
                         display_log=self._display_log,
                         stretch=self._stretch,
                         counts=counts if self._show_counts else None,
                         n_range_sec=self._grid[0], n_beam_sec=self._grid[1],
                         lo_pct=self._lo_pct, hi_pct=self._hi_pct,
                         min_valid_frac=self._min_frac,
                         title=f'scan #{self._scans}')
            cv2.imwrite(self._path, bgr)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'amplitude image render failed: {exc}',
                                    throttle_duration_sec=10.0)
            return
        self._written += 1
        if self._written <= 3 or self._written % 20 == 0:
            fin = amp[np.isfinite(amp)]
            level = (f'median {np.median(fin):.0f} counts' if fin.size
                     else 'all at or below zero')
            hot = ''.join(f', {c}>{e:.0f}: {int(np.sum(fin >= e))}'
                          for e, c in zip(self._bands, self._band_cols))
            self.get_logger().info(
                f'amplitude image #{self._written} (scan {self._scans}): '
                f'{level}{hot} -> {self._path}')

    def _build_frame(self, amp, counts, ranges, bearings):
        """Scan-convert into the display frame and its value planes."""
        import cv2
        self._lut.ensure(ranges, bearings)
        lut = self._lut

        units = 'raw amplitude [counts]'
        amp_fan = lut.map(amp)

        if self._contrast == 'none' and abs(self._gamma - 1.0) <= 1e-6:
            # Untouched path: stretch on the FAN, which is byte-identical to
            # leading_edge_node's sonar_image.png.
            img8, lo, hi = greyscale(amp_fan, self._display_log, self._stretch,
                                     self._lo_pct, self._hi_pct)
        else:
            # Enhance in POLAR, where every pixel is real data, then scan
            # convert. Doing it on the fan would let the black surround drive
            # the local statistics.
            pol8, lo, hi = greyscale(amp, self._display_log, self._stretch,
                                     self._lo_pct, self._hi_pct)
            pol8 = enhance(pol8, self._contrast, self._clip, self._tiles,
                           self._gamma)
            fan8 = lut.map(pol8.astype(np.float64))
            img8 = np.where(np.isfinite(fan8), fan8, 0.0).astype(np.uint8)
        bgr = cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)

        count_fan = lut.map(counts)
        rng_fan, bear_fan = lut.polar_fans(ranges, bearings)

        if self._scale != 1.0:
            w = max(int(round(lut.width * self._scale)), 1)
            h = max(int(round(lut.height * self._scale)), 1)
            interp = cv2.INTER_AREA if self._scale < 1.0 else cv2.INTER_NEAREST
            bgr = cv2.resize(bgr, (w, h), interpolation=interp)
            # Value planes use NEAREST so a reported number is a real cell's
            # value, never an interpolation between two cells.
            def _rs(a):
                return cv2.resize(a, (w, h), interpolation=cv2.INTER_NEAREST)
            amp_fan, count_fan = _rs(amp_fan), _rs(count_fan)
            rng_fan, bear_fan = _rs(rng_fan), _rs(bear_fan)

        # Bands first, so the leading-edge dots stay legible on top of them.
        self._n_band = draw_bands(bgr, amp_fan, self._bands, self._band_cols,
                                  self._band_alpha)
        # Baked in per sweep rather than per GUI tick — the edge belongs to a
        # sweep, and drawing hundreds of dots every redraw is wasted work.
        self._draw_edge(bgr, self._scale)

        self._frame, self._amp, self._counts = bgr, amp_fan, count_fan
        self._rng, self._bear = rng_fan, bear_fan
        glo, ghi = (10**lo, 10**hi) if self._display_log else (lo, hi)
        self._stamp = (f'scan #{self._scans}   {units}   '
                       f'grey {glo:.0f}..{ghi:.0f} '
                       f'{self._stretch}{" log" if self._display_log else ""}'
                       + (f' {self._contrast}' if self._contrast != 'none' else '')
                       + f'   {np.isfinite(amp).mean():.0%} above floor'
                       + ''.join(f'   {c}>{e:.0f}:{n}' for e, c, n
                                  in zip(self._bands, self._band_cols,
                                         self._n_band)))

    # ── the interactive window ───────────────────────────────────────────────

    def _on_mouse(self, event, x, y, flags, param):
        self._cursor = (x, y)

    def _readout(self, x, y):
        """What the cursor is pointing at, in the units classes.yaml uses."""
        if self._amp is None:
            return []
        h, w = self._amp.shape
        if not (0 <= x < w and 0 <= y < h):
            return []
        amp, cnt = self._amp[y, x], self._counts[y, x]
        rng, bear = self._rng[y, x], self._bear[y, x]
        if not np.isfinite(rng):
            return ['outside the swath']

        band = ''
        for i, e in enumerate(self._bands):
            if cnt >= e:
                band = f'  [{self._band_cols[i]}]'
        lines = [f'amplitude {cnt:7.0f} counts{band}',
                 f'range     {rng:7.2f} m',
                 f'bearing   {np.rad2deg(bear):+7.1f} deg']
        if not np.isfinite(amp):
            lines.append('          -- at or below the noise floor')
        elif self._levels is not None:
            # Which class this level looks like, against the SAME numbers the
            # classifier scores with. Reported in sigma: a gap in counts means
            # nothing without the class's own spread, and every class here has
            # sigma/mu ~ 0.5 because speckle is Rayleigh.
            names, mu, sd = self._levels
            z = np.abs(cnt - mu) / sd
            c = int(np.argmin(z))
            lines.append(f'nearest   {names[c]} (mu {mu[c]:.0f}, {z[c]:.1f} sigma)')
        return lines

    def pump_gui(self) -> bool:
        """One GUI tick. Returns False when the user asks to quit.

        Runs on the MAIN thread — OpenCV's highgui requires that — while ROS is
        spun between ticks, so the window stays responsive at the sonar's rate
        without a second thread touching the display.
        """
        import cv2
        if self._frame is None:
            return True
        if not self._window_ready:
            cv2.namedWindow(_WINDOW, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(_WINDOW, self._on_mouse)
            self._window_ready = True

        # A bare frame unless something is switched on: the default is just the
        # sonar image, nothing painted over it.
        needs_copy = self._overlay_info or (self._hover and self._cursor is not None)
        shown = self._frame.copy() if needs_copy else self._frame
        if self._overlay_info:
            cv2.putText(shown, self._stamp, (6, 18), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 255, 255), 1, cv2.LINE_AA)
        if self._hover and self._cursor is not None:
            x, y = self._cursor
            draw_probe(shown, x, y, self._readout(x, y))
        cv2.imshow(_WINDOW, shown)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            return False
        if key == ord('s'):
            cv2.imwrite(self._path, shown)
            self.get_logger().info(f'snapshot -> {self._path}')
        return True

    def close(self):
        if self._window_ready:
            try:
                import cv2
                cv2.destroyWindow(_WINDOW)
            except Exception:  # noqa: BLE001
                pass


def main(args=None):
    rclpy.init(args=args)
    node = SonarViewerNode()
    try:
        if not node.active or node.mode == 'png':
            rclpy.spin(node)
        else:
            # Interleave ROS and the GUI on one thread: spin briefly, then draw.
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.01)
                try:
                    if not node.pump_gui():
                        break
                except Exception as exc:  # noqa: BLE001
                    node.get_logger().error(
                        f'viewer unavailable ({exc}). No display? Set '
                        'sonar_viewer.mode: "png" to write the fan to a file '
                        'instead.')
                    break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
