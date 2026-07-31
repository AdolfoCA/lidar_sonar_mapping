#!/usr/bin/env python3
"""
sonar_inspect.py
================

Interactive sonar viewer — like sonar_image.png, but you can HOVER over any pixel
and read its real intensity, range and bearing live.

It subscribes to the raw ProjectedSonarImage, scan-converts it to the same
Cartesian FAN as leading_edge_node's sonar_image.png, and shows it in an OpenCV
(Qt5) window. Move the cursor over the image: a crosshair follows it and a
read-out shows the RAW sonar intensity at that pixel plus its range (m) and
bearing (deg). Left-click freezes/unfreezes the read-out so you can note a value.

This is a STANDALONE debug tool — it does not touch the pipeline. Run it while a
bag/sensor is publishing:

    ros2 run sonar3d_reconstruction sonar_inspect
    # or directly:
    python3 .../sonar3d_reconstruction/sonar_inspect.py --ros-args \
        -p input_topic:=/blueview_message_polar -p flip_ranges:=true

Needs a display (OpenCV here is built with Qt5 GUI). Press ESC or close the
window to quit.
"""

import numpy as np
import numpy.lib.recfunctions as rfn
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from marine_acoustic_msgs.msg import ProjectedSonarImage
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# Reuse the EXACT polar-image decode (incl. flip_ranges) so the fan matches
# sonar_image.png bin-for-bin.
from sonar3d_reconstruction.leading_edge_node import sonar_image_from_msg

_WIN = "sonar_inspect"


def _read_xy(msg: PointCloud2) -> np.ndarray:
    """Read the (x, y) fields of a PointCloud2 into an (N, 2) float64 array."""
    raw = pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
    if isinstance(raw, np.ndarray):
        if raw.size == 0:
            return np.empty((0, 2), dtype=np.float64)
        if raw.dtype.names is not None:
            return rfn.structured_to_unstructured(raw[["x", "y"]], dtype=np.float64)
        return raw.reshape(-1, 2).astype(np.float64)
    rows = list(raw)
    return (np.array([[r[0], r[1]] for r in rows], dtype=np.float64)
            if rows else np.empty((0, 2), dtype=np.float64))


def build_fan_lut(ranges, bearings):
    """Polar->Cartesian fan scan-conversion LUT (a functional copy of
    leading_edge_node._ensure_fan_lut). Returns (valid, ri, bb, half, W, H):
    for each fan pixel, `valid` says it maps to a real bin, and (ri, bb) are the
    range-row / beam-column indices into the polar image."""
    n_range = int(ranges.shape[0])
    n_beam = int(bearings.shape[0])
    first, last = float(bearings[0]), float(bearings[-1])
    height = n_range
    max_y = float(np.max(np.abs(np.sin(bearings))))
    half = int(np.ceil(max_y * height))
    width = 2 * half
    angle_step = (last - first) / (n_beam - 1) if n_beam > 1 else 1.0
    abs_step = abs(angle_step)
    angle_tol = abs_step * 0.75
    range_scale = (n_range - 1) / height

    py = np.arange(height, dtype=np.float32)[:, None]
    px = np.arange(width, dtype=np.float32)[None, :]
    dz = (height - 1 - py)
    dy = (px - half)
    dist = np.sqrt(dy * dy + dz * dz)
    inside = dist <= height
    pixel_angle = np.arctan2(-dy, dz)
    b_idx = (pixel_angle - first) / angle_step
    best_beam = np.round(b_idx)
    beam_ok = (best_beam >= 0) & (best_beam < n_beam)
    beam_ok &= (np.abs(b_idx - best_beam) * abs_step) <= angle_tol
    range_idx = np.round(dist * range_scale)
    range_ok = (range_idx >= 0) & (range_idx < n_range)

    valid = inside & beam_ok & range_ok
    ri = np.clip(range_idx, 0, n_range - 1).astype(np.int32)
    bb = np.clip(best_beam, 0, n_beam - 1).astype(np.int32)
    return valid, ri, bb, half, width, height


class SonarInspect(Node):
    def __init__(self):
        super().__init__("sonar_inspect")
        self.input_topic = self.declare_parameter(
            "input_topic", "/blueview_message_polar").value
        self.flip_ranges = bool(self.declare_parameter("flip_ranges", True).value)
        # y-sign of the published leading-edge cloud (must match the extractor's
        # flip_beams) so the red edge overlays the right side. Flip if it mirrors.
        self.flip_beams = bool(self.declare_parameter("flip_beams", False).value)
        # Fit the (possibly large) fan into a window at most this many px on its
        # long side; the mouse read-out maps back to full-res so values stay exact.
        self.max_disp = int(self.declare_parameter("max_display_px", 900).value)
        # Text size — bump this if the read-out is hard to read.
        self.font_scale = float(self.declare_parameter("font_scale", 0.8).value)
        self.edge_topic = str(self.declare_parameter(
            "leading_edge_topic", "/blueview/leading_edge").value)
        qos_rel = str(self.declare_parameter("qos_reliability", "best_effort").value).lower()
        rel = (ReliabilityPolicy.RELIABLE if qos_rel == "reliable"
               else ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            ProjectedSonarImage, self.input_topic, self._cb,
            QoSProfile(depth=5, reliability=rel))
        self.edge_sub = self.create_subscription(
            PointCloud2, self.edge_topic, self._edge_cb,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Latest frame + fan LUT (rebuilt only when geometry changes).
        self.image = None            # (n_range, n_beam) raw intensity
        self.ranges = None
        self.bearings = None
        self._lut = None             # (valid, ri, bb, half, W, H)
        self._lut_key = None
        self.edge_pts = None         # (N, 2) leading-edge (x, y) in the sonar frame

        # Interaction state — mouse stored in DISPLAY coords (converted on lookup).
        self.mouse = None            # (x, y) display pixel under the cursor
        self.frozen = None           # left-click freezes this (x, y) display pixel
        self.view_scale = 1.0        # display px -> full-res px factor
        self._frames = 0
        self.get_logger().info(
            f"sonar_inspect: {self.input_topic} (flip_ranges={self.flip_ranges}) "
            f"+ edge {self.edge_topic} — hover to read intensity; click to freeze; "
            f"ESC to quit")

    def _cb(self, msg: ProjectedSonarImage):
        try:
            img, ranges, bearings = sonar_image_from_msg(
                msg, flip_ranges=self.flip_ranges)
        except Exception as exc:  # noqa: BLE001 — one bad frame must not kill the tool
            self.get_logger().warn(f"dropped a frame: {exc!r}",
                                    throttle_duration_sec=5.0)
            return
        self.image = img.astype(np.float32)
        self.ranges = ranges
        self.bearings = bearings
        key = (img.shape[0], img.shape[1], float(bearings[0]), float(bearings[-1]))
        if key != self._lut_key:
            self._lut = build_fan_lut(ranges, bearings)
            self._lut_key = key
            self.get_logger().info(
                f"fan {self._lut[4]}x{self._lut[5]} built for {img.shape}")
        self._frames += 1

    def _edge_cb(self, msg: PointCloud2):
        try:
            self.edge_pts = _read_xy(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"bad leading-edge cloud: {exc!r}",
                                   throttle_duration_sec=5.0)

    # ── geometry ──────────────────────────────────────────────────────────────
    def _fan_px(self, r, b, r0, span, H, half):
        """(range m, bearing rad) -> full-res fan (x, y), same map as the fan."""
        pr = (r - r0) / span * H
        return half - pr * np.sin(b), (H - 1) - pr * np.cos(b)

    # ── mouse (stores DISPLAY coords) ─────────────────────────────────────────
    def on_mouse(self, event, x, y, flags, _param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse = (x, y)
        elif event == cv2.EVENT_LBUTTONDOWN:
            self.frozen = None if self.frozen is not None else (x, y)

    def _readout(self, dx, dy):
        """Read-out for a DISPLAY pixel (dx, dy) — maps back to full-res first."""
        valid, ri, bb, _half, W, H = self._lut
        fx = int(round(dx / self.view_scale))
        fy = int(round(dy / self.view_scale))
        if not (0 <= fx < W and 0 <= fy < H) or not valid[fy, fx]:
            return None
        r = ri[fy, fx]
        b = bb[fy, fx]
        return {
            "intensity": float(self.image[r, b]),
            "range_m": float(self.ranges[r]),
            "bearing_deg": float(np.rad2deg(self.bearings[b])),
            "row": int(r), "col": int(b),
        }

    # ── render ────────────────────────────────────────────────────────────────
    def render(self):
        f = cv2.FONT_HERSHEY_SIMPLEX
        fs = self.font_scale
        if self.image is None or self._lut is None:
            wait = np.zeros((160, 720, 3), np.uint8)
            cv2.putText(wait, f"waiting for {self.input_topic} ...", (16, 90),
                        f, fs, (0, 220, 255), 2, cv2.LINE_AA)
            self.view_scale = 1.0
            return wait

        valid, ri, bb, half, W, H = self._lut
        # Fan intensity (real) -> auto-scaled 8-bit BGR, THEN downscaled for display.
        fan = np.where(valid, self.image[ri, bb], 0.0)
        mx = max(float(fan.max()), 1.0)
        full = cv2.cvtColor(np.clip(fan * (255.0 / mx), 0, 255).astype(np.uint8),
                            cv2.COLOR_GRAY2BGR)
        self.view_scale = vs = min(1.0, self.max_disp / float(max(W, H)))
        disp = (cv2.resize(full, None, fx=vs, fy=vs, interpolation=cv2.INTER_AREA)
                if vs < 1.0 else full.copy())
        Wd, Hd = disp.shape[1], disp.shape[0]

        # ── leading edge (red) — the published cloud mapped onto the fan ───────
        if self.edge_pts is not None and self.edge_pts.shape[0]:
            r0 = float(self.ranges[0]); span = float(self.ranges[-1]) - r0
            if span > 0:
                y_sign = 1.0 if self.flip_beams else -1.0
                cx, cy = self.edge_pts[:, 0], self.edge_pts[:, 1]
                r = np.hypot(cx, cy)
                b = np.arctan2(cy / y_sign, cx)
                fx, fy = self._fan_px(r, b, r0, span, H, half)
                px = np.round(fx * vs).astype(int)
                py = np.round(fy * vs).astype(int)
                inb = (px >= 0) & (px < Wd) & (py >= 0) & (py < Hd)
                for x, y in zip(px[inb].tolist(), py[inb].tolist()):
                    cv2.circle(disp, (x, y), 2, (0, 0, 255), -1)   # BGR red

        # ── labels + hover/frozen read-out, drawn on the DISPLAY (big text) ────
        cv2.putText(disp, f"FAR {self.ranges[-1]:.1f}m", (6, int(24 * fs) + 6),
                    f, fs * 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(disp, f"NEAR {self.ranges[0]:.1f}m",
                    (Wd // 2 - 60, Hd - 10), f, fs * 0.8, (0, 255, 0), 2, cv2.LINE_AA)

        line_h = int(34 * fs)
        yb = int(30 * fs) + 30
        for pt, color, tag in ((self.mouse, (0, 255, 255), "cursor"),
                               (self.frozen, (255, 128, 0), "FROZEN")):
            if pt is None:
                continue
            info = self._readout(*pt)
            if info is None:
                continue
            cv2.drawMarker(disp, pt, color, cv2.MARKER_CROSS, 22, 2, cv2.LINE_AA)
            txt = (f"{tag}: I={info['intensity']:.0f}  "
                   f"r={info['range_m']:.2f}m  b={info['bearing_deg']:+.1f}deg  "
                   f"[row {info['row']}, beam {info['col']}]")
            (tw, th), _ = cv2.getTextSize(txt, f, fs, 2)
            cv2.rectangle(disp, (4, yb - th - 6), (10 + tw, yb + 8), (0, 0, 0), -1)
            cv2.putText(disp, txt, (8, yb), f, fs, color, 2, cv2.LINE_AA)
            yb += line_h + 8
        return disp


def main(args=None):
    rclpy.init(args=args)
    node = SonarInspect()
    cv2.namedWindow(_WIN, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(_WIN, node.on_mouse)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            disp = node.render()
            if disp is not None:
                cv2.imshow(_WIN, disp)
            key = cv2.waitKey(20) & 0xFF
            if key == 27:                                    # ESC
                break
            # Window closed by the user?
            if cv2.getWindowProperty(_WIN, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
