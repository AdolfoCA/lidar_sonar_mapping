#!/usr/bin/env python3
"""Offline preview of the intensity-section grid on a captured frame, so the fan
sonar_image.png layout can be checked without the live node. Reuses the node's
draw_intensity_grid + band_beam_mask on a /tmp/sonar_frames/*.npz frame."""
import sys
import numpy as np
import cv2

from sonar3d_reconstruction.leading_edge_node import draw_intensity_grid, band_beam_mask

FRAME = sys.argv[1] if len(sys.argv) > 1 else "/tmp/sonar_frames/frame_00.npz"
NB = int(sys.argv[2]) if len(sys.argv) > 2 else 9
NR = int(sys.argv[3]) if len(sys.argv) > 3 else 5
OUT = "/home/rosdev/ros2_ws/src/images/blueview/intensity_grid_preview.png"
BAND_AZ = np.deg2rad(np.array([-23.0, 0.0, 23.0]))
BAND_HALF = 17


def fan_lut(ranges, bearings):
    n_range, n_beam = len(ranges), len(bearings)
    first, last = float(bearings[0]), float(bearings[-1])
    H = n_range
    half = int(np.ceil(float(np.max(np.abs(np.sin(bearings)))) * H))
    W = 2 * half
    step = (last - first) / (n_beam - 1)
    tol = abs(step) * 0.75
    rscale = (n_range - 1) / H
    py = np.arange(H, dtype=np.float32)[:, None]
    px = np.arange(W, dtype=np.float32)[None, :]
    dz = (H - 1 - py); dy = (px - half)
    dist = np.sqrt(dy * dy + dz * dz)
    inside = dist <= H
    bi = (np.arctan2(-dy, dz) - first) / step
    bb = np.round(bi)
    ok = (bb >= 0) & (bb < n_beam) & (np.abs(bi - bb) * abs(step) <= tol)
    ri = np.round(dist * rscale)
    rok = (ri >= 0) & (ri < n_range)
    return (inside & ok & rok, np.clip(ri, 0, n_range - 1).astype(int),
            np.clip(bb, 0, n_beam - 1).astype(int), H, W, half)


def main():
    d = np.load(FRAME)
    image, ranges, bearings = d["image"], d["ranges"], d["bearings"]
    fv, ri, bb, H, W, half = fan_lut(ranges, bearings)
    fan = np.where(fv, image[ri, bb], 0.0)
    img8 = np.clip(fan * (255.0 / max(fan.max(), 1.0)), 0, 255).astype(np.uint8)
    bgr = cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)
    r0 = float(ranges[0]); span = float(ranges[-1]) - r0

    def fan_xy(r, b):
        pr = (r - r0) / span * H
        return int(round(half - pr * np.sin(b))), int(round((H - 1) - pr * np.cos(b)))

    bm = band_beam_mask(bearings, BAND_AZ, BAND_HALF)
    draw_intensity_grid(bgr, image, ranges, bearings, fan_xy, n_beam=NB, n_range=NR, band_mask=bm)
    cv2.imwrite(OUT, bgr)
    print(f"raw image max={image.max():.0f} mean={image.mean():.0f}; wrote {OUT} ({NB}x{NR} grid)")


if __name__ == "__main__":
    main()
