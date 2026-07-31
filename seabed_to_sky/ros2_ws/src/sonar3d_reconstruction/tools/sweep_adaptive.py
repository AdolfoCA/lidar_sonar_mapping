#!/usr/bin/env python3
"""Offline adaptive-detector parameter sweep on captured sonar frames.

Reuses the node's own detect_edges / denoise_image so results match the live node.
For each parameter set it computes leading-edge quality metrics (coverage, beam-to-
beam smoothness, scatter vs a local-median trend) aggregated over all frames, and
renders a fan comparison grid for one representative frame so you can SEE which
parameters make the red leading edge hug the bright seabed (the green-curve shape).
"""
import glob
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.ndimage import median_filter

from sonar3d_reconstruction.leading_edge_node import (
    denoise_image, detect_edges, band_beam_mask,
)

FRAMES = sorted(glob.glob("/tmp/sonar_frames/frame_*.npz"))
OUT_DIR = "/home/rosdev/ros2_ws/src/images/blueview"

# ---- fixed gate / band config (matches leading_edge.yaml) ----
MIN_BEARING = np.deg2rad(-41.0)
MAX_BEARING = np.deg2rad(41.0)
BAND_AZ = np.deg2rad(np.array([-23.0, 0.0, 23.0]))
BAND_HALF = 17
DENOISE = dict(standard=True, open_iters=0, destripe=False)


def load():
    out = []
    for f in FRAMES:
        d = np.load(f)
        out.append((d["image"], d["ranges"], d["bearings"]))
    return out


def metrics(edge_range, valid, ranges, bearings, min_range, max_range):
    """Coverage + coherence of the leading edge over eligible (gated, non-band) beams."""
    gate = (bearings >= MIN_BEARING) & (bearings <= MAX_BEARING)
    band = band_beam_mask(bearings, BAND_AZ, BAND_HALF)
    elig = gate & ~band
    n_elig = int(elig.sum())
    v = valid & elig & np.isfinite(edge_range)
    n_v = int(v.sum())
    cov = n_v / max(n_elig, 1)
    if n_v < 5:
        return dict(cov=cov, n=n_v, dstep=np.nan, jump=np.nan, scatter=np.nan)
    idx = np.flatnonzero(v)
    r = edge_range[idx]
    dr = np.abs(np.diff(r))
    dstep = float(np.median(dr))
    jump = float(np.mean(dr > 0.5))                 # fraction of big beam-to-beam jumps
    locmed = median_filter(r, size=min(11, (len(r) // 2) * 2 + 1), mode="nearest")
    scatter = float(1.4826 * np.median(np.abs(r - locmed)))   # robust residual from local trend
    return dict(cov=cov, n=n_v, dstep=dstep, jump=jump, scatter=scatter)


def run_detect(image_den, ranges, bearings, *, k, floor_pct, min_consec,
               min_range, max_range, contrast=False):
    band_mask = band_beam_mask(bearings, BAND_AZ, BAND_HALF)
    er, va, thr = detect_edges(
        image_den, ranges, bearings, mode="adaptive",
        fixed_threshold=0.0, adaptive_k=k, floor_percentile=floor_pct,
        min_range=min_range, max_range=max_range,
        min_bearing=MIN_BEARING, max_bearing=MAX_BEARING,
        min_consecutive=min_consec,
        band_mask=band_mask, band_mode="reject",       # band beams excluded -> measure clean edge
        contrast_enable=contrast, contrast_factor=1.5,
        contrast_train=10, contrast_guard=5,
    )
    return er, va


# ---- fan renderer (same convention as the node) ----
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
    pa = np.arctan2(-dy, dz)
    bi = (pa - first) / step
    bb = np.round(bi)
    ok = (bb >= 0) & (bb < n_beam) & (np.abs(bi - bb) * abs(step) <= tol)
    ri = np.round(dist * rscale)
    rok = (ri >= 0) & (ri < n_range)
    valid = inside & ok & rok
    return valid, np.clip(ri, 0, n_range - 1).astype(int), np.clip(bb, 0, n_beam - 1).astype(int), H, W, half


def render_fan(ax, image, ranges, bearings, edge_range, valid, title):
    fv, ri, bb, H, W, half = fan_lut(ranges, bearings)
    fan = np.where(fv, image[ri, bb], 0.0)
    img8 = np.clip(fan * (255.0 / max(fan.max(), 1.0)), 0, 255).astype(np.uint8)
    bgr = cv2.cvtColor(img8, cv2.COLOR_GRAY2RGB)
    r0 = float(ranges[0]); span = float(ranges[-1]) - r0
    cols = np.flatnonzero(valid)
    if cols.size:
        pr = (edge_range[cols] - r0) / span * H
        bx = np.round(half - pr * np.sin(bearings[cols])).astype(int)
        by = np.round((H - 1) - pr * np.cos(bearings[cols])).astype(int)
        m = (bx >= 0) & (bx < W) & (by >= 0) & (by < H)
        ax.imshow(bgr)
        ax.scatter(bx[m], by[m], s=2, c="red")
    else:
        ax.imshow(bgr)
    ax.set_title(title, fontsize=8)
    ax.axis("off")


def main():
    data = load()
    den = [(denoise_image(im, **DENOISE), r, b) for im, r, b in data]
    print(f"loaded {len(den)} frames\n")

    # range gate currently in YAML
    MINR, MAXR = 1.0, 8.0

    # ---- primary sweep: adaptive_k x min_consecutive (floor=50, contrast off) ----
    ks = [3.0, 5.0, 8.0, 12.0]
    mcs = [10, 20, 30]
    print("=== adaptive_k x min_consecutive  (floor_pct=50, range 1-8m, contrast off) ===")
    print(f"{'k':>4} {'mc':>4} | {'cov':>5} {'n':>4} {'dstep_m':>8} {'jump':>5} {'scat_m':>7}")
    grid_results = {}
    for k in ks:
        for mc in mcs:
            agg = []
            for im, r, b in den:
                er, va = run_detect(im, r, b, k=k, floor_pct=50.0, min_consec=mc,
                                    min_range=MINR, max_range=MAXR)
                agg.append(metrics(er, va, r, b, MINR, MAXR))
            cov = np.nanmean([a["cov"] for a in agg])
            n = np.nanmean([a["n"] for a in agg])
            ds = np.nanmean([a["dstep"] for a in agg])
            jp = np.nanmean([a["jump"] for a in agg])
            sc = np.nanmean([a["scatter"] for a in agg])
            grid_results[(k, mc)] = (cov, ds, jp, sc)
            print(f"{k:>4.0f} {mc:>4} | {cov:>5.2f} {n:>4.0f} {ds:>8.3f} {jp:>5.2f} {sc:>7.3f}")

    # ---- secondary: effect of min_range (kill near-apex streak) on a mid combo ----
    print("\n=== min_range sweep  (k=8, mc=20, floor=50) ===")
    print(f"{'minR':>5} | {'cov':>5} {'dstep_m':>8} {'jump':>5} {'scat_m':>7}")
    for mr in [1.0, 2.0, 3.0]:
        agg = []
        for im, r, b in den:
            er, va = run_detect(im, r, b, k=8.0, floor_pct=50.0, min_consec=20,
                                min_range=mr, max_range=MAXR)
            agg.append(metrics(er, va, r, b, mr, MAXR))
        print(f"{mr:>5.1f} | {np.nanmean([a['cov'] for a in agg]):>5.2f} "
              f"{np.nanmean([a['dstep'] for a in agg]):>8.3f} "
              f"{np.nanmean([a['jump'] for a in agg]):>5.2f} "
              f"{np.nanmean([a['scatter'] for a in agg]):>7.3f}")

    # ---- secondary: contrast gate on/off and floor_pct on a mid combo ----
    print("\n=== contrast & floor_pct  (k=8, mc=20, range 2-8m) ===")
    print(f"{'cfg':>18} | {'cov':>5} {'dstep_m':>8} {'jump':>5} {'scat_m':>7}")
    for label, fp, ct in [("floor50 noct", 50.0, False), ("floor75 noct", 75.0, False),
                          ("floor50 contrast", 50.0, True), ("floor75 contrast", 75.0, True)]:
        agg = []
        for im, r, b in den:
            er, va = run_detect(im, r, b, k=8.0, floor_pct=fp, min_consec=20,
                                min_range=2.0, max_range=MAXR, contrast=ct)
            agg.append(metrics(er, va, r, b, 2.0, MAXR))
        print(f"{label:>18} | {np.nanmean([a['cov'] for a in agg]):>5.2f} "
              f"{np.nanmean([a['dstep'] for a in agg]):>8.3f} "
              f"{np.nanmean([a['jump'] for a in agg]):>5.2f} "
              f"{np.nanmean([a['scatter'] for a in agg]):>7.3f}")

    # ---- render a comparison grid for one representative frame ----
    im0, r0, b0 = den[0]
    combos = [(3, 20), (5, 20), (8, 20), (8, 10), (8, 30), (12, 20)]
    fig, ax = plt.subplots(2, 3, figsize=(15, 12))
    for axi, (k, mc) in zip(ax.ravel(), combos):
        er, va = run_detect(im0, r0, b0, k=float(k), floor_pct=50.0, min_consec=mc,
                            min_range=2.0, max_range=MAXR)
        m = metrics(er, va, r0, b0, 2.0, MAXR)
        render_fan(axi, im0, r0, b0, er, va,
                   f"k={k} mc={mc} | cov={m['cov']:.2f} scat={m['scatter']:.3f}m jump={m['jump']:.2f}")
    fig.suptitle("Adaptive leading edge — frame 0 — range gate 2-8 m, floor_pct=50", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    p = f"{OUT_DIR}/adaptive_sweep_grid.png"
    fig.savefig(p, dpi=90)
    print(f"\nsaved comparison grid -> {p}")


if __name__ == "__main__":
    main()
