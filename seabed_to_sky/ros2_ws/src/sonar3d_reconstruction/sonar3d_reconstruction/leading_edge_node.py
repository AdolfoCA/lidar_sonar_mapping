#!/usr/bin/env python3
"""
leading_edge_node.py
====================

A small, self-contained leading-edge extractor for an imaging sonar.

Pipeline (one frame):
    1. Build the B-scan image (range x beam) from a ProjectedSonarImage, then
       optionally TEMPORALLY AVERAGE it with the previous frame(s)
       (temporal_avg_frames): the speckle is uncorrelated frame to frame while the
       returns are not, so the mean denoises without blurring the image spatially.
    2. Prepare the DETECTION image: either the NN-corrected image (averaged x per-beam
       gain factor, detect_on_corrected) or the legacy subtraction denoise
       (filter_horizontal_image / optional open / optional remove_bscan_stripes).
    3. DETECT the leading edge per beam:
         - "adaptive" (default): a DATA-DRIVEN per-beam threshold from each beam's
           own intensity spectrum, thr = floor(percentile) + k*(1.4826*MAD).
         - "fixed": one `threshold` for all beams (optional auto-lower to a coverage
           target). The edge is the first range bin (near->far) above threshold for
           >= min_consecutive bins, within the range/bearing gate.
         - "window": seed with the "fixed" edge, then REFINE it inside a window
           [seed - l, seed + u] to a bin that (1) is the brightest of the window,
           (2) is a local maximum along range (negative forward gradient) and
           (3) sits as close as possible to its neighbouring beams' picks. The
           three criteria are traded off in ONE global cost, solved exactly by
           dynamic programming over the beams. See detect_window_edges.
       If band_enable, the known-noisy azimuth beams (band_azimuths +- band_half_beams)
       are hard-rejected (blanked).
    4. Build points, applying the per-beam intensity correction, and publish as a
       sensor_msgs/PointCloud2.
    5. Optionally (shape_enable) extract solid bright OBJECTS from the intensity-
       corrected image (top-hat background subtraction -> binarise -> open/close
       morphology -> AKAZE keypoints -> leading-edge association, paper eqs 39-42,
       Alg. 1 line 1) and append the six shape channels (flag, N_R, log-area,
       solidity, shadow, texture) as extra float32 fields on the published cloud,
       feeding the downstream eq-42 shape factor. This REPLACES the old bright-
       region growing. When disabled the cloud keeps the old 4-field layout,
       byte-identical.

All per-frame ops are vectorised numpy -> comfortably >9 Hz.

That's it. No pitch, no TF, no RANSAC/MAD. Geometry only:
    - The polar message is z-forward, y-lateral, so a beam's bearing is
      atan2(y, z) and the in-plane point is (x=r*cos(bearing), y=-r*sin(bearing)).
      The -sin matches the old pipeline (edge_detection.image2Profile); using
      +sin mirrors the beams left<->right.
    - Row i of the image is range bin i; row 0 == ranges[0] is the NEAREST bin
      (ranges are sorted near->far), so the leading edge is argmax over rows.

Everything tunable lives in the YAML; see config/leading_edge.yaml.
"""

import os
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.ndimage import median_filter, uniform_filter1d, gaussian_filter1d

from marine_acoustic_msgs.msg import ProjectedSonarImage
from sensor_msgs.msg import PointCloud2, PointField

# Reuse the OLD pipeline's denoising verbatim so this node's noise processing is
# byte-for-byte the same as /blueview/point2/leading (no re-implementation/drift).
from sonar3d_reconstruction.process_sonar_image import (
    filter_horizontal_image,
    remove_bscan_stripes,
)
# AKAZE/blob morphology + shape descriptors (paper eqs 39-42) live in a pure
# NumPy/OpenCV module so they stay unit-testable without ROS.
from sonar3d_reconstruction.region_shapes import CHANNEL_NAMES, compute_region_channels


# ProjectedSonarImage.image.dtype codes actually emitted by the sonar driver.
_DTYPE = {0: np.uint8, 2: np.uint16}

# Per-class overlay colours (BGR) for the shape_factor.png debug image. Indexed
# by class position in classes.yaml; the catch-all "other" is forced to grey and
# unlabelled regions (no classifier available) to yellow, both handled below.
_SHAPE_CLASS_BGR = [
    (240, 176, 0),    # deep sky blue
    (96, 214, 96),    # green
    (60, 60, 240),    # red
    (0, 176, 255),    # orange
    (240, 96, 240),   # magenta
    (224, 224, 64),   # cyan
]
_SHAPE_OTHER_BGR = (170, 170, 170)     # the "other" catch-all
_SHAPE_UNLABELLED_BGR = (0, 255, 255)  # regions drawn without a class (no sonar_map)


# ─────────────────────────── geometry / image ────────────────────────────
def sonar_image_from_msg(msg: ProjectedSonarImage, flip_ranges: bool = False):
    """Return (image, ranges, bearings) from a ProjectedSonarImage.

    image    : float64 (n_range, n_beam); row i is range bin i, row 0 = nearest.
    ranges   : float64 (n_range,) range of each row in metres (sorted near->far).
    bearings : float64 (n_beam,) bearing of each column in radians, ascending.

    By default the raw buffer is assumed to already run near->far top->bottom, so
    row i pairs with ranges[i]. If `flip_ranges` is True, the range axis (image
    rows) is reversed -- use this when the driver stores the buffer far->near
    while `ranges` is near->far, which otherwise mis-pairs every row with the
    wrong range (the leading edge lands behind the bright return). We reverse the
    image rows, NOT the `ranges` array, so `ranges` stays ascending and the range
    gate / debug NEAR-FAR labels / step computation all remain valid.
    """
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    n_range = ranges.size
    n_beam = msg.image.beam_count

    dtype = _DTYPE.get(msg.image.dtype, np.uint8)
    image = np.frombuffer(msg.image.data, dtype=dtype).reshape(n_range, n_beam).astype(np.float64)

    # Polar frame is z-forward, y-lateral -> in-plane bearing is atan2(y, z).
    bearings = np.arctan2(
        np.fromiter((v.y for v in msg.beam_directions), dtype=np.float64, count=n_beam),
        np.fromiter((v.z for v in msg.beam_directions), dtype=np.float64, count=n_beam),
    )
    # Keep bearings ascending (left -> right); flip the columns to match.
    if n_beam >= 2 and bearings[0] > bearings[-1]:
        bearings = bearings[::-1]
        image = image[:, ::-1]

    # Optional range-axis flip: reverse the rows so the (formerly far) top of the
    # buffer pairs with the near ranges again. `ranges` is left ascending.
    if flip_ranges:
        image = image[::-1, :]

    return image, ranges, bearings


def denoise_image(image: np.ndarray, standard: bool, open_iters: int, destripe: bool) -> np.ndarray:
    """Same noise processing as the old acoustic3d_edge pipeline, in the same order.

    Calls the shared functions from process_sonar_image so the result matches
    /blueview/point2/leading exactly:
      - standard : filter_horizontal_image (per-range-bin background subtraction,
                   global noise floor, center-beam gain mask, range smoothing,
                   soft percentile threshold)
      - open     : morphological opening with a 3x1 kernel, `open_iters` iterations
      - destripe : remove_bscan_stripes (destriping + ring-down suppression)
    Returns a float64 image with the input's near->far row order preserved.
    """
    out = image
    if standard:
        out = filter_horizontal_image(out)                       # -> uint16
    if open_iters > 0:
        if out.dtype != np.uint16:
            out = np.clip(out, 0, 65535).astype(np.uint16)
        kernel = np.ones((3, 1), np.uint8)
        out = cv2.morphologyEx(out, cv2.MORPH_OPEN, kernel, iterations=int(open_iters))
    if destripe:
        if out.dtype != np.uint16:
            out = np.clip(out, 0, 65535).astype(np.uint16)       # remove_bscan_stripes needs an int dtype
        out = remove_bscan_stripes(out)
    return np.asarray(out, dtype=np.float64)


def _runs_start(occ: np.ndarray, k: int) -> np.ndarray:
    """Boolean array, True where a run of >= k consecutive True values starts.

    Used so a single noisy bin above threshold does not count as an edge: the
    edge must be the start of at least ``k`` consecutive supra-threshold bins.
    """
    if k <= 1:
        return occ
    n = occ.shape[0]
    acc = occ.copy()
    for s in range(1, k):
        acc[: n - s] &= occ[s:]
    acc[n - k + 1:] = False
    return acc


def band_beam_mask(bearings, band_azimuths, band_half_beams):
    """Bool[n_beam]: True for beams within +-band_half_beams of any azimuth (radians).
    Marks the known-noisy azimuth bands (driver beam-pattern peaks / self-reflections).
    The same mask drives BOTH the band threshold override and the regularizer
    exclusion, so the two always act on exactly the same beams."""
    n_beam = bearings.shape[0]
    mask = np.zeros(n_beam, dtype=bool)
    for az in np.atleast_1d(np.asarray(band_azimuths, dtype=np.float64)):
        j = int(np.argmin(np.abs(bearings - az)))
        lo = max(0, j - int(band_half_beams))
        hi = min(n_beam, j + int(band_half_beams) + 1)
        mask[lo:hi] = True
    return mask


def detect_edges(
    image, ranges, bearings, *, mode, fixed_threshold, adaptive_k, floor_percentile,
    min_range, max_range, min_bearing, max_bearing, min_consecutive,
    band_mask=None, autolower=False, min_coverage=0.5, threshold_step=200.0,
    threshold_min=0.0, window=None, collect_debug=False,
):
    """Per-beam leading-edge detection -> (meas_range, meas_valid, thr[, dbg]).

    If `collect_debug` is True, a 4th value is returned: a dict of the per-stage
    intermediates (floor, thr, occupancy masks, range_gate, edge) for the debug figure.

    mode="adaptive": each beam's threshold is DATA-DRIVEN from its own intensity
        spectrum over the gated range: thr = percentile(floor) + k*(1.4826*MAD).
    mode="fixed":    one `fixed_threshold` for every beam. If `autolower` is set, the
        threshold is LOWERED by `threshold_step` and re-run until at least
        `min_coverage` of the eligible (gated, non-band) beams have an edge, down to
        a floor of `threshold_min`.
    mode="window":   the "fixed" edge, then refined to the best local maximum inside a
        window around it (see detect_window_edges). `window` carries that mode's
        settings as a dict; `autolower` does not apply.

    band_mask (from band_beam_mask): those known-noisy beams are HARD-REJECTED -- they
    emit no edge, so the noisy azimuths are blanked. Pass None to detect them normally.

    The edge is the first range bin (near->far) at/above the threshold for at least
    `min_consecutive` bins, within [min_range, max_range].
    """
    if mode == "window":
        return detect_window_edges(
            image, ranges, bearings, threshold=fixed_threshold,
            min_consecutive=min_consecutive, min_range=min_range, max_range=max_range,
            min_bearing=min_bearing, max_bearing=max_bearing, band_mask=band_mask,
            collect_debug=collect_debug, **(window or {}))

    n_range, n_beam = image.shape
    range_gate = (ranges >= min_range) & (ranges <= max_range)
    bearing_gate = (bearings >= min_bearing) & (bearings <= max_bearing)
    # Rejected band beams can never be "covered" -> keep them out of the auto-lower denom.
    reject_band = band_mask if band_mask is not None else np.zeros(n_beam, dtype=bool)

    def _detect(thr_vec):
        """thr per beam -> (edge, occ_thr, occ, meas_valid) with band beams rejected."""
        occ_thr = image >= thr_vec[None, :]
        occ = _runs_start(occ_thr, min_consecutive)
        occ &= range_gate[:, None]
        edge = np.argmax(occ, axis=0)
        has = occ[edge, np.arange(n_beam)]
        mv = has & bearing_gate
        if band_mask is not None:
            mv = mv & ~band_mask
        return edge, occ_thr, occ, mv

    if mode == "adaptive" and range_gate.any():
        g = image[range_gate, :]
        med = np.median(g, axis=0)
        # floor_percentile == 50 IS the median, so don't sort twice for it.
        floor = med if floor_percentile == 50.0 else np.percentile(g, floor_percentile, axis=0)
        mad = np.median(np.abs(g - med), axis=0) * 1.4826      # robust std estimate
        thr = floor + adaptive_k * mad                         # (n_beam,)
        edge, occ_thr, occ, meas_valid = _detect(thr)
    else:
        # Fixed threshold, optionally auto-lowered until coverage >= min_coverage.
        n_elig = int((bearing_gate & ~reject_band).sum())
        do_lower = bool(autolower) and threshold_step > 0.0 and n_elig > 0
        thr_val = float(fixed_threshold)
        for _ in range(1000):                                  # bounded by step/floor anyway
            thr = np.full(n_beam, thr_val)
            edge, occ_thr, occ, meas_valid = _detect(thr)
            if not do_lower:
                break
            coverage = int(meas_valid.sum()) / n_elig          # meas_valid is eligible-only
            if coverage >= float(min_coverage) or thr_val <= float(threshold_min):
                break
            thr_val = max(float(threshold_min), thr_val - float(threshold_step))
        floor = thr                                            # no separate floor in fixed mode

    meas_range = ranges[edge]
    if collect_debug:
        dbg = dict(
            floor=floor, thr=thr, range_gate=range_gate,
            occ_floor=image >= floor[None, :],   # above the floor (pre-k*sigma)
            occ_thr=occ_thr,                     # above threshold, pre gate/min_consecutive
            occ_final=occ,                       # final occupancy used to pick the edge
            edge=edge,
        )
        return meas_range, meas_valid, thr, dbg
    return meas_range, meas_valid, thr


# ───────────────────── window detector (peak + cross-beam smoothness) ────────
def _run_min(image, k):
    """rmin[i, b] = min(image[i:i+k, b]) -- -inf where a run of k does not fit.

    This is the threshold-free form of _runs_start: a run of k consecutive bins
    starting at row i clears threshold T exactly when rmin[i] >= T, so

        _runs_start(image >= T, k)  ==  (rmin >= T)      for ANY T.

    One running minimum therefore answers "does this beam have a run at level T"
    for every T at once -- which is what makes the per-beam threshold auto-lower
    a couple of comparisons instead of a re-scan of the image per level.
    """
    n = image.shape[0]
    if k <= 1:
        return image
    if n < k:
        return np.full(image.shape, -np.inf)
    out = image.copy()
    for s in range(1, k):
        np.minimum(out[: n - s], image[s:], out=out[: n - s])
    out[n - k + 1:] = -np.inf
    return out


def _local_peak_mask(image):
    """Bool[n_range, n_beam]: True where a bin is a LOCAL MAXIMUM along range.

    Local maximum == the amplitude rises into the bin and falls out of it, i.e. the
    forward gradient at the bin is strictly negative (criterion 2 of the window
    detector). ``>=`` on the way in so the LAST bin of a flat plateau is the peak
    (a plateau is one return, not several). The first bin has nothing rising into
    it and the last has no forward gradient to test, so neither can qualify.
    """
    peak = np.zeros(image.shape, dtype=bool)
    if image.shape[0] < 3:
        return peak
    d = np.diff(image, axis=0)                     # d[i] = I[i+1] - I[i]
    peak[1:-1] = (d[:-1] >= 0) & (d[1:] < 0)
    return peak


def detect_window_edges(
    image, ranges, bearings, *, threshold, min_consecutive,
    min_range, max_range, min_bearing, max_bearing,
    low_bins=3, high_bins=12, smooth_weight=0.5, smooth_cap_bins=0,
    max_candidates=8, require_peak=False, autolower=False, autolower_step=200.0,
    autolower_min=0.0, band_mask=None, collect_debug=False,
):
    """Window leading-edge detector -> (meas_range, meas_valid, thr[, dbg]).

    Three stages, all working on the (temporally averaged) detection image:

      1. SEED. The first range bin (near->far, inside the range gate) at/above the
         fixed `threshold` t for at least `min_consecutive` consecutive bins --
         exactly the "fixed" detector's edge.

         COVERAGE (`autolower`): t is a compromise -- high enough for the loud beams
         is too high for the quiet ones, which then return nothing at all. So a beam
         that finds no crossing at t retries at t - autolower_step, t - 2*step, ...
         until it does, stopping at `autolower_min`. The lowering is PER BEAM and
         independent: a beam that already has an edge keeps the full t, so the loud
         beams are never made noisier to rescue the quiet ones (unlike the "fixed"
         mode's `fixed_autolower`, which drops the threshold for the WHOLE frame).
         Each pass re-scans only the beams still empty, so the work shrinks fast.
         `thr` comes back holding the threshold each beam actually used, and that
         per-beam value is what stage 2 requires its candidates to clear.

      2. WINDOW + CANDIDATES. Around each seed, look at the bins
         [seed - low_bins, seed + high_bins] (clipped to the range gate) and collect
         every bin that is at/above t AND is a local maximum along range
         (_local_peak_mask -- criterion 2, the gradient at the pick is negative).
         Candidates are ranked by intensity and the brightest `max_candidates` kept.
         A beam whose window holds no peak falls back to the window's plain argmax,
         which still satisfies criterion 1 -- or is dropped entirely when
         `require_peak`, so every published point is a true peak.

      3. GLOBAL PICK (criteria 1 + 3 together). Taking each beam's brightest
         candidate independently satisfies criterion 1 but leaves a ragged edge, so
         the pick is the chain of candidates minimising, over the whole swath,

             sum_b data(b, c_b)  +  lam * sum_b min(|r_b - r_{b+1}|, cap) / span

         data(b, c) = 1 - (I_c - I_min_win) / (I_max_win - I_min_win), in [0, 1] and
         ZERO for the window's brightest bin -- so with lam = 0 every beam takes its
         window maximum (criterion 1 exactly) and nothing else. span = low+high+1
         normalises the jump term, making `smooth_weight` (lam) scale-free: lam = 1
         means "crossing the whole window costs as much as giving up the window's
         entire intensity range". The L1 jump term is criterion 3; `smooth_cap_bins`
         truncates it (0 = uncapped) so a genuine step in the scene costs a bounded
         amount instead of dragging the neighbours off their peaks.

         Solved EXACTLY by dynamic programming (Viterbi) over the per-beam candidate
         sets -- not greedily, so the chosen chain is the global optimum of that cost.
         Only beams ADJACENT in index are coupled: a beam with no seed (or a rejected
         band beam) breaks the chain, and the DP restarts on the far side, so
         smoothness is never enforced across a gap in the swath.

    `band_mask` beams are hard-rejected exactly as in the other modes, and are never
    auto-lowered (a blanked beam must stay blank). `thr` comes back per beam.
    """
    n_range, n_beam = image.shape
    range_gate = (ranges >= min_range) & (ranges <= max_range)
    bearing_gate = (bearings >= min_bearing) & (bearings <= max_bearing)
    eligible = bearing_gate if band_mask is None else (bearing_gate & ~band_mask)
    thr = np.full(n_beam, float(threshold))
    low_bins, high_bins = max(0, int(low_bins)), max(0, int(high_bins))

    # ---- 1. seed = the "fixed" detector's edge, at a PER-BEAM threshold ------
    gate_col = range_gate[:, None]
    rmin = _run_min(image, min_consecutive)     # highest level each run start clears
    occ = (rmin >= thr[None, :]) & gate_col
    seed = np.argmax(occ, axis=0)
    valid = occ[seed, np.arange(n_beam)] & eligible

    # Coverage: a beam with no crossing at t drops to t - step, t - 2*step, ... until
    # it finds one, floored at autolower_min. `best` is the highest level the beam can
    # possibly clear, so the level it would stop at is closed-form -- no re-scanning
    # the image once per threshold level.
    if autolower and float(autolower_step) > 0.0:
        step = float(autolower_step)
        best = np.where(gate_col, rmin, -np.inf).max(axis=0)      # (n_beam,)
        need = eligible & ~valid & np.isfinite(best)
        if need.any():
            dark = np.flatnonzero(need)
            steps = np.ceil((float(threshold) - best[dark]) / step)
            lvl = np.maximum(float(autolower_min), float(threshold) - steps * step)
            ok = lvl <= best[dark]              # False = the floor cut this beam off
            cols, lvl = dark[ok], lvl[ok]
            if cols.size:
                thr[cols] = lvl
                valid[cols] = True
                occ[:, cols] = (rmin[:, cols] >= lvl[None, :]) & gate_col
                seed[cols] = np.argmax(occ[:, cols], axis=0)
    occ_thr = image >= thr[None, :]             # per-beam; gates the candidates below
    edge = seed.copy()

    gate_rows = np.flatnonzero(range_gate)
    if gate_rows.size and valid.any():
        row_lo, row_hi = int(gate_rows[0]), int(gate_rows[-1])

        # ---- 2. candidates: local maxima at/above t inside each beam's window ----
        # Every beam's window is GATHERED into one (span, n_beam) array so the whole
        # stage is vectorised: rows past the range gate are clamped to its edge (which
        # cannot change a min/max) and masked out of the candidate test by `inwin`.
        peak = _local_peak_mask(image) & occ_thr
        n_cand = max(1, int(max_candidates))
        span = float(low_bins + high_bins + 1)
        cols = np.arange(n_beam)
        wrow = seed[None, :] + np.arange(-low_bins, high_bins + 1)[:, None]
        inwin = (wrow >= row_lo) & (wrow <= row_hi)
        np.clip(wrow, row_lo, row_hi, out=wrow)
        win = image[wrow, cols[None, :]]                   # window intensities
        # Rank each beam's candidates by intensity; -inf == "not a candidate".
        score = np.where(peak[wrow, cols[None, :]] & inwin, win, -np.inf)
        order = np.argsort(-score, axis=0)[:n_cand]
        sel_row = np.take_along_axis(wrow, order, axis=0)
        sel_val = np.take_along_axis(score, order, axis=0)
        good = np.isfinite(sel_val)
        none = ~good[0]                                    # window holds no local max
        if none.any():
            if require_peak:
                valid = valid & ~none
            else:                                          # criterion-1 fallback
                am = np.argmax(np.where(inwin, win, -np.inf), axis=0)
                sel_row[0, none] = wrow[am[none], cols[none]]
                sel_val[0, none] = win[am[none], cols[none]]
                good[0, none] = True
        # data cost: 0 for the window's brightest bin, 1 for its darkest.
        w_lo = np.where(inwin, win, np.inf).min(axis=0)
        w_hi = np.where(inwin, win, -np.inf).max(axis=0)
        w_rng = w_hi - w_lo
        with np.errstate(invalid="ignore", divide="ignore"):
            dim = np.where(w_rng > 0.0, (w_hi - sel_val) / w_rng, 0.0)
        cand = np.ascontiguousarray(np.where(good, sel_row, -1).T)          # (n_beam, K)
        cost = np.ascontiguousarray(np.where(good & valid[None, :], dim, np.inf).T)

        # ---- 3. cross-beam smoothness: exact DP over the candidate chains -------
        vb = np.flatnonzero(valid)
        lam = float(smooth_weight)
        if lam <= 0.0:                            # decoupled: each beam's own maximum
            edge[vb] = cand[vb, np.argmin(cost[vb], axis=1)]
        elif vb.size:
            cap = int(smooth_cap_bins)
            candf = cand.astype(np.float64)            # |row difference| in float
            w = lam / span                             # cost of ONE bin of jump
            acc = np.full((n_beam, n_cand), np.inf)    # best cost of a chain ending here
            back = np.zeros((n_beam, n_cand), dtype=np.int64)
            prev = -2
            for b in vb:
                if b != prev + 1:                 # gap -> start a new chain
                    acc[b] = cost[b]
                else:
                    dr = np.abs(candf[b][None, :] - candf[prev][:, None])
                    if cap > 0:
                        np.minimum(dr, cap, out=dr)
                    tot = acc[prev][:, None] + w * dr
                    back[b] = tot.argmin(axis=0)
                    acc[b] = cost[b] + tot.min(axis=0)
                prev = b
            # Backtrack each chain from its last beam's cheapest end state.
            ends = vb[np.r_[np.diff(vb) != 1, True]]       # last beam of every chain
            starts = vb[np.r_[True, np.diff(vb) != 1]]
            for s, e in zip(starts, ends):
                j = int(np.argmin(acc[e]))
                if not np.isfinite(acc[e, j]):    # no finite chain (cannot happen) -> seeds
                    continue
                for b in range(int(e), int(s) - 1, -1):
                    edge[b] = cand[b, j]
                    if b > s:
                        j = int(back[b, j])

    meas_range = ranges[edge]
    if collect_debug:
        dbg = dict(
            floor=thr, thr=thr, range_gate=range_gate,
            occ_floor=occ_thr,                   # above t, pre gate/min_consecutive
            occ_thr=occ_thr,
            occ_final=occ,                       # occupancy the SEED is picked from
            edge=edge, seed=seed,
        )
        return meas_range, valid, thr, dbg
    return meas_range, valid, thr


def _next_edge_by_intensity(col, r0, thr_val, k, dp):
    """The next leading edge after row r0 using an INTENSITY threshold thr_val.

    Same first-crossing principle as the 1st edge (a run of >= k bins with
    col >= thr_val), but: the threshold is thr_val (not the global detection
    threshold), and the edge must start within dp bins of r0 (the zone). `col`
    is one beam's intensity column, already masked to the range gate. Returns
    the run-start row, or -1 if none qualifies within the zone.
    """
    occ = col >= thr_val
    d = np.diff(np.concatenate(([0], occ.astype(np.int8), [0])))
    starts = np.flatnonzero(d == 1)
    ends = np.flatnonzero(d == -1)                          # exclusive
    keep = (ends - starts) >= k
    starts, ends = starts[keep], ends[keep]
    if starts.size == 0:
        return -1
    # End of the return covering r0 (the previous edge's own return under this
    # threshold), then the next qualifying run that starts at/after it.
    cover = np.flatnonzero((starts <= r0) & (r0 < ends))
    if cover.size:
        end0 = int(ends[cover[0]])
    else:                                                  # r0 off a run (rounding)
        before = np.flatnonzero(starts <= r0)
        end0 = int(ends[before[-1]]) if before.size else int(ends[0])
    nxt = starts[starts >= end0]
    if nxt.size == 0:
        return -1
    e = int(nxt[0])
    return e if (e - r0) <= int(dp) else -1                # inside the zone [r0, r0+dp]


def detect_secondary_edges(image, edge_rows, valid, min_consecutive,
                           range_gate, zone_dp):
    """Second and third leading edges per beam (experimental — for sonar_image.png).

    Each edge uses the PREVIOUS edge's INTENSITY as its detection threshold — the
    2nd edge is the next return at least as bright as the 1st edge, the 3rd the
    next at least as bright as the 2nd:

      zone     : the window [prev edge, prev edge + zone_dp] range bins.
      2nd edge : first-crossing (run of >= min_consecutive bins) of the threshold
                 I1 = image[1st edge], after the 1st edge's own return, whose start
                 falls inside the zone [1st edge, 1st edge + zone_dp]. Because the
                 1st return is min_consecutive bins long, zone_dp must exceed
                 min_consecutive for a 2nd edge to fit.
      good beam: a 2nd edge was found in its zone.
      3rd edge : (good beams only) the same, one level down — threshold
                 I2 = image[2nd edge], zone [2nd edge, 2nd edge + zone_dp].
                 Ordering along the beam is 1st < 2nd < 3rd, each >= the previous
                 in intensity.

    Returns (edge2, edge3, good): edge2/edge3 are (n_beam,) int ROW indices (-1
    where absent) and good is a (n_beam,) bool mask. Purely diagnostic — nothing
    here feeds the published cloud.
    """
    n_range, n_beam = image.shape
    k = int(min_consecutive)
    dp = int(zone_dp)
    gate = np.asarray(range_gate, dtype=bool)
    edge2 = np.full(n_beam, -1, dtype=np.int64)
    edge3 = np.full(n_beam, -1, dtype=np.int64)
    good = np.zeros(n_beam, dtype=bool)
    for b in np.flatnonzero(np.asarray(valid, dtype=bool)):
        # Beam column masked to the range gate (bins outside can't be an edge).
        col = np.where(gate, image[:, b].astype(np.float64), -np.inf)
        e1 = int(edge_rows[b])
        i1 = float(image[e1, b])                            # 1st edge intensity = threshold for 2nd
        e2 = _next_edge_by_intensity(col, e1, i1, k, dp)
        if e2 < 0:
            continue
        edge2[b] = e2
        good[b] = True
        i2 = float(image[e2, b])                            # 2nd edge intensity = threshold for 3rd
        e3 = _next_edge_by_intensity(col, e2, i2, k, dp)
        if e3 >= 0:
            edge3[b] = e3
    return edge2, edge3, good


def make_pointcloud2(xyz: np.ndarray, intensity: np.ndarray, stamp, frame_id: str,
                     extra: dict | None = None) -> PointCloud2:
    """Pack Nx3 xyz + intensity into a float32 PointCloud2.

    extra: optional {name: (N,) array} of ADDITIONAL float32 fields, appended
    after `intensity` in dict order (the shape channels when shape_enable).
    With extra=None the layout is the original 4-field / 16-byte cloud,
    byte-identical to the pre-shape node, so existing consumers see no change.
    """
    n = xyz.shape[0]
    names = list(extra.keys()) if extra else []
    buf = np.empty((n, 4 + len(names)), dtype=np.float32)
    buf[:, :3] = xyz
    buf[:, 3] = intensity
    for j, name in enumerate(names):
        buf[:, 4 + j] = extra[name]

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = n
    msg.is_bigendian = False
    msg.is_dense = True
    msg.point_step = 4 * (4 + len(names))
    msg.row_step = msg.point_step * n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ] + [
        PointField(name=name, offset=16 + 4 * j, datatype=PointField.FLOAT32, count=1)
        for j, name in enumerate(names)
    ]
    msg.data = buf.tobytes()
    return msg


def draw_intensity_grid(bgr, image, ranges, bearings, fan_xy, *, n_beam, n_range,
                        band_mask=None, corr_img=None):
    """Overlay a polar section grid on the fan `bgr` and label each cell with the RAW
    mean (top, white/yellow) and median (below, grey) intensity. Beams x range bins are
    split into n_beam x n_range sections. Band-beam cells are YELLOW so you can compare
    band vs neighbour. Read it as: mean >> median -> a few saturated spikes (additive);
    BOTH elevated (esp. in a near-range pure-water row) -> a real per-beam scale/gain
    difference. `image` must be the RAW intensity image (rows=range, cols=beam).
    If `corr_img` (per-pixel correction factor) is given, band cells also show the
    CORRECTED mean (raw*factor) in BLUE -- the new noisy-beam intensity."""
    n_r, n_b = image.shape
    if n_r < 2 or n_b < 2 or ranges[-1] == ranges[0]:
        return
    be = np.linspace(0, n_b, int(n_beam) + 1).astype(int)
    re = np.linspace(0, n_r, int(n_range) + 1).astype(int)
    rr0, rr1 = float(ranges[0]), float(ranges[-1])
    grid = (90, 90, 90)
    for bi in be:                                     # radial lines at beam-section edges
        cv2.line(bgr, fan_xy(rr0, bearings[min(bi, n_b - 1)]),
                 fan_xy(rr1, bearings[min(bi, n_b - 1)]), grid, 1, cv2.LINE_AA)
    for ri in re:                                     # arcs at range-section edges
        r = ranges[min(ri, n_r - 1)]
        cv2.polylines(bgr, [np.array([fan_xy(r, b) for b in bearings], np.int32)],
                      False, grid, 1, cv2.LINE_AA)
    f = cv2.FONT_HERSHEY_SIMPLEX
    for bi in range(int(n_beam)):
        b0, b1 = int(be[bi]), int(be[bi + 1])
        if b1 <= b0:
            continue
        is_band = bool(band_mask[b0:b1].any()) if band_mask is not None else False
        col = (0, 255, 255) if is_band else (235, 235, 235)
        midb = bearings[(b0 + b1) // 2]
        for ri in range(int(n_range)):
            r0i, r1i = int(re[ri]), int(re[ri + 1])
            if r1i <= r0i:
                continue
            sub = image[r0i:r1i, b0:b1]
            mean, med = float(sub.mean()), float(np.median(sub))
            x, y = fan_xy(ranges[(r0i + r1i) // 2], midb)
            t1 = f"{mean:.0f}"
            (tw, _), _ = cv2.getTextSize(t1, f, 0.4, 1)
            cv2.putText(bgr, t1, (x - tw // 2, y), f, 0.4, col, 1, cv2.LINE_AA)
            cv2.putText(bgr, f"{med:.0f}", (x - tw // 2, y + 13), f, 0.38, (150, 150, 150), 1, cv2.LINE_AA)
            # band cells: the CORRECTED mean (raw*factor) in BLUE = the new noisy intensity
            if is_band and corr_img is not None and corr_img.shape == image.shape:
                cmean = float((sub * corr_img[r0i:r1i, b0:b1]).mean())
                cv2.putText(bgr, f"{cmean:.0f}", (x - tw // 2, y + 26), f, 0.4,
                            (255, 128, 0), 1, cv2.LINE_AA)   # BGR -> blue
    cv2.putText(bgr, "raw mean / median (yellow=band) / corrected (blue)", (4, 32),
                f, 0.45, (0, 255, 255), 1, cv2.LINE_AA)


# ──────────────────────────────── node ───────────────────────────────────
class LeadingEdgeNode(Node):
    def __init__(self):
        super().__init__("leading_edge_node")

        p = self.declare_parameter
        # I/O
        self.input_topic = p("input_topic", "/blueview_message_polar").value
        self.output_topic = p("output_topic", "/blueview/leading_edge").value
        self.frame_id = p("frame_id", "blueview_sonar").value
        qos_rel = str(p("qos_reliability", "best_effort").value).lower()
        # TEMPORAL denoise: average the last N consecutive sonar images before any
        # other processing. 1 = off (this frame only), 2 = mean of this frame and the
        # previous one. Speckle is uncorrelated ping to ping while the returns are
        # not, so the mean lifts the SNR without blurring the image spatially the way
        # the morphological/subtraction denoise below does.
        self.temporal_avg_frames = max(1, int(p("temporal_avg_frames", 1).value))
        self._frame_buf = deque(maxlen=self.temporal_avg_frames)
        # denoising (same processing as the old acoustic3d_edge pipeline)
        self.denoise_standard = bool(p("denoise_standard", True).value)
        self.denoise_open = int(p("denoise_open", 0).value)
        self.denoise_destripe = bool(p("denoise_destripe", False).value)
        # detection: "adaptive" (per-beam data-driven threshold), "fixed", or "window"
        self.detector = str(p("detector", "adaptive").value).lower()
        self.adaptive_k = float(p("adaptive_k", 5.0).value)            # sigmas above the floor
        self.floor_percentile = float(p("floor_percentile", 50.0).value)
        self.threshold = float(p("threshold", 3500.0).value)          # fixed/window mode
        # Fixed-mode auto-lower: drop the threshold until coverage >= target.
        self.fixed_autolower = bool(p("fixed_autolower", False).value)
        self.fixed_min_coverage = float(p("fixed_min_coverage", 0.5).value)
        self.fixed_threshold_step = float(p("fixed_threshold_step", 200.0).value)
        self.fixed_threshold_min = float(p("fixed_threshold_min", 0.0).value)
        # Window mode: refine the fixed-threshold seed to the best local maximum in
        # [seed - window_low_bins, seed + window_high_bins], trading intensity against
        # cross-beam continuity in one globally-optimal (DP) pick. See
        # detect_window_edges for the cost function these five knobs parameterise.
        self._window_cfg = dict(
            low_bins=int(p("window_low_bins", 3).value),
            high_bins=int(p("window_high_bins", 12).value),
            smooth_weight=float(p("window_smooth_weight", 0.5).value),
            smooth_cap_bins=int(p("window_smooth_cap_bins", 0).value),
            max_candidates=int(p("window_max_candidates", 8).value),
            require_peak=bool(p("window_require_peak", False).value),
            # Coverage: a beam with no crossing at `threshold` retries at a lower
            # threshold, on its own, until it finds one (floor = autolower_min).
            autolower=bool(p("window_autolower", True).value),
            autolower_step=float(p("window_autolower_step", 200.0).value),
            autolower_min=float(p("window_autolower_min", 500.0).value),
        )
        # Azimuth-band handling: when band_enable, the known-noisy beams (within
        # band_half_beams of each band_azimuths center) are HARD-REJECTED in detection so
        # they emit no edge -- i.e. just blank the noisy beams. Off by default (the
        # intensity correction below normally flattens them instead of blanking them).
        self.band_enable = bool(p("band_enable", False).value)
        self.band_half_beams = int(p("band_half_beams", 17).value)
        self.band_azimuths = np.deg2rad(
            np.asarray(p("band_azimuths_deg", [-23.0, 0.0, 23.0]).value, dtype=np.float64))
        # Multi-edge experiment (diagnostic, drawn on sonar_image.png): the 2nd edge
        # is the next return whose start falls in the zone [1st edge, 1st edge +
        # edge_zone_px]; the 3rd edge is the peak within that 2nd return. Does NOT
        # touch the published cloud. Needs visualize: true to be drawn.
        self.multi_edge_enable = bool(p("multi_edge_enable", False).value)
        self.edge_zone_px = int(p("edge_zone_px", 30).value)  # zone depth (dp) past the 1st edge
        # Shape extraction (paper eqs 39-42, Alg. 1 line 1): isolate solid bright
        # OBJECTS with a top-hat -> binarise -> open/close morphology, anchor them
        # with AKAZE keypoints, associate each to the leading edge, and publish the
        # six per-point shape channels (m_R, eq 40) as extra float32 cloud fields.
        # This REPLACES the old connected-component region growing: the region R is
        # the masked solid-object BLOB the leading edge crosses, not a grow from thr.
        # OFF by default so the published cloud stays byte-identical to the
        # historical 4-field layout unless the eq-24 pipeline asks for shape.
        self.shape_enable = bool(p("shape_enable", False).value)
        # Bright-blob detection on the denoised image: top-hat -> threshold ->
        # connected components -> AREA + local-CONTRAST filter -> edge association.
        # Targets patches that are very bright vs their surroundings (algae/plants).
        self.shape_bg_kernel = int(p("shape_bg_kernel", 25).value)        # top-hat opening (px)
        self.shape_bin_thresh = float(p("shape_bin_thresh", 0.0).value)   # <=0 => Otsu on tophat
        self.shape_open_kernel = int(p("shape_open_kernel", 3).value)     # mask despeckle (px); 0=off
        self.shape_close_kernel = int(p("shape_close_kernel", 3).value)   # mask hole-fill (px); 0=off
        self.shape_min_area_px = int(p("shape_min_area_px", 20).value)    # drop tiny blobs
        self.shape_max_area_px = int(p("shape_max_area_px", 0).value)     # drop huge blobs (0=off)
        self.shape_min_contrast = float(p("shape_min_contrast", 0.15).value)  # bright-vs-ring knob
        self.shape_ring_kernel = int(p("shape_ring_kernel", 9).value)     # contrast ring width (px)
        self.shape_edge_tol_bins = int(p("shape_edge_tol_bins", 5).value)  # edge<->blob range tol
        self.shape_shadow_max_bins = int(p("shape_shadow_max_bins", 40).value)
        self.shape_shadow_dark_frac = float(p("shape_shadow_dark_frac", 0.35).value)
        # Shape-factor DEBUG overlay: every N frames, save <folder>/shape_factor.png
        # — the Cartesian sonar fan (like sonar_image.png) with each grown region
        # outlined and LABELLED by the eq-42 shape factor's argmax class (loaded
        # from sonar_map's classes.yaml). Shows what the SHAPE cue alone believes,
        # in isolation from the other three factors and the map's spatial prior
        # (the full fused label lives downstream in voxel_mapper, where the image
        # is gone). Off by default; needs sonar_map importable at runtime.
        self.shape_factor_viz = bool(p("shape_factor_viz", False).value)
        self.shape_factor_every_n = max(1, int(p("shape_factor_every_n", 10).value))
        self.shape_factor_classes_file = str(p("shape_factor_classes_file", "").value)
        # Background of shape_factor.png: 'tophat' (the contrast map), 'mask' (the
        # binarised object mask), or 'raw' (the sonar fan). Use tophat/mask to SEE
        # the effect of the preprocessing while tuning.
        self.shape_factor_bg = str(p("shape_factor_bg", "tophat").value).lower()
        self._shape_viz_count = 0
        self._shape_cls = None       # cached (ClassSet, factor_shape) or False on failure
        self.min_range = float(p("min_range", 2.0).value)
        self.max_range = float(p("max_range", 20.0).value)
        if self.min_range >= self.max_range:
            self.get_logger().error(
                f"min_range ({self.min_range:g}) >= max_range ({self.max_range:g}): "
                "the range gate is empty, no edges will be detected. Check the YAML.")
        self.min_bearing = np.deg2rad(float(p("min_bearing_deg", -45.0).value))
        self.max_bearing = np.deg2rad(float(p("max_bearing_deg", 45.0).value))
        self.min_consecutive = int(p("min_consecutive", 3).value)
        # orientation flips (fix near/far or port/starboard inversion)
        self.flip_ranges = bool(p("flip_ranges", False).value)   # reverse range axis (rows)
        self.flip_beams = bool(p("flip_beams", False).value)     # mirror beams L<->R (y sign)
        self._frame = 0
        # debug: every frame, save the B-scan + leading edge to sonar_image.png
        self.visualize = bool(p("visualize", False).value)
        # Throttle the PNG write: save 1 of every N frames (it runs in the data
        # callback and costs ~6-16 ms, so writing every frame caps the rate).
        self.visualize_every_n = max(1, int(p("visualize_every_n", 1).value))
        self._viz_count = 0
        self.visualize_folder = str(
            p("visualize_folder", "/home/rosdev/ros2_ws/src/images/blueview").value)
        # Published-intensity source + per-beam correction. The denoise center-beam gain
        # mask amplifies the noisy-beam artifact (raw ~2.6x -> denoised ~7x), so publish
        # RAW by default. intensity_correction_map = path to calibration_map.npz from
        # train_intensity_correction.py: multiplies each point's intensity by the learned
        # per-(beam,range) factor to flatten the noisy beams. "" = off.
        self.intensity_source = str(p("intensity_source", "raw").value).lower()
        # detect_on_corrected: run the (adaptive) detection on the NN-corrected image
        # (raw * per-beam factor) instead of the subtraction-denoise. corrected_smooth_sigma
        # = light range smoothing of that corrected image for detection (0 = off).
        self.detect_on_corrected = bool(p("detect_on_corrected", False).value)
        self.corrected_smooth_sigma = float(p("corrected_smooth_sigma", 2.0).value)
        self._corr_img = None        # per-pixel factor image (n_range, n_beam)
        corr_path = str(p("intensity_correction_map", "").value)
        if corr_path:
            try:
                d = np.load(corr_path)
                nrs, nbs = int(d["n_rsec"]), int(d["n_bsec"])
                nR, nB = int(d["n_range"]), int(d["n_beam"])
                grid = np.ones((nrs, nbs), dtype=np.float32)
                grid[d["ri"].astype(int), d["bi"].astype(int)] = d["factor"].astype(np.float32)
                # Expand per-section factor to a per-pixel image, using the SAME section
                # edges the collector used (linspace over range bins / beams).
                re = np.linspace(0, nR, nrs + 1).astype(int)
                be = np.linspace(0, nB, nbs + 1).astype(int)
                ri_full = np.clip(np.searchsorted(re, np.arange(nR), "right") - 1, 0, nrs - 1)
                bi_full = np.clip(np.searchsorted(be, np.arange(nB), "right") - 1, 0, nbs - 1)
                self._corr_img = grid[np.ix_(ri_full, bi_full)].astype(np.float32)  # (nR, nB)
                self.get_logger().info(
                    f"intensity correction ON: {corr_path} ({nrs}x{nbs} sections -> "
                    f"{nR}x{nB} factor image), detect_on_corrected={self.detect_on_corrected}")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"failed to load intensity_correction_map: {exc!r}")
        # Intensity-section grid baked into the fan: label each polar section with the
        # RAW mean/median intensity (band cells yellow) to debug per-beam intensity scale.
        self.intensity_grid = bool(p("intensity_grid", False).value)
        # What sonar_image.png paints ON TOP of the sweep. All default TRUE so
        # existing setups are unchanged; set them false for a bare image.
        self.viz_leading_edge = bool(p("viz_leading_edge", True).value)
        self.viz_labels = bool(p("viz_labels", True).value)
        self.intensity_grid_beams = int(p("intensity_grid_beams", 9).value)
        self.intensity_grid_ranges = int(p("intensity_grid_ranges", 5).value)
        self._viz_path = os.path.join(self.visualize_folder, "sonar_image.png")
        if self.visualize:
            os.makedirs(self.visualize_folder, exist_ok=True)
            self.get_logger().info(f"visualize ON -> {self._viz_path} (B-scan + leading edge)")
        self._shape_factor_path = os.path.join(self.visualize_folder, "shape_factor.png")
        if self.shape_factor_viz:
            os.makedirs(self.visualize_folder, exist_ok=True)
            self.get_logger().info(
                f"shape_factor_viz ON -> {self._shape_factor_path} "
                f"(fan + shape-factor-labelled regions, every "
                f"{self.shape_factor_every_n} frames)")
            if not self.shape_enable:
                self.get_logger().warn(
                    "shape_factor_viz is ON but shape_enable is OFF — regions are "
                    "extracted for the overlay but NOT published on the cloud.")
        # debug_processing: every N frames, save a multi-panel matplotlib figure
        # showing each processing stage (raw -> denoised -> above floor -> above
        # threshold -> gated occupancy -> leading edge on the raw image).
        self.debug_processing = bool(p("debug_processing", False).value)
        self.debug_processing_every_n = max(1, int(p("debug_processing_every_n", 20).value))
        self._dbg_count = 0
        self._dbg_path = os.path.join(self.visualize_folder, "processing_steps.png")
        if self.debug_processing:
            os.makedirs(self.visualize_folder, exist_ok=True)
            self.get_logger().info(
                f"debug_processing ON -> {self._dbg_path} "
                f"(every {self.debug_processing_every_n} frames)")

        reliability = (
            ReliabilityPolicy.RELIABLE if qos_rel == "reliable" else ReliabilityPolicy.BEST_EFFORT
        )
        qos = QoSProfile(depth=10, reliability=reliability)

        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.sub = self.create_subscription(ProjectedSonarImage, self.input_topic, self.on_image, qos)

        self.get_logger().info(
            f"leading_edge_node: {self.input_topic} -> {self.output_topic} "
            f"[detector={self.detector}"
            + (f"(k={self.adaptive_k:g}, floor_pct={self.floor_percentile:g})"
               if self.detector == "adaptive" else
               f"(thr={self.threshold:g}, window -{self._window_cfg['low_bins']}"
               f"/+{self._window_cfg['high_bins']} bins, "
               f"lambda={self._window_cfg['smooth_weight']:g}, "
               f"peak_required={self._window_cfg['require_peak']}"
               + (f", per-beam autolower -{self._window_cfg['autolower_step']:g}"
                  f" -> {self._window_cfg['autolower_min']:g}"
                  if self._window_cfg["autolower"] else ", no autolower") + ")"
               if self.detector == "window" else f"(thr={self.threshold:g})")
            + (f", temporal-avg({self.temporal_avg_frames} frames)"
               if self.temporal_avg_frames > 1 else "")
            + (f", band-reject(+-{self.band_half_beams}b @ {np.rad2deg(self.band_azimuths)} deg)"
               if self.band_enable else "")
            + (f", shape(bright-blobs bg={self.shape_bg_kernel}, "
               f"area>={self.shape_min_area_px}, contrast>={self.shape_min_contrast:g})"
               if self.shape_enable else "")
            + f", range {self.min_range:g}-{self.max_range:g} m, "
            f"bearing {np.rad2deg(self.min_bearing):g}..{np.rad2deg(self.max_bearing):g} deg, "
            f"min_consec={self.min_consecutive}, "
            f"flip(ranges={self.flip_ranges}, beams={self.flip_beams}), "
            f"denoise(standard={self.denoise_standard}, open={self.denoise_open}, "
            f"destripe={self.denoise_destripe}), qos={reliability.name}]"
        )

    def _build_points(self, image, ranges, bearings, edge_range, valid, flip_beams=False):
        """(edge_range, valid) per beam -> (xyz Nx3 f32, intensity N f32).
        Intensity is sampled from `image` at the (tracked) edge range bin (caller passes
        raw or denoised per `intensity_source`), then optionally divided by the learned
        per-beam gain (intensity_correction_map) to remove the noisy-beam scale artifact.
        `flip_beams` mirrors the cloud left<->right (the y sign)."""
        keep = np.where(valid)[0]
        if keep.size == 0:
            return np.empty((0, 3), np.float32), np.empty((0,), np.float32)
        n_range = ranges.shape[0]
        step = (ranges[-1] - ranges[0]) / (n_range - 1) if n_range > 1 else 1.0
        r = edge_range[keep]
        b = bearings[keep]
        idx = np.clip(np.round((r - ranges[0]) / step).astype(int), 0, n_range - 1)
        intensity = image[idx, keep].astype(np.float32)
        # Per-beam intensity correction: multiply by the learned per-pixel factor.
        # Removes the ~2x noisy-beam gain so the map isn't dominated by bright streaks.
        if self._corr_img is not None and image.shape == self._corr_img.shape:
            intensity = intensity * self._corr_img[idx, keep]
        # Default y = -r*sin(bearing) matches the old pipeline
        # (edge_detection.image2Profile); flip_beams uses +sin to mirror L<->R.
        y_sign = 1.0 if flip_beams else -1.0
        xyz = np.empty((keep.size, 3), np.float32)
        xyz[:, 0] = r * np.cos(b)
        xyz[:, 1] = y_sign * r * np.sin(b)
        xyz[:, 2] = 0.0
        return xyz, intensity

    def _log_spectrum(self, image, thr, n_pts):
        """Every ~20 frames, log the detection image's intensity spectrum + the
        threshold actually used, so you can SEE how big the intensities are and tune
        `adaptive_k` (adaptive) or `threshold` (fixed/window) against real numbers."""
        self._frame += 1
        if self._frame % 20 != 0:
            return
        p50, p90, p99 = np.percentile(image, (50, 90, 99))
        # Window mode lowers the threshold per beam, so report the spread and how
        # many beams needed it -- that is the coverage knob's live feedback.
        n_low = int((thr < self.threshold).sum())
        spread = (f" (min={thr.min():.0f}, {n_low}/{thr.size} beams lowered)"
                  if n_low else "")
        self.get_logger().info(
            f"intensity p50={p50:.0f} p90={p90:.0f} p99={p99:.0f} max={image.max():.0f} | "
            f"thr median={np.median(thr):.0f}{spread} | "
            f"points={n_pts}/{thr.size} beams ({100.0 * n_pts / max(1, thr.size):.0f}%)"
        )

    def _ensure_fan_lut(self, ranges, bearings):
        """Build (and cache) the polar->cartesian fan scan-conversion LUT, matching
        the Foxglove sonar fan script: forward (z) maps to UP, lateral to ACROSS,
        apex (near) at bottom-centre. The LUT is geometry-only, so it is rebuilt
        only when (n_range, n_beam, first/last bearing) changes."""
        n_range = int(ranges.shape[0])
        n_beam = int(bearings.shape[0])
        first = float(bearings[0])
        last = float(bearings[-1])
        key = (n_range, n_beam, first, last)
        if getattr(self, "_fan_key", None) == key:
            return
        height = n_range
        max_y = float(np.max(np.abs(np.sin(bearings))))     # half-FOV lateral extent
        half = int(np.ceil(max_y * height))
        width = 2 * half
        angle_step = (last - first) / (n_beam - 1) if n_beam > 1 else 1.0
        abs_step = abs(angle_step)
        angle_tol = abs_step * 0.75
        range_scale = (n_range - 1) / height

        py = np.arange(height, dtype=np.float32)[:, None]
        px = np.arange(width, dtype=np.float32)[None, :]
        dz = (height - 1 - py)                               # forward distance (up)
        dy = (px - half)                                     # lateral offset
        dist = np.sqrt(dy * dy + dz * dz)
        inside = dist <= height
        pixel_angle = np.arctan2(-dy, dz)                    # -dy: same mirror as the fan script
        b_idx = (pixel_angle - first) / angle_step
        best_beam = np.round(b_idx)
        beam_ok = (best_beam >= 0) & (best_beam < n_beam)
        beam_ok &= (np.abs(b_idx - best_beam) * abs_step) <= angle_tol
        range_idx = np.round(dist * range_scale)
        range_ok = (range_idx >= 0) & (range_idx < n_range)

        self._fan_valid = inside & beam_ok & range_ok
        self._fan_ri = np.clip(range_idx, 0, n_range - 1).astype(np.int32)
        self._fan_bb = np.clip(best_beam, 0, n_beam - 1).astype(np.int32)
        self._fan_half = half
        self._fan_w = width
        self._fan_h = height
        self._fan_key = key
        self.get_logger().info(f"fan LUT built: {width}x{height} (half={half})")

    def _save_debug_image(self, image, ranges, bearings, edge_range, valid,
                          multi_edge=None):
        """Write <folder>/sonar_image.png: the polar image scan-converted into the
        Cartesian FAN (apex/near at bottom-centre, far at top) auto-scaled to 8-bit,
        with the per-beam leading edge drawn in red on top and the intensity-section
        grid overlaid. Faint red wedges mark the noisy azimuth bands when band_enable.

        multi_edge (optional): (edge2_rows, edge3_rows, good) from
        detect_secondary_edges — draws the 2nd leading edge in GREEN and the 3rd
        (good beams only) in CYAN, so the multi-edge experiment is visible."""
        self._ensure_fan_lut(ranges, bearings)
        H, W, half = self._fan_h, self._fan_w, self._fan_half

        fan = np.where(self._fan_valid, image[self._fan_ri, self._fan_bb], 0.0)
        mx = max(float(fan.max()), 1.0)
        img8 = np.clip(fan * (255.0 / mx), 0, 255).astype(np.uint8)
        bgr = cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)

        # Overlay the leading edge: (range, bearing) -> fan pixel (same convention).
        r0 = float(ranges[0])
        span = float(ranges[-1]) - r0

        def _fan_xy(r, b):                                    # (range m, bearing rad) -> px
            pr = (r - r0) / span * H
            return int(round(half - pr * np.sin(b))), int(round((H - 1) - pr * np.cos(b)))

        # Faint red wedges = the noisy-beam bands (band_azimuths +- band_half_beams),
        # shown only when band-reject is enabled, so you can SEE which beams are blanked.
        if self.band_enable and span != 0.0:
            bm = band_beam_mask(bearings, self.band_azimuths, self.band_half_beams)
            if bm.any():
                pad = np.concatenate(([0], bm.astype(np.int8), [0]))
                bnd = np.flatnonzero(np.diff(pad))
                ov = bgr.copy()
                for s, e in zip(bnd[0::2].tolist(), bnd[1::2].tolist()):
                    be = bearings[s:e]
                    poly = np.array([_fan_xy(ranges[0], b) for b in be]
                                    + [_fan_xy(ranges[-1], b) for b in be[::-1]], dtype=np.int32)
                    cv2.fillPoly(ov, [poly], (0, 0, 255))
                cv2.addWeighted(ov, 0.22, bgr, 0.78, 0, dst=bgr)

        cols = np.where(valid)[0] if self.viz_leading_edge else np.empty(0, int)
        if cols.size and span != 0.0:
            pr = (edge_range[cols] - r0) / span * H          # metric range -> pixel radius
            bear = bearings[cols]
            cx = np.round(half - pr * np.sin(bear)).astype(int)
            cy = np.round((H - 1) - pr * np.cos(bear)).astype(int)
            for x, y in zip(cx.tolist(), cy.tolist()):
                if 0 <= x < W and 0 <= y < H:
                    cv2.circle(bgr, (x, y), 2, (0, 0, 255), -1)   # BGR red

        # ── multi-edge experiment: 2nd edge (green) + 3rd edge (cyan, good beams) ─
        if multi_edge is not None and span != 0.0 and self.viz_leading_edge:
            edge2_rows, edge3_rows, good = multi_edge

            def _draw_rows(rows, color, rad=2):
                bs = np.flatnonzero(np.asarray(rows) >= 0)
                for b in bs.tolist():
                    rw = int(rows[b])
                    prr = (ranges[rw] - r0) / span * H
                    x = int(round(half - prr * np.sin(bearings[b])))
                    y = int(round((H - 1) - prr * np.cos(bearings[b])))
                    if 0 <= x < W and 0 <= y < H:
                        cv2.circle(bgr, (x, y), rad, color, -1)

            # Only GOOD beams (dark gap between returns <= edge_gap_px) are drawn,
            # so the display matches the selection rule.
            e2g = np.where(np.asarray(good), np.asarray(edge2_rows), -1)
            _draw_rows(e2g, (0, 255, 0))                      # 2nd edge — green (good)
            e3 = np.where(np.asarray(good), np.asarray(edge3_rows), -1)
            _draw_rows(e3, (255, 255, 0))                     # 3rd edge — cyan (good)
            n_good = int(np.count_nonzero(good))
            for i, (txt, col) in enumerate(
                    [("1st edge", (0, 0, 255)), ("2nd edge", (0, 255, 0)),
                     (f"3rd edge (good {n_good})", (255, 255, 0))]):
                cv2.putText(bgr, txt, (4, 34 + 14 * i),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)

        # Intensity-section grid (raw mean/median per polar section; band cells yellow).
        if self.intensity_grid and span != 0.0:
            # Always mark the band beams in the grid (for the yellow/blue overlay), even
            # when band DETECTION handling is off -- they're still the azimuths to watch.
            bm_grid = band_beam_mask(bearings, self.band_azimuths, self.band_half_beams)
            corr = self._corr_img if (self._corr_img is not None
                                      and self._corr_img.shape == image.shape) else None
            draw_intensity_grid(bgr, image, ranges, bearings, _fan_xy,
                                n_beam=self.intensity_grid_beams,
                                n_range=self.intensity_grid_ranges, band_mask=bm_grid,
                                corr_img=corr)

        if self.viz_labels:
            cv2.putText(bgr, f"FAR {ranges[-1]:.1f}m", (4, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(bgr, f"NEAR {ranges[0]:.1f}m", (half - 30, H - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imwrite(self._viz_path, bgr)

    # ── shape-factor debug overlay ────────────────────────────────────────────

    def _ensure_shape_classifier(self):
        """Lazily import sonar_map's eq-42 shape factor + class set (cached).

        Returns (ClassSet, factor_shape_module) or None. The dependency is
        DEBUG-only and reaches "up" into the sonar_map package, so the import is
        deferred to the first labelled frame and its failure is non-fatal: the
        overlay then draws regions unlabelled instead of taking the node down.
        """
        if self._shape_cls is not None:
            return self._shape_cls or None       # False (cached failure) -> None
        try:
            from sonar_map.dirichlet.classes import load_classes
            from sonar_map.dirichlet import factor_shape
            path = self.shape_factor_classes_file.strip()
            if not path:
                from ament_index_python.packages import get_package_share_directory
                import os as _os
                path = _os.path.join(
                    get_package_share_directory("sonar_map"), "config", "classes.yaml")
            cs = load_classes(path)
            self._shape_cls = (cs, factor_shape)
            self.get_logger().info(
                f"shape_factor.png: labelling regions by the eq-42 shape factor "
                f"over classes {list(cs.names)} ({path})")
        except Exception as exc:  # noqa: BLE001 — debug feature must not kill the node
            self._shape_cls = False
            self.get_logger().warn(
                f"shape_factor.png: sonar_map class set unavailable ({exc!r}); "
                "drawing regions WITHOUT class labels.")
        return self._shape_cls or None

    def _classify_regions(self, regions):
        """eq-42 shape-factor argmax class per region.

        Returns a list of (class_index, class_name, prob) — prob is the softmax
        of the shape log-likelihood over classes (i.e. p(class | shape) under a
        uniform prior). Returns None when no classifier is available, so the
        caller falls back to unlabelled regions.
        """
        got = self._ensure_shape_classifier()
        if got is None or not regions:
            return None
        cs, factor_shape = got
        n = len(regions)
        flag = np.ones(n, dtype=np.float64)
        size = np.array([r["n_r"] for r in regions], dtype=np.float64)
        logarea = np.array([r["logarea"] for r in regions], dtype=np.float64)
        solidity = np.array([r["solidity"] for r in regions], dtype=np.float64)
        shadow = np.array([r["shadow"] for r in regions], dtype=np.float64)
        texture = np.array([r["texture"] for r in regions], dtype=np.float64)
        # Same call the pipeline makes — the label IS the shape factor's opinion.
        log_L = factor_shape.log_lik(flag, size, logarea, solidity, shadow,
                                     texture, cs)                     # (n, K+1)
        idx = np.argmax(log_L, axis=1)
        z = log_L - log_L.max(axis=1, keepdims=True)
        p = np.exp(z); p /= p.sum(axis=1, keepdims=True)
        return [(int(idx[i]), cs.names[int(idx[i])], float(p[i, idx[i]]))
                for i in range(n)]

    def _region_color(self, cs_names, class_idx):
        """BGR colour for a region: palette by index, grey for 'other'."""
        if cs_names is not None and cs_names[class_idx] == "other":
            return _SHAPE_OTHER_BGR
        return _SHAPE_CLASS_BGR[class_idx % len(_SHAPE_CLASS_BGR)]

    def _save_shape_factor_image(self, debug, image, ranges, bearings, edge_range, valid):
        """Write <folder>/shape_factor.png: the debug view of the SHAPE cue alone
        (paper eqs 39-42), built from the compute_region_channels debug bundle.

        The fan renders the PROCESSED image chosen by shape_factor_bg ('tophat' =
        the contrast map, 'mask' = the binarised object mask, 'raw' = the sonar
        fan) so you can SEE the effect of the preprocessing. On top:
          * the bright blobs the leading edge CROSSED (the regions) — hull outline
            in their eq-42 class colour + label + descriptors,
          * candidate bright blobs the edge did NOT cross — dim grey hulls, so you
            can see what the detector found vs what the edge-crossing rule kept.
        The leading edge is NOT drawn here. Reuses _classify_regions /
        _region_color (the eq-42 shape factor's opinion IN ISOLATION).

        `debug` is {'contrast_map','binmask','blobs','regions'} or None/empty on a
        degenerate frame; in the latter case we fall back to the raw fan so the
        overlay never crashes the node.
        """
        self._ensure_fan_lut(ranges, bearings)
        H, W, half = self._fan_h, self._fan_w, self._fan_half
        n_range = int(ranges.shape[0])
        n_beam = int(bearings.shape[0])

        # Fan background = the chosen PROCESSED image, so you can SEE the effect of
        # the preprocessing (shape_factor_bg: 'tophat' = the contrast map, 'mask' =
        # the binarised object mask, 'raw' = the sonar fan).
        bg = str(self.shape_factor_bg)
        src = None
        if debug is not None:
            if bg == "tophat":
                src = debug.get("contrast_map")
            elif bg == "mask":
                bmk = debug.get("binmask")
                src = bmk.astype(np.float32) * 255.0 if bmk is not None else None
        fan_src = src if src is not None else image      # 'raw' (or fallback)
        fan = np.where(self._fan_valid, fan_src[self._fan_ri, self._fan_bb], 0.0)
        mx = max(float(fan.max()), 1.0)
        img8 = np.clip(fan * (255.0 / mx), 0, 255).astype(np.uint8)
        bgr = cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)
        cv2.putText(bgr, f"bg: {bg}", (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 255), 1, cv2.LINE_AA)

        r0 = float(ranges[0])
        span = float(ranges[-1]) - r0
        f = cv2.FONT_HERSHEY_SIMPLEX
        if span == 0.0:
            cv2.imwrite(self._shape_factor_path, bgr)
            return

        def _fan_xy(r, b):
            """(range m, bearing rad) -> (x, y) fan pixel, same map as the edge."""
            pr = (r - r0) / span * H
            return int(round(half - pr * np.sin(b))), int(round((H - 1) - pr * np.cos(b)))

        def _fan_pts(rows, cols):
            """(image rows, cols) -> (M, 2) int fan pixels (blob body / seed anchor)."""
            pr = (ranges[np.asarray(rows)] - r0) / span * H
            b = bearings[np.asarray(cols)]
            fx = np.round(half - pr * np.sin(b)).astype(int)
            fy = np.round((H - 1) - pr * np.cos(b)).astype(int)
            return np.stack([fx, fy], axis=1)

        def _kp_fan(row, col):
            """AKAZE keypoint image (row, col) float -> fan pixel, or None if off-fan."""
            ri = int(np.clip(round(float(row)), 0, n_range - 1))
            ci = int(np.clip(round(float(col)), 0, n_beam - 1))
            x, y = _fan_xy(ranges[ri], bearings[ci])
            return (x, y) if (0 <= x < W and 0 <= y < H) else None

        # (Leading edge intentionally NOT drawn here — this view is for seeing the
        #  preprocessing + the blobs, not the edge.)
        blobs = debug.get("blobs", []) if debug else []
        regions = debug.get("regions", []) if debug else []

        # eq-42 argmax class per region (None -> unlabelled); the hull colours
        # below read it, so each region is drawn in its class colour.
        labels = self._classify_regions(regions)
        cs_names = self._shape_cls[0].names if self._shape_cls else None

        # ── candidate bright blobs NOT crossed by the leading edge ────────────
        # Detected (bright vs surroundings, passed area + contrast) but the leading
        # edge did not enter them, so they are NOT regions — drawn as a dim grey
        # hull so you can SEE what the detector found vs what the edge-crossing
        # rule actually kept.
        for bl in blobs:
            if bl.get("crossed"):
                continue
            pix = _fan_pts(bl["ys"], bl["xs"])
            inb = ((pix[:, 0] >= 0) & (pix[:, 0] < W)
                   & (pix[:, 1] >= 0) & (pix[:, 1] < H))
            pix = pix[inb]
            if pix.shape[0] >= 3:
                hull = cv2.convexHull(pix.reshape(-1, 1, 2))
                cv2.polylines(bgr, [hull], True, (90, 90, 90), 1, cv2.LINE_AA)

        # ── regions: blob-hull outline + eq-42 label + descriptor line ────────
        for i, reg in enumerate(regions):
            if labels is not None:
                cidx, cname, prob = labels[i]
                color = self._region_color(cs_names, cidx)
                text = f"{cname} {prob:.0%}"
            else:
                color = _SHAPE_UNLABELLED_BGR
                text = f"R{i} (Nr={reg['n_r']})"

            pix = _fan_pts(reg["ys"], reg["xs"])                    # blob body
            inb = ((pix[:, 0] >= 0) & (pix[:, 0] < W)
                   & (pix[:, 1] >= 0) & (pix[:, 1] < H))
            pix = pix[inb]
            if pix.shape[0] == 0:
                continue
            if pix.shape[0] >= 3:
                hull = cv2.convexHull(pix.reshape(-1, 1, 2))
                cv2.polylines(bgr, [hull], True, color, 2, cv2.LINE_AA)

            # Label at the leading-edge member centroid (the region's front).
            seed = _fan_pts(reg["edge_rows"], reg["member_cols"])
            tx, ty = int(np.mean(seed[:, 0])), int(np.mean(seed[:, 1]))
            (tw, th), _ = cv2.getTextSize(text, f, 0.5, 1)
            tx = int(np.clip(tx - tw // 2, 2, W - tw - 2))
            ty = int(np.clip(ty, th + 2, H - 22))
            cv2.rectangle(bgr, (tx - 2, ty - th - 2), (tx + tw + 2, ty + 2),
                          (0, 0, 0), -1)
            cv2.putText(bgr, text, (tx, ty), f, 0.5, color, 1, cv2.LINE_AA)
            # Descriptor line (a=log-area, s=solidity, sh=shadow, t=texture).
            sub = (f"a{reg['logarea']:+.1f} s{reg['solidity']:.2f} "
                   f"sh{reg['shadow']:.2f} t{reg['texture']:.1f}")
            cv2.putText(bgr, sub, (tx, ty + 13), f, 0.36, color, 1, cv2.LINE_AA)

        # ── legend (class -> colour) + region/keypoint count + NEAR/FAR ───────
        y = 30
        if cs_names is not None:
            for ci, nm in enumerate(cs_names):
                col = self._region_color(cs_names, ci)
                cv2.rectangle(bgr, (6, y - 9), (18, y + 2), col, -1)
                cv2.putText(bgr, nm, (22, y), f, 0.42, col, 1, cv2.LINE_AA)
                y += 16
        cv2.putText(bgr, f"regions: {len(regions)}  candidates: {len(blobs)}",
                    (6, y + 2), f, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(bgr, f"FAR {ranges[-1]:.1f}m", (4, 16),
                    f, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(bgr, f"NEAR {ranges[0]:.1f}m", (half - 30, H - 8),
                    f, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imwrite(self._shape_factor_path, bgr)

    def _save_processing_debug(self, raw, det, ranges, dbg, edge_range, valid):
        """Save a 2x3 matplotlib figure walking through every processing stage on
        the sonar image, from raw to the leading edge drawn on the raw image:
          1. raw B-scan
          2. denoised (filter_horizontal_image)
          3. above per-beam floor (percentile, pre k*sigma)
          4. above per-beam threshold (floor + k*sigma), pre gate/min_consecutive
          5. final occupancy: threshold AND range gate AND min_consecutive runs
          6. leading edge (red) on the raw image
        Dashed cyan lines mark the [min_range, max_range] gate. Imported lazily so
        matplotlib never touches the fast path. Runs only every N frames."""
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        # y axis in metres, NEAR (ranges[0]) at top to match the B-scan convention.
        extent = [0, raw.shape[1], float(ranges[-1]), float(ranges[0])]

        def show(ax, img, title, cmap="viridis", binary=False):
            ax.imshow(img, aspect="auto", cmap=("gray" if binary else cmap),
                      extent=extent, interpolation="nearest")
            ax.set_title(title, fontsize=10)
            ax.set_xlabel("beam"); ax.set_ylabel("range [m]")
            ax.axhline(self.min_range, color="cyan", lw=0.7, ls="--")
            ax.axhline(self.max_range, color="cyan", lw=0.7, ls="--")

        fig, ax = plt.subplots(2, 3, figsize=(16, 9))
        show(ax[0, 0], raw, "1. Raw sonar image")
        show(ax[0, 1], det, "2. Denoised")
        if self.detector == "adaptive":
            t3 = f"3. Above floor (p{self.floor_percentile:g})"
            t4 = f"4. Above threshold (floor + {self.adaptive_k:g}·σ)"
        else:
            t3 = t4 = f"3./4. Above threshold t={self.threshold:g}"
        show(ax[0, 2], dbg["occ_floor"], t3, binary=True)
        show(ax[1, 0], dbg["occ_thr"], t4, binary=True)
        show(ax[1, 1], dbg["occ_final"],
             f"5. + range gate + min_consec={self.min_consecutive}", binary=True)
        show(ax[1, 2], raw, "6. Leading edge on raw")
        cols = np.where(valid)[0]
        if cols.size:
            # window mode: show the threshold-crossing SEED too, so the window
            # offsets can be read straight off the gap between the two curves.
            has_seed = "seed" in dbg
            if has_seed:
                ax[1, 2].scatter(cols, ranges[dbg["seed"][cols]], s=3, c="deepskyblue",
                                 marker=".", label="seed (threshold crossing)")
            ax[1, 2].scatter(cols, edge_range[cols], s=3, c="red", marker=".",
                             label="published edge")
            if has_seed:
                ax[1, 2].legend(loc="upper right", fontsize=7, markerscale=3)

        if self.detector == "adaptive":
            det_mode = f"adaptive k={self.adaptive_k:g}"
        elif self.detector == "window":
            w = self._window_cfg
            det_mode = (f"window thr={self.threshold:g}, -{w['low_bins']}/+{w['high_bins']} "
                        f"bins, λ={w['smooth_weight']:g}")
        else:
            det_mode = f"fixed thr={self.threshold:g}"
        fig.suptitle(f"leading-edge processing — frame {self._dbg_count} | {det_mode} | "
                     f"points={int(valid.sum())}", fontsize=12)
        fig.tight_layout(rect=[0, 0, 1, 0.97])
        fig.savefig(self._dbg_path, dpi=100)
        plt.close(fig)

    def on_image(self, msg: ProjectedSonarImage):
        # One bad frame must not stop the subscription.
        try:
            raw, ranges, bearings = sonar_image_from_msg(msg, flip_ranges=self.flip_ranges)
            # TEMPORAL denoise: mean of the last `temporal_avg_frames` sonar images.
            # A shape change (new range/beam count, or a different sonar) restarts the
            # buffer rather than averaging incompatible frames; the first frames after
            # a restart are simply averaged over fewer images.
            avg = raw
            if self.temporal_avg_frames > 1:
                if self._frame_buf and self._frame_buf[-1].shape != raw.shape:
                    self._frame_buf.clear()
                self._frame_buf.append(raw)
                if len(self._frame_buf) > 1:
                    avg = np.mean(np.stack(self._frame_buf, axis=0), axis=0)
            # Detection image. detect_on_corrected: use the NN-corrected image
            # (averaged * per-beam factor) -> the noisy beams are flattened at the source,
            # no subtraction-denoise to amplify them. Else the subtraction denoise.
            if (self.detect_on_corrected and self._corr_img is not None
                    and avg.shape == self._corr_img.shape):
                det = avg * self._corr_img
                if self.corrected_smooth_sigma > 0:
                    det = gaussian_filter1d(det, sigma=self.corrected_smooth_sigma, axis=0)
            else:
                det = denoise_image(
                    avg, self.denoise_standard, self.denoise_open, self.denoise_destripe,
                )
            # Decide up front whether this frame gets the step-by-step debug figure,
            # so detect_edges can hand back its per-stage intermediates in one pass.
            want_dbg = False
            if self.debug_processing:
                self._dbg_count += 1
                want_dbg = self._dbg_count % self.debug_processing_every_n == 0
            # band_enable -> hard-reject the known-noisy beams (they emit no edge).
            band_mask = (band_beam_mask(bearings, self.band_azimuths, self.band_half_beams)
                         if self.band_enable else None)
            result = detect_edges(
                det, ranges, bearings,
                mode=self.detector, fixed_threshold=self.threshold,
                adaptive_k=self.adaptive_k, floor_percentile=self.floor_percentile,
                min_range=self.min_range, max_range=self.max_range,
                min_bearing=self.min_bearing, max_bearing=self.max_bearing,
                min_consecutive=self.min_consecutive, band_mask=band_mask,
                autolower=self.fixed_autolower, min_coverage=self.fixed_min_coverage,
                threshold_step=self.fixed_threshold_step, threshold_min=self.fixed_threshold_min,
                window=self._window_cfg, collect_debug=want_dbg,
            )
            dbg = None
            if want_dbg:
                edge_range, valid, thr, dbg = result
            else:
                edge_range, valid, thr = result

            # Published intensity comes from RAW by default (the denoise center-beam gain
            # mask amplifies the noisy-beam artifact ~2.6x->7x); the correction map is
            # raw-derived, so raw + correction is consistent. "averaged" keeps that scale
            # but takes the temporally-averaged image (identical to raw when averaging
            # is off); "denoised" samples the detection image.
            intensity_img = (raw if self.intensity_source == "raw" else
                             avg if self.intensity_source == "averaged" else det)
            xyz, intensity = self._build_points(
                intensity_img, ranges, bearings, edge_range, valid, self.flip_beams)
            # Shape channels (paper eqs 39-42, Alg. 1 line 1): extract solid bright
            # blobs (top-hat -> binarise -> open/close), anchor them with AKAZE
            # keypoints, associate each blob to the leading edge, and attach the
            # per-beam m_R descriptors to the cloud. Run on the intensity-CORRECTED
            # image, NOT the detection image — the noisy beams must not fake blobs.
            # Is THIS frame a shape-factor debug frame? (needs the region debug bundle)
            shape_viz_frame = False
            if self.shape_factor_viz:
                self._shape_viz_count += 1
                shape_viz_frame = (
                    self._shape_viz_count % self.shape_factor_every_n == 0)

            extra = None
            debug = None
            if self.shape_enable or shape_viz_frame:
                n_range = ranges.shape[0]
                step = (ranges[-1] - ranges[0]) / (n_range - 1) if n_range > 1 else 1.0
                # Same integer edge rows as _build_points samples intensity at, so
                # the leading-edge association lands on the published edge pixel.
                edge_rows = np.clip(
                    np.round((edge_range - ranges[0]) / step).astype(int), 0, n_range - 1)
                # Bright-blob detection runs on the DENOISED detection image `det`
                # (the same image the leading edge is found on).
                result = compute_region_channels(
                    det, ranges, bearings, edge_rows, valid,
                    bg_kernel=self.shape_bg_kernel,
                    bin_thresh=self.shape_bin_thresh,
                    open_kernel=self.shape_open_kernel,
                    close_kernel=self.shape_close_kernel,
                    min_area_px=self.shape_min_area_px,
                    max_area_px=self.shape_max_area_px,
                    min_contrast=self.shape_min_contrast,
                    ring_kernel=self.shape_ring_kernel,
                    edge_tol_bins=self.shape_edge_tol_bins,
                    shadow_max_bins=self.shape_shadow_max_bins,
                    shadow_dark_frac=self.shape_shadow_dark_frac,
                    return_regions=shape_viz_frame,     # debug bundle only on viz frames
                )
                channels, debug = result if shape_viz_frame else (result, None)
                if self.shape_enable:
                    # Cloud rows are the valid beams in ascending beam order
                    # (_build_points' keep) -> subset each channel the same way.
                    extra = {name: channels[name][valid] for name in CHANNEL_NAMES}
            self.pub.publish(
                make_pointcloud2(xyz, intensity, msg.header.stamp, self.frame_id, extra=extra))
            if shape_viz_frame:
                # Track the blob CONTRAST distribution so min_contrast can be picked
                # off real numbers (all candidate blobs that passed area).
                bl = debug.get("blobs", []) if debug else []
                if bl:
                    ct = np.array([b["contrast"] for b in bl], dtype=np.float64)
                    crossed = int(sum(b.get("crossed", False) for b in bl))
                    self.get_logger().info(
                        f"shape blobs: {ct.size} candidates | contrast "
                        f"min/med/max={ct.min():.2f}/{np.median(ct):.2f}/{ct.max():.2f}"
                        f" | edge-crossed(regions)={crossed}")
                self._save_shape_factor_image(
                    debug, raw, ranges, bearings, edge_range, valid)
            if self.visualize:
                self._viz_count += 1
                if self._viz_count % self.visualize_every_n == 0:
                    multi_edge = None
                    if self.multi_edge_enable:
                        n_range = ranges.shape[0]
                        step = ((ranges[-1] - ranges[0]) / (n_range - 1)
                                if n_range > 1 else 1.0)
                        e1_rows = np.clip(
                            np.round((edge_range - ranges[0]) / step).astype(int),
                            0, n_range - 1)
                        rgate = (ranges >= self.min_range) & (ranges <= self.max_range)
                        multi_edge = detect_secondary_edges(
                            det, e1_rows, valid, self.min_consecutive,
                            rgate, self.edge_zone_px)
                    self._save_debug_image(raw, ranges, bearings, edge_range, valid,
                                           multi_edge=multi_edge)  # fan + edges + grid
            if dbg is not None:
                self._save_processing_debug(raw, det, ranges, dbg, edge_range, valid)
            self._log_spectrum(det, thr, int(valid.sum()))
        except Exception as exc:  # noqa: BLE001 - keep the node alive
            self.get_logger().warn(f"Dropped a sonar frame: {exc!r}")


def main(args=None):
    rclpy.init(args=args)
    node = LeadingEdgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
