#!/usr/bin/env python3
"""
region_shapes.py
================

Image-domain BRIGHT-BLOB detection + shape descriptors for the leading-edge
extractor (paper eqs 39-42, Algorithm 1 line 1: "grow bright regions from the
leading edge; set s^m_i, N_R, m_R").

WHY: the leading edge alone throws away morphology. Per the paper, a return may
carry "an optional morphology descriptor ... when the return belongs to a
connected bright region" (eq 9), and each such region R carries

    m_R = (a_R, ς_R, ξ_R, τ_R)                                        (eq 40)

with a_R the log physical area of the bright region, ς_R ∈ [0,1] its solidity,
ξ_R a shadow signature ("the depth, length, and sharpness of the acoustic
shadow trailing the region in range" — the principal discriminator), and τ_R its
internal intensity variance. Membership is s^m_i = 1[∃R : i ∈ R] (eq 39); "most
returns are bare seabed belonging to none", and that absence is itself
informative — so non-region valid beams get flag 0, NOT NaN (NaN = the extractor
did not run at all, which makes the eq-42 shape factor ABSTAIN).

THE TARGET IS BRIGHTNESS CONTRAST, NOT TEXTURE. The features of interest (algae /
plants) are patches that are VERY BRIGHT RELATIVE TO THEIR LOCAL SURROUNDINGS.
That is a segmentation-by-local-contrast problem, so — instead of a texture
keypoint detector (AKAZE) — we detect them with a white TOP-HAT and an explicit
local-contrast filter, and keep the CONNECTED REGION (its real 2D shape):

    1. white TOP-HAT = image − morphological opening(image, bg_kernel). Removes
       anything larger than bg_kernel (the slowly-varying seabed floor), leaving
       the compact bright peaks — a "how much brighter than the local background"
       map,
    2. THRESHOLD the top-hat (Otsu over the positive top-hat, or a fixed cut),
    3. clean the mask (open away speckle, close small holes),
    4. CONNECTED COMPONENTS → candidate blobs; filter by AREA,
    5. LOCAL-CONTRAST filter: for each blob, the Michelson contrast between its
       interior and a surrounding ring,  c = (m_in − m_ring)/(m_in + m_ring);
       keep blobs with c ≥ min_contrast — this is literally "bright vs around it",
    6. LEADING-EDGE ASSOCIATION: a valid beam whose leading-edge pixel lies in (or
       within edge_tol_bins of) a kept blob crosses it (eq 39 membership); a blob
       crossed by ≥1 beam becomes a region R,
    7. m_R (eq 40) — area, SOLIDITY (real now: a compact blob is convex, an
       irregular plant is not), shadow, texture — is measured on the blob's
       pixels and written onto every member beam.

Because the region is a real connected blob again, ALL FOUR shape channels are
meaningful (solidity is no longer NaN). This module is pure NumPy/OpenCV — NO ROS
imports — so it is unit-testable on synthetic images.

Output channels (all (B,) float32, one value per beam / image column):
    shape_flag       s^m: 1 = beam's leading edge falls in a bright blob region,
                     0 = valid beam with no region, NaN = beam had no edge.
    region_size      N_R, the number of member beams (crossing leading edges) in
                     the beam's blob; 0 when flag != 1.
    region_logarea   a_R    (NaN when flag != 1)
    region_solidity  ς_R    (NaN when flag != 1)
    region_shadow    ξ_R    (NaN when flag != 1 or the shadow is unobservable)
    region_texture   τ_R    (NaN when flag != 1)
"""

import cv2
import numpy as np

# Channel order is the wire order: the node appends the extra PointCloud2
# fields in exactly this sequence, and the classifier's Features block maps
# them back by name.
CHANNEL_NAMES = (
    "shape_flag",
    "region_size",
    "region_logarea",
    "region_solidity",
    "region_shadow",
    "region_texture",
)

_EPS = 1e-9


def _blob_contrast(image, labels, lab, left, top, w, h, ring, n_range, n_beam):
    """Michelson contrast (m_in − m_ring)/(m_in + m_ring) of blob `lab` against a
    surrounding ring of half-width `ring` px. Computed on a padded bounding box so
    the ring is complete. Returns c ∈ [-1, 1] (≈ "fraction brighter than around")."""
    pad = int(ring)
    r0 = max(0, top - pad); r1 = min(n_range, top + h + pad)
    c0 = max(0, left - pad); c1 = min(n_beam, left + w + pad)
    sub = labels[r0:r1, c0:c1] == lab
    se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * pad + 1, 2 * pad + 1))
    dil = cv2.dilate(sub.astype(np.uint8), se) > 0
    ring_mask = dil & ~sub
    img_sub = image[r0:r1, c0:c1]
    m_in = float(img_sub[sub].mean()) if sub.any() else 0.0
    m_ring = float(img_sub[ring_mask].mean()) if ring_mask.any() else 0.0
    return (m_in - m_ring) / (m_in + m_ring + _EPS)


# ───────────────────────── top-hat bright-blob detection ────────────────────
def compute_region_channels(
    image,
    ranges,
    bearings,
    edge_rows,
    valid,
    *,
    bg_kernel=25,
    bin_thresh=0.0,
    open_kernel=3,
    close_kernel=3,
    min_area_px=20,
    max_area_px=0,
    min_contrast=0.15,
    ring_kernel=9,
    edge_tol_bins=5,
    shadow_max_bins=40,
    shadow_dark_frac=0.35,
    return_regions=False,
):
    """Detect bright-contrast blobs, associate the leading edge with them, and
    return the per-beam shape channels (paper eqs 39-42).

    Parameters
    ----------
    image : (n_range, n_beam) float array — the DENOISED sonar image (row 0 =
        nearest range bin). The top-hat needs the real backscatter values.
    ranges, bearings, edge_rows, valid : as elsewhere in the extractor.

    bg_kernel : diameter (px) of the top-hat opening — the object/seabed scale.
        Bright features SMALLER than this survive; the slowly-varying seabed
        larger than it is subtracted. Set a bit above the biggest blob you want.
    bin_thresh : threshold on the top-hat. ``<= 0`` (default) = Otsu over the
        positive top-hat pixels; a positive value pins a fixed contrast cut.
    open_kernel : opening diameter (px) removing speckle from the mask; 0 = off.
    close_kernel : closing diameter (px) filling small holes; 0 = off.
    min_area_px : drop blobs smaller than this many pixels (discretisation noise).
    max_area_px : drop blobs larger than this (0 = no upper limit) — rejects a
        whole bright seabed swath that isn't a discrete object.
    min_contrast : keep a blob only if its Michelson interior-vs-ring contrast is
        ≥ this (0..1). THE knob for "very bright compared to around it": ↑ demands
        a starker blob, ↓ admits fainter ones.
    ring_kernel : half-width (px) of the surrounding ring the contrast is measured
        against — roughly how far out "the area around" extends.
    edge_tol_bins : a leading-edge pixel counts as inside a blob if the blob lies
        within ± this many range bins of it at that beam (the leading edge sits at
        the blob's near face, so a small tolerance ties them together).
    shadow_max_bins, shadow_dark_frac : shadow-signature window (as elsewhere).
    return_regions : also return a DEBUG bundle for the overlay.

    Returns
    -------
    dict name -> (n_beam,) float32 (CHANNEL_NAMES). If ``return_regions``:
    ``(dict, debug)`` with::

        {
          'contrast_map': (n_range, n_beam) float32,   # the top-hat map
          'blobs':   [ {'ys':int[],'xs':int[],'crossed':bool,'contrast':float}, ...],
          'regions': [ {'ys','xs','member_cols','edge_rows','n_r','contrast',
                        'logarea','solidity','shadow','texture'}, ... ]
        }
    """
    image = np.asarray(image, dtype=np.float64)
    n_range, n_beam = image.shape
    ranges = np.asarray(ranges, dtype=np.float64)
    bearings = np.asarray(bearings, dtype=np.float64)
    valid = np.asarray(valid, dtype=bool)
    edge_rows = np.clip(np.asarray(edge_rows, dtype=np.int64), 0, n_range - 1)

    out = {
        "shape_flag": np.full(n_beam, np.nan, dtype=np.float32),
        "region_size": np.zeros(n_beam, dtype=np.float32),
        "region_logarea": np.full(n_beam, np.nan, dtype=np.float32),
        "region_solidity": np.full(n_beam, np.nan, dtype=np.float32),
        "region_shadow": np.full(n_beam, np.nan, dtype=np.float32),
        "region_texture": np.full(n_beam, np.nan, dtype=np.float32),
    }
    out["shape_flag"][valid] = 0.0

    contrast_map = np.zeros((n_range, n_beam), dtype=np.float32)
    binmask_dbg = np.zeros((n_range, n_beam), dtype=np.uint8)
    blobs = []
    regions = []

    def _ret():
        if return_regions:
            return out, {"contrast_map": contrast_map, "binmask": binmask_dbg,
                         "blobs": blobs, "regions": regions}
        return out

    if not valid.any() or n_range < 2 or n_beam < 2:
        return _ret()

    # ── 1. robust uint8 scaling + white TOP-HAT (local bright-contrast map) ─
    robust_max = float(np.percentile(image, 99.0))
    if robust_max <= _EPS:
        robust_max = float(np.nanmax(image)) if image.size else 0.0
    if robust_max <= _EPS:
        return _ret()
    img8 = np.clip(image / robust_max * 255.0, 0.0, 255.0).astype(np.uint8)
    bg_se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (int(bg_kernel), int(bg_kernel)))
    tophat = cv2.morphologyEx(img8, cv2.MORPH_TOPHAT, bg_se)
    contrast_map = tophat.astype(np.float32)

    # ── 2. threshold the top-hat (Otsu over the positive part, or a fixed cut) ─
    if bin_thresh <= 0.0:
        pos = tophat[tophat > 0]
        if pos.size == 0:
            return _ret()
        thr, _ = cv2.threshold(
            pos.reshape(-1, 1), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    else:
        thr = float(bin_thresh)
    binmask = (tophat >= thr).astype(np.uint8)

    # ── 3. clean the mask: open away speckle, close small holes ────────────
    if open_kernel > 0:
        binmask = cv2.morphologyEx(
            binmask, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (int(open_kernel), int(open_kernel))))
    if close_kernel > 0:
        binmask = cv2.morphologyEx(
            binmask, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (int(close_kernel), int(close_kernel))))
    binmask_dbg = binmask                            # expose the object mask for the overlay
    if not binmask.any():
        return _ret()

    # ── 4. connected components; filter by AREA and 5. by LOCAL CONTRAST ───
    n_lab, labels, stats, _ = cv2.connectedComponentsWithStats(binmask, connectivity=8)
    kept = []                              # (lab, contrast) surviving both filters
    for lab in range(1, n_lab):
        area_px = int(stats[lab, cv2.CC_STAT_AREA])
        if area_px < int(min_area_px):
            continue
        if max_area_px > 0 and area_px > int(max_area_px):
            continue
        c = _blob_contrast(
            image, labels, lab,
            int(stats[lab, cv2.CC_STAT_LEFT]), int(stats[lab, cv2.CC_STAT_TOP]),
            int(stats[lab, cv2.CC_STAT_WIDTH]), int(stats[lab, cv2.CC_STAT_HEIGHT]),
            ring_kernel, n_range, n_beam)
        if c < float(min_contrast):
            continue
        kept.append((lab, c))
    if not kept:
        return _ret()
    kept_labels = np.array([lab for lab, _ in kept], dtype=np.int64)
    contrast_of = {lab: c for lab, c in kept}
    # label image restricted to kept blobs (0 elsewhere), for the edge test.
    labels_kept = np.where(np.isin(labels, kept_labels), labels, 0)

    # ── 6. leading-edge association ───────────────────────────────────────
    # A valid beam b crosses a kept blob when that blob's label appears within
    # ± edge_tol_bins of the beam's leading-edge row (the edge sits at the blob's
    # near face). The beam joins the most-present blob in that window.
    tol = int(edge_tol_bins)
    members = {int(lab): [] for lab in kept_labels}   # lab -> [beam cols]
    for b in np.flatnonzero(valid):
        er = int(edge_rows[b])
        w = labels_kept[max(0, er - tol):min(n_range, er + tol + 1), b]
        nz = w[w > 0]
        if nz.size == 0:
            continue
        u, cnt = np.unique(nz, return_counts=True)
        lab = int(u[int(np.argmax(cnt))])             # most-present kept blob
        members[lab].append(int(b))

    # ── overlay: record every kept blob (crossed flag) for the debug bundle ─
    if return_regions:
        for lab in kept_labels:
            ys, xs = np.nonzero(labels == lab)
            blobs.append({"ys": ys, "xs": xs,
                          "crossed": len(members[int(lab)]) > 0,
                          "contrast": float(contrast_of[int(lab)])})

    # Physical pixel footprint: cell at range rho spans drho × rho*dbeta.
    drho = (ranges[-1] - ranges[0]) / (n_range - 1)
    dbeta = abs(float(np.mean(np.diff(bearings))))

    # ── 7. regions: every kept blob with ≥1 crossing beam ─────────────────
    for lab in kept_labels:
        mcols = members[int(lab)]
        if not mcols:
            continue
        mcols = np.sort(np.asarray(mcols, dtype=np.int64))
        ys, xs = np.nonzero(labels == lab)
        vals = image[ys, xs]

        # a_R (eq 40): LOG physical area of the blob.
        area = drho * dbeta * float(np.sum(ranges[ys]))
        logarea = float(np.log(max(area, _EPS)))

        # ς_R (eq 40): solidity = pixel area / convex-hull area (REAL now — a
        # compact blob ~1, an irregular plant < 1); clipped to [0, 1].
        pts = np.stack([xs, ys], axis=1).astype(np.int32).reshape(-1, 1, 2)
        hull_area = float(cv2.contourArea(cv2.convexHull(pts)))
        solidity = 1.0 if hull_area <= _EPS else min(1.0, float(ys.size) / hull_area)

        # ξ_R (eq 40): shadow signature behind the blob, per MEMBER column.
        scores = []
        for c in mcols:
            col_rows = ys[xs == c]
            if col_rows.size == 0:
                continue
            rear = int(col_rows.max())
            col_mean = float(image[col_rows, c].mean())
            w0, w1 = rear + 1, min(rear + int(shadow_max_bins), n_range - 1)
            if w0 > w1:
                continue
            window = image[w0:w1 + 1, c]
            contrast = float(np.clip(1.0 - window.mean() / (col_mean + _EPS), 0.0, 1.0))
            dark = window < shadow_dark_frac * col_mean
            run = int(dark.size) if dark.all() else int(np.argmin(dark))
            scores.append(contrast * min(1.0, run / float(shadow_max_bins)))
        shadow = float(np.mean(scores)) if scores else np.nan

        # τ_R (eq 40): internal intensity variance, LOG scale.
        texture = float(np.log1p(np.var(vals))) if vals.size else np.nan

        n_r = int(mcols.size)
        out["shape_flag"][mcols] = 1.0
        out["region_size"][mcols] = float(n_r)
        out["region_logarea"][mcols] = logarea
        out["region_solidity"][mcols] = solidity
        out["region_shadow"][mcols] = shadow
        out["region_texture"][mcols] = texture

        if return_regions:
            regions.append(
                {
                    "ys": ys, "xs": xs,
                    "member_cols": mcols, "edge_rows": edge_rows[mcols],
                    "n_r": n_r, "contrast": float(contrast_of[int(lab)]),
                    "logarea": logarea, "solidity": solidity,
                    "shadow": shadow, "texture": texture,
                }
            )

    return _ret()
