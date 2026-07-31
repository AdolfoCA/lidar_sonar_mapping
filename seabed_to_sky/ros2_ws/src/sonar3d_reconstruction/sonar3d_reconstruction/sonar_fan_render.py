"""
db_render — render a sonar sweep as a Cartesian fan in COMPENSATED dB.

The debug picture that answers "what does the intensity factor actually see?".
It is the same fan layout as the detection stack's sonar_image.png — apex/near
at bottom-centre, far at the top, a polar section grid with a per-section
numeric readout — with two deliberate differences:

  * THE UNITS ARE dB, not raw counts. Every cell is passed through the same
    radiometric compensation (eq 1) the classifier consumes, so the numbers on
    this image are directly comparable with the `mu_c` in classes.yaml. Reading
    a section as "-21 dB" and seeing seabed's mu at -22 dB is the whole point;
    reading it as "4238 counts" tells you nothing about class membership,
    because the same material reads a different count at every range.
  * NO LEADING EDGE is drawn. This is a picture of the measurement field, not
    of the detection. Detection overlays belong on the detector's own debug
    image.

The greyscale itself is mapped from the dB image over a robust percentile
window, so the picture is a picture OF the compensated field rather than of the
raw counts stretched by whatever the brightest specular hit happened to be.

Pure NumPy + OpenCV — no ROS — so the renderer can be exercised on a synthetic
sweep in the tests.
"""

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - exercised only where OpenCV is absent
    cv2 = None

_GRID = (90, 90, 90)
_INK = (235, 235, 235)
_SUB = (150, 150, 150)
_ACCENT = (0, 255, 255)
_GREEN = (0, 255, 0)
_RAW = (255, 160, 60)      # BGR -> blue: the raw count behind a dB number
#: Named tints for the amplitude bands, BGR.
PALETTE = {
    'pink':    (203, 192, 255),
    'yellow':  (0, 255, 255),
    'red':     (0, 0, 255),
    'orange':  (0, 140, 255),
    'magenta': (255, 0, 255),
    'cyan':    (255, 255, 0),
    'green':   (0, 255, 0),
    'blue':    (255, 128, 0),
    'white':   (255, 255, 255),
}


class FanLUT:
    """Cached polar -> Cartesian scan-conversion table.

    Geometry only, so it is rebuilt only when the sweep's shape or angular span
    changes. Forward maps to UP, lateral to ACROSS, apex (near) at bottom-centre
    — the same convention as the rest of the stack's fan images, so the two can
    be compared side by side without mentally mirroring one of them.
    """

    def __init__(self):
        self._key = None

    def ensure(self, ranges: np.ndarray, bearings: np.ndarray) -> None:
        n_range, n_beam = int(ranges.size), int(bearings.size)
        first, last = float(bearings[0]), float(bearings[-1])
        key = (n_range, n_beam, first, last)
        if self._key == key:
            return

        height = n_range
        half = int(np.ceil(float(np.max(np.abs(np.sin(bearings)))) * height))
        width = 2 * half
        step = (last - first) / (n_beam - 1) if n_beam > 1 else 1.0
        tol = abs(step) * 0.75

        py = np.arange(height, dtype=np.float32)[:, None]
        px = np.arange(width, dtype=np.float32)[None, :]
        dz = height - 1 - py                      # forward distance (up)
        dy = px - half                            # lateral offset
        dist = np.sqrt(dy * dy + dz * dz)

        # Radial scale maps pixel radius [0, height-1] onto range index
        # [0, n_range-1]. The height-1 matters: the reachable radius along
        # boresight is height-1 (py = 0), so scaling by height would leave the
        # outermost range bin permanently off-screen and — worse — put the
        # section grid, which uses xy() below, a pixel off the imagery it is
        # annotating.
        self._radial = float(height - 1)
        b_idx = (np.arctan2(-dy, dz) - first) / step
        best = np.round(b_idx)
        ok = (dist <= self._radial) & (best >= 0) & (best < n_beam)
        ok &= (np.abs(b_idx - best) * abs(step)) <= tol
        r_idx = np.round(dist * ((n_range - 1) / self._radial))
        ok &= (r_idx >= 0) & (r_idx < n_range)

        self.valid = ok
        self.ri = np.clip(r_idx, 0, n_range - 1).astype(np.int32)
        self.bi = np.clip(best, 0, n_beam - 1).astype(np.int32)
        self.half, self.width, self.height = half, width, height
        self.n_range, self.n_beam = n_range, n_beam
        self.r0 = float(ranges[0])
        self.span = float(ranges[-1]) - self.r0
        self._key = key

    def map(self, arr, fill=np.nan) -> np.ndarray:
        """Scan-convert any per-cell (n_range, n_beam) array onto the fan.

        The same table that builds the picture also carries the NUMBERS, so a
        pixel and the value reported for it can never disagree — which is the
        whole basis of the interactive probe.
        """
        arr = np.asarray(arr, dtype=np.float64)
        out = np.full((self.height, self.width), fill, dtype=np.float64)
        out[self.valid] = arr[self.ri[self.valid], self.bi[self.valid]]
        return out

    def polar_fans(self, ranges, bearings):
        """(range, bearing) of every fan pixel — what the probe reads back."""
        shape = (self.n_range, self.n_beam)
        return (self.map(np.broadcast_to(np.asarray(ranges, float)[:, None], shape)),
                self.map(np.broadcast_to(np.asarray(bearings, float)[None, :], shape)))

    def xy(self, r: float, bearing: float):
        """(range [m], bearing [rad]) -> integer fan pixel.

        Uses the same radial scale as the scan-conversion table, so an arc drawn
        at range r lands on the pixels that actually hold range r.
        """
        pr = 0.0 if self.span == 0.0 else (r - self.r0) / self.span * self._radial
        return (int(round(self.half - pr * np.sin(bearing))),
                int(round((self.height - 1) - pr * np.cos(bearing))))


def section_stats(db: np.ndarray, n_range_sec: int, n_beam_sec: int):
    """Per-section (mean, median, valid-fraction) of the dB image.

    Cells below the noise floor are NaN in ``db`` and are excluded from both
    statistics rather than counted as some floor value — a section that is
    mostly water should report the level of the few real returns in it, and say
    how few they were, not average in a fictitious number.
    """
    n_r, n_b = db.shape
    re = np.linspace(0, n_r, int(n_range_sec) + 1).astype(int)
    be = np.linspace(0, n_b, int(n_beam_sec) + 1).astype(int)

    out = []
    for ri in range(int(n_range_sec)):
        r0, r1 = int(re[ri]), int(re[ri + 1])
        for bi in range(int(n_beam_sec)):
            b0, b1 = int(be[bi]), int(be[bi + 1])
            if r1 <= r0 or b1 <= b0:
                continue
            sub = db[r0:r1, b0:b1]
            fin = sub[np.isfinite(sub)]
            frac = fin.size / float(sub.size)
            if fin.size:
                out.append((ri, bi, (r0 + r1) // 2, (b0 + b1) // 2,
                            float(fin.mean()), float(np.median(fin)), frac))
            else:
                out.append((ri, bi, (r0 + r1) // 2, (b0 + b1) // 2,
                            np.nan, np.nan, 0.0))
    return out


def range_profile(db, ranges, n_range_sec):
    """Median dB and mid-range of each range BAND (all beams pooled).

    The conversion's own self-check. Compensation exists to make the level
    independent of range, so over homogeneous ground these medians should be
    the same number all the way down the swath. They are pooled across beams
    because a per-section number is noisy while a whole band is not, and the
    question here is about range alone.

    Returns a list of (mid_range_m, median_dB, valid_fraction), near -> far.
    """
    db = np.asarray(db, dtype=np.float64)
    n_r = db.shape[0]
    re = np.linspace(0, n_r, int(n_range_sec) + 1).astype(int)
    out = []
    for ri in range(int(n_range_sec)):
        r0, r1 = int(re[ri]), int(re[ri + 1])
        if r1 <= r0:
            continue
        band = db[r0:r1, :]
        fin = band[np.isfinite(band)]
        mid = float(ranges[(r0 + r1) // 2])
        frac = fin.size / float(band.size)
        out.append((mid, float(np.median(fin)) if fin.size else np.nan, frac))
    return out


def greyscale(fan, display_log: bool, stretch: str, lo_pct: float, hi_pct: float):
    """Map a scan-converted fan onto 0-255. DISPLAY ONLY.

    Two stretches, because they answer different questions:

      max        lo = 0, hi = the frame maximum. Byte-identical to what
                 leading_edge_node writes to sonar_image.png, so the two
                 pictures can be compared directly. Absolute: a cell's shade
                 means the same thing sweep to sweep — but ONE saturated return
                 darkens the entire frame, and near the 65535 clamp that
                 happens often.
      percentile lo/hi from the given percentiles. Robust to a single specular
                 spike, so faint structure stays visible — but the mapping
                 floats with the scene, so a shade is not comparable between
                 sweeps.

    `display_log` compresses the tone curve first. Amplitude spans decades in
    one sweep, so linear crushes most of the image dark; log spreads it. Either
    way this touches only which count becomes which shade — the value planes
    behind the picture stay linear counts, so the readout and everything the
    classifier scores are unaffected.
    """
    plane = _tone(fan, display_log)
    finite = plane[np.isfinite(plane)]
    if finite.size == 0:
        return np.zeros(fan.shape, np.uint8), 0.0, 1.0
    if stretch == 'max':
        lo = _tone(np.array([0.0]), display_log)[0] if display_log else 0.0
        lo = 0.0 if not np.isfinite(lo) else lo
        hi = float(finite.max())
    else:
        lo, hi = float(np.percentile(finite, lo_pct)), float(np.percentile(finite, hi_pct))
    if hi - lo < 1e-12:
        hi = lo + 1.0
    img = np.where(np.isfinite(plane), plane, lo)
    return (np.clip((img - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8),
            lo, hi)


def enhance(img8, method: str, clip: float = 2.0, tiles: int = 8,
            gamma: float = 1.0):
    """Contrast-enhance an 8-bit sonar image. DISPLAY ONLY.

    APPLY THIS IN THE POLAR DOMAIN, before scan conversion. The Cartesian fan
    is more than half black outside the swath, and any local method — CLAHE
    above all — would key its tile statistics off that void, blowing up noise
    along the fan edge and near the apex. The polar image is a full rectangle
    of real data, and its tiles are meaningful blocks of range x beam.

      clahe     contrast-limited adaptive histogram equalisation. The right
                default for sonar: it lifts local structure (a dim target
                against dim background) without the global stretch being set
                by the one bright specular return, and `clip` bounds how much
                noise it may amplify.
      equalize  plain global histogram equalisation. Cruder — it will happily
                amplify the water-column speckle to full scale.
      none      leave it alone.

    `gamma` < 1 lifts the dark end further, applied after either method.

    None of this touches the value planes: the readout and everything the
    classifier scores remain linear counts.
    """
    if cv2 is None or img8 is None:
        return img8
    out = img8
    if method == 'clahe':
        t = max(int(tiles), 1)
        out = cv2.createCLAHE(clipLimit=float(clip),
                              tileGridSize=(t, t)).apply(out)
    elif method == 'equalize':
        out = cv2.equalizeHist(out)
    if abs(gamma - 1.0) > 1e-6 and gamma > 0.0:
        lut8 = np.clip(((np.arange(256) / 255.0) ** (1.0 / gamma)) * 255.0,
                       0, 255).astype(np.uint8)
        out = lut8[out]
    return out


def _tone(plane, display_log: bool):
    """The plane the GREYSCALE is stretched over. Display only.

    Amplitude spans decades — a typical sweep runs a few hundred counts of
    water column to over ten thousand on a hard face, a 55x range. Mapped
    LINEARLY onto 0-255 that puts the median return around grey 57 and crushes
    more than half the image below quarter-brightness: technically faithful,
    visually unreadable. Taking log10 first spreads the same data evenly and
    lands the median near grey 158.

    This changes ONLY which count maps to which shade. The value planes behind
    the picture stay linear, so the hover readout and everything the classifier
    scores are untouched — the tone curve is a property of the monitor, not of
    the measurement.
    """
    if not display_log:
        return plane
    with np.errstate(divide='ignore', invalid='ignore'):
        return np.where(np.isfinite(plane) & (plane > 0.0),
                        np.log10(np.maximum(plane, 1e-9)), np.nan)


def render(db, ranges, bearings, lut: FanLUT, *, counts=None, n_range_sec=6,
           n_beam_sec=24, lo_pct=5.0, hi_pct=99.0, min_valid_frac=0.05,
           bands=(), band_colours=(), band_alpha=0.8,
           display_log=False, stretch='max', title=''):
    """Render the compensated-dB sweep as an annotated BGR fan image.

    Parameters
    ----------
    db        : (n_range, n_beam) compensated intensity [dB]; NaN below the floor.
    ranges    : (n_range,) slant range of each row [m], ascending.
    bearings  : (n_beam,) bearing of each column [rad], ascending.
    lut       : a :class:`FanLUT` (``ensure`` is called for you).
    counts    : optional (n_range, n_beam) RAW counts. When given, each section
                also prints its mean raw count, and the range-band table gains
                a raw column — which is what makes the conversion checkable BY
                HAND: take a section's count and range off the picture, run
                them through eq 1 yourself, and compare with the dB printed
                right above. Nothing else can distinguish "the arithmetic is
                wrong" from "the calibration is wrong".
    lo_pct,
    hi_pct    : percentile window the greyscale is stretched over. Percentiles,
                not min/max, so one saturated specular return cannot wash the
                whole picture out.
    min_valid_frac : sections with fewer than this fraction of returns above the
                floor are drawn dimmed — the number is real but thinly sampled.
    """
    if cv2 is None:
        raise RuntimeError('db_render.render needs OpenCV (python3-opencv)')

    db = np.asarray(db, dtype=np.float64)
    lut.ensure(np.asarray(ranges, float), np.asarray(bearings, float))

    fan = np.where(lut.valid, lut.map(db), np.nan)
    img8, lo, hi = greyscale(fan, display_log, stretch, lo_pct, hi_pct)
    bgr = cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)

    draw_bands(bgr, lut.map(db), bands, band_colours, band_alpha)
    _draw_grid(bgr, ranges, bearings, lut, n_range_sec, n_beam_sec)
    _draw_sections(bgr, db, ranges, bearings, lut,
                   n_range_sec, n_beam_sec, min_valid_frac, counts)
    _draw_range_table(bgr, db, ranges, counts, n_range_sec, lut)
    _draw_legend(bgr, ranges, lut, lo, hi, title, counts is not None)
    return bgr


def _draw_grid(bgr, ranges, bearings, lut, n_range_sec, n_beam_sec):
    n_r, n_b = len(ranges), len(bearings)
    be = np.linspace(0, n_b, int(n_beam_sec) + 1).astype(int)
    re = np.linspace(0, n_r, int(n_range_sec) + 1).astype(int)
    r_lo, r_hi = float(ranges[0]), float(ranges[-1])
    for bi in be:
        b = bearings[min(int(bi), n_b - 1)]
        cv2.line(bgr, lut.xy(r_lo, b), lut.xy(r_hi, b), _GRID, 1, cv2.LINE_AA)
    for ri in re:
        r = float(ranges[min(int(ri), n_r - 1)])
        cv2.polylines(bgr, [np.array([lut.xy(r, b) for b in bearings], np.int32)],
                      False, _GRID, 1, cv2.LINE_AA)


def _draw_sections(bgr, db, ranges, bearings, lut, n_range_sec, n_beam_sec,
                   min_valid_frac, counts=None):
    f = cv2.FONT_HERSHEY_SIMPLEX
    raw_stats = (section_stats(np.where(np.isfinite(db), counts, np.nan),
                               n_range_sec, n_beam_sec)
                 if counts is not None else None)
    for i, (_, _, r_mid, b_mid, mean, med, frac) in enumerate(
            section_stats(db, n_range_sec, n_beam_sec)):
        x, y = lut.xy(float(ranges[r_mid]), float(bearings[b_mid]))
        if not np.isfinite(mean):
            continue
        thin = frac < min_valid_frac
        top = _SUB if thin else _INK
        t1 = f'{mean:+.1f}'
        (tw, _), _ = cv2.getTextSize(t1, f, 0.42, 1)
        cv2.putText(bgr, t1, (x - tw // 2, y), f, 0.42, top, 1, cv2.LINE_AA)
        cv2.putText(bgr, f'{med:+.1f}', (x - tw // 2, y + 13), f, 0.38,
                    _SUB, 1, cv2.LINE_AA)
        row = y + 25
        if raw_stats is not None and np.isfinite(raw_stats[i][4]):
            # The raw count that produced the dB above it — the pair is what
            # makes the conversion hand-checkable.
            cv2.putText(bgr, f'{raw_stats[i][4]:.0f}', (x - tw // 2, row), f,
                        0.36, _RAW, 1, cv2.LINE_AA)
            row += 12
        if thin:
            cv2.putText(bgr, f'{frac * 100:.0f}%', (x - tw // 2, row), f,
                        0.32, (90, 90, 160), 1, cv2.LINE_AA)


def _draw_range_table(bgr, db, ranges, counts, n_range_sec, lut):
    """A near->far readout down the left margin: the conversion's own check.

    Over homogeneous ground these dB medians should not change with range. If
    they march steadily, the compensation curve is wrong; if they jump around
    incoherently while the raw counts vary smoothly, rows are being paired with
    the wrong ranges (the flip_ranges trap).
    """
    f = cv2.FONT_HERSHEY_SIMPLEX
    prof = range_profile(db, ranges, n_range_sec)
    if not prof:
        return
    raw = (range_profile(np.where(np.isfinite(db), counts, np.nan),
                         ranges, n_range_sec)
           if counts is not None else None)

    y0 = 110
    header = 'range   dB' + ('     raw' if raw is not None else '')
    cv2.putText(bgr, header, (6, y0), f, 0.4, _ACCENT, 1, cv2.LINE_AA)
    finite = [d for _, d, _ in prof if np.isfinite(d)]
    for i, (mid, med, _) in enumerate(prof):
        txt = f'{mid:5.1f}m {med:+7.1f}' if np.isfinite(med) else f'{mid:5.1f}m      --'
        if raw is not None and np.isfinite(raw[i][1]):
            txt += f' {raw[i][1]:7.0f}'
        cv2.putText(bgr, txt, (6, y0 + 14 * (i + 1)), f, 0.38, _INK, 1,
                    cv2.LINE_AA)
    if len(finite) >= 2:
        swing = max(finite) - min(finite)
        col = _GREEN if swing <= 3.0 else (0, 140, 255)
        cv2.putText(bgr, f'near-far spread {swing:.1f} dB',
                    (6, y0 + 14 * (len(prof) + 1)), f, 0.4, col, 1, cv2.LINE_AA)


def draw_bands(bgr, plane, edges, colours, alpha=0.8):
    """Tint cells by amplitude band. Returns the population of each band.

    ``edges`` are ascending lower bounds; band i covers [edges[i], edges[i+1])
    and the last is open-ended. Half-open so the bands are disjoint and the
    paint order cannot matter.

    Blended rather than painted flat, so the structure inside a band stays
    visible — a solid patch would hide whether a hot region is one specular
    return or a genuinely bright extended surface, which is usually what you
    are looking at it to find out. Raise ``alpha`` if the bands are hard to
    tell apart: at these levels the underlying greyscale is already near white,
    and it washes every tint toward white as it shows through.
    """
    n_bands = min(len(edges), len(colours))
    counts = [0] * n_bands
    if cv2 is None or n_bands == 0:
        return counts
    finite = np.isfinite(plane)
    for i in range(n_bands):
        lo = float(edges[i])
        hi = float(edges[i + 1]) if i + 1 < n_bands else np.inf
        mask = finite & (plane >= lo) & (plane < hi)
        counts[i] = int(np.count_nonzero(mask))
        if not counts[i]:
            continue
        tint = np.asarray(PALETTE.get(str(colours[i]).lower(), PALETTE['magenta']),
                          dtype=np.float64)
        bgr[mask] = (alpha * tint + (1.0 - alpha) * bgr[mask]).astype(bgr.dtype)
    return counts


def draw_probe(bgr, x, y, lines, *, colour=_ACCENT):
    """Crosshair at (x, y) plus a readout box, for the interactive viewer.

    The box is placed on whichever side of the cursor keeps it fully on screen,
    so the numbers never run off the edge exactly when you reach for the
    interesting part of the image.
    """
    if cv2 is None or not lines:
        return
    h, w = bgr.shape[:2]
    f = cv2.FONT_HERSHEY_SIMPLEX
    scale, thick, pad, lh = 0.45, 1, 8, 17

    cv2.line(bgr, (x - 11, y), (x - 4, y), colour, 1, cv2.LINE_AA)
    cv2.line(bgr, (x + 4, y), (x + 11, y), colour, 1, cv2.LINE_AA)
    cv2.line(bgr, (x, y - 11), (x, y - 4), colour, 1, cv2.LINE_AA)
    cv2.line(bgr, (x, y + 4), (x, y + 11), colour, 1, cv2.LINE_AA)

    box_w = max(cv2.getTextSize(t, f, scale, thick)[0][0] for t in lines) + 2 * pad
    box_h = lh * len(lines) + 2 * pad - 4
    bx = x + 16 if x + 16 + box_w < w else x - 16 - box_w
    by = y + 16 if y + 16 + box_h < h else y - 16 - box_h
    bx, by = max(0, min(bx, w - box_w)), max(0, min(by, h - box_h))

    panel = bgr[by:by + box_h, bx:bx + box_w]
    if panel.size:
        # Dim rather than fill, so the imagery under the box stays readable.
        panel[:] = (panel * 0.25).astype(bgr.dtype)
        cv2.rectangle(bgr, (bx, by), (bx + box_w - 1, by + box_h - 1),
                      colour, 1, cv2.LINE_AA)
        for i, t in enumerate(lines):
            cv2.putText(bgr, t, (bx + pad, by + pad + lh * i + 9), f, scale,
                        _INK, thick, cv2.LINE_AA)


def _draw_legend(bgr, ranges, lut, lo, hi, title, has_counts=False):
    f = cv2.FONT_HERSHEY_SIMPLEX
    readout = ('compensated dB — mean / median / RAW count per section'
               if has_counts else 'compensated dB — mean / median per section')
    lines = [
        (f'FAR {float(ranges[-1]):.1f}m', _GREEN),
        (readout, _ACCENT),
        (f'greyscale {lo:+.1f} .. {hi:+.1f} dB', _SUB),
        ('dim + % = section mostly below the detection floor', _SUB),
    ]
    if title:
        lines.append((title, _SUB))
    for i, (txt, col) in enumerate(lines):
        cv2.putText(bgr, txt, (4, 16 + 15 * i), f, 0.45, col, 1, cv2.LINE_AA)
    cv2.putText(bgr, f'NEAR {float(ranges[0]):.1f}m',
                (lut.half - 30, lut.height - 8), f, 0.5, _GREEN, 1, cv2.LINE_AA)
