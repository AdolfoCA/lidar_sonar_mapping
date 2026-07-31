"""
bottom_grid — rolling ẑ_bed(x, y) bottom reference for the position factor.

The position factor (eq 31) scores the height ABOVE THE SEABED,
h_i = z_i − ẑ_bed(x_i, y_i) (eq 25), so every return needs a local bottom
reference at its XY. The paper posits a "locally fitted bottom surface"; this
module is its online stand-in: a coarse XY grid whose cells hold a robust
running estimate of the local bed height, built from the very returns being
classified — no separate bathymetry input, no batch fit.

Two choices make the reference robust to the structure it must NOT follow:

  * LOW QUANTILE, not mean — per scan, each occupied cell contributes only
    its `quantile` (default 0.25) of z. Walls, pillars and vegetation return
    ABOVE the bed, so they populate the upper tail of a cell's z sample and
    barely move a low quantile while any bed is visible in the cell. The
    quantile of symmetric noise sits slightly below the true bed, which is
    harmless: the position factor carries a per-class spread ω²_c precisely
    because this reference is approximate.
  * EMA ACROSS SCANS (alpha = `forget`, first touch initialises) — the cell
    estimate tracks slowly varying bathymetry (and slow z drift in odometry)
    while single noisy scans are averaged out.

QUERY FALLBACK CHAIN (query()): the return's own cell if ever observed, else
the mean of known cells within `neighbor_radius` (Chebyshev) cells, else a
global fallback (EMA of the scan-level low quantile of ALL z), else NaN.
NaN propagates through geometry.height_above_bed to h_i = NaN, so the
position factor ABSTAINS — it never guesses — over ground nobody has seen.

Pure NumPy, no ROS imports (dirichlet/__init__ design rules). update() is
vectorised per scan via np.unique on packed cell indices: the only Python
loop runs over the occupied CELLS of the scan, never over returns.
"""

import numpy as np

# Cell indices are packed into one int64 (ix in the high 32 bits, iy in the
# low 32) so np.unique groups a scan on a flat array. Valid while
# |ix|, |iy| < 2³¹ — i.e. maps narrower than ~10⁹ cells, always true here.
_LOW32 = 0xFFFFFFFF


class BottomGrid:
    """Rolling per-cell EMA of the per-scan low-quantile z (ẑ_bed of eq 25)."""

    def __init__(self, cell_size: float = 0.5, quantile: float = 0.25,
                 forget: float = 0.10, neighbor_radius: int = 1,
                 min_scan_points: int = 1):
        """
        Parameters
        ----------
        cell_size        XY cell edge [m]. Coarser cells see more bed returns
                         per scan (a steadier quantile) but blur real slopes.
        quantile         per-scan, per-cell low quantile of z that enters the
                         EMA. Low enough that structure returns (above the
                         bed) stay in the discarded upper tail; high enough
                         not to chase the noise floor.
        forget           EMA weight of the newest scan, in (0, 1]. First
                         touch of a cell initialises it outright.
        neighbor_radius  Chebyshev radius [cells] searched by query() when
                         the return's own cell was never observed.
        min_scan_points  minimum returns a cell needs IN ONE SCAN for that
                         scan's quantile to be trusted (a lone stray return
                         is no bed evidence).
        """
        if cell_size <= 0.0:
            raise ValueError('BottomGrid: cell_size must be > 0')
        if not 0.0 < quantile < 1.0:
            raise ValueError('BottomGrid: quantile must be in (0, 1)')
        if not 0.0 < forget <= 1.0:
            raise ValueError('BottomGrid: forget must be in (0, 1]')
        if neighbor_radius < 0:
            raise ValueError('BottomGrid: neighbor_radius must be >= 0')
        if min_scan_points < 1:
            raise ValueError('BottomGrid: min_scan_points must be >= 1')

        self.cell_size = float(cell_size)
        self.quantile = float(quantile)
        self.forget = float(forget)
        self.neighbor_radius = int(neighbor_radius)
        self.min_scan_points = int(min_scan_points)

        self._cells: dict = {}    # (ix, iy) -> EMA of per-scan low-quantile z
        self._global = np.nan     # EMA of the scan-level low quantile of all z

    # ── per-scan update ───────────────────────────────────────────────────────
    def update(self, x, y, z) -> None:
        """Fold one scan's returns into the grid. Non-finite rows are ignored.

        x, y, z : array-like, same length — return positions (odom, z up).
        """
        x = np.asarray(x, dtype=np.float64).ravel()
        y = np.asarray(y, dtype=np.float64).ravel()
        z = np.asarray(z, dtype=np.float64).ravel()
        if not (x.size == y.size == z.size):
            raise ValueError('BottomGrid.update: x, y, z lengths differ')

        good = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        if not np.any(good):
            return
        x, y, z = x[good], y[good], z[good]
        a = self.forget

        # Global fallback: the whole scan's low quantile — robust to the same
        # above-bed structure as the per-cell estimate, for the same reason.
        gq = float(np.quantile(z, self.quantile))
        self._global = gq if np.isnan(self._global) \
            else (1.0 - a) * self._global + a * gq

        ix = np.floor(x / self.cell_size).astype(np.int64)
        iy = np.floor(y / self.cell_size).astype(np.int64)
        keys = (ix << 32) ^ (iy & _LOW32)
        _, first, inv, counts = np.unique(
            keys, return_index=True, return_inverse=True, return_counts=True)

        # Grouped low quantile without a per-return loop: sort z within each
        # cell group, then read the interpolated quantile straight off the
        # sorted array (matches np.quantile's default linear interpolation).
        zs = z[np.lexsort((z, inv))]
        starts = np.concatenate(([0], np.cumsum(counts[:-1])))
        pos = self.quantile * (counts - 1)
        lo = np.floor(pos).astype(np.int64)
        frac = pos - lo
        hi = np.ceil(pos).astype(np.int64)
        qz = zs[starts + lo] * (1.0 - frac) + zs[starts + hi] * frac

        keep = counts >= self.min_scan_points
        cells = self._cells
        for cix, ciy, q in zip(ix[first][keep], iy[first][keep], qz[keep]):
            key = (int(cix), int(ciy))
            old = cells.get(key)
            cells[key] = float(q) if old is None else (1.0 - a) * old + a * q

    # ── query ─────────────────────────────────────────────────────────────────
    def query(self, x, y) -> np.ndarray:
        """ẑ_bed at each (x, y): cell value → neighbour mean → global → NaN.

        Returns (N,) float64. Non-finite query coordinates yield NaN.
        """
        x = np.asarray(x, dtype=np.float64).ravel()
        y = np.asarray(y, dtype=np.float64).ravel()
        if x.size != y.size:
            raise ValueError('BottomGrid.query: x, y lengths differ')

        out = np.full(x.size, np.nan, dtype=np.float64)
        good = np.isfinite(x) & np.isfinite(y)
        if not np.any(good):
            return out

        ix = np.floor(x[good] / self.cell_size).astype(np.int64)
        iy = np.floor(y[good] / self.cell_size).astype(np.int64)
        keys = (ix << 32) ^ (iy & _LOW32)
        # Resolve each distinct cell once; a scan's returns share few cells.
        _, first, inv = np.unique(keys, return_index=True, return_inverse=True)
        uvals = np.array([self._resolve(int(cix), int(ciy))
                          for cix, ciy in zip(ix[first], iy[first])],
                         dtype=np.float64)
        out[good] = uvals[inv]
        return out

    def _resolve(self, cix: int, ciy: int) -> float:
        """One cell through the fallback chain (see module docstring)."""
        v = self._cells.get((cix, ciy))
        if v is not None:
            return v
        r = self.neighbor_radius
        acc, n = 0.0, 0
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx == 0 and dy == 0:      # own cell already known missing
                    continue
                w = self._cells.get((cix + dx, ciy + dy))
                if w is not None:
                    acc += w
                    n += 1
        if n:
            return acc / n
        return self._global                  # NaN while the grid is empty
