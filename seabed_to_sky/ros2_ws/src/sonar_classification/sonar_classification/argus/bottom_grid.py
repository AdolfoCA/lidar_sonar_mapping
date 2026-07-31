"""
bottom_grid — the rolling local bed fit z_bed_hat(x, y) behind h_i (eq 18).

The position factor scores HEIGHT ABOVE THE SEABED, so every return needs a
local bottom reference at its XY. This is the online stand-in for the paper's
"locally fitted bottom surface": a coarse XY grid whose cells hold a robust
running estimate of the bed, built from the very returns being classified — no
separate bathymetry input, no batch fit.

Two choices make the reference robust to the structure it must NOT follow:

  * LOW QUANTILE, NOT MEAN — per sweep each occupied cell contributes only its
    ``quantile`` of z. Walls, pillars and vegetation return ABOVE the bed, so
    they populate the upper tail of a cell's sample and barely move a low
    quantile while any bed is visible in the cell. The quantile of symmetric
    noise sits slightly below the true bed, which is harmless: tau_c for
    bed-anchored classes exists precisely to absorb bed-fit error at the
    lower edge.
  * EMA ACROSS SWEEPS (alpha = ``forget``; first touch initialises) — the cell
    tracks slowly varying bathymetry and slow odometry drift in z, while a
    single noisy sweep is averaged out.

QUERY FALLBACK CHAIN: the return's own cell if ever observed, else the mean of
known cells within ``neighbor_radius`` (Chebyshev) cells, else a global
fallback (EMA of the sweep-level low quantile of all z), else NaN. NaN
propagates into h_i, so the position factor ABSTAINS over ground nobody has
seen instead of guessing.

Pure NumPy. ``update`` is vectorised per sweep via np.unique on packed cell
indices: the only Python loop runs over the occupied CELLS of a sweep, never
over returns.

(Ported from the proven implementation in sonar_map/dirichlet/bottom_grid.py;
the algorithm is unchanged, the vocabulary follows this package.)
"""

import numpy as np

# Cell indices are packed into one int64 (ix high, iy low) so np.unique groups a
# sweep on a flat array. Valid while |ix|, |iy| < 2^31 — always true here.
_LOW32 = 0xFFFFFFFF


class BottomGrid:
    """Rolling per-cell EMA of the per-sweep low-quantile z (z_bed_hat, eq 18)."""

    def __init__(self, cell_size: float = 0.5, quantile: float = 0.25,
                 forget: float = 0.10, neighbor_radius: int = 1,
                 min_scan_points: int = 1):
        """
        Parameters
        ----------
        cell_size        XY cell edge [m]. Coarser cells see more bed returns
                         per sweep (a steadier quantile) but blur real slopes.
        quantile         per-sweep, per-cell low quantile of z entering the EMA.
                         Low enough that structure returns stay in the discarded
                         upper tail; high enough not to chase the noise floor.
        forget           EMA weight of the newest sweep, in (0, 1].
        neighbor_radius  Chebyshev radius [cells] searched when the return's own
                         cell was never observed.
        min_scan_points  returns a cell needs IN ONE SWEEP for that sweep's
                         quantile to be trusted (a lone stray is no bed evidence).
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

        self._cells: dict = {}    # (ix, iy) -> EMA of per-sweep low-quantile z
        self._global = np.nan     # EMA of the sweep-level low quantile of all z

    # ── per-sweep update ─────────────────────────────────────────────────────
    def update(self, x, y, z) -> None:
        """Fold one sweep's returns into the grid. Non-finite rows are ignored."""
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
        zs = z[np.lexsort((z, inv.ravel()))]
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

    # ── query ────────────────────────────────────────────────────────────────
    def query(self, x, y) -> np.ndarray:
        """z_bed_hat at each (x, y): cell -> neighbour mean -> global -> NaN."""
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
        _, first, inv = np.unique(keys, return_index=True, return_inverse=True)
        uvals = np.array([self._resolve(int(cix), int(ciy))
                          for cix, ciy in zip(ix[first], iy[first])],
                         dtype=np.float64)
        out[good] = uvals[inv.ravel()]
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
                if dx == 0 and dy == 0:
                    continue
                w = self._cells.get((cix + dx, ciy + dy))
                if w is not None:
                    acc += w
                    n += 1
        if n:
            return acc / n
        return self._global
