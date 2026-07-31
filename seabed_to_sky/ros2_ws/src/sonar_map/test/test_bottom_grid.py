#!/usr/bin/env python3
"""Unit tests for sonar_map.dirichlet.bottom_grid — the rolling ẑ_bed reference.

Covers: flat-bottom convergence everywhere observed, robustness of the low
quantile to a wall sharing a cell with bed returns, the query fallback chain
(own cell → neighbour mean → global → NaN), EMA tracking of a slowly
deepening bed, the min_scan_points gate and non-finite input handling.

Run:  cd src/sonar_map && python3 -m pytest test/test_bottom_grid.py -v
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sonar_map.dirichlet.bottom_grid import BottomGrid  # noqa: E402


def _flat_scan(rng, n, bed, extent=10.0, noise=0.05):
    x = rng.uniform(0.0, extent, n)
    y = rng.uniform(0.0, extent, n)
    z = rng.normal(bed, noise, n)
    return x, y, z


# ── flat bottom ───────────────────────────────────────────────────────────────

def test_flat_bottom_converges_everywhere_observed():
    rng = np.random.default_rng(0)
    g = BottomGrid(cell_size=1.0)
    for _ in range(5):
        g.update(*_flat_scan(rng, 4000, bed=-10.0))

    # fresh query points strictly inside the observed extent — every cell there
    # has been hit several times, so the direct estimate answers everywhere
    qx = rng.uniform(0.5, 9.5, 500)
    qy = rng.uniform(0.5, 9.5, 500)
    zb = g.query(qx, qy)
    assert zb.shape == (500,)
    assert zb.dtype == np.float64
    assert np.all(np.isfinite(zb))
    assert np.allclose(zb, -10.0, atol=0.15)


# ── wall robustness ───────────────────────────────────────────────────────────

def test_wall_in_cell_barely_moves_estimate():
    """A cluster of returns 5 m above the bed shares the cell with bed returns;
    the low quantile must stay on the bed while a mean would be dragged ~2 m up."""
    rng = np.random.default_rng(1)
    n_bed, n_wall = 60, 40
    xb, yb = rng.uniform(0, 1, n_bed), rng.uniform(0, 1, n_bed)
    zb = rng.normal(-10.0, 0.05, n_bed)
    xw, yw = rng.uniform(0, 1, n_wall), rng.uniform(0, 1, n_wall)
    zw = rng.normal(-5.0, 0.30, n_wall)

    g_ref = BottomGrid(cell_size=1.0)
    g_ref.update(xb, yb, zb)                      # bed only
    ref = g_ref.query([0.5], [0.5])[0]

    g = BottomGrid(cell_size=1.0)
    g.update(np.concatenate([xb, xw]),            # bed + wall, one scan
             np.concatenate([yb, yw]),
             np.concatenate([zb, zw]))
    est = g.query([0.5], [0.5])[0]

    assert abs(est - ref) < 0.15                  # wall barely registers
    assert abs(est - (-10.0)) < 0.2               # still on the bed
    # sanity: the naive mean IS pulled far off the bed by the same wall
    assert np.mean(np.concatenate([zb, zw])) > -8.5


# ── query fallback chain ──────────────────────────────────────────────────────

def test_query_fallback_neighbor_then_global():
    """Two regions at different depths make the neighbour and global fallbacks
    distinguishable: adjacent to the shallow region → its value; far from
    everything → the global (whole-scan) quantile, dominated by the deep one."""
    rng = np.random.default_rng(2)
    g = BottomGrid(cell_size=1.0, neighbor_radius=1)
    xa, ya = rng.uniform(0, 1, 50), rng.uniform(0, 1, 50)      # cell (0, 0)
    za = rng.normal(-10.0, 0.02, 50)
    xb, yb = rng.uniform(20, 21, 50), rng.uniform(20, 21, 50)  # cell (20, 20)
    zb = rng.normal(-20.0, 0.02, 50)
    g.update(np.concatenate([xa, xb]),
             np.concatenate([ya, yb]),
             np.concatenate([za, zb]))

    # cell (1, 0) never observed; its only known Chebyshev-1 neighbour is (0, 0)
    v_adj = g.query([1.5], [0.5])[0]
    assert abs(v_adj - (-10.0)) < 0.2

    # cell (100, 100): no neighbours known → global = 0.25 quantile of ALL z,
    # which lands inside the deeper half of the combined scan
    v_far = g.query([100.5], [100.5])[0]
    assert abs(v_far - (-20.0)) < 0.3
    assert abs(v_far - v_adj) > 5.0               # genuinely distinct fallbacks


def test_empty_grid_returns_nan():
    g = BottomGrid()
    out = g.query(np.array([0.0, 5.0]), np.array([0.0, 5.0]))
    assert out.shape == (2,)
    assert np.all(np.isnan(out))


def test_nonfinite_query_coordinates_return_nan():
    g = BottomGrid(cell_size=1.0)
    g.update(np.full(5, 0.5), np.full(5, 0.5), np.full(5, -10.0))
    out = g.query([np.nan, 0.5], [0.5, 0.5])
    assert np.isnan(out[0])
    assert out[1] == -10.0


# ── EMA tracking ──────────────────────────────────────────────────────────────

def test_ema_tracks_slowly_deepening_bottom():
    rng = np.random.default_rng(3)
    g = BottomGrid(cell_size=1.0, forget=0.3)
    n_scans, drift = 60, 0.05                     # 3 m total descent
    for t in range(n_scans):
        bed = -10.0 - drift * t
        g.update(rng.uniform(0, 1, 50), rng.uniform(0, 1, 50),
                 rng.normal(bed, 0.02, 50))

    final_bed = -10.0 - drift * (n_scans - 1)
    est = g.query([0.5], [0.5])[0]
    # steady-state EMA lag behind a linear drift is drift·(1−a)/a ≈ 0.12 m
    assert abs(est - final_bed) < 0.3
    assert est < -12.0                            # followed most of the descent


# ── gating and input hygiene ──────────────────────────────────────────────────

def test_min_scan_points_gates_sparse_cells():
    g = BottomGrid(cell_size=1.0, min_scan_points=3)
    g.update([5.5], [5.5], [-4.0])                # lone stray: below the gate
    g.update(np.full(10, 0.5), np.full(10, 0.5), np.full(10, -10.0))

    assert len(g._cells) == 1                     # only the populated cell stored
    assert g.query([0.5], [0.5])[0] == -10.0
    # the stray's cell answers through the global fallback, not the stray itself
    v = g.query([5.5], [5.5])[0]
    assert np.isfinite(v)
    assert v != -4.0


def test_update_ignores_nonfinite_rows():
    g = BottomGrid(cell_size=1.0)
    g.update(np.array([0.5, np.nan, 0.5]),
             np.array([0.5, 0.5, 0.5]),
             np.array([-10.0, -3.0, np.nan]))
    assert g.query([0.5], [0.5])[0] == -10.0
