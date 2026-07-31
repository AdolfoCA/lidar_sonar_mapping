#!/usr/bin/env python3
"""Unit tests for the position (eq 31) and intensity (eq 34) factors.

These pin the two Gaussian factors of eq 24 against scipy.stats.norm with the
combined variances (ω²_c + ϱ_i and σ²_c + λ²·ϱ_i), and exercise the two
behaviours the contract makes load-bearing:

  * heteroscedastic widening — a far return (large ϱ_i) discriminates less:
    the gap between the best and worst class log-likelihood shrinks;
  * abstention vs lift-dropping — NaN h zeroes a POSITION row, but the
    INTENSITY factor only abstains on NaN intensity; with NaN h it drops the
    λ·h lift and keeps the base Gaussian.

The ClassSet is built directly (no YAML), so the factors are tested in
isolation from the loader.

Run:  cd src/sonar_map && python3 -m pytest test/test_factor_position_intensity.py -v
"""
import os
import sys

import numpy as np
import pytest
from scipy import stats

# Make the package importable regardless of cwd (test/ has no __init__.py).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sonar_map.dirichlet import factor_intensity, factor_position  # noqa: E402
from sonar_map.dirichlet.classes import FACTOR_NAMES, ClassSet  # noqa: E402


# ── fixture: hand-built K=2 (+other) class set ───────────────────────────────

def _make_cs(lambda_lift: float = 25.0) -> ClassSet:
    """seabed / wall / other — distinct height bands and intensity means."""
    return ClassSet(
        names=('seabed', 'wall', 'other'),
        nu=np.array([0.0, 1.5, 0.5]),
        omega2=np.array([0.05, 0.8, 4.0]),
        mu=np.array([30.0, 90.0, 60.0]),
        sigma2=np.array([100.0, 225.0, 900.0]),
        pi_l=np.array([0.05, 0.9, 0.5]),
        kappa=np.array([1.0, 6.0, 2.0]),
        gamma_rate=np.array([1.0, 0.5, 0.8]),
        rho=np.array([0.02, 0.1, 0.05]),
        shape_mean=np.zeros((3, 4)),
        shape_var=np.ones((3, 4)),
        lambda_lift=lambda_lift,
        tilt_rad=float(np.deg2rad(15.0)),
        aperture_rad=float(np.deg2rad(20.0)),
        other_weight=0.3,
        factors_enabled={f: True for f in FACTOR_NAMES},
    )


# ── scipy reference values ───────────────────────────────────────────────────

def test_position_matches_scipy_norm_with_combined_variance():
    """eq 31: log N(h; ν_c, ω²_c + ϱ_i) element-by-element."""
    cs = _make_cs()
    h = np.array([0.1, 1.2, -0.3, 2.7])
    rho = np.array([0.02, 0.5, 1.3, 0.0])           # ϱ = 0 (range → 0) is legal
    got = factor_position.log_lik(h, rho, cs)
    for i in range(h.size):
        for c in range(cs.n_classes):
            ref = stats.norm.logpdf(
                h[i], loc=cs.nu[c], scale=np.sqrt(cs.omega2[c] + rho[i]))
            assert got[i, c] == pytest.approx(ref, rel=1e-12), (i, c)


def test_intensity_matches_scipy_norm_with_lift_and_widening():
    """eq 34: log N(I; μ_c + λh, σ²_c + λ²ϱ_i) element-by-element."""
    cs = _make_cs(lambda_lift=25.0)
    inten = np.array([35.0, 80.0, 120.0])
    h = np.array([0.2, 1.0, -0.4])
    rho = np.array([0.1, 0.6, 0.0])
    got = factor_intensity.log_lik(inten, h, rho, cs)
    lam = cs.lambda_lift
    for i in range(inten.size):
        for c in range(cs.n_classes):
            ref = stats.norm.logpdf(
                inten[i],
                loc=cs.mu[c] + lam * h[i],
                scale=np.sqrt(cs.sigma2[c] + lam ** 2 * rho[i]))
            assert got[i, c] == pytest.approx(ref, rel=1e-12), (i, c)


# ── heteroscedastic widening ─────────────────────────────────────────────────

def test_position_far_return_discriminates_less():
    """Larger ϱ_i shrinks the best-vs-worst class log-lik gap (eq 31 widening):
    the height cue attenuates exactly where it is unreliable."""
    cs = _make_cs()
    h = np.array([0.0])                              # on the seabed band, off the wall band
    gap_near = np.ptp(factor_position.log_lik(h, np.array([0.01]), cs)[0])
    gap_far = np.ptp(factor_position.log_lik(h, np.array([50.0]), cs)[0])
    assert gap_far < gap_near
    # monotone in ϱ, not just two-point
    gaps = [np.ptp(factor_position.log_lik(h, np.array([r]), cs)[0])
            for r in (0.01, 0.1, 1.0, 10.0, 50.0)]
    assert all(a > b for a, b in zip(gaps, gaps[1:]))


def test_intensity_far_return_discriminates_less():
    """λ²ϱ_i widens eq 34 the same way: far returns lean on the base Gaussian."""
    cs = _make_cs(lambda_lift=25.0)
    inten, h = np.array([30.0]), np.array([0.0])     # matches seabed μ, off wall μ
    gap_near = np.ptp(factor_intensity.log_lik(inten, h, np.array([0.01]), cs)[0])
    gap_far = np.ptp(factor_intensity.log_lik(inten, h, np.array([50.0]), cs)[0])
    assert gap_far < gap_near


# ── abstention / lift-dropping ───────────────────────────────────────────────

def test_position_abstains_on_nan_h_or_nan_rho():
    """NaN in either gating channel zeroes that row ONLY; no NaN/-inf leaks."""
    cs = _make_cs()
    h = np.array([np.nan, 0.1, 0.1])
    rho = np.array([0.2, np.nan, 0.2])
    out = factor_position.log_lik(h, rho, cs)
    assert np.all(out[0] == 0.0)                     # NaN h
    assert np.all(out[1] == 0.0)                     # NaN ϱ
    assert np.all(out[2] != 0.0) and np.all(np.isfinite(out[2]))
    assert np.all(np.isfinite(out))


def test_intensity_abstains_only_on_nan_intensity():
    """NaN intensity → zero row; NaN h with finite intensity still evaluates."""
    cs = _make_cs()
    inten = np.array([np.nan, 55.0])
    h = np.array([0.3, 0.3])
    rho = np.array([0.1, 0.1])
    out = factor_intensity.log_lik(inten, h, rho, cs)
    assert np.all(out[0] == 0.0)
    assert np.all(out[1] != 0.0) and np.all(np.isfinite(out))


def test_intensity_nan_h_drops_lift_to_base_gaussian():
    """h = ϱ = NaN (bed grid cold): eq 34 degrades to N(I; μ_c, σ²_c) exactly —
    the intensity cue survives the bottom-grid warm-up."""
    cs = _make_cs(lambda_lift=25.0)
    inten = np.array([70.0])
    out = factor_intensity.log_lik(inten, np.array([np.nan]), np.array([np.nan]), cs)
    ref = stats.norm.logpdf(inten[0], loc=cs.mu, scale=np.sqrt(cs.sigma2))
    assert np.allclose(out[0], ref, rtol=1e-12)


def test_intensity_nan_h_finite_rho_keeps_widening():
    """h NaN but ϱ known (slant range measured, bed unknown): the lift is
    dropped from the MEAN but the λ²ϱ widening still applies to the VARIANCE —
    the two channels gate independently."""
    cs = _make_cs(lambda_lift=25.0)
    inten, rho = np.array([70.0]), np.array([0.4])
    out = factor_intensity.log_lik(inten, np.array([np.nan]), rho, cs)
    ref = stats.norm.logpdf(
        inten[0], loc=cs.mu,
        scale=np.sqrt(cs.sigma2 + cs.lambda_lift ** 2 * rho[0]))
    assert np.allclose(out[0], ref, rtol=1e-12)


# ── shapes ───────────────────────────────────────────────────────────────────

def test_shapes_are_n_by_kplus1():
    """K=2 predefined + trailing 'other' ⇒ (N, 3) from both factors."""
    cs = _make_cs()
    assert cs.k_predefined == 2 and cs.n_classes == 3
    n = 7
    h = np.linspace(-1.0, 2.0, n)
    rho = np.full(n, 0.3)
    inten = np.linspace(20.0, 120.0, n)
    pos = factor_position.log_lik(h, rho, cs)
    ints = factor_intensity.log_lik(inten, h, rho, cs)
    assert pos.shape == (n, 3) and pos.dtype == np.float64
    assert ints.shape == (n, 3) and ints.dtype == np.float64
