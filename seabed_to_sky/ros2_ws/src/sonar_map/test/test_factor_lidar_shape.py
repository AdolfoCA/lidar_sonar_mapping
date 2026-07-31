#!/usr/bin/env python3
"""Unit tests for the eq-24 hurdle factors: factor_lidar (eq 37) and
factor_shape (eq 42).

References are computed with scipy.stats (poisson / expon / norm) — i.e.
independently of the factors' gammaln / log1p implementations — plus
hand-assembled hurdle combinations. Also pinned: the ABSTENTION contract
(NaN gating channel → all-zero row; NaN magnitude channel → that term alone
dropped), the informative s=0 cases, and the 1/N_R de-duplication power.

Run:  cd src/sonar_map && python3 -m pytest test/test_factor_lidar_shape.py -q
"""
import os
import sys

import numpy as np
from scipy import stats
from scipy.special import logit

# Make the package importable regardless of cwd (test/ has no __init__.py).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sonar_map.dirichlet import factor_lidar, factor_shape  # noqa: E402
from sonar_map.dirichlet.classes import ClassSet  # noqa: E402

# Column indices in the synthetic inventory below ("other" LAST, as always).
SEABED, STRUCTURE, OTHER = 0, 1, 2


def _cs() -> ClassSet:
    """Synthetic 2-class inventory (+ catch-all), shaped exactly like the
    output of load_classes: a tight seabed-like class and a structure-like
    class (overhead LiDAR likely, big regions, deep shadows)."""
    return ClassSet(
        names=('seabedish', 'structureish', 'other'),
        nu=np.array([0.0, 1.2, 0.5]),
        omega2=np.array([0.20, 1.00, 2.0]) ** 2,
        mu=np.array([5000.0, 9000.0, 6000.0]),
        sigma2=np.array([1200.0, 2500.0, 4000.0]) ** 2,
        pi_l=np.array([0.05, 0.90, 0.5]),
        kappa=np.array([2.0, 8.0, 3.0]),
        gamma_rate=np.array([2.0, 0.8, 0.5]),
        rho=np.array([0.03, 0.70, 0.5]),
        # columns: logarea, solidity_logit, shadow, texture (SHAPE_CHANNELS)
        shape_mean=np.array([[-1.5, 0.0, 0.10, 12.0],
                             [0.7, 1.0, 0.75, 15.0],
                             [0.0, 0.0, 0.50, 13.0]]),
        shape_var=np.array([[1.0, 1.5, 0.20, 3.0],
                            [0.9, 1.2, 0.15, 2.5],
                            [2.0, 2.0, 0.35, 5.0]]) ** 2,
        lambda_lift=0.0,
        tilt_rad=float(np.deg2rad(15.0)),
        aperture_rad=float(np.deg2rad(20.0)),
        other_weight=1.0,
        factors_enabled={'position': True, 'intensity': True,
                         'lidar': True, 'shape': True},
    )


# ── lidar factor (eq 37) ─────────────────────────────────────────────────────

def _ztpois_ref(k, cs):
    """log Pois>0(k; κ_c) per eq 38, via scipy's plain Poisson."""
    return (stats.poisson.logpmf(k, cs.kappa)
            - np.log(1.0 - stats.poisson.pmf(0, cs.kappa)))


def test_lidar_s0_rows_equal_log_one_minus_pi():
    cs = _cs()
    # gap channel must be irrelevant when nothing is overhead
    out = factor_lidar.log_lik(np.array([0.0, 0.0]),
                               np.array([np.nan, 0.4]), cs)
    assert out.shape == (2, 3)
    assert np.array_equal(out, np.tile(np.log1p(-cs.pi_l), (2, 1)))
    assert np.allclose(out, np.log(1.0 - cs.pi_l), rtol=1e-12, atol=0.0)


def test_lidar_s1_matches_scipy_reference():
    cs = _cs()
    k = np.array([1.0, 3.0, 7.0])
    g = np.array([0.05, 0.7, 2.3])
    out = factor_lidar.log_lik(k, g, cs)
    for i in range(k.size):
        ref = (np.log(cs.pi_l)
               + _ztpois_ref(k[i], cs)
               + stats.expon.logpdf(g[i], scale=1.0 / cs.gamma_rate))
        assert np.allclose(out[i], ref, rtol=1e-10, atol=1e-12)
    assert np.isfinite(out).all()


def test_lidar_abstains_on_nan_over_count():
    cs = _cs()
    out = factor_lidar.log_lik(np.array([np.nan, np.nan]),
                               np.array([0.3, np.nan]), cs)
    assert np.array_equal(out, np.zeros((2, 3)))


def test_lidar_nan_gap_drops_gap_term_only():
    cs = _cs()
    out = factor_lidar.log_lik(np.array([2.0, 2.0]),
                               np.array([np.nan, 0.6]), cs)
    ref_nogap = np.log(cs.pi_l) + _ztpois_ref(2.0, cs)
    gap_term = stats.expon.logpdf(0.6, scale=1.0 / cs.gamma_rate)
    assert np.allclose(out[0], ref_nogap, rtol=1e-10, atol=1e-12)
    assert np.allclose(out[1], ref_nogap + gap_term, rtol=1e-10, atol=1e-12)
    assert np.isfinite(out).all()


def test_lidar_mixed_batch_rows_are_independent():
    """abstain / s=0 / s=1 in one call — each row per its own regime."""
    cs = _cs()
    out = factor_lidar.log_lik(np.array([np.nan, 0.0, 4.0]),
                               np.array([1.0, 1.0, 1.0]), cs)
    assert np.array_equal(out[0], np.zeros(3))
    assert np.array_equal(out[1], np.log1p(-cs.pi_l))
    ref = (np.log(cs.pi_l) + _ztpois_ref(4.0, cs)
           + stats.expon.logpdf(1.0, scale=1.0 / cs.gamma_rate))
    assert np.allclose(out[2], ref, rtol=1e-10, atol=1e-12)


# ── shape factor (eq 42) ─────────────────────────────────────────────────────

def _shape_ref_row(cs, logarea, sol, shadow, texture, nr, keep=(0, 1, 2, 3)):
    """log(ρ) + Σ_kept norm.logpdf / max(N_R, 1) — eq 42 assembled by hand."""
    x = np.array([logarea, logit(np.clip(sol, 1e-6, 1.0 - 1e-6)),
                  shadow, texture])
    terms = stats.norm.logpdf(x[None, :], loc=cs.shape_mean,
                              scale=np.sqrt(cs.shape_var))   # (K+1, 4)
    return np.log(cs.rho) + terms[:, list(keep)].sum(axis=1) / max(nr, 1.0)


def test_shape_s0_rows_equal_log_one_minus_rho():
    cs = _cs()
    out = factor_shape.log_lik(np.array([0.0, 0.0]),      # shape_flag
                               np.array([np.nan, 3.0]),   # region_size
                               np.array([np.nan, 0.1]),   # logarea
                               np.array([np.nan, 0.5]),   # solidity
                               np.array([np.nan, 0.2]),   # shadow
                               np.array([np.nan, 13.0]),  # texture
                               cs)
    assert out.shape == (2, 3)
    assert np.array_equal(out, np.tile(np.log1p(-cs.rho), (2, 1)))
    assert np.allclose(out, np.log(1.0 - cs.rho), rtol=1e-12, atol=0.0)


def test_shape_s1_matches_scipy_reference():
    cs = _cs()
    out = factor_shape.log_lik(np.array([1.0]), np.array([7.0]),
                               np.array([0.5]), np.array([0.8]),
                               np.array([0.6]), np.array([14.0]), cs)
    ref = _shape_ref_row(cs, 0.5, 0.8, 0.6, 14.0, nr=7.0)
    assert np.allclose(out[0], ref, rtol=1e-10, atol=1e-12)
    assert np.isfinite(out).all()


def test_shape_dedup_power_scales_with_region_size():
    """The descriptor part must shrink as 1/N_R (eq 42); size < 1 floors at 1."""
    cs = _cs()
    args = (np.array([1.0, 1.0, 1.0]),                 # shape_flag
            np.array([1.0, 10.0, 0.0]),               # region_size (0 → floor 1)
            np.array([0.5, 0.5, 0.5]), np.array([0.8, 0.8, 0.8]),
            np.array([0.6, 0.6, 0.6]), np.array([14.0, 14.0, 14.0]))
    out = factor_shape.log_lik(*args, cs)
    desc1 = out[0] - np.log(cs.rho)
    desc10 = out[1] - np.log(cs.rho)
    assert np.allclose(desc10, desc1 / 10.0, rtol=1e-10, atol=1e-12)
    assert np.allclose(out[2], out[0], rtol=0.0, atol=0.0)


def test_shape_structure_region_prefers_structure_class():
    """A big, solid region with a deep shadow must score higher under the
    structure-like class than under the seabed-like class — both through the
    presence odds (ρ) and the descriptor Gaussians."""
    cs = _cs()
    out = factor_shape.log_lik(np.array([1.0]), np.array([20.0]),
                               np.array([1.0]),    # ~2.7 m² region
                               np.array([0.75]),
                               np.array([0.85]),   # deep, long, sharp shadow
                               np.array([15.0]), cs)
    assert out[0, STRUCTURE] > out[0, SEABED]
    # the bare seabed return, conversely, is better explained by seabedish
    out0 = factor_shape.log_lik(np.array([0.0]), np.array([np.nan]),
                                np.array([np.nan]), np.array([np.nan]),
                                np.array([np.nan]), np.array([np.nan]), cs)
    assert out0[0, SEABED] > out0[0, STRUCTURE]


def test_shape_abstains_on_nan_shape_flag():
    cs = _cs()
    out = factor_shape.log_lik(np.array([np.nan, np.nan]),
                               np.array([5.0, np.nan]),
                               np.array([0.2, np.nan]),
                               np.array([0.6, np.nan]),
                               np.array([0.4, np.nan]),
                               np.array([13.0, np.nan]), cs)
    assert np.array_equal(out, np.zeros((2, 3)))


def test_shape_nan_texture_drops_texture_term_only():
    cs = _cs()
    out = factor_shape.log_lik(np.array([1.0, 1.0]), np.array([5.0, 5.0]),
                               np.array([0.2, 0.2]), np.array([0.6, 0.6]),
                               np.array([0.4, 0.4]),
                               np.array([np.nan, 13.0]), cs)
    ref3 = _shape_ref_row(cs, 0.2, 0.6, 0.4, 0.0, nr=5.0, keep=(0, 1, 2))
    ref4 = _shape_ref_row(cs, 0.2, 0.6, 0.4, 13.0, nr=5.0)
    assert np.allclose(out[0], ref3, rtol=1e-10, atol=1e-12)
    assert np.allclose(out[1], ref4, rtol=1e-10, atol=1e-12)
    assert np.isfinite(out).all()
