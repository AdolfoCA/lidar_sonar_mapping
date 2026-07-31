#!/usr/bin/env python3
"""Unit tests for sonar_map.dirichlet.likelihood — the eq-24 assembler.

These pin the assembler's contract rather than the factor math (each factor
module has its own test file):

  * total == Σ of the four factor log-matrices, plus log(other_weight) on the
    trailing "other" column only (the paper's γ, Alg. 1 line 9 — applied ONCE
    on the total, never inside a factor);
  * a factor disabled via cs.factors_enabled contributes a ZERO matrix but
    keeps its key in by_factor (stable telemetry rows across ablations);
  * responsibilities (eq 16) is a numerically stable row softmax: rows sum
    to 1, match a hand-computed softmax, survive |log L| ~ 1e4, and accept
    both a shared (K+1,) and a per-return (N, K+1) log-prior;
  * end-to-end on the real config/classes.yaml: an outlandish return lands
    on 'other' under a uniform prior (the catch-all actually catches).

Run:  cd src/sonar_map && python3 -m pytest test/test_likelihood.py -v
"""
import os
import sys

import numpy as np
import pytest

# Make the package importable regardless of cwd (test/ has no __init__.py).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sonar_map.dirichlet import (  # noqa: E402
    factor_intensity,
    factor_lidar,
    factor_position,
    factor_shape,
)
from sonar_map.dirichlet.classes import FACTOR_NAMES, ClassSet, load_classes  # noqa: E402
from sonar_map.dirichlet.likelihood import (  # noqa: E402
    Features,
    log_likelihood,
    responsibilities,
)

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'config', 'classes.yaml')


# ── fixtures: a tiny hand-built ClassSet and a mixed feature batch ───────────

def _tiny_cs(factors_enabled=None, other_weight=2.0):
    """Two predefined classes + 'other' (K+1 = 3), all parameters by hand.

    other_weight deliberately != 1 so a missing/misplaced log(other_weight)
    term is caught by the equality tests below.
    """
    if factors_enabled is None:
        factors_enabled = {name: True for name in FACTOR_NAMES}
    return ClassSet(
        names=('flat', 'bump', 'other'),
        nu=np.array([0.0, 0.4, 0.5]),
        omega2=np.array([0.04, 0.09, 4.0]),
        mu=np.array([5000.0, 7000.0, 6000.0]),
        sigma2=np.array([1200.0, 1500.0, 4000.0]) ** 2,
        pi_l=np.array([0.05, 0.40, 0.50]),
        kappa=np.array([2.0, 3.0, 3.0]),
        gamma_rate=np.array([2.0, 1.0, 0.5]),
        rho=np.array([0.05, 0.50, 0.50]),
        shape_mean=np.array([[-1.5, 0.0, 0.1, 12.0],
                             [-0.7, 1.5, 0.35, 14.0],
                             [0.0, 0.0, 0.5, 13.0]]),
        shape_var=np.array([[1.0, 1.5, 0.2, 3.0],
                            [0.8, 1.0, 0.2, 2.5],
                            [2.0, 2.0, 0.35, 5.0]]) ** 2,
        lambda_lift=150.0,
        tilt_rad=float(np.deg2rad(15.0)),
        aperture_rad=float(np.deg2rad(20.0)),
        other_weight=other_weight,
        factors_enabled=factors_enabled,
    )


def _mixed_features():
    """Four returns exercising every abstention path the assembler forwards:
    0 fully observed region return, 1 bare return (s=0 hurdles), 2 cold
    bottom grid + no intensity, 3 no LiDAR cloud / no shape extractor."""
    return Features(
        h=np.array([0.10, 0.50, np.nan, 2.00]),
        rho=np.array([0.05, 0.20, 0.10, np.nan]),
        intensity=np.array([5200.0, 7100.0, np.nan, 9000.0]),
        over_count=np.array([2.0, 0.0, 5.0, np.nan]),
        over_gap=np.array([0.5, np.nan, np.nan, 1.0]),
        shape_flag=np.array([1.0, 0.0, 1.0, np.nan]),
        region_size=np.array([3.0, 0.0, 2.0, np.nan]),
        region_logarea=np.array([-0.5, np.nan, 0.4, np.nan]),
        region_solidity=np.array([0.8, np.nan, np.nan, np.nan]),
        region_shadow=np.array([0.3, np.nan, 0.6, np.nan]),
        region_texture=np.array([13.0, np.nan, 12.0, np.nan]),
    )


def _direct_factor_matrices(f, cs):
    """The four factor matrices computed WITHOUT the assembler."""
    return {
        'position': factor_position.log_lik(f.h, f.rho, cs),
        'intensity': factor_intensity.log_lik(f.intensity, f.h, f.rho, cs),
        'lidar': factor_lidar.log_lik(f.over_count, f.over_gap, cs),
        'shape': factor_shape.log_lik(f.shape_flag, f.region_size,
                                      f.region_logarea, f.region_solidity,
                                      f.region_shadow, f.region_texture, cs),
    }


# ── log_likelihood: eq-24 assembly ───────────────────────────────────────────

def test_total_is_sum_of_factors_plus_other_weight():
    """total == Σ factor matrices, + log(other_weight) on the LAST column only."""
    cs = _tiny_cs(other_weight=2.0)
    f = _mixed_features()

    total, by_factor = log_likelihood(f, cs)
    direct = _direct_factor_matrices(f, cs)

    expected = sum(direct[name] for name in FACTOR_NAMES)
    expected[:, -1] += np.log(cs.other_weight)

    assert total.shape == (4, cs.n_classes)
    assert total.dtype == np.float64
    assert np.allclose(total, expected, rtol=1e-12, atol=1e-12)
    # design rule 3: abstention is zeros, never NaN/-inf
    assert np.isfinite(total).all()

    # by_factor mirrors the direct calls exactly, in FACTOR_NAMES order
    assert tuple(by_factor.keys()) == FACTOR_NAMES
    for name in FACTOR_NAMES:
        assert np.allclose(by_factor[name], direct[name], rtol=1e-12, atol=1e-12)


def test_other_weight_applied_once_on_total_not_per_factor():
    """γ shifts only the total's 'other' column; by_factor is γ-free."""
    f = _mixed_features()
    t1, by1 = log_likelihood(f, _tiny_cs(other_weight=1.0))
    t3, by3 = log_likelihood(f, _tiny_cs(other_weight=3.0))

    for name in FACTOR_NAMES:
        assert np.allclose(by1[name], by3[name])
    assert np.allclose(t3[:, :-1], t1[:, :-1])
    assert np.allclose(t3[:, -1] - t1[:, -1], np.log(3.0))


@pytest.mark.parametrize('disabled', FACTOR_NAMES)
def test_disabled_factor_is_zero_but_key_survives(disabled):
    """Disabling a factor zeroes its matrix, keeps its by_factor key, and
    the total is exactly the sum of the remaining factors (+ γ term)."""
    enabled = {name: name != disabled for name in FACTOR_NAMES}
    cs = _tiny_cs(factors_enabled=enabled)
    f = _mixed_features()

    total, by_factor = log_likelihood(f, cs)
    direct = _direct_factor_matrices(f, cs)

    assert tuple(by_factor.keys()) == FACTOR_NAMES          # key survives
    assert np.all(by_factor[disabled] == 0.0)               # contribution zeroed

    expected = sum(direct[n] for n in FACTOR_NAMES if n != disabled)
    expected[:, -1] += np.log(cs.other_weight)
    assert np.allclose(total, expected, rtol=1e-12, atol=1e-12)


# ── responsibilities: eq 16 as a stable row softmax ──────────────────────────

def test_responsibilities_match_hand_softmax():
    log_L = np.array([[0.0, 1.0, 2.0],
                      [3.0, 3.0, 3.0],
                      [-1.0, 0.5, 0.0]])
    log_prior = np.log(np.array([0.5, 0.3, 0.2]))

    r = responsibilities(log_L, log_prior)

    z = np.exp(log_L + log_prior)                            # safe: small logs
    expected = z / z.sum(axis=1, keepdims=True)
    assert np.allclose(r, expected, rtol=1e-12, atol=1e-15)
    assert np.allclose(r.sum(axis=1), 1.0, rtol=1e-12)


def test_responsibilities_rows_sum_to_one_random():
    rng = np.random.default_rng(7)
    log_L = rng.normal(scale=30.0, size=(50, 4))
    log_prior = rng.normal(scale=5.0, size=4)
    r = responsibilities(log_L, log_prior)
    assert np.all(r >= 0.0)
    assert np.allclose(r.sum(axis=1), 1.0, rtol=1e-12)


def test_responsibilities_stable_for_huge_log_magnitudes():
    """A naive exp() would overflow at +1e4 and flush to a 0/0 row at -1e4;
    the row-max-subtracted softmax must return finite one-hot-ish rows."""
    log_L = np.array([[1.0e4, 0.0, -1.0e4],
                      [-1.0e4, -1.0e4 + 1.0, -1.0e4],
                      [-1.0e4, -2.0e4, -3.0e4]])
    r = responsibilities(log_L, np.zeros(3))

    assert np.isfinite(r).all()
    assert np.allclose(r.sum(axis=1), 1.0, rtol=1e-12)
    assert r[0, 0] == pytest.approx(1.0)
    assert r[2, 0] == pytest.approx(1.0)
    # row 1 only shifts by a constant: must equal softmax([0, 1, 0])
    z = np.exp([0.0, 1.0, 0.0])
    assert np.allclose(r[1], z / z.sum(), rtol=1e-12)


def test_responsibilities_accepts_shared_and_per_return_priors():
    """(K+1,) and (N, K+1) log-priors both work; a tiled shared prior gives
    identical rows, and per-return rows match row-by-row evaluation."""
    rng = np.random.default_rng(11)
    log_L = rng.normal(scale=10.0, size=(6, 4))
    shared = rng.normal(scale=2.0, size=4)
    per_return = rng.normal(scale=2.0, size=(6, 4))

    r_shared = responsibilities(log_L, shared)
    r_tiled = responsibilities(log_L, np.tile(shared, (6, 1)))
    assert np.allclose(r_shared, r_tiled, rtol=1e-12)

    r_per = responsibilities(log_L, per_return)
    assert r_per.shape == (6, 4)
    assert np.allclose(r_per.sum(axis=1), 1.0, rtol=1e-12)
    for i in range(6):
        row = responsibilities(log_L[i:i + 1], per_return[i])
        assert np.allclose(r_per[i], row[0], rtol=1e-12)


def test_responsibilities_do_not_mutate_inputs():
    log_L = np.array([[1.0, 2.0, 3.0]])
    log_prior = np.array([0.1, 0.2, 0.3])
    keep_L, keep_p = log_L.copy(), log_prior.copy()
    responsibilities(log_L, log_prior)
    assert np.array_equal(log_L, keep_L)
    assert np.array_equal(log_prior, keep_p)


# ── end-to-end on the real class inventory ───────────────────────────────────

def test_outlandish_return_lands_on_other():
    """intensity 40000 (≈ 8σ beyond 'structure'), h = 5 m, k = 50 overhead
    points, 20 m gap: no predefined class explains ALL channels at once, so
    under a uniform prior the catch-all must claim the return (the base
    measure Λ_H doing its job, Alg. 1 line 8)."""
    cs = load_classes(CONFIG)
    nan = np.array([np.nan])
    f = Features(
        h=np.array([5.0]),
        rho=np.array([0.1]),
        intensity=np.array([40000.0]),
        over_count=np.array([50.0]),
        over_gap=np.array([20.0]),
        shape_flag=np.array([0.0]),       # bare return: shape hurdle s=0
        region_size=np.array([0.0]),
        region_logarea=nan, region_solidity=nan,
        region_shadow=nan, region_texture=nan,
    )

    total, by_factor = log_likelihood(f, cs)
    assert total.shape == (1, cs.n_classes)
    assert np.isfinite(total).all()
    assert tuple(by_factor.keys()) == FACTOR_NAMES

    uniform = np.full(cs.n_classes, -np.log(cs.n_classes))
    r = responsibilities(total, uniform)
    assert np.allclose(r.sum(axis=1), 1.0, rtol=1e-12)
    assert cs.names[int(np.argmax(r[0]))] == 'other'


def test_ordinary_seabed_return_does_not_land_on_other():
    """Counter-probe: a plain on-bottom return near the seabed intensity mean
    with no overhead LiDAR and no region must NOT be claimed by 'other'."""
    cs = load_classes(CONFIG)
    nan = np.array([np.nan])
    f = Features(
        h=np.array([0.0]),
        rho=np.array([0.1]),
        intensity=np.array([5000.0]),
        over_count=np.array([0.0]),
        over_gap=nan,
        shape_flag=np.array([0.0]),
        region_size=np.array([0.0]),
        region_logarea=nan, region_solidity=nan,
        region_shadow=nan, region_texture=nan,
    )
    total, _ = log_likelihood(f, cs)
    r = responsibilities(total, np.full(cs.n_classes, -np.log(cs.n_classes)))
    assert cs.names[int(np.argmax(r[0]))] == 'seabed'
