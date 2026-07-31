#!/usr/bin/env python3
"""Unit tests for sonar_map.conjugate — the conjugate-family math library.

These pin the accumulation recursions (paper eqs 47–50) and the posterior
predictives (eq 48 + the NegBinom/Lomax/Beta-Bernoulli counterparts) against
independent scipy.stats references and against batch/naive recomputations.

The library itself uses NO scipy (math.lgamma only), so agreement here is a
genuine cross-check rather than a tautology.

Run:  cd src/sonar_map && python3 -m pytest test/test_conjugate.py -v
"""
import os
import sys

import numpy as np
import pytest
from scipy import stats

# Make the package importable regardless of cwd (test/ has no __init__.py).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sonar_map.conjugate import NIG, GammaRate, BetaBernoulli  # noqa: E402


# ── NIG ──────────────────────────────────────────────────────────────────────

def _nig_prior():
    return NIG(m=2.0, kappa=0.5, a=1.5, b=3.0)


def test_nig_recursive_equals_batch():
    """Sequential eq-47 updates == the Welford batch combination (any order)."""
    rng = np.random.default_rng(0)
    x = rng.normal(5.0, 2.0, size=37)
    w = rng.uniform(0.1, 1.5, size=37)

    seq = _nig_prior()
    for xi, wi in zip(x, w):
        seq.update(xi, wi)

    bat = _nig_prior()
    bat.batch_update(x, w)

    for attr in ('m', 'kappa', 'a', 'b'):
        assert getattr(seq, attr) == pytest.approx(getattr(bat, attr), rel=1e-10)


def test_nig_predictive_matches_scipy_t():
    """log_predictive == Student-t logpdf with df=2a, loc=m, scale²=b(κ+1)/(aκ)."""
    nig = _nig_prior()
    nig.batch_update(np.array([4.0, 4.5, 3.5, 5.0]))
    df = 2.0 * nig.a
    scale = np.sqrt(nig.b * (nig.kappa + 1.0) / (nig.a * nig.kappa))
    x = np.array([-3.0, 0.0, 2.0, 4.1, 12.0])
    ref = stats.t.logpdf(x, df=df, loc=nig.m, scale=scale)
    got = nig.log_predictive(x)
    assert np.allclose(got, ref, rtol=1e-10, atol=1e-12)
    # scalar path too
    assert nig.log_predictive(4.1) == pytest.approx(float(ref[3]), rel=1e-10)


def test_nig_scatter_matches_naive_sum_of_squares():
    """The eq-47 stable scatter equals the closed-form batch scatter, itself
    equal to a naive Σx²−Nx̄² recomputation."""
    x = np.array([9.0, 11.0, 10.0, 8.0, 12.0])
    m0, k0, a0, b0 = 1.0, 0.3, 2.0, 5.0
    nig = NIG(m0, k0, a0, b0)
    nig.batch_update(x)

    N = x.size
    xbar = x.mean()
    S_direct = float(np.sum(x ** 2) - N * xbar ** 2)      # naive, cancellation-prone
    kp = k0 + N
    b_expected = b0 + 0.5 * S_direct + 0.5 * (k0 * N / kp) * (xbar - m0) ** 2
    assert nig.b == pytest.approx(b_expected, rel=1e-10)


def test_nig_weight_two_equals_point_twice():
    a = _nig_prior(); a.update(7.0, 2.0)
    b = _nig_prior(); b.update(7.0, 1.0); b.update(7.0, 1.0)
    for attr in ('m', 'kappa', 'a', 'b'):
        assert getattr(a, attr) == pytest.approx(getattr(b, attr), rel=1e-12)


def test_nig_zero_and_negative_weight_are_noops():
    for w in (0.0, -1.0):
        nig = _nig_prior()
        before = (nig.m, nig.kappa, nig.a, nig.b)
        nig.update(1000.0, w)
        assert (nig.m, nig.kappa, nig.a, nig.b) == before
    # batch with all-zero weights is also a no-op
    nig = _nig_prior()
    before = (nig.m, nig.kappa, nig.a, nig.b)
    nig.batch_update([1.0, 2.0, 3.0], [0.0, 0.0, 0.0])
    assert (nig.m, nig.kappa, nig.a, nig.b) == before


def test_nig_confirmation_budget_mean_shift():
    """A larger prior pseudo-count κ0 damps the shift from one observation."""
    weak = NIG(0.0, 0.5, 2.0, 2.0); weak.update(10.0, 1.0)
    strong = NIG(0.0, 50.0, 2.0, 2.0); strong.update(10.0, 1.0)
    assert abs(weak.m - 0.0) > abs(strong.m - 0.0)
    # exact: Δm = (x-m)·w/(κ+w)
    assert weak.m == pytest.approx(10.0 * 1.0 / (0.5 + 1.0), rel=1e-12)


def test_nig_fresh_class_scores_novel_returns_cautiously():
    """Heavy-tailed predictive: a fresh (small-a) class assigns MORE density to a
    far return than a sharp, well-fed class — the paper's cautious novelty."""
    fresh = NIG(0.0, 1.0, 1.0, 1.0)
    sharp = NIG(0.0, 1.0, 1.0, 1.0)
    sharp.batch_update(np.zeros(200))                       # concentrate at 0
    far = 10.0
    assert fresh.log_predictive(far) > sharp.log_predictive(far)


def test_nig_converges_to_generating_gaussian():
    rng = np.random.default_rng(7)
    mu, sig = -3.0, 1.7
    x = rng.normal(mu, sig, size=20000)
    nig = NIG(0.0, 1e-3, 1e-3, 1e-3)                        # diffuse prior
    nig.batch_update(x)
    assert nig.mean == pytest.approx(mu, abs=0.05)
    assert np.sqrt(nig.var) == pytest.approx(sig, rel=0.03)


# ── GammaRate ────────────────────────────────────────────────────────────────

def test_gamma_count_predictive_matches_nbinom():
    g = GammaRate(a=3.0, b=2.0, kind='count')
    k = np.array([0, 1, 2, 5, 9], dtype=float)
    ref = stats.nbinom.logpmf(k, n=g.a, p=g.b / (g.b + 1.0))
    assert np.allclose(g.log_predictive(k), ref, rtol=1e-10, atol=1e-12)


def test_gamma_gap_predictive_matches_lomax():
    g = GammaRate(a=2.5, b=4.0, kind='gap')
    x = np.array([0.0, 0.5, 1.0, 3.0, 8.0])
    ref = stats.lomax.logpdf(x, c=g.a, scale=g.b)
    assert np.allclose(g.log_predictive(x), ref, rtol=1e-10, atol=1e-12)


@pytest.mark.parametrize('kind', ['count', 'gap'])
def test_gamma_recursive_equals_batch_and_weight2(kind):
    rng = np.random.default_rng(1)
    x = rng.integers(0, 8, size=25).astype(float) if kind == 'count' \
        else rng.uniform(0.0, 5.0, size=25)
    w = rng.uniform(0.1, 2.0, size=25)

    seq = GammaRate(1.0, 1.0, kind)
    for xi, wi in zip(x, w):
        seq.update(xi, wi)
    bat = GammaRate(1.0, 1.0, kind)
    bat.batch_update(x, w)
    assert seq.a == pytest.approx(bat.a, rel=1e-12)
    assert seq.b == pytest.approx(bat.b, rel=1e-12)

    one = GammaRate(1.0, 1.0, kind); one.update(3.0, 2.0)
    two = GammaRate(1.0, 1.0, kind); two.update(3.0, 1.0); two.update(3.0, 1.0)
    assert (one.a, one.b) == pytest.approx((two.a, two.b), rel=1e-12)


def test_gamma_zero_weight_noop():
    g = GammaRate(2.0, 3.0, 'count')
    g.update(100.0, 0.0)
    assert (g.a, g.b) == (2.0, 3.0)


# ── BetaBernoulli ────────────────────────────────────────────────────────────

def test_beta_predictive_and_estimate():
    beta = BetaBernoulli(u=2.0, v=3.0)
    assert beta.prob == pytest.approx(2.0 / 5.0)
    assert beta.log_predictive(1) == pytest.approx(np.log(2.0 / 5.0), rel=1e-12)
    assert beta.log_predictive(0) == pytest.approx(np.log(3.0 / 5.0), rel=1e-12)
    got = beta.log_predictive(np.array([1.0, 0.0, 1.0]))
    assert np.allclose(got, np.log([0.4, 0.6, 0.4]), rtol=1e-12)


def test_beta_update_batch_weight_and_noop():
    b1 = BetaBernoulli(1.0, 1.0)
    b1.update(1.0, 2.0)
    b2 = BetaBernoulli(1.0, 1.0)
    b2.update(1.0, 1.0); b2.update(1.0, 1.0)
    assert (b1.u, b1.v) == pytest.approx((b2.u, b2.v), rel=1e-12)

    seq = BetaBernoulli(0.5, 0.5)
    s = np.array([1, 0, 1, 1, 0], dtype=float)
    w = np.array([1.0, 0.5, 2.0, 0.3, 1.1])
    for si, wi in zip(s, w):
        seq.update(si, wi)
    bat = BetaBernoulli(0.5, 0.5); bat.batch_update(s, w)
    assert (seq.u, seq.v) == pytest.approx((bat.u, bat.v), rel=1e-12)

    beta = BetaBernoulli(2.0, 2.0); beta.update(1.0, 0.0)
    assert (beta.u, beta.v) == (2.0, 2.0)
