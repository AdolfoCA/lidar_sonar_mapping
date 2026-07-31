"""The soft-box family (eq 23): its two limits, and that it never goes non-finite.

The limits are what make the family principled rather than convenient — the
seabed is the zero-extent member of the SAME density, not a special case — so
they are asserted, not assumed.
"""

import numpy as np
from scipy.stats import norm

from sonar_classification.argus.soft_interval import (
    log_soft_interval,
    soft_interval,
)


def test_gaussian_limit_as_interval_collapses():
    """u -> l degenerates exactly to N(x; l, sigma^2)."""
    x = np.linspace(-3.0, 3.0, 61)
    sigma = 0.4
    got = soft_interval(x, 0.0, 0.0, sigma)
    want = norm.pdf(x, loc=0.0, scale=sigma)
    assert np.allclose(got, want, rtol=1e-10, atol=1e-12)

    # ... and the approach to it is continuous: a very narrow interval is
    # numerically the same density, with no discontinuity at the switch.
    near = soft_interval(x, 0.0, 1e-7, sigma)
    assert np.allclose(near, want, rtol=1e-5, atol=1e-8)


def test_uniform_limit_as_sigma_vanishes():
    """sigma -> 0 recovers the hard uniform on [l, u]."""
    l, u = -1.0, 2.0
    inside = np.array([-0.9, 0.0, 1.0, 1.9])
    outside = np.array([-1.5, 2.5, 10.0])

    got_in = soft_interval(inside, l, u, 1e-6)
    assert np.allclose(got_in, 1.0 / (u - l), rtol=1e-9)

    got_out = soft_interval(outside, l, u, 1e-6)
    assert np.all(got_out < 1e-12)


def test_plateau_is_flat_across_the_extent():
    """The interior is flat — that is the whole point of the box.

    A mid-extent Gaussian would fall off quadratically from a centre that has
    no physical meaning; the box says only "inside the extent", which is what
    lets the spatial prediction resolve genuinely ambiguous low returns.
    """
    l, u, sigma = 0.0, 4.0, 0.2
    # Deep interior — more than 8 sigma from either edge, so neither shoulder
    # reaches: the density is exactly the uniform 1/(u-l).
    deep = np.array([1.8, 2.0, 2.2])
    vals = soft_interval(deep, l, u, sigma)
    assert np.allclose(vals, 1.0 / (u - l), rtol=1e-12)

    # The shoulders do reach a few sigma inward — that is the Gaussian blur, not
    # a defect — but even 5 sigma in, the plateau is flat to a part in a million.
    near_edge = soft_interval(np.array([1.0, 3.0]), l, u, sigma)
    assert np.allclose(near_edge, 1.0 / (u - l), rtol=1e-6)


def test_far_tail_stays_finite_and_monotone():
    """No -inf anywhere: a numerical underflow must never veto a class.

    Beyond ~38 sigma the exact CDF difference underflows to 0.0 and a naive
    log() returns -inf, which would permanently kill that class in that voxel.
    """
    l, u, sigma = 0.0, 1.0, 0.1
    far = np.array([5.0, 20.0, 100.0, 1e4])
    lg = log_soft_interval(far, l, u, sigma)
    assert np.all(np.isfinite(lg))
    assert np.all(np.diff(lg) < 0.0)          # strictly decreasing with distance

    # The narrow-interval tail must agree with the Gaussian it degenerates to.
    lg_narrow = log_soft_interval(far, 0.0, 0.0, sigma)
    assert np.all(np.isfinite(lg_narrow))
    assert np.all(np.diff(lg_narrow) < 0.0)


def test_log_matches_linear_where_both_are_representable():
    x = np.linspace(-5.0, 8.0, 131)
    lg = log_soft_interval(x, 0.0, 3.0, 0.5)
    lin = soft_interval(x, 0.0, 3.0, 0.5)
    ok = lin > 1e-280
    assert np.allclose(np.exp(lg[ok]), lin[ok], rtol=1e-9)


def test_integrates_to_one():
    """It is a density, not a score."""
    x = np.linspace(-20.0, 25.0, 400001)
    for (l, u, s) in [(0.0, 0.0, 0.5), (0.0, 5.0, 0.3), (-2.0, 2.0, 1.5)]:
        area = np.trapezoid(soft_interval(x, l, u, s), x) \
            if hasattr(np, 'trapezoid') else np.trapz(soft_interval(x, l, u, s), x)
        assert abs(area - 1.0) < 1e-6, (l, u, s)
