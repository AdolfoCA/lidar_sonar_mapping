"""
soft_interval — the soft-box density B(x; l, u, sig) of eq 23.

    B(x; l, u, sig) = [ Phi((x-l)/sig) - Phi((x-u)/sig) ] / (u - l)      (23)

A plateau over [l, u] with Gaussian shoulders of width sig: the convolution of
a uniform occupancy interval with Gaussian blur. ONE family serves two
channels — the height occupancy of the position factor (eq 24) and the
"beneath cover" clearance of the LiDAR factor (eq 29) — which is the whole
point of introducing it.

WHY A BOX AND NOT A GAUSSIAN. A class owns a height *interval*, not a typical
height. The top of a canopy and its base are both fully consistent with
vegetation; a return anywhere on a wall between bed and surface is fully
consistent with the wall. A mid-extent Gaussian would assert a fictitious
typical height and punish a legitimate base-of-canopy return at h ~ 0
quadratically. The box is flat across the extent and decisive only at the
edges — exactly the shape that lets the spatial prediction resolve the
genuinely ambiguous low returns instead of the likelihood overruling them.

TWO LIMITS (asserted in the tests) show the family is not ad hoc:
  * u -> l   degenerates exactly to N(x; l, sig^2). The seabed is not a
    special case, it is the zero-extent member of the same family.
  * sig -> 0 recovers the hard uniform on [l, u].

NUMERICS. Two failure modes are handled explicitly, because the naive
expression hits both on real data:

  1. CANCELLATION. For x well above u both CDFs are ~1 and their difference
     loses every significant digit. When both standardised bounds are
     positive we evaluate the complementary form Phi(-b) - Phi(-a), whose
     terms are small and whose difference keeps full relative accuracy.
  2. UNDERFLOW. Beyond ~38 sigma the tail probability underflows to exactly
     0.0 and log() would return -inf, permanently vetoing that class in that
     voxel. Past that point we switch to the closed-form tail asymptote
     (``_log_tail``), which stays finite, monotone in |x|, and agrees with
     both limits above.
"""

import numpy as np
from scipy.special import ndtr

_LOG_2PI = float(np.log(2.0 * np.pi))
# Standardised distance beyond which Phi's tail underflows in float64.
_TAIL_SWITCH = 37.0


def _phi_diff(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Phi(a) - Phi(b) for a >= b, without catastrophic cancellation.

    Both bounds positive (x above the interval) is the cancellation case:
    1 - Phi(-a) - (1 - Phi(-b)) = Phi(-b) - Phi(-a), whose terms are the small
    upper-tail masses. Every other sign pattern is well conditioned directly.
    """
    upper = (a > 0.0) & (b > 0.0)
    return np.where(upper, ndtr(-b) - ndtr(-a), ndtr(a) - ndtr(b))


def _log_tail(t: np.ndarray, width_sig: np.ndarray, log_sigma: np.ndarray) -> np.ndarray:
    """log B in the far tail, where the exact difference has underflowed to 0.

    ``t`` is the distance from x to the NEAREST interval edge in units of sig
    (always >= _TAIL_SWITCH here) and ``width_sig`` = (u - l)/sig. Using the
    Mills-ratio asymptote Q(t) ~ phi(t)/t for the dominant edge,

        log B  ~  log phi(t) - log sig - log( max(1, t * width_sig) )

    which reproduces both limits of the family: as width_sig -> 0 the last
    term vanishes and this is exactly log N(x; edge, sig^2); for a wide
    interval it becomes log phi(t) - log t - log(u - l), the Mills asymptote
    of eq 23 itself. Finite and monotone decreasing in t either way.
    """
    log_phi = -0.5 * (t * t + _LOG_2PI)
    return log_phi - log_sigma - np.log(np.maximum(1.0, t * width_sig))


def log_soft_interval(x, l, u, sigma) -> np.ndarray:
    """log B(x; l, u, sig) (eq 23), broadcast over its arguments.

    Parameters
    ----------
    x     : values at which to evaluate (any broadcastable shape).
    l, u  : interval bounds, u >= l. u == l is the Gaussian limit and is
            handled exactly (not as a degenerate 0/0).
    sigma : shoulder width > 0. Combines class softness and per-return
            geometric spread in quadrature at the CALL SITE, never inside a
            stored parameter.

    Returns
    -------
    float64 array of the broadcast shape. Always finite.
    """
    x = np.asarray(x, dtype=np.float64)
    l = np.asarray(l, dtype=np.float64)
    u = np.asarray(u, dtype=np.float64)
    sigma = np.asarray(sigma, dtype=np.float64)

    x, l, u, sigma = np.broadcast_arrays(x, l, u, sigma)
    sigma = np.maximum(sigma, 1e-12)
    log_sigma = np.log(sigma)

    a = (x - l) / sigma          # standardised distance to the lower edge
    b = (x - u) / sigma          # ... to the upper edge; b <= a always
    width_sig = (u - l) / sigma  # interval width in sigma units

    # Distance to the nearest edge, in sigma. Zero while x is inside [l, u].
    t = np.maximum(np.maximum(b, -a), 0.0)

    # ── Gaussian limit: a zero-width interval is N(x; l, sigma^2) ─────────────
    degenerate = width_sig <= 1e-9
    out = np.empty(x.shape, dtype=np.float64)
    if np.any(degenerate):
        z = a[degenerate]
        out[degenerate] = -0.5 * (z * z + _LOG_2PI) - log_sigma[degenerate]

    # ── Exact soft box wherever the CDF difference still carries digits ──────
    body = ~degenerate & (t < _TAIL_SWITCH)
    if np.any(body):
        diff = _phi_diff(a[body], b[body])
        # A residual zero here can only come from an interval so narrow that
        # the difference underflows despite t being small; the tail form is
        # the correct continuation in exactly that case too.
        safe = diff > 0.0
        idx = np.flatnonzero(body)
        ok, bad = idx[safe], idx[~safe]
        out.flat[ok] = (np.log(diff[safe])
                        - np.log(u.flat[ok] - l.flat[ok]))
        if bad.size:
            out.flat[bad] = _log_tail(t.flat[bad], width_sig.flat[bad],
                                      log_sigma.flat[bad])

    # ── Far tail: exact difference has underflowed to 0.0 ────────────────────
    tail = ~degenerate & (t >= _TAIL_SWITCH)
    if np.any(tail):
        out[tail] = _log_tail(t[tail], width_sig[tail], log_sigma[tail])

    return out


def soft_interval(x, l, u, sigma) -> np.ndarray:
    """B(x; l, u, sig) (eq 23) in the linear domain — tests and plots only.

    Everything in the pipeline works in log space (I5); this is the readable
    reference the unit tests compare the log form against.
    """
    return np.exp(log_soft_interval(x, l, u, sigma))
