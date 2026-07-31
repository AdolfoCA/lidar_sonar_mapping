"""
factor_intensity — the intensity factor p(I_i|p_i,c) of eq 24, per eq 34.

Each class has a characteristic base backscatter μ_c, and a return higher
above the bottom lies closer to the array and is brighter. The height-lifted
Gaussian (eq 33) explains that geometric brightening away through the shared,
FIXED slope λ (a sensor-geometry effect, not a class property — only μ_c and
σ²_c are class parameters):

    p(I_i|p_i,c) = N(I_i; μ_c + λ·h_i, σ²_c)                          (eq 33)

The elevation ambiguity of the boresight height propagates into eq 33 through
the lift λ·h_i; marginalising it sharpens the factor to (with ĥ_i the
boresight height estimate — our h)

    p(I_i|p_i,c) = N(I_i; μ_c + λ·ĥ_i, σ²_c + λ²·ϱ_i)                 (eq 34)

so, as in the position factor, far returns lean on the base Gaussian rather
than on a height lift they cannot localise.

MISSING BOTTOM REFERENCE: h (and possibly ϱ) is NaN while the bed grid warms
up, but the return's intensity is still measured and still informative. The
lift is therefore DROPPED — NaN h contributes 0 to the mean, NaN ϱ contributes
0 to the variance — reducing eq 34 to the base Gaussian N(I_i; μ_c, σ²_c)
instead of abstaining: the intensity cue must not die before the bottom grid
has coverage.

ABSTENTION: only a NaN intensity makes the factor unevaluable; those rows are
all ZEROS (log-space identity — the remaining eq-24 factors still combine).
"""

import numpy as np

from sonar_map.dirichlet.classes import ClassSet

# ── constants ────────────────────────────────────────────────────────────────

_LOG_2PI = float(np.log(2.0 * np.pi))


# ── eq 34 ────────────────────────────────────────────────────────────────────

def log_lik(intensity: np.ndarray, h: np.ndarray, rho_i: np.ndarray,
            cs: ClassSet) -> np.ndarray:
    """(N, K+1) log N(I_i; μ_c + λ·ĥ_i, σ²_c + λ²·ϱ_i) (eq 34), abstained rows zeroed.

    Parameters
    ----------
    intensity : (N,) float64 — backscatter intensity I_i of the leading-edge
        return. NaN ⇒ the row abstains.
    h : (N,) float64 — boresight height above the bottom reference ĥ_i
        (eq 25). NaN ⇒ the lift λ·ĥ_i is dropped for that return (see module
        docstring), NOT an abstention.
    rho_i : (N,) float64 — per-return elevation variance ϱ_i ≥ 0 (eq 30).
        NaN ⇒ the λ²·ϱ_i widening is dropped for that return.
    cs : the fixed class inventory; μ_c = cs.mu, σ²_c = cs.sigma2,
        λ = cs.lambda_lift, class axis parallels cs.names ("other" last).

    Returns
    -------
    (N, K+1) float64 log-likelihood matrix. Never NaN or -inf: rows where the
    factor cannot be evaluated are all zeros.
    """
    intensity = np.asarray(intensity, dtype=np.float64)
    h = np.asarray(h, dtype=np.float64)
    rho_i = np.asarray(rho_i, dtype=np.float64)

    abstain = np.isnan(intensity)
    # No bottom reference ⇒ lift dropped, base Gaussian kept (module docstring).
    h_eff = np.where(np.isnan(h), 0.0, h)[:, None]           # (N, 1)
    rho_eff = np.where(np.isnan(rho_i), 0.0, rho_i)[:, None]
    # NaN intensity would poison its row through broadcasting; evaluate those
    # rows at a benign placeholder, then overwrite with zeros.
    i_safe = np.where(abstain, 0.0, intensity)[:, None]

    mean = cs.mu[None, :] + cs.lambda_lift * h_eff           # (N, K+1)
    var = cs.sigma2[None, :] + cs.lambda_lift ** 2 * rho_eff  # > 0: loader enforces σ² > 0
    out = -0.5 * (_LOG_2PI + np.log(var) + (i_safe - mean) ** 2 / var)
    out[abstain] = 0.0
    return out
