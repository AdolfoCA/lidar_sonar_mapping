"""
factor_position — the position factor p(p_i|c) of eq 24, evaluated per eq 31.

The semantically informative part of a return's position is its height above
the locally fitted bottom surface, h_i = z_i − ẑ_bed(x_i, y_i) (eq 25). Each
class occupies a height band N(ν_c, ω²_c); the exact height likelihood
marginalises the unobserved elevation angle along the arc (eq 28), which after
the boresight linearisation (eq 29) and the variance-matched Gaussian of the
uniform spread (eq 30) collapses to a Gaussian with the two variances ADDED:

    p(p_i|c) = N(h_i; ν_c, ω²_c + ϱ_i)                               (eq 31)

ω²_c is a learned class property while ϱ_i is a geometric one growing as r_i²
(geometry.height_variance); they combine per evaluation but are never folded
together. A far return therefore widens eq 31 for EVERY class identically, so
the height cue discriminates weakly exactly where it is unreliable — the
heteroscedastic attenuation of §II-D.a.

ABSTENTION: h_i is NaN while the bottom grid holds no reference at the
return's XY, and ϱ_i is NaN when the slant range is missing; eq 31 is not
evaluable without either, so those rows are all ZEROS (log-space identity —
the remaining eq-24 factors still combine).
"""

import numpy as np

from sonar_map.dirichlet.classes import ClassSet

# ── constants ────────────────────────────────────────────────────────────────

_LOG_2PI = float(np.log(2.0 * np.pi))


# ── eq 31 ────────────────────────────────────────────────────────────────────

def log_lik(h: np.ndarray, rho_i: np.ndarray, cs: ClassSet) -> np.ndarray:
    """(N, K+1) log N(h_i; ν_c, ω²_c + ϱ_i) (eq 31), abstained rows zeroed.

    Parameters
    ----------
    h : (N,) float64 — height above the bottom reference, h_i = z_i − ẑ_bed
        (eq 25, geometry.height_above_bed). NaN ⇒ the row abstains.
    rho_i : (N,) float64 — per-return elevation variance ϱ_i ≥ 0 (eq 30,
        geometry.height_variance). NaN ⇒ the row abstains.
    cs : the fixed class inventory; ν_c = cs.nu, ω²_c = cs.omega2, class axis
        parallels cs.names ("other" last).

    Returns
    -------
    (N, K+1) float64 log-likelihood matrix. Never NaN or -inf: rows where the
    factor cannot be evaluated are all zeros.
    """
    h = np.asarray(h, dtype=np.float64)
    rho_i = np.asarray(rho_i, dtype=np.float64)

    abstain = np.isnan(h) | np.isnan(rho_i)
    # A NaN input would poison its whole row through broadcasting; evaluate
    # abstaining rows at benign placeholders, then overwrite with zeros.
    h_safe = np.where(abstain, 0.0, h)[:, None]              # (N, 1)
    rho_safe = np.where(abstain, 0.0, rho_i)[:, None]        # (N, 1)

    var = cs.omega2[None, :] + rho_safe                      # > 0: loader enforces ω² > 0
    out = -0.5 * (_LOG_2PI + np.log(var) + (h_safe - cs.nu[None, :]) ** 2 / var)
    out[abstain] = 0.0
    return out
