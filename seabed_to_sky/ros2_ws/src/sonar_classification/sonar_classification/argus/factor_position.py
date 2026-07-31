"""
factor_position — p(p_i | c), the height factor (eq 24).

    p(p_i | c) = B(h_i; l_c, u_c, sigma_ic),   sigma_ic^2 = tau_c^2 + rho_var_i

A class owns a height INTERVAL, not a typical height, so the family is the soft
box of eq 23 (see soft_interval for why a Gaussian is the wrong choice). The
two spreads combine in quadrature AT EVALUATION and are never folded into one
another: tau_c is a class property (the ragged moving top of a canopy against
the sharp top of a rigid object; at the lower edge it also absorbs bed-fit
error), while rho_var_i is per-return geometry growing as r_i^2.

The factor is decisive where the physics is decisive — a return well above a
class's upper edge is strongly penalised — and deliberately flat where it is
not: a low return is equally consistent with the bed and with the base of a
bed-anchored extended class, and the box says so instead of inventing a
preference. Resolving exactly those returns is the spatial prediction's job.

ABLATION ``position_gaussian`` swaps in the mid-extent Gaussian the box
replaces — N(h_i; (l_c+u_c)/2, sigma_ic^2) — and changes nothing else, so the
study isolates the extended-class fix.

ABSTENTION: h_i is NaN while no bed reference exists at the return's XY, and
rho_var_i is NaN without a slant range. Neither is evaluable, so those rows are
zeros and the remaining factors still combine.
"""

import numpy as np

from sonar_classification.argus.classes import ClassSet
from sonar_classification.argus.soft_interval import log_soft_interval

_LOG_2PI = float(np.log(2.0 * np.pi))


def log_lik(h, rho_var, cs: ClassSet) -> np.ndarray:
    """(N, C) log p(p_i | c) (eq 24). Abstained rows are zero; never non-finite.

    Parameters
    ----------
    h       : (N,) h_i, height above the fitted bed [m] (eq 18).
    rho_var : (N,) rho_var_i, elevation-induced height variance [m^2] (eq 22).
    """
    h = np.asarray(h, dtype=np.float64)
    rho_var = np.asarray(rho_var, dtype=np.float64)

    abstain = ~np.isfinite(h) | ~np.isfinite(rho_var)
    # Evaluate abstaining rows at benign placeholders — a NaN would poison the
    # whole row through broadcasting — then overwrite them with zeros.
    h_s = np.where(abstain, 0.0, h)[:, None]                    # (N, 1)
    rv_s = np.where(abstain, 0.0, np.maximum(rho_var, 0.0))[:, None]

    # sigma_ic^2 = tau_c^2 + rho_var_i — class softness and per-return geometry,
    # in quadrature, here and nowhere else.
    sigma = np.sqrt(cs.tau[None, :] ** 2 + rv_s)                # (N, C)

    if cs.ablation.get('position_gaussian', False):
        centre = 0.5 * (cs.l + cs.u)[None, :]
        out = -0.5 * (_LOG_2PI + 2.0 * np.log(sigma)
                      + ((h_s - centre) / sigma) ** 2)
    else:
        out = log_soft_interval(h_s, cs.l[None, :], cs.u[None, :], sigma)

    out = np.where(abstain[:, None], 0.0, out)
    return out
