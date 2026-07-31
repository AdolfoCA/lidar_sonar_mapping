"""
factor_shape — p(m_i | c), the morphology factor (eqs 32-33).

The other three factors apply to every return, but morphology is a property of
a connected bright REGION and most returns are bare seabed belonging to none.
So, as with the LiDAR channel, it enters in hurdle form:

    p(m_i|c)  = rho_c^{s_m} (1-rho_c)^{1-s_m} [ p(m_R|c) ]^{s_m d_i}         (33)
    p(m_R|c)  = N(log e_R; e_bar_c, E_c^2) N(log w_R; w_bar_c, W_c^2)        (32)

Both lengths are positive and vary across classes MULTIPLICATIVELY — by
factors, not increments — so each is Gaussian in the log domain, which places
no mass on negative lengths and stops the largest regions dominating the
spread.

THE DEDUP GATE d_i. Within a region the N_R returns share ONE descriptor: the
region is one observation about each voxel it covers, not N_R observations.
d_i in {0,1} is 1 iff return i is the FIRST return of its region to fall in its
voxel this sweep. This replaces a 1/N_R weighting, which belonged to the
(removed) class-parameter update and is actively wrong for voxel inference:
dividing a 100-return wall's shape evidence by 100 per return would neuter
morphology for exactly the large structures it exists to describe — and
morphology is the ONE channel that separates wall from pillar.
ABLATION ``shape_dedup_mode: inv_nr`` restores the 1/N_R exponent so the
difference can be measured; nothing else changes.

rho_c is a FRACTION OF RETURNS (assign the complement, as for pi_lidar). It is
the least physical parameter in the model — it depends on the region-grower's
threshold — so the robustness result matters: scaling ALL rho_c by a common
factor cancels from the present branch of the ratio, and only the ABSENT
branch (the missing-blob evidence) moves. A bare-seabed return contributes only
log(1 - rho_c), and that absence is itself informative.

ABSTENTION: s_m = NaN means the extractor did not run at all — the row is
zeros. s_m = 0 is a genuine observation ("this return formed no region") and
contributes the absent branch. A present region whose e_R or w_R could not be
measured keeps its presence term and drops only the missing length.
"""

import numpy as np

from sonar_classification.argus.classes import ClassSet

_LOG_2PI = float(np.log(2.0 * np.pi))


def _log_gauss(x_col, mean, std):
    """log N(x; mean, std^2) broadcast to (M, C)."""
    return -0.5 * (_LOG_2PI + 2.0 * np.log(std)[None, :]
                   + ((x_col - mean[None, :]) / std[None, :]) ** 2)


def log_lik(flag, extent, thickness, dedup, cs: ClassSet,
            n_region=None) -> np.ndarray:
    """(N, C) log p(m_i | c) (eq 33). Abstained rows are zero.

    Parameters
    ----------
    flag      : (N,) s^m_i in {0, 1}; NaN = extractor absent, row abstains.
    extent    : (N,) e_R [m], beam-width corrected. NaN drops the extent term.
    thickness : (N,) w_R [m]. NaN drops the thickness term.
    dedup     : (N,) d_i in {0, 1} — the per-voxel-per-sweep carrier gate,
                computed by the mapper, which is the only place that knows
                which voxel a return landed in.
    n_region  : (N,) N_R, required only by the ``inv_nr`` ablation.
    """
    s_m = np.asarray(flag, dtype=np.float64)
    e_r = np.asarray(extent, dtype=np.float64)
    w_r = np.asarray(thickness, dtype=np.float64)
    d_i = np.asarray(dedup, dtype=np.float64)

    n = s_m.shape[0]
    out = np.zeros((n, cs.n_classes), dtype=np.float64)

    known = np.isfinite(s_m)
    s1 = known & (s_m > 0.5)        # return belongs to a grown region
    s0 = known & ~s1                # bare return — the informative absence

    out[s0] = np.log1p(-cs.rho)[None, :]
    if not np.any(s1):
        return out
    out[s1] = np.log(cs.rho)[None, :]

    # ── descriptor exponent: the dedup gate, or the 1/N_R ablation ───────────
    if cs.ablation.get('shape_dedup_mode', 'per_voxel') == 'inv_nr':
        nr = np.asarray(n_region, dtype=np.float64) if n_region is not None \
            else np.ones(n, dtype=np.float64)
        expo = np.where(np.isfinite(nr) & (nr > 0.0), 1.0 / np.maximum(nr, 1.0), 0.0)
    else:
        expo = np.where(np.isfinite(d_i), d_i, 0.0)

    carriers = s1 & (expo > 0.0)
    if not np.any(carriers):
        return out

    # The two lengths contribute independently, so a region measured on only
    # one axis still contributes what it does know.
    contrib = np.zeros((int(carriers.sum()), cs.n_classes), dtype=np.float64)
    sub_e = e_r[carriers]
    sub_w = w_r[carriers]

    ok_e = np.isfinite(sub_e) & (sub_e > 0.0)
    if np.any(ok_e):
        contrib[ok_e] += _log_gauss(np.log(sub_e[ok_e])[:, None],
                                    cs.e_bar, cs.e_std)
    ok_w = np.isfinite(sub_w) & (sub_w > 0.0)
    if np.any(ok_w):
        contrib[ok_w] += _log_gauss(np.log(sub_w[ok_w])[:, None],
                                    cs.w_bar, cs.w_std)

    out[carriers] += expo[carriers][:, None] * contrib
    return out
