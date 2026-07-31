"""
factor_shape — the shape factor p(m_i | c) of eq 24 (paper eqs 39–42).

Morphology is a property of a grown bright REGION, not of a single return, and
most returns are bare seabed belonging to none — so shape enters as a hurdle
(eq 42): membership first (s_i = 1[∃R: i ∈ R], eq 39), morphology only when a
region exists. A member return carries its region's descriptor
m_R = (a_R, ς_R, ξ_R, τ_R) (eq 40): log physical area, solidity, shadow
signature (the principal discriminator — an opaque wall casts a deep, long,
sharp shadow, vegetation a weak ragged one) and internal intensity texture.
Each channel is an independent Gaussian, solidity on the LOGIT scale because
it is bounded in [0, 1] (eq 41):

    p(m_i|c) = ρ_c^s (1−ρ_c)^{1−s} × p(m_R|c)^{s/N_R}                  (eq 42)

DE-DUPLICATION: all N_R returns of a region share the ONE descriptor of
eq 40; summing the region log-likelihood over them would count the same
evidence N_R times, so it is split evenly across the region's points — raised
to the power 1/N_R (eq 42).

ABSTENTION vs s=0: shape_flag = NaN means the region extractor did not run
(old bag, factor upstream disabled) — the row abstains. s = 0 is a genuine
observation and contributes log(1 − ρ_c): the ABSENCE of shape is itself
informative — bare-seabed classes have ρ → 0, structure ρ → 1. A NaN in a
single descriptor channel drops that channel's term only, so partial
descriptors keep contributing.
"""

import numpy as np

from sonar_map.dirichlet.classes import SHAPE_CHANNELS, ClassSet

_LOG_2PI = np.log(2.0 * np.pi)

# ── numerical guard ──────────────────────────────────────────────────────────
# logit(ς) = log ς − log(1 − ς) diverges at the closed ends of [0, 1]; a
# degenerate extractor output (a filled or hairline region with ς = 0 or 1
# exactly) must land on a large-but-finite logit (≈ ±13.8), never ±inf
# (dirichlet design rule 3).
_SOLIDITY_EPS = 1e-6


def log_lik(shape_flag, region_size, logarea, solidity, shadow, texture,
            cs: ClassSet) -> np.ndarray:
    """log p(m_i | c) (eq 42) — (N, K+1) float64, class axis = cs.names.

    shape_flag  : (N,) s_i (eq 39). NaN = extractor did not run → the row
                  ABSTAINS (all zeros); 0 is the informative "no region" case.
    region_size : (N,) N_R, returns in i's region — the de-duplication power
                  1/N_R of eq 42. Floored at 1; a NaN size falls back to 1
                  (count the evidence once rather than poison the row).
    logarea     : (N,) a_R, log physical area of the bright region [log m²].
    solidity    : (N,) ς_R ∈ [0, 1]; enters on the logit scale (eq 41).
    shadow      : (N,) ξ_R, shadow-signature composite of the extractor.
    texture     : (N,) τ_R, internal intensity texture.
    cs          : class inventory; uses cs.rho (ρ_c) and cs.shape_mean /
                  cs.shape_var — (K+1, 4), columns in SHAPE_CHANNELS order.
    """
    flag = np.asarray(shape_flag, dtype=np.float64)

    out = np.zeros((flag.shape[0], cs.n_classes), dtype=np.float64)
    valid = ~np.isnan(flag)
    s1 = valid & (flag > 0)   # member of a grown region
    s0 = valid & ~s1          # bare return — informative absence

    # ── s = 0: no region — log(1 − ρ_c) ─────────────────────────────────────
    out[s0] = np.log1p(-cs.rho)[None, :]

    if not np.any(s1):
        return out

    # ── descriptor matrix, columns in SHAPE_CHANNELS order (eq 40) ──────────
    sol = np.clip(np.asarray(solidity, dtype=np.float64),
                  _SOLIDITY_EPS, 1.0 - _SOLIDITY_EPS)
    channels = {
        'logarea': np.asarray(logarea, dtype=np.float64),
        'solidity_logit': np.log(sol) - np.log1p(-sol),   # NaN ς stays NaN
        'shadow': np.asarray(shadow, dtype=np.float64),
        'texture': np.asarray(texture, dtype=np.float64),
    }
    x = np.stack([channels[ch] for ch in SHAPE_CHANNELS], axis=1)[s1]  # (M, 4)

    # ── eq 41: independent Gaussians per channel; NaN drops that term ───────
    xb = x[:, None, :]                              # (M, 1,   4)
    mean = cs.shape_mean[None, :, :]                # (1, K+1, 4)
    var = cs.shape_var[None, :, :]
    logpdf = -0.5 * (_LOG_2PI + np.log(var) + (xb - mean) ** 2 / var)
    logpdf = np.where(np.isnan(xb), 0.0, logpdf)    # partial descriptors count

    # ── eq 42: presence + region likelihood de-duplicated by 1/N_R ──────────
    nr = np.asarray(region_size, dtype=np.float64)[s1]
    nr = np.where(np.isnan(nr), 1.0, np.maximum(nr, 1.0))
    out[s1] = np.log(cs.rho)[None, :] + logpdf.sum(axis=2) / nr[:, None]

    return out
