"""
likelihood — assemble the class-conditional likelihood (eq 24) and close the
responsibilities (eq 16).

The four channels of a return probe distinct physical aspects of the scene —
geometry, backscatter, overhead structure, morphology — so conditional on the
class they are modelled as independent and the likelihood factors as

    L_{i,c} = p(p_i|c) · p(I_i|p_i,c) · p(ℓ_i|c) · p(m_i|c)          (eq 24)

Each factor module returns an (N, K+1) LOG-likelihood matrix, so the product
above is a SUM here; factors that abstain (NaN gating channel, or globally
disabled in the config) contribute rows/matrices of zeros, which is exactly
the multiplicative identity of eq 24 — the remaining factors still combine
correctly and no return is ever dropped.

Two deliberate asymmetries versus a naive sum:

  * ``other_weight`` — the analogue of the paper's stick-breaking
    concentration γ (Alg. 1 line 9: r_new ∝ γ·L_new) — is added to the LAST
    column of the TOTAL, once. It is a responsibility weight on the catch-all
    class, not a property of any single factor, so folding it into a factor
    matrix would corrupt the per-factor telemetry.
  * ``by_factor`` always contains all four factor names, a disabled factor
    mapping to a zero matrix, so downstream telemetry (Eq24Telemetry) keeps a
    stable row layout across ablation configs.

``responsibilities`` then evaluates eq 16 as a numerically stable row softmax:
the responsibilities are invariant to a per-row additive constant in log
space, so subtracting the row maximum before exponentiating changes nothing
mathematically while keeping every exponent ≤ 0 (no overflow, and at least
one term is exp(0)=1 so the row sum never vanishes).
"""

from dataclasses import dataclass

import numpy as np

from sonar_map.dirichlet import (
    factor_intensity,
    factor_lidar,
    factor_position,
    factor_shape,
)
from sonar_map.dirichlet.classes import FACTOR_NAMES, ClassSet  # noqa: F401


# ── per-return feature bundle ────────────────────────────────────────────────

@dataclass
class Features:
    """The per-return channels of eq 24, as parallel (N,) float64 arrays.

    NaN is the ABSTENTION marker throughout (see the package docstring): a
    NaN in a factor's gating channel makes that factor contribute zeros for
    that return. Mapping from ClassifiedReturns:

        h          = z − ẑ_bed   (geometry.height_above_bed, eq 25;
                                   NaN while the bottom grid is cold there)
        rho        = ϱ_i          (geometry.height_variance, eq 30)
        over_count = k_i, over_gap = g_i (eq 36; over_count NaN = no LiDAR
                     cloud yet, over_gap NaN with k_i>0 = gap term dropped)
        shape_flag = s^m_i (eq 39; NaN = extractor did not run / old bag),
        region_*   = N_R and the mR descriptor channels (eq 40)
    """

    h: np.ndarray                # h_i  height above bed [m] (eq 25)
    rho: np.ndarray              # ϱ_i  elevation-ambiguity variance [m²] (eq 30)
    intensity: np.ndarray        # I_i  raw backscatter intensity
    over_count: np.ndarray       # k_i  overhead LiDAR support count (eq 36)
    over_gap: np.ndarray         # g_i  vertical gap to lowest overhead point [m]
    shape_flag: np.ndarray       # s^m_i region-membership indicator (eq 39)
    region_size: np.ndarray      # N_R  returns in the region (de-dup exponent)
    region_logarea: np.ndarray   # a_R  log physical area (eq 40)
    region_solidity: np.ndarray  # ς_R  solidity in [0, 1] (logit taken inside)
    region_shadow: np.ndarray    # ξ_R  shadow signature composite
    region_texture: np.ndarray   # τ_R  internal texture, log(1 + intensity var)


# ── eq 24 in log space ───────────────────────────────────────────────────────

def log_likelihood(f: Features, cs: ClassSet) -> tuple:
    """Assemble log L (eq 24) for every return × class.

    Returns ``(total, by_factor)``:
      total     — (N, K+1) float64, Σ of the factor log-matrices with
                  log(other_weight) added to the trailing "other" column.
      by_factor — {factor name: (N, K+1)} in FACTOR_NAMES order; a factor
                  disabled via cs.factors_enabled maps to a zero matrix
                  (present so telemetry rows stay stable across ablations).

    The factor modules themselves never consult cs.factors_enabled — the
    enable/disable gate lives here, in one place, so an ablation run and a
    NaN-channel abstention are indistinguishable downstream (both are zeros).
    """
    n = np.asarray(f.h, dtype=np.float64).shape[0]

    evaluators = {
        'position': lambda: factor_position.log_lik(f.h, f.rho, cs),
        'intensity': lambda: factor_intensity.log_lik(f.intensity, f.h, f.rho, cs),
        'lidar': lambda: factor_lidar.log_lik(f.over_count, f.over_gap, cs),
        'shape': lambda: factor_shape.log_lik(
            f.shape_flag, f.region_size, f.region_logarea,
            f.region_solidity, f.region_shadow, f.region_texture, cs),
    }

    total = np.zeros((n, cs.n_classes), dtype=np.float64)
    by_factor = {}
    for name in FACTOR_NAMES:
        if cs.factors_enabled.get(name, True):
            mat = np.asarray(evaluators[name](), dtype=np.float64)
        else:
            mat = np.zeros((n, cs.n_classes), dtype=np.float64)
        by_factor[name] = mat
        total += mat

    # The paper's γ (Alg. 1 line 9) — applied ONCE, on the total, never per
    # factor: it weights the catch-all's responsibility, not any single cue.
    total[:, -1] += np.log(cs.other_weight)
    return total, by_factor


# ── eq 16: responsibilities ──────────────────────────────────────────────────

def responsibilities(log_L: np.ndarray, log_prior) -> np.ndarray:
    """r_{i,c} = L_{i,c}·π_c(N_v) / Σ_c' L_{i,c'}·π_c'(N_v)  (eq 16), stably.

    log_prior is log π_c(N_v) (eq 23): either a shared (K+1,) vector (e.g.
    the uniform fallback) or a per-return (N, K+1) matrix when each return
    hits a different voxel neighbourhood — broadcasting covers both.

    Never exponentiate raw log-likelihoods: a single return with |log L|
    ~ 1e4 (an outlandish intensity is enough) would overflow/underflow
    float64. Subtracting the row maximum first is exact — eq 16 is invariant
    to per-row constants — and guarantees one exp(0)=1 term per row, so the
    normaliser is always positive and rows always sum to 1.
    """
    z = np.asarray(log_L, dtype=np.float64) + np.asarray(log_prior, dtype=np.float64)
    z -= z.max(axis=-1, keepdims=True)
    np.exp(z, out=z)
    z /= z.sum(axis=-1, keepdims=True)
    return z
