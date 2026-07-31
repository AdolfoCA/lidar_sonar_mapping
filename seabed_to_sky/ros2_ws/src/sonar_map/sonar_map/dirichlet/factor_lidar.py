"""
factor_lidar — the LiDAR-support factor p(ℓ_i | c) of eq 24 (paper eqs 35–38).

The LiDAR images the air–water interface downward, so it never sees the
submerged return itself — it sees what stands ABOVE it. ℓ_i is therefore an
overhead-column query (eq 35): k_i counts the LiDAR points in the vertical
column above return i, g_i is the vertical gap to the lowest of them, and
s_i = 1[k_i > 0] flags presence (eq 36). The channels are degenerate — absent
support leaves g_i undefined — so eq 37 is a HURDLE model, presence first,
magnitude only when present:

    p(ℓ_i|c) = (π^ℓ_c)^s (1−π^ℓ_c)^{1−s} × [Pois>0(k_i; κ_c)·Exp(g_i; γ_c)]^s

The count term is the ZERO-TRUNCATED Poisson (eq 38) — s=1 implies k ≥ 1, so
the k=0 mass must be renormalised away — and the gap is Exponential. π^ℓ_c is
the decisive parameter: surface-piercing structure has π → 1, seabed and
submerged classes π → 0.

ABSTENTION vs s=0 (they are NOT the same thing): k_i = NaN means no LiDAR
cloud was available for the query — the factor abstains (zero row) rather than
asserting "not structure", so a fully submerged wall can still be claimed as
structure by the intensity/shape cues. k_i = 0 is a genuine observation
("nothing overhead") and contributes log(1 − π^ℓ_c).
"""

import numpy as np
from scipy.special import gammaln

from sonar_map.dirichlet.classes import ClassSet

# ── numerical guard ──────────────────────────────────────────────────────────
# The Pois>0 normaliser log1p(−e^{−κ}) → −inf as κ → 0⁺. load_classes enforces
# κ > 0, but a pathological config value must degrade to "astronomically
# unlikely", never to a non-finite matrix entry (dirichlet design rule 3).
# Below the floor 1 − e^{−κ} ≈ κ, so flooring κ caps the normaliser at
# ≈ log(1e−12) ≈ −27.6 while leaving any sane κ untouched.
_KAPPA_FLOOR = 1e-12


def log_lik(over_count, over_gap, cs: ClassSet) -> np.ndarray:
    """log p(ℓ_i | c) (eq 37) — (N, K+1) float64, class axis = cs.names.

    over_count : (N,) k_i, LiDAR points in the column above the return
                 (eq 36). NaN = no LiDAR cloud received yet → the row
                 ABSTAINS (all zeros); k = 0 is the informative s=0 case.
    over_gap   : (N,) g_i [m], vertical gap to the lowest overhead point
                 (eq 36). NaN while k > 0 drops the gap term only —
                 presence and count still contribute.
    cs         : class inventory; uses cs.pi_l (π^ℓ_c), cs.kappa (κ_c),
                 cs.gamma_rate (γ_c).
    """
    k = np.asarray(over_count, dtype=np.float64)
    g = np.asarray(over_gap, dtype=np.float64)

    out = np.zeros((k.shape[0], cs.n_classes), dtype=np.float64)
    valid = ~np.isnan(k)
    s1 = valid & (k > 0)      # hurdle crossed: overhead support present
    s0 = valid & ~s1          # genuine "nothing overhead"

    # ── s = 0: presence miss — log(1 − π^ℓ_c) ───────────────────────────────
    out[s0] = np.log1p(-cs.pi_l)[None, :]

    if not np.any(s1):
        return out

    # ── s = 1: presence + zero-truncated Poisson count (eq 38) ──────────────
    #   log Pois>0(k; κ) = k·log κ − κ − log k! − log(1 − e^{−κ})
    kappa = np.maximum(cs.kappa, _KAPPA_FLOOR)
    log_norm = np.log1p(-np.exp(-kappa))
    k1 = k[s1][:, None]
    out[s1] = (np.log(cs.pi_l)[None, :]
               + k1 * np.log(kappa)[None, :] - kappa[None, :]
               - gammaln(k1 + 1.0) - log_norm[None, :])

    # ── gap term Exp(g; γ) = γ e^{−γg}, only where the gap is defined ───────
    has_gap = s1 & ~np.isnan(g)
    if np.any(has_gap):
        g1 = g[has_gap][:, None]
        out[has_gap] += (np.log(cs.gamma_rate)[None, :]
                         - cs.gamma_rate[None, :] * g1)

    return out
