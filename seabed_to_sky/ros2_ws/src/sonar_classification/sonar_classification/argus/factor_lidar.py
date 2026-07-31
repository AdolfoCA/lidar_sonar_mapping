"""
factor_lidar — p(l_i | c), the overhead-support factor (eqs 28-29).

The LiDAR images the air-water interface downward, so it never sees the
submerged return itself — it sees what stands ABOVE it. l_i is therefore an
overhead-column query (eq 26): for the LiDAR cloud P_L and a horizontal radius
eps_xy,

    K_i       = { q in P_L : ||(x_q,y_q) - (x_i,y_i)|| <= eps_xy }
    o_i       = 1 if the column was OBSERVED by the LiDAR, else 0
    s_lidar_i = 1 if |K_i| >= k_min, else 0
    g_tilde_i = min_q(z_q) - z_waterline

and the factor is a hurdle — presence first, magnitude only when present —
gated by coverage:

    p(l_i|c) = [ pi_c^s (1-pi_c)^(1-s) p_clr(g_tilde_i|c)^s ]^{o_i}          (28)
    p_clr(g|c) = gamma_c exp(-gamma_c g)     surface-piercing classes
               = B(g; lg_c, ug_c, sg_c)      classes beneath cover            (29)

THREE DESIGN DECISIONS, EACH PREVENTING A SPECIFIC FAILURE.

1. THE COUNT VALIDATES BUT NEVER DISCRIMINATES (I8). An angularly scanning
   LiDAR deposits whole line segments on any surface a column clips, so the
   count scales with dwell time and standoff (as 1/R^2) and is bimodal — zero
   or hundreds. It is a property of the TRAJECTORY, not of the class, so no
   class rate can honestly be assigned to it and it is excluded from the
   likelihood. Its only legitimate role is reliability: a single overhead point
   may be spray, a bird, or sensor noise and must not assert structure, hence
   the k_min gate that produces s_lidar_i upstream.

2. CLEARANCE IS REFERENCED TO THE WATERLINE, NOT TO THE RETURN. The vehicle
   floats on the waterline, so z_waterline is directly available. Referencing
   the return would make this channel a deterministic function of the return's
   own height — contradicting the conditional independence of eq 17 and mixing
   "how deep is the return" (already owned by the position factor) into "what
   hangs overhead". The waterline-referenced clearance is a property of the
   scene alone, and it is assignable by inspection: does the class reach the
   surface, and if not, how high does its cover hang.
   ABLATION ``lidar_legacy_gap_ref`` restores the return-referenced clearance
   so the bridge-problem fix can be measured.

3. COVERAGE GATES EVERYTHING. When o_i = 0 the factor is unity for EVERY class:
   absence of observation is not evidence of absence. Without this gate,
   correlated coverage gaps — a whole stretch of missing deck — stack many mild
   misses into a false veto against structure that the intensity and morphology
   channels could still have recovered. The better the coverage mask, the
   closer to 1 you can honestly set pi_c for piercing classes.

TWO DESIGN GROUPS ONLY (I7). Piercing (wall, pillar): high pi_c, exponential
clearance with its mode at 0. Submerged (seabed, vegetation, object): low
pi_c, with exp/box chosen by whether the class can sit under cover. WITHIN a
group this channel cannot — and must not be tuned to pretend it can —
separate classes.

pi_c is a FRACTION OF RETURNS, not a property of the object. Assign the
complement ("what fraction of wall returns lack validated support?"): all the
sensitivity is in the absent branch. It is never allowed to be exactly 1 (the
config validator rejects it) — one gap would give log(0) = -inf and
permanently veto the correct class in that voxel.
"""

import numpy as np

from sonar_classification.argus.classes import ClassSet
from sonar_classification.argus.soft_interval import log_soft_interval


def log_lik(observed, present, clearance, cs: ClassSet) -> np.ndarray:
    """(N, C) log p(l_i | c) (eq 28). Abstained rows are zero.

    Parameters
    ----------
    observed  : (N,) o_i in {0, 1} — the coverage gate. NaN (no LiDAR data at
                all yet) abstains, exactly like o_i = 0.
    present   : (N,) s_lidar_i in {0, 1} — the k_min-validated presence bit.
    clearance : (N,) g_tilde_i [m], waterline-referenced. Only read where
                s_lidar_i = 1; NaN there drops the magnitude term but keeps
                the presence term.
    """
    o = np.asarray(observed, dtype=np.float64)
    s = np.asarray(present, dtype=np.float64)
    g = np.asarray(clearance, dtype=np.float64)

    n = o.shape[0]
    out = np.zeros((n, cs.n_classes), dtype=np.float64)

    covered = np.isfinite(o) & (o > 0.5)
    if not np.any(covered):
        return out

    s1 = covered & np.isfinite(s) & (s > 0.5)     # validated overhead support
    s0 = covered & ~s1                            # observed, nothing overhead

    # ── presence hurdle ──────────────────────────────────────────────────────
    out[s0] = np.log1p(-cs.pi_lidar)[None, :]
    if not np.any(s1):
        return out
    out[s1] = np.log(cs.pi_lidar)[None, :]

    # ── magnitude: the clearance density, only where support is present ──────
    has_g = s1 & np.isfinite(g)
    if not np.any(has_g):
        return out
    g1 = g[has_g][:, None]                                      # (M, 1)

    # Piercing classes: exponential, mode at 0. A negative clearance (the
    # lowest overhead point below the waterline — wave crest, spray, a badly
    # registered point) is clamped to the mode rather than rewarded for it.
    g_pos = np.maximum(g1, 0.0)
    log_exp = np.log(cs.clr_gamma)[None, :] - cs.clr_gamma[None, :] * g_pos

    # Classes beneath cover: the soft interval again — one family for both the
    # height and the clearance channel, every parameter a tape-measure quantity.
    log_box = log_soft_interval(g1, cs.clr_l[None, :], cs.clr_u[None, :],
                                cs.clr_sigma[None, :])

    out[has_g] += np.where(cs.clr_is_exp[None, :], log_exp, log_box)
    return out
