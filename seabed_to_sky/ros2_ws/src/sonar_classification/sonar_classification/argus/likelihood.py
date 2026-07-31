r"""
likelihood — assemble the class-conditional likelihood (eq 34) and close the
responsibilities (eq 9).

The four channels of a measurement probe distinct physical aspects of the scene
— geometry, scattering strength, overhead structure, region morphology — so,
conditional on the class, they are modelled as independent and the likelihood
factors:

    L_{i,c} = p(p_i|c) p(A_i|c) p(l_i|c) p(m_i|c)                          (34)
              \_____ every _____/  \_ unity  _/  \_ hurdle over _/
                     return          outside        bright regions
                                     coverage

Each factor module returns an (N, C) LOG matrix, so the product above is a SUM
here (I5). A factor that abstains contributes zeros — the multiplicative
identity — so the remaining factors still combine correctly.

THE RESPONSIBILITY IS A RATIO (I7). Any factor whose parameters coincide for
two classes cancels from their pairwise comparison: two classes are separated
ONLY by the channels in which their assigned parameters differ. Every
distinction the map is required to draw must therefore be carried by at least
one factor with genuinely distinct parameters.

``responsibilities`` evaluates eq 9 as a numerically stable row softmax: the
result is invariant to a per-row additive constant in log space, so subtracting
the row maximum is exact while keeping every exponent <= 0 (no overflow) and
guaranteeing one exp(0) = 1 term (the normaliser never vanishes).
"""

from dataclasses import dataclass, field

import numpy as np

from sonar_classification.argus import (
    factor_amplitude,
    factor_lidar,
    factor_position,
    factor_shape,
)
from sonar_classification.argus.classes import FACTOR_NAMES, ClassSet  # noqa: F401


@dataclass
class Measurements:
    """One sweep's measurement tuples y_i = (p_i, A_i, l_i, m_i), as arrays.

    NaN is the ABSTENTION marker throughout: a NaN in a factor's gating
    channel makes that factor contribute zeros for that return.
    """

    h: np.ndarray                # h_i        height above the fitted bed [m]
    rho_var: np.ndarray          # rho_var_i  elevation height variance [m^2]
    count_raw: np.ndarray        # A_i        the pristine uint16 count
    observed: np.ndarray         # o_i        LiDAR coverage gate {0,1}
    present: np.ndarray          # s_lidar_i  validated overhead presence {0,1}
    clearance: np.ndarray        # g_tilde_i  waterline-referenced clearance [m]
    shape_flag: np.ndarray       # s^m_i      region membership {0,1}
    extent: np.ndarray           # e_R        beam-corrected bearing extent [m]
    thickness: np.ndarray        # w_R        range thickness [m]
    dedup: np.ndarray            # d_i        per-voxel-per-sweep carrier gate
    n_region: np.ndarray = field(default=None)   # N_R — only the inv_nr ablation

    def __len__(self):
        return int(np.asarray(self.h).shape[0])


def log_likelihood(m: Measurements, cs: ClassSet):
    """Assemble log L (eq 34) for every return x class.

    Returns ``(total, by_factor)``: the (N, C) sum, and the per-factor matrices
    keyed by FACTOR_NAMES. A factor disabled in config maps to a zero matrix
    and is reported as such, so telemetry keeps a stable layout across ablation
    runs and a disabled factor is indistinguishable downstream from an
    abstaining one — which is exactly right, since both mean "this channel says
    nothing".
    """
    n = len(m)

    evaluators = {
        'position': lambda: factor_position.log_lik(m.h, m.rho_var, cs),
        'amplitude': lambda: factor_amplitude.log_lik(m.count_raw, cs),
        'lidar': lambda: factor_lidar.log_lik(
            m.observed, m.present, m.clearance, cs),
        'shape': lambda: factor_shape.log_lik(
            m.shape_flag, m.extent, m.thickness, m.dedup, cs,
            n_region=m.n_region),
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
    return total, by_factor


def responsibilities(log_L: np.ndarray, log_prior) -> np.ndarray:
    """r_{i,c} = L_{i,c} pi_c(N_v) / sum_c' L_{i,c'} pi_c'(N_v)   (eq 9).

    ``log_prior`` is log pi_c(N_v) (eq 16): either a shared (C,) vector — the
    uniform fallback — or an (N, C) matrix when each return sits in a different
    neighbourhood. Broadcasting covers both.

    Never exponentiate raw log-likelihoods: one return with |log L| ~ 1e4 (an
    outlandish intensity is enough) would flush float64. Subtracting the row
    maximum first is exact and keeps the normaliser positive.
    """
    z = np.asarray(log_L, dtype=np.float64) + np.asarray(log_prior, dtype=np.float64)
    z = z - z.max(axis=-1, keepdims=True)
    np.exp(z, out=z)
    z /= z.sum(axis=-1, keepdims=True)
    return z
