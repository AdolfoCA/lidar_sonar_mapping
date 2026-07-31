"""
geometry — the shared per-return quantities of the paper's position model.

The sonar does not observe the elevation angle ϕ within its vertical aperture,
so the published return position uses the BORESIGHT assumption ϕ = 0 (eq 26 at
boresight: the z that arrives in ClassifiedReturns IS z_w(0)). The residual
elevation ambiguity is carried as a per-return VARIANCE ϱ_i instead:
linearising the arc about boresight (eq 29) maps the uniform elevation prior
(eq 27) to a uniform height spread of width s_i·Δϕ, whose variance-matched
Gaussian has

    s_i = ∂z_w/∂ϕ |_{ϕ=0} = r_i · cos θ                         (eq 29)
    ϱ_i = (s_i · Δϕ)² / 12                                       (eq 30)

ϱ_i is a GEOMETRIC property growing as r_i² — it widens the position (eq 31)
and intensity (eq 34) factors so far returns discriminate weakly, exactly
where the height cue is unreliable. It is never folded into the class spreads.
"""

import numpy as np

from sonar_map.dirichlet.classes import ClassSet


def height_variance(slant_range: np.ndarray, cs: ClassSet) -> np.ndarray:
    """ϱ_i (eq 30) from the slant range r_i, the tilt θ and the aperture Δϕ.

    Returns (N,) float64, ≥ 0. NaN slant ranges propagate to NaN (the factors
    abstain on those returns).
    """
    r = np.asarray(slant_range, dtype=np.float64)
    s = r * np.cos(cs.tilt_rad)                 # height sensitivity (eq 29)
    return (s * cs.aperture_rad) ** 2 / 12.0    # variance of the matched Gaussian


def height_above_bed(z: np.ndarray, z_bed: np.ndarray) -> np.ndarray:
    """h_i = z_i − ẑ_bed(x_i, y_i) (eq 25), the boresight height estimate.

    z is the boresight return height (odom, z up) and z_bed the local bottom
    reference at the return's XY. Where z_bed is NaN (bottom not yet observed
    there) the result is NaN and the position factor abstains for that return.
    """
    return np.asarray(z, dtype=np.float64) - np.asarray(z_bed, dtype=np.float64)
