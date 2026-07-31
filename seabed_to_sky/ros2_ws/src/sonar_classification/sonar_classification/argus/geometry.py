"""
geometry — the per-return geometric quantities behind the position factor.

The multibeam resolves BEARING but not ELEVATION: the vertical aperture
Delta_phi is not beamformed, so a pixel corresponds to an ARC over
phi in [phi_min, phi_max], not a point. phi is marginalised, never measured.

    z_i(phi) = r_i * [ cos(phi) cos(beta_i) sin(theta) + sin(phi) cos(theta) ]  (19)
    h_i      = z_i - z_bed_hat(x_i, y_i)                                        (18)

Linearising the arc about boresight (phi = 0) gives the height sensitivity and
the elevation-induced height variance. Note there is NO bearing dependence —
it cancels exactly at phi = 0:

    s_i       = dz/dphi |_{phi=0} = r_i cos(theta)                              (21)
    rho_var_i = (s_i * Delta_phi)^2 / 12                                        (22)

the variance of a uniform of that width, matched by its Gaussian. rho_var_i
grows as r_i^2, so it is a PER-RETURN GEOMETRIC quantity, never a class
property: it is combined with the class edge softness tau_c in quadrature at
evaluation time (``sigma_ic^2 = tau_c^2 + rho_var_i``) and never folded into a
stored parameter. A far return therefore blurs only the SHOULDERS of the soft
box while its plateau stays flat — the interior verdict "within this class's
extent" survives at long range and only the boundary decisions soften.
"""

import numpy as np

from sonar_classification.argus.classes import ClassSet


def height_sensitivity(slant_range, cs: ClassSet) -> np.ndarray:
    """s_i = r_i cos(theta) (eq 21) — metres of height per radian of elevation."""
    r = np.asarray(slant_range, dtype=np.float64)
    return r * np.cos(cs.tilt_rad)


def elevation_variance(slant_range, cs: ClassSet) -> np.ndarray:
    """rho_var_i = (s_i Delta_phi)^2 / 12 (eq 22), in m^2. NaN propagates."""
    s = height_sensitivity(slant_range, cs)
    return (s * cs.aperture_rad) ** 2 / 12.0


def height_above_bed(z, z_bed) -> np.ndarray:
    """h_i = z_i - z_bed_hat(x_i, y_i) (eq 18).

    ``z`` is the boresight (phi = 0) height of the return and ``z_bed`` the
    locally fitted bottom at its XY. Where the bed reference is unknown the
    result is NaN and the position factor abstains — it never guesses over
    ground nobody has seen.
    """
    return np.asarray(z, dtype=np.float64) - np.asarray(z_bed, dtype=np.float64)
