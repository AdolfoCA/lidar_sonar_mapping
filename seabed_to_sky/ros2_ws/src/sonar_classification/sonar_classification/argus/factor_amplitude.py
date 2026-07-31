"""
factor_amplitude — p(A_i | c), the backscatter factor.

    p(A_i | c) = N(A_i; mu_c, sigma_c^2)

A_i is the RAW uint16 count read from the sonar image at the detected cell —
the linear envelope amplitude the sonar produces, with no conversion. mu_c and
sigma_c are therefore in COUNTS.

WHAT THE COUNT IS. The vendor SDK forms it as

    count = min( trunc( sqrt(re^2 + im^2) * norm[beam] ), 65535 )

i.e. the magnitude of the beamformed complex sample, scaled by a fixed per-beam
calibration constant and truncated. It is an AMPLITUDE (pressure), not a power,
and it is not normalised per ping — the map from (cell, raw sample) to count is
fixed, which is what lets a class own a level at all.

THREE PROPERTIES OF THIS MODEL, STATED SO THEY ARE NOT REDISCOVERED AS BUGS:

  * sigma_c CANNOT MEAN "MATERIAL VARIABILITY". Speckle is Rayleigh, so
    sigma/mu is pinned near 0.52 at every level, and a class's spread is
    therefore dominated by its brightness. A class at 20000 counts will have a
    spread roughly 7x one at 2800 counts even if the two materials are equally
    homogeneous. Expect bright classes to have wide skirts that reach down into
    dim ones.
  * THE LEVEL IS TIED TO RANGE AND TO THE ACQUISITION SETTINGS. The count still
    carries the two-way transmission loss and the head's TVG and gain, so a
    class level measured at 6-9 m does not transfer to a pass at 2-5 m, and any
    change to the head's Gain or TVGSlope invalidates every mu_c. Re-measure
    after any change to either.
  * A GAUSSIAN PUTS MASS BELOW ZERO. With sigma/mu ~ 0.5, ~2.8% of each class's
    probability lies at negative counts, which cannot occur. It is a modelling
    approximation, not an error in the data.

If those become limiting, the principled upgrade inside the linear domain is a
Rayleigh or Nakagami likelihood — it matches the physics of the count exactly
and keeps a level parameter that means what it says.

ABSTENTION: a count at or below zero is "below the noise floor", not a
measurement of zero brightness, so the row is zeros and the remaining factors
still combine.
"""

import numpy as np

from sonar_classification.argus.classes import ClassSet

_LOG_2PI = float(np.log(2.0 * np.pi))


def log_lik(count_raw, cs: ClassSet) -> np.ndarray:
    """(N, C) log N(A_i; mu_c, sigma_c^2). Abstained rows are zero.

    Parameters
    ----------
    count_raw : (N,) the pristine uint16 count at each detected cell. Read from
        the UNCONDITIONED image — detection may run on a filtered copy, but the
        measured value never does, or the level becomes a property of the
        processing rather than of the scene.
    """
    a = np.asarray(count_raw, dtype=np.float64)

    abstain = ~np.isfinite(a) | (a <= 0.0)
    a_s = np.where(abstain, 0.0, a)[:, None]                    # (N, 1)

    sigma = cs.sigma[None, :]
    out = -0.5 * (_LOG_2PI + 2.0 * np.log(sigma)
                  + ((a_s - cs.mu[None, :]) / sigma) ** 2)
    return np.where(abstain[:, None], 0.0, out)
