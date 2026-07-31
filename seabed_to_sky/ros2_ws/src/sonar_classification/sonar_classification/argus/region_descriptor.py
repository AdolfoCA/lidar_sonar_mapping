"""
region_descriptor — the morphology descriptor m_R = (e_R, w_R) of eq 31.

The descriptor set is deliberately just two lengths, the two axes the sonar
natively resolves:

    e_R  bearing extent   — lateral physical size. Splits wall (large) from
                            pillar (small). BEAM-WIDTH CORRECTED: the beam
                            convolves the scene, so the physical extent is
                            recovered by subtracting one beam footprint,
                                e_R = e_R_meas - rho_R * Delta_beta,
                            a per-measurement geometric correction of the same
                            status as rho_var_i — NOT a class property.
    w_R  range thickness  — radial penetration. Splits a hard face (scatters at
                            its surface, one or two range bins thick regardless
                            of lateral size) from a volume scatterer such as
                            vegetation (returns energy throughout its depth,
                            producing a thick ragged highlight). No beam
                            correction: range resolution is centimetres.

Known blind spot, documented rather than papered over: a bright,
bearing-elongated, range-thin bedform (a sand ripple) shares the wall
signature. It is left to the intensity and height channels and to the spatial
prediction.

WHERE THE NUMBERS COME FROM. Region growing runs upstream, in the leading-edge
extractor, on the CONDITIONED image (I1). Two source modes are supported:

  * ``native``  — the cloud already carries ``region_extent`` and
                  ``region_thickness`` in metres (plus ``region_id``). Used
                  verbatim. This is the target state; it needs the extractor to
                  publish the two lengths.
  * ``derived`` — the DEFAULT today, because the current extractor publishes
                  the older descriptor set (region_size N_R, region_logarea
                  a_R). Both lengths follow from those by geometry alone:

                      e_meas = N_R * rho_R * Delta_beta
                      e_R    = e_meas - rho_R * Delta_beta
                             = (N_R - 1) * rho_R * Delta_beta
                      w_R    = exp(a_R) / e_meas

                  The first is exact: N_R is the number of member BEAMS the
                  region spans, and each beam subtends Delta_beta at range
                  rho_R, so the measured lateral size is N_R beam footprints
                  and the beam-width correction removes exactly one of them.
                  The second is an EQUIVALENT-RECTANGLE approximation: the
                  extractor reports the region's physical area, and area
                  divided by measured lateral size is its mean radial
                  thickness. It is exact for a rectangular highlight and
                  biased low for a ragged one — acceptable, because w_R is
                  used on a log scale against a spread factor of ~2, but it is
                  an approximation and the ``native`` path should replace it.

Region identity. The per-voxel-per-sweep dedup gate d_i needs to know which
returns belong to the same region. ``native`` reads ``region_id`` directly;
``derived`` synthesises one by hashing the descriptor tuple, since two distinct
regions in one sweep sharing bit-identical float32 area AND beam count is not a
practical concern.
"""

import numpy as np


def _synthetic_region_id(n_beams, log_area) -> np.ndarray:
    """A per-sweep region id from the descriptor tuple (``derived`` mode).

    Returns float64 ids (-1 where the return belongs to no region), suitable
    for grouping within ONE sweep. Ids are not stable across sweeps and must
    never be used for anything but the d_i dedup, which is per-sweep by
    definition.
    """
    n_beams = np.asarray(n_beams, dtype=np.float64)
    log_area = np.asarray(log_area, dtype=np.float64)
    key = np.stack([np.nan_to_num(n_beams, nan=-1.0),
                    np.round(np.nan_to_num(log_area, nan=-1e9), 6)], axis=1)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    return inv.ravel().astype(np.float64)


def derive(n_beams, log_area, region_range, beam_spacing_rad: float,
           beamwidth_rad: float):
    """(e_R, w_R) in metres from the legacy descriptor channels.

    TWO DIFFERENT ANGLES ARE INVOLVED, and conflating them is a large error:

      * BEAM SPACING is the angular step between adjacent beams. It converts a
        count of member beams into an angular SPAN:
            e_meas = N_R * rho_R * spacing
      * BEAM WIDTH is the angular size of one beam — the kernel the scene is
        convolved with. It is what gets DECONVOLVED, once:
            e_R = e_meas - rho_R * width

    On this sonar the two differ by more than 5x (0.18 deg spacing against a
    1 deg beam), because the beams deliberately oversample the aperture. Using
    the width for both would multiply every extent by that ratio and then
    subtract the wrong amount — turning point targets into metre-wide walls.

    A CONSEQUENCE WORTH EXPECTING: with ~5.6 beams inside one beamwidth, a
    region spanning fewer than that many beams is NARROWER THAN THE BEAM and
    has no resolvable lateral size at all. Such regions return NaN extent, and
    the morphology factor drops the extent term for them rather than inventing
    a width. That is the honest answer — you cannot resolve a target smaller
    than your beam — and it is exactly the regime a pillar lives in.

    Parameters
    ----------
    n_beams           N_R, member beams of the region.
    log_area          a_R, log physical area of the region [log m^2].
    region_range      rho_R, the region's slant range [m].
    beam_spacing_rad  angular step between adjacent beams [rad].
    beamwidth_rad     angular width of one beam [rad].

    Non-region returns (NaN inputs) come back NaN, which makes the morphology
    factor take its "no region" branch rather than inventing a shape.
    """
    n = np.asarray(n_beams, dtype=np.float64)
    a = np.asarray(log_area, dtype=np.float64)
    rho_r = np.asarray(region_range, dtype=np.float64)

    e_meas = n * rho_r * beam_spacing_rad          # angular span -> metres
    e_r = e_meas - rho_r * beamwidth_rad           # deconvolve ONE beam

    with np.errstate(divide='ignore', invalid='ignore'):
        w_r = np.exp(a) / e_meas

    # Unresolved (narrower than a beam) or degenerate -> NaN, never log(0).
    bad_e = ~np.isfinite(e_r) | (e_r <= 0.0)
    bad_w = ~np.isfinite(w_r) | (w_r <= 0.0)
    e_r = np.where(bad_e, np.nan, e_r)
    w_r = np.where(bad_w, np.nan, w_r)
    return e_r, w_r


def build(flag, *, n_beams=None, log_area=None, region_range=None,
          extent=None, thickness=None, region_id=None,
          beam_spacing_rad: float = 0.0, beamwidth_rad: float = 0.0,
          mode: str = 'derived'):
    """Assemble (s_m, e_R, w_R, region_id) for one sweep.

    ``flag`` is the extractor's membership indicator s^m_i: 1 = the return's
    leading edge falls in a grown region, 0 = a valid return in no region (the
    ABSENCE of morphology, itself informative), NaN = the extractor did not run
    at all (the factor abstains entirely).
    """
    s_m = np.asarray(flag, dtype=np.float64)

    if mode == 'native':
        e_r = np.asarray(extent, dtype=np.float64)
        w_r = np.asarray(thickness, dtype=np.float64)
        rid = (np.asarray(region_id, dtype=np.float64) if region_id is not None
               else _synthetic_region_id(e_r, w_r))
    else:
        e_r, w_r = derive(n_beams, log_area, region_range,
                          beam_spacing_rad, beamwidth_rad)
        rid = _synthetic_region_id(n_beams, log_area)

    # Membership is only meaningful where a descriptor actually exists.
    rid = np.where(s_m == 1.0, rid, -1.0)
    return s_m, e_r, w_r, rid
