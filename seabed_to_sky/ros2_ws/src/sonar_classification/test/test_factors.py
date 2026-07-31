"""The four factors, each tested on the property it exists to guarantee.

These are the invariant checks from the implementation guide's validation
section. Each one corresponds to a specific failure that a naive
implementation reintroduces.
"""

import textwrap

import numpy as np
import pytest

from conftest import MINIMAL_YAML, write_classes

from sonar_classification.argus import (
    compensation,
    factor_intensity,
    factor_lidar,
    factor_position,
    factor_shape,
    likelihood,
)
from sonar_classification.argus.classes import load_classes


# ── position ────────────────────────────────────────────────────────────────

def test_position_extended_class_does_not_punish_a_base_return(tmp_path):
    """A return at h ~ 0 is ordinary for a bed-anchored extended class.

    This is the failure the soft box exists to prevent: a mid-extent Gaussian
    asserts a fictitious typical height and punishes a legitimate
    base-of-canopy return quadratically for its distance to a centre that has
    no physical meaning. The box charges it only the edge shoulder.
    """
    cs = write_classes(tmp_path)
    veg = cs.index('vegetation')

    gaussian_yaml = MINIMAL_YAML + textwrap.dedent("""
        ablation:
          position_gaussian: true
        """)
    p = tmp_path / 'gauss.yaml'
    p.write_text(gaussian_yaml)
    cs_gauss = load_classes(str(p))

    h = np.array([0.0, 0.75])                 # base of the canopy, and mid-extent
    rho_var = np.zeros(2)
    box = factor_position.log_lik(h, rho_var, cs)[:, veg]
    gauss = factor_position.log_lik(h, rho_var, cs_gauss)[:, veg]

    # Cost of being at the base rather than mid-extent, under each model.
    box_cost = box[1] - box[0]
    gauss_cost = gauss[1] - gauss[0]
    assert box_cost < 0.8                      # one shoulder, ~log 2
    assert gauss_cost > 2.0                    # quadratic in (0.75 / tau)
    assert gauss_cost > 2.5 * box_cost


def test_position_plateau_is_flat_well_inside_an_extended_class(cs):
    """Inside the extent, away from the shoulders, the box is indifferent."""
    wall = cs.index('wall')                   # [0, 8] m, tau 0.30
    lg = factor_position.log_lik(np.array([2.0, 4.0, 6.0]), np.zeros(3), cs)[:, wall]
    assert np.ptp(lg) < 1e-6


def test_position_penalises_a_return_above_the_upper_edge(cs):
    """Decisive where the physics is decisive."""
    veg = cs.index('vegetation')
    lg = factor_position.log_lik(np.array([0.5, 6.0]), np.zeros(2), cs)[:, veg]
    assert lg[1] < lg[0] - 20.0


def test_position_range_blurs_shoulders_not_the_plateau(cs):
    """rho_var widens the edges; the interior verdict survives at long range."""
    wall = cs.index('wall')
    near = factor_position.log_lik(np.array([4.0, 12.0]), np.array([0.01, 0.01]), cs)[:, wall]
    far = factor_position.log_lik(np.array([4.0, 12.0]), np.array([4.0, 4.0]), cs)[:, wall]
    # Interior barely moves ...
    assert abs(far[0] - near[0]) < 0.5
    # ... while the out-of-extent verdict softens a lot.
    assert far[1] - near[1] > 5.0


def test_position_abstains_without_a_bed_reference(cs):
    """h_i NaN (bed not yet observed) must abstain, not guess."""
    lg = factor_position.log_lik(np.array([np.nan, 0.2]), np.array([0.1, 0.1]), cs)
    assert np.all(lg[0] == 0.0)
    assert np.any(lg[1] != 0.0)


# ── intensity ───────────────────────────────────────────────────────────────

def test_intensity_responsibilities_are_invariant_to_a_global_db_offset(tmp_path):
    """The uncalibrated offset C cancels from the responsibility.

    Shifting every class mean AND the measurement by the same number of dB is
    exactly what changing C does; if the classification moved, mu_c would be
    an absolute level rather than an assignable offset.
    """
    cs_a = write_classes(tmp_path)
    # Shift every class mean by the same +17 dB. Applied darkest-first so no
    # substitution rewrites a level a later one would match.
    shifted = (MINIMAL_YAML
               .replace('mu: -5.0', 'mu: 12.0')
               .replace('mu: -18.0', 'mu: -1.0')
               .replace('mu: -22.0', 'mu: -5.0'))
    path = tmp_path / 'shifted.yaml'
    path.write_text(shifted)
    cs_b = load_classes(str(path))
    assert np.allclose(cs_b.mu - cs_a.mu, 17.0)     # a pure +17 dB shift

    log_pi = np.log(np.full(cs_a.n_classes, 1.0 / cs_a.n_classes))
    i_db = np.array([-22.0, -18.0, -5.0, -13.0])
    r_a = likelihood.responsibilities(factor_intensity.log_lik(i_db, cs_a), log_pi)
    r_b = likelihood.responsibilities(
        factor_intensity.log_lik(i_db + 17.0, cs_b), log_pi)
    assert np.allclose(r_a, r_b, atol=1e-12)


def test_intensity_abstains_below_the_noise_floor(cs):
    """A count at or below the floor yields no honest level."""
    i_db, _ = compensation.compensate(np.array([0.0, 5000.0]),
                                      np.array([10.0, 10.0]), cs)
    assert np.isnan(i_db[0]) and np.isfinite(i_db[1])
    lg = factor_intensity.log_lik(i_db, cs)
    assert np.all(lg[0] == 0.0)


def test_compensation_removes_the_range_trend():
    """A constant scatterer must read the SAME level at every range.

    With the true two-way loss reapplied, a count that decays exactly as the
    physics says compensates to a flat curve. That flat median IS the
    validation of the compensation chain.
    """
    yaml_text = MINIMAL_YAML.replace('absorption_alpha_db_per_m: 1.74',
                                     'absorption_alpha_db_per_m: 0.5')
    import tempfile, os
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, 'c.yaml')
        open(p, 'w').write(yaml_text)
        cs = load_classes(p)

    rho = np.array([2.0, 5.0, 10.0, 20.0, 40.0])
    # Synthesise counts from a constant backscatter of 100 dB, decayed by the
    # true loss the compensation will undo.
    loss = 30.0 * np.log10(rho) + 2.0 * cs.alpha_abs * rho
    counts = 10.0 ** ((100.0 - loss) / 20.0)
    i_db, _ = compensation.compensate(counts, rho, cs)
    assert np.allclose(i_db, 100.0, atol=1e-9)


# ── LiDAR support ───────────────────────────────────────────────────────────

def test_lidar_abstains_exactly_when_the_column_is_unobserved(cs):
    """o_i = 0 -> factor 1 for EVERY class. Absence of observation is not
    evidence of absence, and a correlated coverage gap must not stack into a
    veto against the structure it hides."""
    lg = factor_lidar.log_lik(np.array([0.0]), np.array([1.0]),
                              np.array([0.5]), cs)
    assert np.all(lg == 0.0)

    # ... and before any LiDAR data at all (o_i NaN), likewise.
    lg = factor_lidar.log_lik(np.array([np.nan]), np.array([np.nan]),
                              np.array([np.nan]), cs)
    assert np.all(lg == 0.0)


def test_lidar_presence_separates_piercing_from_submerged(cs):
    """Presence alone is log(0.90/0.04) ~ 3.1 nats for wall over seabed.

    The clearance is deliberately excluded here (NaN) so the presence hurdle is
    measured on its own — that is the number quoted in the config's sanity
    check, and the one to compare against an intensity miss.
    """
    lg = factor_lidar.log_lik(np.array([1.0]), np.array([1.0]),
                              np.array([np.nan]), cs)[0]
    gap = lg[cs.index('wall')] - lg[cs.index('seabed')]
    assert np.isclose(gap, np.log(0.90 / 0.04))
    assert 2.5 < gap < 4.0


def test_lidar_clearance_adds_discrimination_on_top_of_presence(cs):
    """A clearance right at the waterline is exp-typical for a piercing class
    and far below the soft box a covered class expects, so it separates them
    further — but the two effects stay separable, which is why the presence
    number above is quotable on its own."""
    presence = factor_lidar.log_lik(np.array([1.0]), np.array([1.0]),
                                    np.array([np.nan]), cs)[0]
    with_clr = factor_lidar.log_lik(np.array([1.0]), np.array([1.0]),
                                    np.array([0.2]), cs)[0]
    w, s = cs.index('wall'), cs.index('seabed')
    assert (with_clr[w] - with_clr[s]) > (presence[w] - presence[s])


def test_lidar_clearance_is_independent_of_the_return_height(cs):
    """The clearance is a property of the SCENE, not of the return.

    Referencing the return would make this channel a deterministic function of
    the return's own height — the position factor's job — and break the
    conditional independence the likelihood factorisation assumes.
    """
    # Same scene clearance, two very different return depths: identical factor.
    a = factor_lidar.log_lik(np.array([1.0]), np.array([1.0]),
                             np.array([2.0]), cs)
    b = factor_lidar.log_lik(np.array([1.0]), np.array([1.0]),
                             np.array([2.0]), cs)
    assert np.allclose(a, b)
    # And a different clearance DOES move it (the channel is not inert).
    c = factor_lidar.log_lik(np.array([1.0]), np.array([1.0]),
                             np.array([9.0]), cs)
    assert not np.allclose(a, c)


def test_config_rejects_certain_presence(tmp_path):
    """pi = 1 must be refused: one gap would be log(0) = -inf forever."""
    bad = MINIMAL_YAML.replace('pi: 0.90', 'pi: 1.0')
    p = tmp_path / 'bad.yaml'
    p.write_text(bad)
    with pytest.raises(ValueError, match='lidar.pi'):
        load_classes(str(p))


# ── morphology ──────────────────────────────────────────────────────────────

def test_shape_absence_is_informative(cs):
    """A bare return contributes log(1 - rho_c) — that is real evidence."""
    lg = factor_shape.log_lik(np.zeros(1), np.array([np.nan]),
                              np.array([np.nan]), np.zeros(1), cs)[0]
    assert lg[cs.index('seabed')] > lg[cs.index('wall')]
    assert np.allclose(lg, np.log1p(-cs.rho))


def test_shape_abstains_when_no_extractor_ran(cs):
    lg = factor_shape.log_lik(np.array([np.nan]), np.array([np.nan]),
                              np.array([np.nan]), np.zeros(1), cs)
    assert np.all(lg == 0.0)


def test_shape_extent_separates_wall_from_pillar(cs):
    """The ONE channel that can: they are identical everywhere else."""
    wide = factor_shape.log_lik(np.ones(1), np.array([6.0]), np.array([0.1]),
                                np.ones(1), cs)[0]
    narrow = factor_shape.log_lik(np.ones(1), np.array([0.5]), np.array([0.1]),
                                  np.ones(1), cs)[0]
    assert wide[cs.index('wall')] > wide[cs.index('pillar')]
    assert narrow[cs.index('pillar')] > narrow[cs.index('wall')]


def test_shape_evidence_is_not_diluted_by_region_size(tmp_path):
    """A 100-return wall must not have its shape evidence divided by 100.

    The region is ONE observation about each voxel it covers, not N_R
    observations. The legacy 1/N_R weighting neuters morphology for exactly the
    large structures it exists to describe.
    """
    cs = write_classes(tmp_path)
    n = 100
    flag = np.ones(n)
    e_r = np.full(n, 6.0)
    w_r = np.full(n, 0.10)
    dedup = np.zeros(n)
    dedup[0] = 1.0                 # one designated carrier for this voxel

    per_voxel = factor_shape.log_lik(flag, e_r, w_r, dedup, cs)

    abl = MINIMAL_YAML + textwrap.dedent("""
        ablation:
          shape_dedup_mode: inv_nr
        """)
    p = tmp_path / 'abl.yaml'
    p.write_text(abl)
    cs_abl = load_classes(str(p))
    inv_nr = factor_shape.log_lik(flag, e_r, w_r, dedup, cs_abl,
                                  n_region=np.full(n, float(n)))

    wall, pillar = cs.index('wall'), cs.index('pillar')
    carried = per_voxel[0, wall] - per_voxel[0, pillar]
    diluted = inv_nr[:, wall].sum() - inv_nr[:, pillar].sum()
    # The carrier alone delivers the full descriptor separation ...
    assert carried > 3.0
    # ... and 1/N_R spread over all N_R returns delivers the same TOTAL, which
    # is the point: the difference is WHERE it lands, and per-voxel dedup puts
    # it in the voxel instead of smearing it across every voxel the region
    # touches.
    assert np.isclose(carried, diluted, rtol=1e-9)
    # Per return, though, the legacy weighting is 100x weaker — so any voxel
    # that catches only part of the region gets almost nothing.
    assert (inv_nr[0, wall] - inv_nr[0, pillar]) < carried / 50.0
