"""The class-inventory loader: it is the last line of defence against a config
that would silently produce a broken map."""

import numpy as np
import pytest

from conftest import MINIMAL_YAML, write_classes

from sonar_classification.argus.classes import load_classes
from sonar_classification.argus.region_descriptor import derive


def test_loads_the_inventory_in_file_order(cs):
    assert cs.names == ('seabed', 'vegetation', 'wall', 'pillar')
    assert cs.n_classes == 4
    assert cs.index('wall') == 2


def test_shape_lengths_convert_to_log_gaussian(cs):
    """median_m / spread_factor are the assignable form; the factor wants logs."""
    wall = cs.index('wall')
    assert np.isclose(cs.e_bar[wall], np.log(6.0))
    assert np.isclose(cs.e_std[wall], np.log(2.0))
    assert np.isclose(cs.w_bar[wall], np.log(0.10))


def test_shared_channels_are_bit_identical_between_wall_and_pillar(cs):
    """The responsibility is a ratio: shared parameters must CANCEL exactly.

    If a copy-paste drifted, wall and pillar would start separating on a
    channel whose physics is identical — a wrong answer that looks confident.
    """
    w, p = cs.index('wall'), cs.index('pillar')
    for name in ('l', 'u', 'tau', 'mu', 'sigma', 'rho', 'w_bar', 'w_std'):
        assert getattr(cs, name)[w] == getattr(cs, name)[p], name
    assert cs.e_bar[w] != cs.e_bar[p]          # ... and the discriminator differs


def test_globals_are_converted_to_radians(cs):
    assert np.isclose(cs.tilt_rad, np.deg2rad(15.0))
    assert np.isclose(cs.aperture_rad, np.deg2rad(20.0))
    assert np.isclose(cs.beamwidth_rad, np.deg2rad(1.0))


@pytest.mark.parametrize('subst,message', [
    (('rho: 0.90', 'rho: 1.0'), 'shape.rho'),
    (('tau: 0.30', 'tau: 0.0'), 'position.tau'),
    (('sigma: 5.0', 'sigma: 0.0'), 'intensity.sigma'),
    (('spread_factor: 2.0', 'spread_factor: 1.0'), 'spread_factor'),
    (('median_m: 6.00', 'median_m: 0.0'), 'median_m'),
    (('k_min: 3', 'k_min: 0'), 'k_min'),
    (('eps_xy: 0.30', 'eps_xy: 0.0'), 'eps_xy'),
])
def test_rejects_inadmissible_parameters(tmp_path, subst, message):
    bad = MINIMAL_YAML.replace(*subst)
    p = tmp_path / 'bad.yaml'
    p.write_text(bad)
    with pytest.raises(ValueError, match=message):
        load_classes(str(p))


def test_rejects_an_inverted_height_interval(tmp_path):
    bad = MINIMAL_YAML.replace('{l: 0.0, u: 1.5, tau: 0.35}',
                               '{l: 2.0, u: 1.5, tau: 0.35}')
    p = tmp_path / 'bad.yaml'
    p.write_text(bad)
    with pytest.raises(ValueError, match='position.u'):
        load_classes(str(p))


def test_rejects_duplicate_class_names(tmp_path):
    bad = MINIMAL_YAML.replace('- name: pillar', '- name: wall')
    p = tmp_path / 'bad.yaml'
    p.write_text(bad)
    with pytest.raises(ValueError, match='duplicate'):
        load_classes(str(p))


def test_zero_extent_class_is_accepted(tmp_path):
    """u == l is the legitimate seabed member, not a validation error."""
    cs = write_classes(tmp_path)
    assert cs.l[cs.index('seabed')] == cs.u[cs.index('seabed')] == 0.0


def test_ablation_defaults(cs):
    assert cs.ablation['shape_dedup_mode'] == 'per_voxel'
    assert cs.ablation['spatial_prior'] is True
    assert cs.ablation['position_gaussian'] is False
    assert cs.ablation['lidar_legacy_gap_ref'] is False


# ── the derived morphology descriptor ───────────────────────────────────────

#: This sonar at 2250 kHz: beams step 0.18 deg apart but each is 1 deg wide,
#: so ~5.6 beams fall inside one beamwidth.
SPACING = np.deg2rad(0.18)
WIDTH = np.deg2rad(1.0)


def test_derived_extent_uses_spacing_for_span_and_width_for_deconvolution():
    """e_R = N_R * rho * spacing - rho * width. Two DIFFERENT angles.

    Conflating them — the bug this test exists to prevent — would scale every
    extent by width/spacing (5.6x here) and then subtract the wrong amount.
    """
    rho_r = np.array([10.0])
    n_beams = np.array([20.0])                       # 20 * 0.18 = 3.6 deg span
    e_meas = n_beams * rho_r * SPACING
    area = e_meas * 0.2                              # a 0.2 m thick highlight
    e_r, w_r = derive(n_beams, np.log(area), rho_r, SPACING, WIDTH)

    assert np.isclose(e_r[0], 20.0 * 10.0 * SPACING - 10.0 * WIDTH)
    # Thickness divides the area by the MEASURED span, not the corrected one.
    assert np.isclose(w_r[0], 0.2)


def test_region_narrower_than_the_beam_has_no_resolved_extent():
    """Fewer than ~width/spacing beams means the target is sub-beam.

    You cannot resolve a target smaller than your beam, so the extent term is
    dropped (NaN) rather than reported as a spurious width or log(0). A pillar
    lives in exactly this regime.
    """
    n_in_one_beam = WIDTH / SPACING                  # ~5.6
    for n in (1.0, 3.0, np.floor(n_in_one_beam)):
        e_r, _ = derive(np.array([n]), np.array([np.log(0.1)]),
                        np.array([10.0]), SPACING, WIDTH)
        assert np.isnan(e_r[0]), n

    # Comfortably wider than a beam resolves normally.
    e_r, _ = derive(np.array([30.0]), np.array([np.log(0.5)]),
                    np.array([10.0]), SPACING, WIDTH)
    assert np.isfinite(e_r[0]) and e_r[0] > 0.0


def test_loader_rejects_spacing_wider_than_the_beam(tmp_path):
    """Beams stepping further apart than they are wide would leave gaps in the
    fan. No multibeam does that, so it almost certainly means the two were
    swapped."""
    bad = MINIMAL_YAML.replace('beamwidth_deg: 1.0',
                               'beamwidth_deg: 0.18\n      beam_spacing_deg: 1.0')
    p = tmp_path / 'bad.yaml'
    p.write_text(bad)
    with pytest.raises(ValueError, match='beam_spacing_deg'):
        load_classes(str(p))


def test_beam_spacing_defaults_to_the_beamwidth(cs):
    """An inventory written before beam_spacing_deg existed keeps its old
    behaviour rather than silently changing every extent."""
    assert np.isclose(cs.beam_spacing_rad, cs.beamwidth_rad)
