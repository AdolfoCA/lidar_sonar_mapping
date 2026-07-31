#!/usr/bin/env python3
"""Unit tests for sonar_map.class_registry — dynamic class creation.

Pins the assignment-over-active∪{new} normalisation, the sustained-novelty
creation trigger (spawns on a coherent group, with the correct nearest-anchor
tag), the count-gate boundary, and the confirmation-budget guarantee that an
isolated outlier cannot spawn a class.

Run:  cd src/sonar_map && python3 -m pytest test/test_class_registry.py -v
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sonar_map.conjugate import NIG                                    # noqa: E402
from sonar_map.class_model import (                                    # noqa: E402
    ClassModel, SEABED, OBJECT, STRUCTURE)
from sonar_map.class_registry import ClassRegistry                     # noqa: E402


def _fresh_registry(**kw):
    # Anchors seeded as STABLE semantic priors (κ0=a0=10): confident enough that
    # a far intensity population cannot drag an anchor over to absorb it, so
    # genuine novelty is forced into a new class instead. This seed strength is
    # itself a class-generation knob (weaker → anchors drift and swallow novelty).
    anchors = [
        ClassModel.seed_intensity(SEABED,    100.0, 60.0, kappa0=10.0, a0=10.0, class_id=0),
        ClassModel.seed_intensity(OBJECT,    500.0, 60.0, kappa0=10.0, a0=10.0, class_id=1),
        ClassModel.seed_intensity(STRUCTURE, 900.0, 60.0, kappa0=10.0, a0=10.0, class_id=2),
    ]
    # Base measure Λ_H: UNCERTAIN mean (tiny κ0 → wide prior predictive that
    # catches novelty) but a SENSIBLE within-class variance prior (σ0≈100, so a
    # created class sharpens and claims its own returns instead of re-spawning).
    lambda_h = NIG(m=500.0, kappa=0.01, a=2.0, b=2.0 * 100.0 ** 2)
    params = dict(gamma=0.5, novelty_return_floor=0.5,
                  novelty_min_count=20, novelty_mass_thresh=10.0)
    params.update(kw)
    return ClassRegistry(anchors, lambda_h, **params)


def test_assign_responsibilities_normalised():
    reg = _fresh_registry()
    x = np.array([100.0, 500.0, 900.0, 1300.0, 50.0, 1500.0])
    r_active, r_new = reg.assign(x)
    total = r_active.sum(axis=1) + r_new
    assert np.allclose(total, 1.0, atol=1e-12)
    # A far, bright return is dominated by the novelty column.
    assert r_new[3] > 0.5 and r_new[5] > 0.5
    # An on-anchor return is claimed by its class, not novelty.
    assert r_new[0] < 0.1


def test_base_populations_do_not_spawn():
    """Well-explained returns (r_new < floor) never accrue novelty, even when
    tightly clustered in one cell."""
    rng = np.random.default_rng(3)
    reg = _fresh_registry()
    for mu, key in [(100.0, -1), (500.0, -2), (900.0, -3)]:
        x = rng.normal(mu, 40.0, 500)
        reg.step(x, np.full(x.size, key))
    assert reg.n_classes == 3


def test_spawns_on_sustained_novel_population_with_correct_tag():
    rng = np.random.default_rng(11)
    reg = _fresh_registry()

    # Base populations (clustered, well-explained) + a sustained novel population
    # at 1300 (brighter than every anchor), all sharing novelty cell 999.
    parts = []
    for mu, key in [(100.0, -1), (500.0, -2), (900.0, -3)]:
        xi = rng.normal(mu, 40.0, 1000)
        parts.append((xi, np.full(xi.size, key)))
    novel = rng.normal(1300.0, 40.0, 300)
    parts.append((novel, np.full(novel.size, 999)))
    X = np.concatenate([p[0] for p in parts])
    K = np.concatenate([p[1] for p in parts])
    perm = rng.permutation(X.size)
    X, K = X[perm], K[perm]

    created = []
    for s in range(0, X.size, 50):
        _, _, made = reg.step(X[s:s + 50], K[s:s + 50])
        created += made

    assert len(created) == 1                       # exactly one new class
    assert reg.n_classes == 4
    new_class = reg.classes[-1]
    assert new_class.class_id == created[0]
    assert new_class.intensity_mean == pytest.approx(1300.0, abs=80.0)
    # Nearest anchor to 1300 is STRUCTURE (900), so it inherits that tag.
    assert new_class.semantic_tag == STRUCTURE

    # After creation the new class claims its returns (novelty collapses there).
    _, r_new_after = reg.assign(np.array([1300.0]))
    assert r_new_after[0] < 0.2


def test_count_gate_boundary_and_outlier_cannot_spawn():
    reg = _fresh_registry()
    novel_x = 1300.0

    # 19 sustained novel returns (mass ~19 ≥ 10, but count 19 < 20): NO spawn.
    x19 = np.full(19, novel_x)
    _, _, made = reg.step(x19, np.full(19, 999))
    assert made == [] and reg.n_classes == 3

    # One more return trips the count gate → spawn.
    _, _, made = reg.step(np.array([novel_x]), np.array([999]))
    assert len(made) == 1 and reg.n_classes == 4


def test_isolated_outlier_does_not_spawn():
    """A single bright outlier (count 1) cannot instantiate a class — the
    confirmation budget κ0/a0 keeps the candidate near the prior."""
    reg = _fresh_registry()
    _, _, made = reg.step(np.array([2000.0]), np.array([999]))
    assert made == [] and reg.n_classes == 3


def test_unsupervised_empty_start_grows_three_classes():
    """Fully unsupervised: start with NO classes; three distinct intensity modes
    each become their own class, learned from data, tagged dim->seabed /
    bright->structure by brightness relative to the dimmest class."""
    rng = np.random.default_rng(5)
    lambda_h = NIG(800.0, 0.01, 2.0, 2.0 * 200.0 ** 2)
    reg = ClassRegistry([], lambda_h, gamma=0.5, novelty_return_floor=0.5,
                        novelty_min_count=25, novelty_mass_thresh=12.0)
    assert reg.n_classes == 0

    modes = [(300.0, 0), (1200.0, 1), (2500.0, 2)]   # (mean, own voxel)
    parts = [(rng.normal(mu, 60.0, 200), np.full(200, k)) for mu, k in modes]
    X = np.concatenate([p[0] for p in parts])
    K = np.concatenate([p[1] for p in parts])
    perm = rng.permutation(X.size)
    X, K = X[perm], K[perm]
    for s in range(0, X.size, 40):
        reg.step(X[s:s + 40], K[s:s + 40])

    assert reg.n_classes == 3
    mus = np.array(sorted(c.intensity_mean for c in reg.classes))
    assert np.allclose(mus, [300.0, 1200.0, 2500.0], atol=120.0)

    tags = reg.brightness_tags(object_offset=400.0, structure_offset=1000.0)
    order = np.argsort([c.intensity_mean for c in reg.classes])
    assert [tags[i] for i in order] == [SEABED, OBJECT, STRUCTURE]


def test_no_duplicate_class_for_same_signature_across_voxels():
    """The SAME intensity mode appearing in several voxels in one scan must yield
    ONE class, not one per voxel (create_merge_sigmas dedup guard)."""
    lambda_h = NIG(800.0, 0.01, 2.0, 2.0 * 200.0 ** 2)
    reg = ClassRegistry([], lambda_h, gamma=0.5, novelty_return_floor=0.5,
                        novelty_min_count=25, novelty_mass_thresh=12.0,
                        create_merge_sigmas=3.0)
    rng = np.random.default_rng(2)
    x = np.concatenate([rng.normal(1200.0, 40.0, 30) for _ in range(3)])
    k = np.concatenate([np.full(30, v) for v in (10, 20, 30)])   # 3 distinct voxels
    reg.step(x, k)
    assert reg.n_classes == 1
    assert abs(reg.classes[0].intensity_mean - 1200.0) < 80.0


def test_distinct_signatures_same_scan_both_create():
    """Two genuinely different novel modes in the same scan DO both spawn."""
    lambda_h = NIG(800.0, 0.01, 2.0, 2.0 * 200.0 ** 2)
    reg = ClassRegistry([], lambda_h, gamma=0.5, novelty_return_floor=0.5,
                        novelty_min_count=25, novelty_mass_thresh=12.0,
                        create_merge_sigmas=3.0)
    rng = np.random.default_rng(3)
    x = np.concatenate([rng.normal(400.0, 40.0, 30), rng.normal(3000.0, 40.0, 30)])
    k = np.concatenate([np.full(30, 10), np.full(30, 20)])
    reg.step(x, k)
    assert reg.n_classes == 2


def test_brightness_tags_relative_to_dimmest():
    lambda_h = NIG(800.0, 0.01, 2.0, 2.0 * 200.0 ** 2)
    reg = ClassRegistry([
        ClassModel.seed_intensity(SEABED, 500.0, 50.0, 10.0, 10.0, 0),
        ClassModel.seed_intensity(SEABED, 950.0, 50.0, 10.0, 10.0, 1),
        ClassModel.seed_intensity(SEABED, 1800.0, 50.0, 10.0, 10.0, 2),
    ], lambda_h)
    # dimmest=500 → seabed; 950 is +450 → object; 1800 is +1300 → structure
    tags = reg.brightness_tags(object_offset=400.0, structure_offset=1000.0)
    assert tags == [SEABED, OBJECT, STRUCTURE]
    reg.apply_tags(tags)
    assert [c.semantic_tag for c in reg.classes] == [SEABED, OBJECT, STRUCTURE]


def test_tag_mass_collapse_to_three():
    reg = _fresh_registry()
    # Force a 4th class tagged STRUCTURE by promoting a novel population.
    x = np.full(40, 1300.0)
    reg.step(x, np.full(40, 999))
    assert reg.n_classes == 4
    r_active = np.zeros((1, 4))
    r_active[0] = [0.1, 0.2, 0.3, 0.4]             # sb, obj, str-anchor, str-new
    tag_mass = reg.tag_mass(r_active)
    assert tag_mass.shape == (1, 3)
    # class 3 is STRUCTURE-tagged, so its 0.4 adds to the structure column.
    assert tag_mass[0, STRUCTURE] == pytest.approx(0.3 + 0.4)
    assert tag_mass[0, SEABED] == pytest.approx(0.1)
    assert tag_mass[0, OBJECT] == pytest.approx(0.2)
    assert tag_mass.sum() == pytest.approx(1.0)
