#!/usr/bin/env python3
"""Unit tests for sonar_map.class_model — the learned class + responsibility loop.

Covers the two half-steps (log_likelihood / update), the eq-16 responsibility
softmax, and an end-to-end offline check that the streaming assign↔accumulate
loop recovers a set of generating Gaussians (learned means/stds converge, MAP
labels are recovered, by-tag Dirichlet mass is unbiased).

Run:  cd src/sonar_map && python3 -m pytest test/test_class_model.py -v
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sonar_map.class_model import (  # noqa: E402
    ClassModel, responsibilities, stack_log_likelihoods,
    SEABED, OBJECT, STRUCTURE)


def test_seed_intensity_sets_mean_and_var():
    m = ClassModel.seed_intensity(SEABED, mu=530.0, sigma=40.0, kappa0=3.0, a0=2.0)
    assert m.intensity_mean == pytest.approx(530.0)
    assert m.intensity_std == pytest.approx(40.0, rel=1e-9)
    assert m.evidence == pytest.approx(3.0)
    assert m.semantic_tag == SEABED


def test_log_likelihood_scalar_and_array_agree():
    m = ClassModel.seed_intensity(OBJECT, mu=800.0, sigma=100.0)
    x = np.array([600.0, 800.0, 1000.0])
    arr = m.log_likelihood(x)
    assert arr.shape == (3,)
    assert m.log_likelihood(800.0) == pytest.approx(float(arr[1]), rel=1e-12)


def test_responsibilities_uniform_prior_is_softmax_and_normalised():
    logL = np.array([[0.0, np.log(3.0), np.log(6.0)],
                     [1.0, 1.0, 1.0]])
    r = responsibilities(logL)
    assert np.allclose(r.sum(axis=1), 1.0)
    # row 0: proportions 1:3:6
    assert np.allclose(r[0], np.array([1.0, 3.0, 6.0]) / 10.0)
    # row 1: uniform
    assert np.allclose(r[1], np.full(3, 1.0 / 3.0))


def test_responsibilities_prior_tilts_assignment():
    logL = np.array([[0.0, 0.0]])            # likelihood indifferent
    log_prior = np.log(np.array([0.25, 0.75]))
    r = responsibilities(logL, log_prior)
    assert np.allclose(r[0], [0.25, 0.75])   # prior decides


def test_empty_model_list_shapes():
    logL = stack_log_likelihoods([], np.arange(5.0))
    assert logL.shape == (5, 0)
    r = responsibilities(logL)
    assert r.shape == (5, 0)


def test_update_accepts_scalar_and_array_weight():
    a = ClassModel.seed_intensity(SEABED, 0.0, 1.0, kappa0=1.0, a0=2.0)
    a.update(np.array([1.0, 1.0]), weight=1.0)      # array obs, scalar weight
    b = ClassModel.seed_intensity(SEABED, 0.0, 1.0, kappa0=1.0, a0=2.0)
    b.update(1.0, 1.0); b.update(1.0, 1.0)          # two scalars
    assert a.intensity_mean == pytest.approx(b.intensity_mean, rel=1e-12)
    assert a.evidence == pytest.approx(b.evidence, rel=1e-12)


def test_streaming_loop_recovers_generating_gaussians():
    """Three well-separated intensity populations, anchors seeded OFFSET from the
    truth. Streaming responsibility-weighted updates must migrate the means to the
    truth, recover ≥95% of MAP labels, and split the Dirichlet mass by population."""
    rng = np.random.default_rng(42)
    true_mu = np.array([100.0, 500.0, 900.0])
    true_sig = 40.0
    n_per = 3000
    X = np.concatenate([rng.normal(mu, true_sig, n_per) for mu in true_mu])
    label = np.concatenate([np.full(n_per, c) for c in range(3)])
    perm = rng.permutation(X.size)
    X, label = X[perm], label[perm]

    # Anchors seeded OFFSET from truth (130/470/870) with an informative prior
    # (σ0=60, the scale the real node uses from μ_I/δ_str) — so assignment is
    # near-hard from the first return and the means migrate cleanly to truth.
    models = [
        ClassModel.seed_intensity(SEABED,    130.0, 60.0, kappa0=1.0, a0=2.0, class_id=0),
        ClassModel.seed_intensity(OBJECT,    470.0, 60.0, kappa0=1.0, a0=2.0, class_id=1),
        ClassModel.seed_intensity(STRUCTURE, 870.0, 60.0, kappa0=1.0, a0=2.0, class_id=2),
    ]
    dirichlet_mass = np.zeros(3)

    chunk = 100
    for s in range(0, X.size, chunk):
        xi = X[s:s + chunk]
        logL = stack_log_likelihoods(models, xi)       # (n, 3)
        r = responsibilities(logL)                     # uniform prior (offline)
        dirichlet_mass += r.sum(axis=0)
        for c, m in enumerate(models):
            m.update(xi, r[:, c])

    # 1) learned means converged to the generating means
    learned_mu = np.array([m.intensity_mean for m in models])
    assert np.allclose(learned_mu, true_mu, atol=8.0)
    # 2) learned stds near the generating std
    learned_sig = np.array([m.intensity_std for m in models])
    assert np.allclose(learned_sig, true_sig, rtol=0.15)
    # 3) MAP labels under the final models recover the generating cluster
    final_logL = stack_log_likelihoods(models, X)
    pred = np.argmax(final_logL, axis=1)
    assert (pred == label).mean() >= 0.95
    # 4) Dirichlet mass split ~evenly (each population equal), sums to N
    assert dirichlet_mass.sum() == pytest.approx(X.size, rel=1e-9)
    assert np.allclose(dirichlet_mass, n_per, atol=0.05 * n_per)
