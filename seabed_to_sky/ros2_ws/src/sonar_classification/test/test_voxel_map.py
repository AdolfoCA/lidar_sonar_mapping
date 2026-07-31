"""The Dirichlet map: order independence, the spatial prediction, the census."""

import numpy as np

from sonar_classification.argus import likelihood
from sonar_classification.argus.voxel_map import VoxelMap, pack, unpack


def _sweep(vmap, keys, log_L):
    """One sweep, exactly as classification_node runs it: score everything
    against the frozen snapshot FIRST, then accumulate, then commit."""
    pi = vmap.spatial_prediction(keys)
    r = likelihood.responsibilities(log_L, np.log(pi))
    vmap.accumulate(keys, r)
    vmap.commit()
    return r


def test_voxel_key_roundtrip():
    ix = np.array([-100000, -1, 0, 1, 100000])
    iy = np.array([5, -5, 0, 7, -7])
    iz = np.array([0, 1, -1, 999, -999])
    got = unpack(pack(ix, iy, iz))
    assert np.array_equal(got[0], ix)
    assert np.array_equal(got[1], iy)
    assert np.array_equal(got[2], iz)


def test_sweep_result_is_independent_of_return_order():
    """The map must not depend on the order returns arrive in.

    This is what the frozen (t-1) snapshot buys: every responsibility in a
    sweep is computed against the same state, and accumulation is a sum.
    Reading live neighbour beliefs mid-sweep would make the result a function
    of the message's point ordering.
    """
    rng = np.random.default_rng(0)
    n, c = 200, 4
    log_L = rng.normal(size=(n, c))
    xyz = rng.uniform(-2.0, 2.0, size=(n, 3))

    a = VoxelMap(c, resolution=0.2, alpha_0=0.01, eps=0.5)
    keys_a = a.keys_of(xyz[:, 0], xyz[:, 1], xyz[:, 2])
    # Two sweeps, so the second genuinely reads a populated snapshot.
    _sweep(a, keys_a, log_L)
    _sweep(a, keys_a, log_L)

    perm = rng.permutation(n)
    b = VoxelMap(c, resolution=0.2, alpha_0=0.01, eps=0.5)
    keys_b = b.keys_of(xyz[perm, 0], xyz[perm, 1], xyz[perm, 2])
    _sweep(b, keys_b, log_L[perm])
    _sweep(b, keys_b, log_L[perm])

    ka, ma, Ma = a.snapshot_arrays()
    kb, mb, Mb = b.snapshot_arrays()
    oa, ob = np.argsort(ka), np.argsort(kb)
    assert np.array_equal(ka[oa], kb[ob])
    assert np.allclose(ma[oa], mb[ob], atol=1e-12)
    assert np.allclose(Ma[oa], Mb[ob], atol=1e-12)


def test_spatial_prediction_falls_back_to_uniform_with_no_evidence():
    vmap = VoxelMap(3, resolution=0.2, alpha_0=0.01, eps=0.5)
    pi = vmap.spatial_prediction(vmap.keys_of([0.0], [0.0], [0.0]))
    assert np.allclose(pi, 1.0 / 3.0)


def test_spatial_prediction_is_evidence_weighted():
    """A well-observed neighbour dominates; an unobserved one carries no weight."""
    vmap = VoxelMap(3, resolution=0.2, alpha_0=0.01, eps=0.5)
    # Pile evidence for class 0 into one voxel, a whisker for class 2 nearby.
    heavy = vmap.keys_of([0.0], [0.0], [0.0])
    light = vmap.keys_of([0.1], [0.0], [0.0])
    vmap.accumulate(heavy, np.array([[100.0, 0.0, 0.0]]))
    vmap.accumulate(light, np.array([[0.0, 0.0, 1.0]]))
    vmap.commit()

    pi = vmap.spatial_prediction(vmap.keys_of([0.05], [0.0], [0.0]))[0]
    assert pi.argmax() == 0
    assert pi[0] > 0.95


def test_spatial_prediction_reads_the_frozen_snapshot_only():
    """Accumulating without committing must not change what a query returns."""
    vmap = VoxelMap(3, resolution=0.2, alpha_0=0.01, eps=0.5)
    k = vmap.keys_of([0.0], [0.0], [0.0])
    vmap.accumulate(k, np.array([[10.0, 0.0, 0.0]]))
    vmap.commit()
    before = vmap.spatial_prediction(k).copy()

    vmap.accumulate(k, np.array([[0.0, 100.0, 0.0]]))       # no commit
    assert np.allclose(vmap.spatial_prediction(k), before)

    vmap.commit()
    assert not np.allclose(vmap.spatial_prediction(k), before)


def test_disabled_prior_skips_the_snapshot_entirely():
    vmap = VoxelMap(3, resolution=0.2, alpha_0=0.01, eps=0.0)
    assert not vmap.use_prior
    k = vmap.keys_of([0.0], [0.0], [0.0])
    vmap.accumulate(k, np.array([[10.0, 0.0, 0.0]]))
    vmap.commit()
    assert np.allclose(vmap.spatial_prediction(k), 1.0 / 3.0)


def test_posterior_mean_and_map_label():
    vmap = VoxelMap(3, resolution=0.2, alpha_0=0.01, eps=0.0)
    k = vmap.keys_of([0.0], [0.0], [0.0])
    vmap.accumulate(k, np.array([[2.0, 7.0, 1.0]]))
    _, m, M = vmap.snapshot_arrays()
    theta = vmap.posterior_mean(m, M)
    assert np.isclose(theta.sum(), 1.0)
    assert theta.argmax() == 1
    assert np.isclose(theta[0, 1], (0.01 + 7.0) / (0.03 + 10.0))


def test_census_counts_voxels_by_map_label_and_respects_the_floor():
    vmap = VoxelMap(3, resolution=1.0, alpha_0=0.01, eps=0.0)
    # Three confident voxels for class 0, one for class 1, one whisper-thin.
    for x in (0.5, 1.5, 2.5):
        vmap.accumulate(vmap.keys_of([x], [0.0], [0.0]),
                        np.array([[5.0, 0.0, 0.0]]))
    vmap.accumulate(vmap.keys_of([3.5], [0.0], [0.0]),
                    np.array([[0.0, 5.0, 0.0]]))
    vmap.accumulate(vmap.keys_of([4.5], [0.0], [0.0]),
                    np.array([[0.0, 0.0, 0.2]]))

    counts, mass, n_vox = vmap.census(min_evidence=1.0)
    assert list(counts) == [3, 1, 0]           # the thin voxel is not counted
    assert n_vox == 4
    assert np.isclose(mass[0], 15.0)

    counts_all, _, n_all = vmap.census(min_evidence=0.0)
    assert list(counts_all) == [3, 1, 1]
    assert n_all == 5


def test_census_is_empty_on_a_fresh_map():
    vmap = VoxelMap(4, resolution=0.2, alpha_0=0.01, eps=0.0)
    counts, mass, n_vox = vmap.census()
    assert n_vox == 0 and counts.sum() == 0 and mass.sum() == 0.0
