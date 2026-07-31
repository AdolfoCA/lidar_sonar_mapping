"""Unit tests for region_shapes.compute_region_channels (paper eqs 39-42).

These exercise the REDESIGNED morphology pipeline: top-hat background
subtraction -> Otsu -> solid-object open/close -> connected-component blobs ->
AKAZE salient keypoints on the masked clean image -> leading-edge association ->
blob descriptors m_R = (a_R, ς_R, ξ_R, τ_R).

Synthetic-image design (see the module helpers below):

  * BACKGROUND is low, high-frequency seabed speckle. It is deliberately NOT a
    flat zero field: the top-hat needs a broad low-level population so Otsu
    separates OBJECT from seabed (on a pure-zero background Otsu splits the blob
    internally), and the speckle is dim enough that the morphological OPEN erases
    it, leaving only solid blobs.

  * BLOBS are SOLID (high base, so every pixel stays far above the Otsu
    threshold => one connected component smaller than the top-hat kernel) but
    carry BLOCKY internal contrast. That block structure is what makes AKAZE
    actually fire — a flat fill yields at most the odd corner keypoint — so the
    contract's requirement ("blobs MUST be textured so AKAZE produces
    keypoints") is met and explicitly verified in
    test_akaze_fires_on_textured_blob. The internal contrast also makes the
    texture descriptor τ_R > 0.

AKAZE tuning for the synthetic images: the detector params are fixed by
region_shapes (only its response threshold is exposed), and on these small
blobs AKAZE keypoints come out tiny (~5 px diameter). We therefore pass
assoc_scale=ASSOC (3.0, up from the 1.5 default) so a keypoint's association
disc reaches the blob's top-row leading edge deterministically. Everything is
seeded (numpy default_rng) so the tests are reproducible; AKAZE itself is
deterministic per image.

Pure NumPy/OpenCV — no ROS needed. Run with:
    PYTHONPATH=/home/rosdev/ros2_ws/src/sonar3d_reconstruction \
        python3 -m pytest test/test_region_shapes.py -q
"""

import numpy as np
import pytest

from sonar3d_reconstruction.region_shapes import CHANNEL_NAMES, compute_region_channels

# ─────────────────────────── fixture geometry ────────────────────────────
N_RANGE, N_BEAM = 150, 120
RANGES = np.linspace(2.0, 14.0, N_RANGE)          # drho ~ 0.081 m
BEARINGS = np.linspace(-0.7, 0.7, N_BEAM)         # dbeta ~ 0.0118 rad
DRHO = (RANGES[-1] - RANGES[0]) / (N_RANGE - 1)
DBETA = abs(float(np.mean(np.diff(BEARINGS))))

# AKAZE association reach (see module docstring): widened from the 1.5 default so
# the tiny keypoints on small synthetic blobs reach the top-row leading edge.
ASSOC = 3.0
BLOB_H = BLOB_W = 22                               # < bg_kernel (25) -> one blob


# ─────────────────────────── synthetic builders ──────────────────────────
def background(seed):
    """Low, high-frequency seabed speckle (see module docstring for why it is
    textured rather than zero)."""
    rng = np.random.default_rng(seed)
    return np.abs(rng.normal(0.0, 18.0, size=(N_RANGE, N_BEAM))) + 15.0


def textured_blob(h, w, seed, base=1400.0, amp=400.0, block=4):
    """A SOLID bright object with BLOCKY internal contrast (fires AKAZE while
    staying one connected component; see module docstring)."""
    rng = np.random.default_rng(seed)
    nbh, nbw = (h + block - 1) // block, (w + block - 1) // block
    blocks = rng.integers(0, 2, size=(nbh, nbw)) * 2 - 1        # ±1 block checker
    big = np.kron(blocks, np.ones((block, block)))[:h, :w]
    return base + big * amp + rng.normal(0.0, 25.0, size=(h, w))


def canvas(seed):
    """A seabed-speckle image with no edges: the base each test paints onto."""
    det = background(seed)
    edge = np.zeros(N_BEAM, dtype=int)
    valid = np.zeros(N_BEAM, dtype=bool)
    return det, edge, valid


def paint_blob(det, edge, valid, r0, c0, seed, h=BLOB_H, w=BLOB_W, shadow=True):
    """Stamp a textured blob; its top row r0 is the leading edge of every column
    it spans (the crossing seeds), with an optional dark trailing shadow."""
    det[r0 : r0 + h, c0 : c0 + w] = textured_blob(h, w, seed)
    edge[c0 : c0 + w] = r0
    valid[c0 : c0 + w] = True
    if shadow:
        det[r0 + h : r0 + h + 40, c0 : c0 + w] = 3.0
    return r0, c0, h, w


def run(det, edge, valid, **kw):
    kw.setdefault("assoc_scale", ASSOC)
    return compute_region_channels(det, RANGES, BEARINGS, edge, valid, **kw)


# ─────────────────────────────── tests ───────────────────────────────────
def test_channels_shape_and_keys():
    """Output is exactly the six CHANNEL_NAMES, each a (n_beam,) float32."""
    det, edge, valid = canvas(9)
    paint_blob(det, edge, valid, 45, 48, 7)
    ch = run(det, edge, valid)
    assert set(ch.keys()) == set(CHANNEL_NAMES)
    for name in CHANNEL_NAMES:
        assert ch[name].shape == (N_BEAM,)
        assert ch[name].dtype == np.float32


def test_akaze_fires_on_textured_blob():
    """Setup verification (contract): the textured blob must actually produce
    AKAZE keypoints, at least one of which is crossed by the leading edge, and
    the resulting region's texture descriptor must be > 0 (internal variance)."""
    det, edge, valid = canvas(9)
    paint_blob(det, edge, valid, 45, 48, 7)
    ch, dbg = run(det, edge, valid, return_regions=True)

    assert len(dbg["keypoints"]) >= 1, "textured blob produced no AKAZE keypoints"
    assert any(k["crossed"] for k in dbg["keypoints"]), "no keypoint was crossed"
    assert len(dbg["regions"]) == 1
    mem = np.flatnonzero(ch["shape_flag"] == 1.0)
    assert mem.size >= 1
    assert np.all(ch["region_texture"][mem] > 0.0)     # blocky contrast -> tau>0


def test_single_textured_blob_one_region():
    """One textured blob crossed by its top-row leading edge -> exactly one
    region with analytic log-area, bounded solidity, positive texture, and a
    strong shadow; plus the three per-beam semantics in one image."""
    det, edge, valid = canvas(9)
    r0, c0, h, w = paint_blob(det, edge, valid, 45, 48, 7)
    # A group of VALID beams far from the blob (edge near range 0) that cannot
    # cross any keypoint -> they must resolve to flag 0 (informative absence).
    far = slice(5, 16)
    edge[far] = 5
    valid[far] = True

    ch, dbg = run(det, edge, valid, return_regions=True)

    # Exactly one region, all members inside the blob's column span.
    assert len(dbg["regions"]) == 1
    mem = np.flatnonzero(ch["shape_flag"] == 1.0)
    assert mem.size > 0
    assert mem.min() >= c0 and mem.max() < c0 + w
    # N_R = member count, written onto every member beam.
    assert np.all(ch["region_size"][mem] == float(mem.size))
    assert mem.size > 0                                  # N_R > 0

    # a_R: log physical area. Check the eq-40 formula is applied to the blob's
    # pixels exactly, and that it lands within tolerance of the analytic area of
    # the painted rectangle (the mask is ~98% of it).
    ys = dbg["regions"][0]["ys"]
    exact = np.log(DRHO * DBETA * float(RANGES[ys].sum()))
    assert np.allclose(ch["region_logarea"][mem], exact, atol=1e-5)
    rect = np.log(DRHO * DBETA * w * float(RANGES[r0 : r0 + h].sum()))
    assert abs(float(ch["region_logarea"][mem[0]]) - rect) < 0.1

    # Solid rectangle: solidity ∈ (0, 1]; blocky fill: texture > 0.
    assert np.all(ch["region_solidity"][mem] > 0.0)
    assert np.all(ch["region_solidity"][mem] <= 1.0)
    assert np.all(ch["region_texture"][mem] > 0.0)

    # Dark trailing window -> a strong shadow signature in [0, 1].
    assert np.all(ch["region_shadow"][mem] >= 0.0)
    assert np.all(ch["region_shadow"][mem] <= 1.0)
    assert np.all(ch["region_shadow"][mem] > 0.3)

    # Valid-but-uncrossed beams: informative absence (flag 0, no descriptor).
    assert np.all(ch["shape_flag"][far] == 0.0)
    assert np.all(ch["region_size"][far] == 0.0)
    assert np.all(np.isnan(ch["region_logarea"][far]))

    # Invalid beams (never produced an edge) stay NaN and publish no point.
    invalid = ~valid
    assert np.all(np.isnan(ch["shape_flag"][invalid]))
    assert np.all(ch["region_size"][invalid] == 0.0)


def test_blob_not_crossed_is_no_region():
    """A textured blob is present (it even yields keypoints) but every leading
    edge sits far away in range, so nothing crosses -> no region at all."""
    det, edge, valid = canvas(9)
    # Paint the blob's PIXELS but keep the leading edge near range 0 for all beams.
    det[45 : 45 + BLOB_H, 48 : 48 + BLOB_W] = textured_blob(BLOB_H, BLOB_W, 7)
    valid[:] = True
    edge[:] = 5                                          # far from the blob (row 45+)

    ch, dbg = run(det, edge, valid, return_regions=True)

    assert len(dbg["regions"]) == 0
    assert not np.any(ch["shape_flag"] == 1.0)
    assert np.all(ch["shape_flag"][valid] == 0.0)       # every valid beam -> flag 0
    assert np.all(ch["region_size"] == 0.0)
    for name in ("region_logarea", "region_solidity", "region_shadow", "region_texture"):
        assert np.all(np.isnan(ch[name]))


@pytest.mark.parametrize("seed", [0, 3, 7, 11])
def test_background_only_is_no_region(seed):
    """Seabed speckle with no solid object -> the open/min-px gates leave no blob,
    so every valid beam resolves to flag 0 and no region forms."""
    det, edge, valid = canvas(seed)
    valid[:] = True
    edge[:] = 20

    ch, dbg = run(det, edge, valid, return_regions=True)

    assert len(dbg["regions"]) == 0
    assert not np.any(ch["shape_flag"] == 1.0)
    assert np.all(ch["shape_flag"][valid] == 0.0)
    assert np.all(ch["region_size"] == 0.0)


def test_two_separated_blobs_two_regions():
    """Two well-separated textured blobs, each crossed -> two distinct regions,
    each with its own member column cluster and its own N_R."""
    det, edge, valid = canvas(9)
    paint_blob(det, edge, valid, 45, 25, 3)             # left blob
    paint_blob(det, edge, valid, 45, 72, 4)             # right blob

    ch, dbg = run(det, edge, valid, return_regions=True)

    assert len(dbg["regions"]) == 2
    mem = np.flatnonzero(ch["shape_flag"] == 1.0)
    left = mem[mem < 50]
    right = mem[mem >= 50]
    assert left.size > 0 and right.size > 0             # one cluster per blob
    # Distinct N_R per side proves the blobs were NOT merged into one region.
    assert np.all(ch["region_size"][left] == float(left.size))
    assert np.all(ch["region_size"][right] == float(right.size))
    # The gap beams between the blobs carry nothing.
    gap = mem[(mem >= left.max() + 1) & (mem <= right.min() - 1)]
    assert gap.size == 0


def test_return_regions_debug_bundle():
    """return_regions=True yields the masked image, keypoint list, and region
    dicts with the exact keys/shapes the debug overlay consumes; masked is 0
    everywhere off the blob."""
    det, edge, valid = canvas(9)
    paint_blob(det, edge, valid, 45, 48, 7)
    ch, dbg = run(det, edge, valid, return_regions=True)

    assert set(dbg.keys()) == {"masked", "keypoints", "regions"}

    masked = dbg["masked"]
    assert masked.shape == (N_RANGE, N_BEAM)
    assert masked.dtype == np.float32

    # keypoints: list of dicts with the overlay's keys and types.
    assert isinstance(dbg["keypoints"], list) and dbg["keypoints"]
    for k in dbg["keypoints"]:
        assert set(k.keys()) == {"x", "y", "size", "crossed"}
        assert isinstance(k["crossed"], bool)
        assert np.isfinite(k["x"]) and np.isfinite(k["y"]) and k["size"] > 0

    # regions: one dict carrying pixels, members, N_R, anchor keypoint, m_R.
    assert len(dbg["regions"]) == 1
    reg = dbg["regions"][0]
    assert set(reg.keys()) == {
        "ys", "xs", "member_cols", "edge_rows", "n_r",
        "kp_x", "kp_y", "kp_size", "logarea", "solidity", "shadow", "texture",
    }
    assert reg["ys"].shape == reg["xs"].shape
    assert reg["member_cols"].shape == reg["edge_rows"].shape
    assert reg["n_r"] == reg["member_cols"].size
    assert reg["n_r"] == int(np.sum(ch["shape_flag"] == 1.0))
    for key in ("logarea", "solidity", "shadow", "texture", "kp_x", "kp_y", "kp_size"):
        assert np.isfinite(reg[key])

    # masked is EXACTLY the blob pixels (0 off the blob) for a single clean blob.
    blob_pix = np.zeros((N_RANGE, N_BEAM), dtype=bool)
    blob_pix[reg["ys"], reg["xs"]] = True
    assert np.array_equal(masked != 0.0, blob_pix)
    assert np.all(masked[:35, :] == 0.0)                # top rows are pure seabed


def test_invalid_beams_nan_valid_nonmember_zero():
    """Focused check of the per-beam absence semantics: invalid beams stay NaN,
    valid beams that join no region are 0 (not NaN)."""
    det, edge, valid = canvas(9)
    paint_blob(det, edge, valid, 45, 48, 7)
    # Mark a block of far beams valid so some valid beams are guaranteed
    # non-members regardless of how many blob columns cross.
    valid[5:16] = True
    edge[5:16] = 5

    ch = run(det, edge, valid)

    members = ch["shape_flag"] == 1.0
    valid_nonmember = valid & ~members
    assert valid_nonmember.any()
    assert np.all(ch["shape_flag"][valid_nonmember] == 0.0)
    assert np.all(np.isnan(ch["shape_flag"][~valid]))
    # Descriptors present only on members.
    assert np.all(np.isnan(ch["region_logarea"][~members]))


def test_no_valid_beams_all_nan():
    """No leading edge anywhere -> the extractor abstains on every beam."""
    det, edge, valid = canvas(9)
    ch = run(det, edge, valid)
    assert np.all(np.isnan(ch["shape_flag"]))
    assert np.all(ch["region_size"] == 0.0)


def test_degenerate_geometry_no_crash():
    """A single-range-bin image is degenerate (drho undefined) -> the guard
    returns all-absence channels and a well-formed (empty) debug bundle, no
    crash."""
    det = np.full((1, N_BEAM), 1000.0)
    edge = np.zeros(N_BEAM, dtype=int)
    valid = np.ones(N_BEAM, dtype=bool)
    ch, dbg = compute_region_channels(
        det, np.array([2.0]), BEARINGS, edge, valid, return_regions=True
    )
    assert not np.any(ch["shape_flag"] == 1.0)          # no region formed
    assert np.all(ch["region_size"] == 0.0)
    assert dbg["regions"] == [] and dbg["keypoints"] == []
    assert dbg["masked"].shape == (1, N_BEAM)
