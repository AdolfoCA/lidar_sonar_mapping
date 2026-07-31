#!/usr/bin/env python3
"""
class_registry — the class inventory + dynamic (stick-breaking) class creation.
===============================================================================

Holds the live set of ``ClassModel``s and implements the two extra half-steps
of the paper's online sweep that go beyond a fixed inventory (Algorithm 1,
§II-G):

  • Assignment over active ∪ {new}  (Alg 1 L8–10). Each return is scored against
    every active class AND against a "new-class" column whose likelihood is the
    PRIOR predictive p(y | Λ_H) times a concentration γ (the stick-breaking mass
    reserved for unseen classes). Responsibilities are normalised over all
    columns (eq 16); the last column's mass is the novelty r_{i,new}.

  • Creation from unclaimed mass  (Alg 1 L17–19, eq 53). Novelty is accrued per
    spatial cell into a candidate class that starts from Λ_H and folds each
    sufficiently-novel return via ⊕. When a cell's candidate has gathered enough
    SUSTAINED evidence — mass ≥ ``novelty_mass_thresh`` across ≥ ``novelty_min_
    count`` distinct sufficiently-novel returns — it is promoted to an active
    class. Its intensity block IS exactly Λ_H ⊕ {triggering returns} (eq 53); no
    bespoke seeding rule. The new class inherits the semantic tag of the nearest
    anchor by intensity mean (the two-level scheme).

Why a lone outlier cannot spawn a class: the confirmation budget κ0/a0 in Λ_H
means one return moves the candidate barely off the prior, and count = 1 <
``novelty_min_count`` fails the trigger regardless of mass. Only a coherent,
sustained group clears both gates.

Spatial coherence is expressed through the ``cell_keys`` argument — an integer
per return (the voxel key in the live node). Novelty is accumulated per cell, so
"sustained in one place" is the spatial signal. Neighbourhood (ε-ball)
aggregation across adjacent cells is a documented extension hook
(``_maybe_create``) and is NOT required for the first cut: a single voxel is
already a small coherent region, and the count/mass gates give the sustained-ness.

ROS-free and importable on its own for offline testing.
"""

import math
from typing import List, Optional, Tuple

import numpy as np

from sonar_map.conjugate import NIG
from sonar_map.class_model import (
    ClassModel, responsibilities, stack_log_likelihoods,
    SEABED, OBJECT, STRUCTURE, N_TAGS)


class _NoveltyCell:
    """Per-cell accumulator: a candidate class (Λ_H ⊕ novel returns) + gate stats."""

    __slots__ = ('cand', 'count', 'mass')

    def __init__(self, lambda_h: NIG):
        self.cand = lambda_h.copy()   # eq-53 candidate, grown from the base prior
        self.count = 0                # # distinct sufficiently-novel returns
        self.mass = 0.0               # Σ r_{i,new} of those returns

    def add(self, x: float, w: float):
        self.cand.update(x, w)
        self.count += 1
        self.mass += w


class ClassRegistry:
    """Active class inventory with online creation from unclaimed responsibility."""

    def __init__(self, classes: List[ClassModel], base_prior: NIG, *,
                 gamma: float = 1.0,
                 novelty_return_floor: float = 0.5,
                 novelty_min_count: int = 20,
                 novelty_mass_thresh: float = 10.0,
                 create_merge_sigmas: float = 3.0,
                 anchors: Optional[List[Tuple[int, float]]] = None):
        self.classes: List[ClassModel] = list(classes)
        self.LambdaH = base_prior                       # Λ_H (intensity NIG)
        self.gamma = float(gamma)
        self._log_gamma = math.log(self.gamma) if self.gamma > 0.0 else -math.inf
        self.novelty_return_floor = float(novelty_return_floor)
        self.novelty_min_count = int(novelty_min_count)
        self.novelty_mass_thresh = float(novelty_mass_thresh)
        # A candidate within this many σ of an existing class is a spatial replica
        # of that class, not a new one — so don't duplicate it (prevents the same
        # intensity signature in many voxels spawning many identical classes).
        self.create_merge_sigmas = float(create_merge_sigmas)
        # Anchors (tag, intensity-mean) used to tag new classes. Default: the
        # classes present at construction (the seeded sb/obj/str anchors).
        self._anchors: List[Tuple[int, float]] = (
            list(anchors) if anchors is not None
            else [(c.semantic_tag, c.intensity_mean) for c in classes])
        self._cells: dict = {}                          # cell_key -> _NoveltyCell
        self._next_id = (1 + max((c.class_id for c in classes
                                  if c.class_id is not None), default=-1))

    # -- assignment over active ∪ {new}  (Alg 1 L8–10, eq 16) -----------------
    def assign(self, intensity, log_prior=None):
        """Return (r_active (N, C), r_new (N,)).

        ``log_prior`` (N, C) optionally supplies the eq-23 spatial prior for the
        active classes (the live node); None ⇒ uniform (offline). The new column
        carries log γ + log p(y | Λ_H)."""
        intensity = np.atleast_1d(np.asarray(intensity, dtype=np.float64))
        n, C = intensity.shape[0], len(self.classes)
        active = stack_log_likelihoods(self.classes, intensity)     # (n, C)
        if log_prior is not None and C > 0:
            active = active + np.asarray(log_prior, dtype=np.float64)
        new_col = self._log_gamma + self.LambdaH.log_predictive(intensity)  # (n,)
        full = np.concatenate([active, new_col[:, None]], axis=1)   # (n, C+1)
        r = responsibilities(full)
        return r[:, :C], r[:, C]

    # -- accumulation + creation  (Alg 1 L11–19, eqs 52–53) -------------------
    def accumulate(self, intensity, cell_keys, r_active, r_new) -> List[int]:
        """Fold returns into active classes (eq 52) and accrue novelty per cell;
        promote any cell that clears the sustained-novelty gate. Returns the list
        of newly created class ids (possibly empty)."""
        intensity = np.atleast_1d(np.asarray(intensity, dtype=np.float64))
        cell_keys = np.atleast_1d(np.asarray(cell_keys))
        r_active = np.atleast_2d(r_active)
        r_new = np.atleast_1d(np.asarray(r_new, dtype=np.float64))

        # eq-52 update of each active class with its responsibility column.
        for c, m in enumerate(self.classes):
            m.update(intensity, r_active[:, c])

        # Accrue novelty for sufficiently-novel returns, grouped by cell.
        touched = set()
        novel = r_new >= self.novelty_return_floor
        for i in np.nonzero(novel)[0]:
            key = cell_keys[i].item() if hasattr(cell_keys[i], 'item') else cell_keys[i]
            cell = self._cells.get(key)
            if cell is None:
                cell = self._cells[key] = _NoveltyCell(self.LambdaH)
            cell.add(float(intensity[i]), float(r_new[i]))
            touched.add(key)

        # Check the gate on cells that changed this step.
        created = []
        for key in touched:
            cid = self._maybe_create(key)
            if cid is not None:
                created.append(cid)
        return created

    def step(self, intensity, cell_keys, log_prior=None):
        """Convenience: assign → accumulate. Returns (r_active, r_new, created)."""
        r_active, r_new = self.assign(intensity, log_prior=log_prior)
        created = self.accumulate(intensity, cell_keys, r_active, r_new)
        return r_active, r_new, created

    # -- creation trigger (eq 53) ---------------------------------------------
    def _maybe_create(self, cell_key) -> Optional[int]:
        """Promote a cell's candidate to an active class if it clears both gates.

        Extension hook: to require coherence across an ε-ball rather than a single
        voxel, aggregate ``mass``/``count`` (and merge candidates) over the
        neighbourhood of ``cell_key`` here before the threshold test."""
        cell = self._cells.get(cell_key)
        if cell is None:
            return None
        if cell.count < self.novelty_min_count or cell.mass < self.novelty_mass_thresh:
            return None
        # Duplicate guard: if an existing class already explains this candidate
        # (within create_merge_sigmas · σ), it is a spatial replica, not a new
        # class — drop the accumulator instead of spawning a twin.
        cand_mu = cell.cand.mean
        for c in self.classes:
            if abs(c.intensity_mean - cand_mu) <= self.create_merge_sigmas * c.intensity_std:
                del self._cells[cell_key]
                return None
        # Placeholder tag at birth; in unsupervised mode it is overwritten every
        # scan by brightness_tags(). With seeded anchors, nearest-anchor is used.
        tag = self._nearest_anchor_tag(cell.cand.mean) if self._anchors else SEABED
        cid = self._next_id
        self._next_id += 1
        self.classes.append(ClassModel(tag, cell.cand, class_id=cid))
        del self._cells[cell_key]           # consumed; region now has an active class
        return cid

    def _nearest_anchor_tag(self, mean: float) -> int:
        """Semantic tag of the anchor whose intensity mean is closest to ``mean``."""
        tag, best = self._anchors[0][0], abs(self._anchors[0][1] - mean)
        for t, mu in self._anchors[1:]:
            d = abs(mu - mean)
            if d < best:
                tag, best = t, d
        return tag

    # -- introspection --------------------------------------------------------
    @property
    def n_classes(self) -> int:
        return len(self.classes)

    def brightness_tags(self, object_offset: float,
                        structure_offset: float) -> list:
        """Assign a semantic tag to each active class from its LEARNED intensity
        mean, relative to the dimmest (seabed-reference) class — fully
        unsupervised, no seeded μ. Returns a list[int] aligned with self.classes.

        A class is SEABED near the dimmest learned class, STRUCTURE once it is
        ``structure_offset`` brighter, OBJECT in between. Because the reference is
        the dimmest *learned* class (not a hand-set value), the split follows the
        data; when every class is within ``object_offset`` they are all seabed."""
        if not self.classes:
            return []
        mus = np.array([c.intensity_mean for c in self.classes], dtype=np.float64)
        d = mus - float(mus.min())
        tags = np.full(len(self.classes), SEABED, dtype=int)
        tags[d >= object_offset] = OBJECT
        tags[d >= structure_offset] = STRUCTURE
        return tags.tolist()

    def apply_tags(self, tags) -> None:
        """Write a per-class tag list back onto the class models (for logging /
        snapshot). ``tags`` is aligned with self.classes."""
        for c, t in zip(self.classes, tags):
            c.semantic_tag = int(t)

    def tag_mass(self, r_active, tags=None) -> np.ndarray:
        """Aggregate an (N, C) responsibility matrix down to the 3 semantic tags
        (the two-level collapse feeding the voxel Dirichlet). Returns (N, 3).

        ``tags`` (optional) supplies a per-class tag list (e.g. from
        ``brightness_tags``); None ⇒ each class's stored ``semantic_tag``. Only the
        first ``r_active.shape[1]`` classes are aggregated, so it is safe to call
        after a class was appended this scan (its triggering returns were already
        folded via ⊕ and carry no column here)."""
        r_active = np.atleast_2d(r_active)
        C = r_active.shape[1]
        out = np.zeros((r_active.shape[0], N_TAGS), dtype=np.float64)
        for c in range(C):
            t = self.classes[c].semantic_tag if tags is None else int(tags[c])
            out[:, t] += r_active[:, c]
        return out
