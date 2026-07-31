#!/usr/bin/env python3
"""
class_model — one online-learned semantic class + the responsibility softmax.
===============================================================================

A ``ClassModel`` bundles the conjugate posteriors of one semantic class (paper
§II-E, eq 44/45) and exposes the two half-steps of the online update:

    log_likelihood(y)  →  log L_{i,c} = log p(y | Λ_c)     (predictive, eq 51)
    update(y, w)       →  Λ_c ← Λ_c ⊕ (y, w)               (accumulation, eq 52)

For the first cut only the INTENSITY channel is active: a Normal–Inverse–Gamma
on the backscatter intensity I (eq 33 with the height-lift λ·h dropped, since
height-above-seabed is not yet available). The height / LiDAR-hurdle / shape
channels are reserved as optional slots so adding them later is a field
addition, not a rewrite — ``log_likelihood`` and ``update`` already sum/iterate
over whatever blocks are present.

Each model also carries a fixed ``semantic_tag`` ∈ {SEABED, OBJECT, STRUCTURE}.
Many appearance-classes may share a tag (the two-level scheme): the voxel
Dirichlet is kept over the 3 tags, while learning and novelty run at the finer
appearance-class resolution. The tag is assigned once (at seeding or creation)
and never changes.

The module-level ``responsibilities`` implements the soft assignment of eq 16,
    r_{i,c} = L_{i,c} π_c(N_v) / Σ_c' L_{i,c'} π_c'(N_v),
as a numerically-stable softmax over (log L + log π). Offline (no map) π is a
uniform categorical, so it drops out and r = softmax(log L) — exactly the
``_use_prior=False`` fast path of voxel_mapper.py (lines 370–374). Online the
caller passes the eq-23 spatial prior as ``log_prior``.

ROS-free and importable on its own for offline testing.
"""

from typing import Optional

import numpy as np

from sonar_map.conjugate import NIG, GammaRate, BetaBernoulli

# Semantic tags — same encoding as voxel_mapper / ClassifiedReturns.
SEABED = 0
OBJECT = 1
STRUCTURE = 2
N_TAGS = 3
TAG_NAMES = ('seabed', 'object', 'structure')


class ClassModel:
    """One online-learned class: its conjugate blocks + a fixed semantic tag."""

    __slots__ = ('semantic_tag', 'class_id', 'intensity',
                 'lidar_presence', 'lidar_count', 'lidar_gap', 'height')

    def __init__(self, semantic_tag: int, intensity: NIG,
                 class_id: Optional[int] = None,
                 lidar_presence: Optional[BetaBernoulli] = None,
                 lidar_count: Optional[GammaRate] = None,
                 lidar_gap: Optional[GammaRate] = None,
                 height: Optional[NIG] = None):
        self.semantic_tag = int(semantic_tag)
        self.class_id = class_id
        self.intensity = intensity            # NIG on I  (active)
        # Reserved for later channels — None means "abstain / not modelled yet".
        self.lidar_presence = lidar_presence  # BetaBernoulli on s_ℓ (eq 37)
        self.lidar_count = lidar_count        # GammaRate 'count' on k (eq 37)
        self.lidar_gap = lidar_gap            # GammaRate 'gap'   on g (eq 37)
        self.height = height                  # NIG on h          (eq 31)

    # -- seeding convenience --------------------------------------------------
    @classmethod
    def seed_intensity(cls, semantic_tag: int, mu: float, sigma: float,
                       kappa0: float = 1.0, a0: float = 2.0,
                       class_id: Optional[int] = None) -> 'ClassModel':
        """Build a class from an intensity prior N(mu, sigma²) with pseudo-counts.

        κ0 and a0 are the *confirmation budget*: the effective number of returns
        the class must gather before its estimate moves off the seed (paper
        §II-G). b0 = a0·sigma² so that the point estimate σ̂² = b/a starts at
        sigma²."""
        b0 = a0 * sigma * sigma
        return cls(semantic_tag, NIG(mu, kappa0, a0, b0), class_id=class_id)

    # -- assignment half-step (eq 51) -----------------------------------------
    def log_likelihood(self, intensity):
        """log L_{i,c} over the active channels (intensity-only for now).

        Returns scalar or ndarray matching ``intensity``. Extra channels, when
        present, add their own log-predictive here."""
        logL = self.intensity.log_predictive(intensity)
        # Future channels (each abstains while None):
        #   if self.height is not None:        logL = logL + self.height.log_predictive(h)
        #   if self.lidar_presence is not None: logL = logL + self.lidar_presence.log_predictive(s_l)
        #   ... (gated count/gap) ...
        return logL

    # -- accumulation half-step (eq 52) ---------------------------------------
    def update(self, intensity, weight=1.0):
        """Fold weighted return(s) into the class state: Λ_c ← Λ_c ⊕ (y, w).

        ``intensity``/``weight`` may be scalars or arrays (weight broadcasts)."""
        x = np.atleast_1d(np.asarray(intensity, dtype=np.float64))
        w = np.broadcast_to(np.asarray(weight, dtype=np.float64), x.shape)
        self.intensity.batch_update(x, w)
        return self

    # -- point estimates ------------------------------------------------------
    @property
    def intensity_mean(self) -> float:
        return self.intensity.mean

    @property
    def intensity_std(self) -> float:
        return float(np.sqrt(self.intensity.var))

    @property
    def evidence(self) -> float:
        """κ of the intensity block — the effective #returns seen by this class."""
        return self.intensity.kappa

    def copy(self) -> 'ClassModel':
        return ClassModel(
            self.semantic_tag, self.intensity.copy(), class_id=self.class_id,
            lidar_presence=None if self.lidar_presence is None else self.lidar_presence.copy(),
            lidar_count=None if self.lidar_count is None else self.lidar_count.copy(),
            lidar_gap=None if self.lidar_gap is None else self.lidar_gap.copy(),
            height=None if self.height is None else self.height.copy())

    def __repr__(self) -> str:
        return (f'ClassModel(id={self.class_id}, tag={TAG_NAMES[self.semantic_tag]}, '
                f'I~N({self.intensity_mean:.1f}, {self.intensity_std:.1f}²), '
                f'kappa={self.evidence:.1f})')


# ─────────────────────────────────────────────────────────────────────────────
#  Soft assignment — eq 16
# ─────────────────────────────────────────────────────────────────────────────
def stack_log_likelihoods(models, intensity) -> np.ndarray:
    """Assemble the (N, C) matrix of log L_{i,c} for a list of C ClassModels
    over N returns' intensities."""
    intensity = np.atleast_1d(np.asarray(intensity, dtype=np.float64))
    if not models:
        return np.empty((intensity.shape[0], 0), dtype=np.float64)
    return np.stack([m.log_likelihood(intensity) for m in models], axis=-1)


def responsibilities(log_like, log_prior=None) -> np.ndarray:
    """r_{i,c} = softmax_c( log L_{i,c} + log π_c )  — the eq-16 responsibility.

    ``log_like`` is (..., C). ``log_prior`` (optional) broadcasts against it;
    None ⇒ uniform prior (offline / voxel_mapper's vectorised fast path). The
    softmax is shift-stabilised, so rows always sum to 1."""
    z = np.asarray(log_like, dtype=np.float64)
    if z.shape[-1] == 0:
        return np.zeros_like(z)
    if log_prior is not None:
        z = z + np.asarray(log_prior, dtype=np.float64)
    z = z - np.max(z, axis=-1, keepdims=True)
    e = np.exp(z)
    return e / np.sum(e, axis=-1, keepdims=True)
