#!/usr/bin/env python3
"""
conjugate — the three conjugate families behind the learned class models.
===============================================================================

Pure-math foundation (NO ROS, NO scipy) for the online-learned likelihoods of
the probabilistic seabed-mapping paper. Each class in the paper is a bundle of
conjugate posteriors, one per measurement channel; this module implements the
three families those channels use, together with the accumulation operator ⊕
(the weighted rank-one updates) and the posterior predictives used to score a
return (the L_{i,c} of the responsibility, eq 16).

Families (paper §II-E, eqs 45–50, Tables I–II):

  NIG  — Normal–Inverse–Gamma over a Gaussian channel (μ_c, σ²_c). Used for the
         intensity factor (eq 33) and — later — the height band (eq 31) and the
         continuous shape descriptors (eq 41). Hyperparameters (m, κ, a, b):
             μ̂ = m,  σ̂² = b / a           (paper's point estimates, §II-E-a)
         Update for a weighted obs (x, w)  (eq 47):
             κ' = κ + w
             m' = (κ m + w x) / κ'
             a' = a + w/2
             b' = b + w κ / (2 κ') · (x − m)²      ← stable scatter form
         Posterior predictive (eq 48): Student-t, df = 2a, loc = m,
             scale² = b (κ + 1) / (a κ).

  GammaRate — Gamma over a Poisson-count rate κ_c OR an exponential-gap rate γ_c
         (paper eq 37, §II-E-b). Hyperparameters (a, b), rate estimate a/b.
         Update (eq 49), weighted obs (x, w):
             count (x = k):  a' = a + w x,  b' = b + w      → predictive NegBinom
             gap   (x = g):  a' = a + w,    b' = b + w x    → predictive Lomax

  BetaBernoulli — Beta over a presence probability π^ℓ_c or ρ_c (eq 37/42,
         §II-E-c). Hyperparameters (u, v), estimate u/(u+v). Update (eq 50),
         weighted indicator (s, w):  u' = u + w s,  v' = v + w (1 − s).
         Predictive p(s=1) = u/(u+v).

Only NIG is on the hot path for the intensity-only first cut; GammaRate and
BetaBernoulli are here so the LiDAR-hurdle and shape channels drop in later
without touching this module.

Design notes
  • Updates MUTATE in place (cheap for the online map) and also expose a
    ``batch_update`` that folds an array of weighted observations at once. For
    every family the batch form is mathematically identical to applying the
    per-observation recursion in sequence — for NIG this is the Welford/parallel
    combination that avoids the Σ w x² cancellation; for the additive Gamma/Beta
    updates it is a plain sum. ``test_conjugate.py`` pins this equivalence.
  • A weight w ≤ 0 is a strict no-op, so a zero responsibility r_{i,c}=0 leaves a
    class untouched exactly.
  • ``log_predictive`` accepts a scalar or an ndarray and returns the matching
    shape; the Gamma-function terms depend only on the (scalar) hyperparameters,
    so scoring a whole scan is vectorised over the observation.
  • No scipy: the log-Gamma is math.lgamma, vectorised. The unit tests compute
    the same quantities with scipy.stats, so agreement is an independent check.
"""

import math

import numpy as np

# Vectorised log-Gamma (numpy has none; math.lgamma is scalar-only).
_gammaln_vec = np.frompyfunc(math.lgamma, 1, 1)


def _gammaln(x):
    """log Γ(x) for a scalar or ndarray, returned as float/float64 array."""
    if np.isscalar(x):
        return math.lgamma(float(x))
    return _gammaln_vec(np.asarray(x, dtype=np.float64)).astype(np.float64)


_LOG_PI = math.log(math.pi)


# ─────────────────────────────────────────────────────────────────────────────
#  Normal–Inverse–Gamma  (Gaussian channel with unknown mean & variance)
# ─────────────────────────────────────────────────────────────────────────────
class NIG:
    """Normal–Inverse–Gamma posterior over a Gaussian channel (μ, σ²).

    Hyperparameters (m, κ, a, b): m the current mean estimate, κ the pseudo-count
    of observations backing it, (a, b) the shape/scale of the variance posterior.
    See module docstring for the update (eq 47) and predictive (eq 48)."""

    __slots__ = ('m', 'kappa', 'a', 'b')

    def __init__(self, m: float, kappa: float, a: float, b: float):
        self.m = float(m)
        self.kappa = float(kappa)
        self.a = float(a)
        self.b = float(b)

    # -- point estimates (paper §II-E-a) --------------------------------------
    @property
    def mean(self) -> float:
        """μ̂ = m."""
        return self.m

    @property
    def var(self) -> float:
        """σ̂² = b / a  (the paper's plug-in point estimate)."""
        return self.b / self.a

    # -- accumulation ⊕ (eq 47) -----------------------------------------------
    def update(self, x: float, w: float = 1.0) -> 'NIG':
        """Fold one weighted observation (x, w) via the eq-47 rank-one recursion.

        The scatter grows by the squared deviation of x from the OLD mean, scaled
        by κ/κ', which is the cancellation-free form of the batch scatter."""
        if w <= 0.0:
            return self
        kp = self.kappa + w
        dm = x - self.m
        self.m = self.m + (w / kp) * dm          # = (κ m + w x)/κ'
        self.b = self.b + (w * self.kappa) / (2.0 * kp) * dm * dm
        self.a = self.a + 0.5 * w
        self.kappa = kp
        return self

    def batch_update(self, x, w=None) -> 'NIG':
        """Fold an array of weighted observations at once (Welford combination).

        Mathematically identical to calling :meth:`update` for each (x_i, w_i) in
        any order. ``w`` defaults to unit weights."""
        x = np.asarray(x, dtype=np.float64).ravel()
        if x.size == 0:
            return self
        w = (np.ones_like(x) if w is None
             else np.asarray(w, dtype=np.float64).ravel())
        keep = w > 0.0
        if not np.any(keep):
            return self
        x, w = x[keep], w[keep]
        W = float(w.sum())
        xbar = float(np.dot(w, x) / W)                 # weighted mean
        S = float(np.dot(w, (x - xbar) ** 2))          # weighted scatter about xbar
        kp = self.kappa + W
        dm = xbar - self.m
        self.b = self.b + 0.5 * S + 0.5 * (self.kappa * W / kp) * dm * dm
        self.m = self.m + (W / kp) * dm
        self.a = self.a + 0.5 * W
        self.kappa = kp
        return self

    # -- posterior predictive (eq 48): Student-t ------------------------------
    def log_predictive(self, x):
        """log t_{2a}(x; m, b(κ+1)/(aκ)) — the L factor for this channel.

        Scalar or ndarray x; heavy-tailed for small a, so a freshly created class
        scores novel returns cautiously (paper §II-E-a)."""
        nu = 2.0 * self.a
        scale2 = self.b * (self.kappa + 1.0) / (self.a * self.kappa)
        z2 = np.asarray(x, dtype=np.float64)
        z2 = (z2 - self.m) ** 2 / (nu * scale2)
        const = (math.lgamma(0.5 * (nu + 1.0)) - math.lgamma(0.5 * nu)
                 - 0.5 * (math.log(nu) + _LOG_PI + math.log(scale2)))
        return const - 0.5 * (nu + 1.0) * np.log1p(z2)

    def copy(self) -> 'NIG':
        return NIG(self.m, self.kappa, self.a, self.b)

    def __repr__(self) -> str:
        return (f'NIG(m={self.m:.4g}, kappa={self.kappa:.4g}, '
                f'a={self.a:.4g}, b={self.b:.4g})')


# ─────────────────────────────────────────────────────────────────────────────
#  Gamma rate  (Poisson count OR exponential gap)
# ─────────────────────────────────────────────────────────────────────────────
class GammaRate:
    """Gamma posterior over a rate (paper eq 49). ``kind`` selects the channel:

      'count' — Poisson likelihood on k; predictive NegativeBinomial.
      'gap'   — Exponential likelihood on g; predictive Lomax (Pareto-II)."""

    __slots__ = ('a', 'b', 'kind')

    def __init__(self, a: float, b: float, kind: str = 'count'):
        if kind not in ('count', 'gap'):
            raise ValueError("kind must be 'count' or 'gap'")
        self.a = float(a)
        self.b = float(b)
        self.kind = kind

    @property
    def rate(self) -> float:
        """point estimate a/b (mean of the Gamma)."""
        return self.a / self.b

    def update(self, x: float, w: float = 1.0) -> 'GammaRate':
        if w <= 0.0:
            return self
        if self.kind == 'count':
            self.a += w * x
            self.b += w
        else:  # gap
            self.a += w
            self.b += w * x
        return self

    def batch_update(self, x, w=None) -> 'GammaRate':
        x = np.asarray(x, dtype=np.float64).ravel()
        if x.size == 0:
            return self
        w = (np.ones_like(x) if w is None
             else np.asarray(w, dtype=np.float64).ravel())
        keep = w > 0.0
        x, w = x[keep], w[keep]
        if self.kind == 'count':
            self.a += float(np.dot(w, x))
            self.b += float(w.sum())
        else:
            self.a += float(w.sum())
            self.b += float(np.dot(w, x))
        return self

    def log_predictive(self, x):
        """Predictive log-pmf/pdf: NegBinom (count) or Lomax (gap)."""
        if self.kind == 'count':
            # Gamma(a,b) rate, Poisson count → NB: n=a, p=b/(b+1).
            k = np.asarray(x, dtype=np.float64)
            p = self.b / (self.b + 1.0)
            return (_gammaln(self.a + k) - _gammaln(k + 1.0) - math.lgamma(self.a)
                    + self.a * math.log(p) + k * math.log1p(-p))
        # Gamma(a,b) rate, Exp gap → Lomax(shape=a, scale=b):
        #   p(g) = a b^a / (b + g)^{a+1}
        g = np.asarray(x, dtype=np.float64)
        return (math.log(self.a) + self.a * math.log(self.b)
                - (self.a + 1.0) * np.log(self.b + g))

    def copy(self) -> 'GammaRate':
        return GammaRate(self.a, self.b, self.kind)

    def __repr__(self) -> str:
        return f'GammaRate(a={self.a:.4g}, b={self.b:.4g}, kind={self.kind!r})'


# ─────────────────────────────────────────────────────────────────────────────
#  Beta–Bernoulli  (presence indicator)
# ─────────────────────────────────────────────────────────────────────────────
class BetaBernoulli:
    """Beta posterior over a presence probability (paper eq 50).

    Hyperparameters (u, v); estimate û = u/(u+v)."""

    __slots__ = ('u', 'v')

    def __init__(self, u: float, v: float):
        self.u = float(u)
        self.v = float(v)

    @property
    def prob(self) -> float:
        """û = u/(u+v)."""
        return self.u / (self.u + self.v)

    def update(self, s: float, w: float = 1.0) -> 'BetaBernoulli':
        if w <= 0.0:
            return self
        self.u += w * s
        self.v += w * (1.0 - s)
        return self

    def batch_update(self, s, w=None) -> 'BetaBernoulli':
        s = np.asarray(s, dtype=np.float64).ravel()
        if s.size == 0:
            return self
        w = (np.ones_like(s) if w is None
             else np.asarray(w, dtype=np.float64).ravel())
        keep = w > 0.0
        s, w = s[keep], w[keep]
        self.u += float(np.dot(w, s))
        self.v += float(np.dot(w, 1.0 - s))
        return self

    def log_predictive(self, s):
        """log Beta–Bernoulli predictive: log û for s=1, log(1−û) for s=0."""
        s = np.asarray(s, dtype=np.float64)
        p = self.u / (self.u + self.v)
        return np.where(s >= 0.5, math.log(p), math.log1p(-p))

    def copy(self) -> 'BetaBernoulli':
        return BetaBernoulli(self.u, self.v)

    def __repr__(self) -> str:
        return f'BetaBernoulli(u={self.u:.4g}, v={self.v:.4g})'
