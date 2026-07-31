"""
voxel_map — the persistent Dirichlet map, its frozen snapshot, the spatial
prediction (eq 16) and the MAP report (eq 15).

Each voxel v holds a categorical theta_v over the classes with a conjugate
Dirichlet prior. The per-voxel update derived in the methodology reduces to an
additive accumulation of responsibilities onto the concentrations:

    m_{v,c} += r_{i,c}                                                    (13)
    theta_hat_{v,c} = (alpha_c + m_{v,c}) / (alpha_0_total + M_v)         (14)
    c_MAP_v         = argmax_c (alpha_c + m_{v,c})                        (15)

with M_v = sum_c m_{v,c}. The map is PERSISTENT: voxels accumulate evidence
indefinitely and are never retired, and as evidence accumulates the posterior
concentrates and the MAP label converges.

SPATIAL PREDICTION (eq 16). The seabed, objects and structures are spatially
coherent, so the responsibility's prior over the measurement's class is a
confidence-weighted average of the neighbourhood's beliefs, over the voxels
within radius eps:

    pi_c(N_v) = sum_{u in N_v} M_u theta_hat^{(t-1)}_{u,c} / sum_{u in N_v} M_u

The evidence weight gates each vote automatically: an unobserved neighbour
(M_u = 0) carries no weight, well-observed neighbours dominate. When the
neighbourhood carries no evidence at all the prediction is undefined and falls
back to uniform 1/|C| — so the spatial coupling is inactive exactly when
beliefs are evidence-poor, and influence flows only from evidence-rich voxels
to evidence-poor ones as the map fills in. Its role is complementary to the
likelihoods: those are decisive where the physics is decisive and abstain where
it is not, and the prediction resolves precisely the returns on which they
abstain.

E/M DISCIPLINE (I6). Neighbour beliefs are read from an explicit IMMUTABLE
snapshot frozen at the previous sweep's commit — never from live values
mid-sweep. Every voxel in a sweep is therefore scored against the same frozen
(t-1) state, which is what makes the result independent of the order in which
returns are processed. ``snapshot()`` is called once per sweep, at commit.

The voxel's own previous belief is deliberately INCLUDED in its neighbourhood:
it is a legitimate prior, and the (t-1) freeze means it introduces no
within-sweep feedback.
"""

import numpy as np

# ── 21-bit-per-axis voxel hashing (signed, packs into one int64) ─────────────
_BITS = 21
_MASK = (1 << _BITS) - 1
_SIGN = 1 << (_BITS - 1)
_SPAN = 1 << _BITS


def pack(ix, iy, iz):
    """Pack signed voxel indices into one int64 key (vectorised or scalar)."""
    ix = np.asarray(ix, dtype=np.int64)
    iy = np.asarray(iy, dtype=np.int64)
    iz = np.asarray(iz, dtype=np.int64)
    return ((ix & _MASK) << (2 * _BITS)) | ((iy & _MASK) << _BITS) | (iz & _MASK)


def unpack(keys):
    """Vectorised inverse of :func:`pack` — signed (ix, iy, iz) int64 arrays."""
    keys = np.asarray(keys, dtype=np.int64)
    iz = keys & _MASK
    iy = (keys >> _BITS) & _MASK
    ix = (keys >> (2 * _BITS)) & _MASK
    iz = np.where(iz >= _SIGN, iz - _SPAN, iz)
    iy = np.where(iy >= _SIGN, iy - _SPAN, iy)
    ix = np.where(ix >= _SIGN, ix - _SPAN, ix)
    return ix, iy, iz


class VoxelMap:
    """Persistent sparse Dirichlet map over a fixed class axis."""

    def __init__(self, n_classes: int, resolution: float = 0.20,
                 alpha_0: float = 0.01, eps: float = 0.5):
        """
        Parameters
        ----------
        n_classes  |C| — length of the class axis (classes.yaml order).
        resolution voxel edge [m].
        alpha_0    symmetric Dirichlet concentration alpha_c. Small (<< 1)
                   keeps the prior weak so a handful of returns already
                   dominates it.
        eps        spatial-prediction neighbourhood radius [m]. <= 0 disables
                   the coupling: pi_c becomes uniform, the responsibility
                   reduces to the normalised likelihood, and the whole sweep
                   runs vectorised with no ball query and no snapshot.
        """
        if resolution <= 0.0:
            raise ValueError('VoxelMap: resolution must be > 0')
        if alpha_0 <= 0.0:
            raise ValueError('VoxelMap: alpha_0 must be > 0')

        self.n_classes = int(n_classes)
        self.resolution = float(resolution)
        self._inv_res = 1.0 / self.resolution
        self.alpha_0 = float(alpha_0)
        self._alpha_total = self.alpha_0 * self.n_classes
        self.eps = float(max(eps, 0.0))
        self.use_prior = self.eps > 0.0

        self.uniform = np.full(self.n_classes, 1.0 / self.n_classes)
        self.log_uniform = np.log(self.uniform)

        # A true Euclidean ball of offsets, matching "voxels within radius eps".
        r = int(np.ceil(self.eps * self._inv_res))
        eps2 = self.eps * self.eps
        res2 = self.resolution * self.resolution
        self._offsets = np.array(
            [(dx, dy, dz)
             for dx in range(-r, r + 1)
             for dy in range(-r, r + 1)
             for dz in range(-r, r + 1)
             if (dx * dx + dy * dy + dz * dz) * res2 <= eps2],
            dtype=np.int64).reshape(-1, 3) if self.use_prior else \
            np.zeros((0, 3), dtype=np.int64)

        self._m: dict = {}          # key -> (C,) soft counts m_v
        self._M: dict = {}          # key -> M_v = sum_c m_{v,c}
        self._snapshot: dict = {}   # key -> ((C,) theta_hat, M)  frozen at t-1

    # ── indexing ─────────────────────────────────────────────────────────────

    def indices(self, x, y, z):
        """Voxel indices (ix, iy, iz) of Cartesian positions."""
        return (np.floor(np.asarray(x, dtype=np.float64) * self._inv_res).astype(np.int64),
                np.floor(np.asarray(y, dtype=np.float64) * self._inv_res).astype(np.int64),
                np.floor(np.asarray(z, dtype=np.float64) * self._inv_res).astype(np.int64))

    def keys_of(self, x, y, z):
        """Packed voxel keys of Cartesian positions."""
        return pack(*self.indices(x, y, z))

    def centres(self, keys):
        """(N, 3) Cartesian centres of the given voxel keys."""
        ix, iy, iz = unpack(keys)
        return np.stack([(ix + 0.5) * self.resolution,
                         (iy + 0.5) * self.resolution,
                         (iz + 0.5) * self.resolution], axis=1)

    # ── (t-1) snapshot: the frozen state a whole sweep is scored against ─────

    def commit(self) -> None:
        """Freeze theta_hat and M for every voxel (I6). Call once per sweep."""
        if not self.use_prior:
            return
        a0, at = self.alpha_0, self._alpha_total
        self._snapshot = {
            k: ((a0 + m) / (at + self._M[k]), self._M[k])
            for k, m in self._m.items()
        }

    def spatial_prediction(self, keys) -> np.ndarray:
        """(N, C) pi_c(N_v) (eq 16) for the given voxel keys, from the snapshot.

        Resolved once per DISTINCT voxel and broadcast back: a sweep's returns
        share far fewer voxels than it has returns.
        """
        keys = np.asarray(keys, dtype=np.int64)
        if not self.use_prior or not self._snapshot:
            return np.broadcast_to(self.uniform, (keys.size, self.n_classes))

        uniq, inv = np.unique(keys, return_inverse=True)
        ix, iy, iz = unpack(uniq)
        out = np.empty((uniq.size, self.n_classes), dtype=np.float64)
        snap = self._snapshot

        for j in range(uniq.size):
            num = np.zeros(self.n_classes, dtype=np.float64)
            den = 0.0
            for dx, dy, dz in self._offsets:
                e = snap.get(int(pack(ix[j] + dx, iy[j] + dy, iz[j] + dz)))
                if e is None:
                    continue
                theta, mass = e
                num += mass * theta
                den += mass
            out[j] = self.uniform if den <= 0.0 else num / den

        return out[inv.ravel()]

    # ── accumulation (eq 13) ─────────────────────────────────────────────────

    def accumulate(self, keys, r: np.ndarray) -> None:
        """Add each return's responsibility vector to its voxel's soft counts.

        Returns sharing a voxel are summed first, so the store is touched once
        per distinct voxel rather than once per return.
        """
        keys = np.asarray(keys, dtype=np.int64)
        r = np.asarray(r, dtype=np.float64)
        if keys.size == 0:
            return

        uniq, inv = np.unique(keys, return_inverse=True)
        sums = np.zeros((uniq.size, self.n_classes), dtype=np.float64)
        np.add.at(sums, inv.ravel(), r)

        m_store, M_store = self._m, self._M
        for j, k in enumerate(uniq):
            key = int(k)
            row = m_store.get(key)
            if row is None:
                m_store[key] = sums[j].copy()
                M_store[key] = float(sums[j].sum())
            else:
                row += sums[j]
                M_store[key] += float(sums[j].sum())

    # ── reporting (eqs 14-15) ────────────────────────────────────────────────

    def __len__(self):
        return len(self._m)

    def snapshot_arrays(self):
        """(keys, m (N, C), M (N,)) of the live store, in dict order."""
        n = len(self._m)
        keys = np.empty(n, dtype=np.int64)
        m = np.empty((n, self.n_classes), dtype=np.float64)
        M = np.empty(n, dtype=np.float64)
        for i, (k, row) in enumerate(self._m.items()):
            keys[i] = k
            m[i] = row
            M[i] = self._M[k]
        return keys, m, M

    def posterior_mean(self, m: np.ndarray, M: np.ndarray) -> np.ndarray:
        """theta_hat = (alpha_c + m_c) / (alpha_0_total + M_v)  (eq 14)."""
        return (self.alpha_0 + m) / (self._alpha_total + M)[:, None]

    def census(self, min_evidence: float = 0.0):
        """Per-class voxel counts and accumulated soft mass — the debug curve.

        Returns ``(counts, mass, n_voxels)``:
          counts   (C,) int   — voxels whose MAP label (eq 15) is class c
          mass     (C,) float — sum over voxels of m_{v,c}, the total soft
                                evidence each class has attracted
          n_voxels int        — voxels that passed ``min_evidence``

        ``min_evidence`` drops voxels with M_v below it, so a single grazing
        return cannot register as a confidently labelled voxel. Because
        alpha_0 is symmetric, argmax_c (alpha_c + m_c) = argmax_c m_c; ties
        (an untouched or perfectly balanced voxel) fall to the first class, so
        the evidence floor matters for an honest curve.
        """
        counts = np.zeros(self.n_classes, dtype=np.int64)
        mass = np.zeros(self.n_classes, dtype=np.float64)
        if not self._m:
            return counts, mass, 0

        _, m, M = self.snapshot_arrays()
        keep = M >= float(min_evidence)
        if not np.any(keep):
            return counts, mass, 0

        m = m[keep]
        labels = np.argmax(m, axis=1)
        counts = np.bincount(labels, minlength=self.n_classes).astype(np.int64)
        mass = m.sum(axis=0)
        return counts, mass, int(keep.sum())
