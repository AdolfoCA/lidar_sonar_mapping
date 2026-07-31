"""
classes — load and validate the FIXED class inventory from classes.yaml.

The paper learns each class's parameters online (Table I / §II-E–G); this
pipeline instead reads them as fixed values from a YAML file and never updates
them. The catch-all "other" (the paper's base measure Λ_H, Alg. 1 line 8) is
appended as the LAST class, so every per-class array below has length K+1 and
column K is always "other".

All parameters are stored as the arrays the factor modules consume directly
(variances, radians, logits precomputed where that is the natural unit), so a
factor evaluation is pure array broadcasting with no per-call conversion.
"""

from dataclasses import dataclass, field

import numpy as np
import yaml

# Order of the shape descriptor channels everywhere in this package
# (matrices are (K+1, 4) with columns in this order — eq 41).
SHAPE_CHANNELS = ('logarea', 'solidity_logit', 'shadow', 'texture')

FACTOR_NAMES = ('position', 'intensity', 'lidar', 'shape')

OTHER = 'other'  # reserved name of the trailing catch-all class


@dataclass(frozen=True)
class ClassSet:
    """The fixed class inventory: per-class parameter arrays, "other" LAST.

    Every array has length K+1 (or (K+1, 4) for the shape channels), where
    K = number of predefined classes; index K is the catch-all "other".
    """

    names: tuple            # (K+1,) class names, names[-1] == 'other'
    # position factor (eq 31)
    nu: np.ndarray          # ν_c  height-band mean [m]
    omega2: np.ndarray      # ω²_c height-band intrinsic variance [m²]
    # intensity factor (eq 34)
    mu: np.ndarray          # μ_c  base backscatter intensity [I]
    sigma2: np.ndarray      # σ²_c residual intensity variance [I²]
    # lidar factor (eq 37)
    pi_l: np.ndarray        # π^ℓ_c presence probability ∈ (0, 1)
    kappa: np.ndarray       # κ_c  zero-truncated-Poisson count rate > 0
    gamma_rate: np.ndarray  # γ_c  Exponential gap rate [1/m] > 0
    # shape factor (eqs 41–42)
    rho: np.ndarray         # ρ_c  region-presence probability ∈ (0, 1)
    shape_mean: np.ndarray  # (K+1, 4) descriptor means, columns = SHAPE_CHANNELS
    shape_var: np.ndarray   # (K+1, 4) descriptor variances
    # global constants (shared across classes — sensor geometry, not class props)
    lambda_lift: float      # λ  height–brightness lift [I/m] (eq 33)
    tilt_rad: float         # θ  sonar tilt [rad] (eq 26)
    aperture_rad: float     # Δϕ vertical aperture [rad] (eq 27)
    other_weight: float     # γ-analogue multiplying the "other" responsibility
    factors_enabled: dict = field(default_factory=dict)  # factor name -> bool

    @property
    def n_classes(self) -> int:
        """K+1 — predefined classes plus the trailing 'other'."""
        return len(self.names)

    @property
    def k_predefined(self) -> int:
        """K — number of predefined classes (excluding 'other')."""
        return len(self.names) - 1


def _require(cond: bool, msg: str):
    if not cond:
        raise ValueError(f'classes.yaml: {msg}')


def _class_row(name: str, blk: dict):
    """Validate one class block and return its flat parameter tuple."""
    for section in ('position', 'intensity', 'lidar', 'shape'):
        _require(section in blk, f'class "{name}" is missing the "{section}" block')

    pos, inten, lid, shp = blk['position'], blk['intensity'], blk['lidar'], blk['shape']
    _require(float(pos['omega']) > 0.0, f'class "{name}": position.omega must be > 0')
    _require(float(inten['sigma']) > 0.0, f'class "{name}": intensity.sigma must be > 0')
    _require(0.0 < float(lid['pi']) < 1.0, f'class "{name}": lidar.pi must be in (0, 1)')
    _require(float(lid['kappa']) > 0.0, f'class "{name}": lidar.kappa must be > 0')
    _require(float(lid['gamma']) > 0.0, f'class "{name}": lidar.gamma must be > 0')
    _require(0.0 < float(shp['rho']) < 1.0, f'class "{name}": shape.rho must be in (0, 1)')

    means, variances = [], []
    for ch in SHAPE_CHANNELS:
        _require(ch in shp, f'class "{name}": shape is missing the "{ch}" channel')
        _require(float(shp[ch]['std']) > 0.0, f'class "{name}": shape.{ch}.std must be > 0')
        means.append(float(shp[ch]['mean']))
        variances.append(float(shp[ch]['std']) ** 2)

    return (
        float(pos['nu']), float(pos['omega']) ** 2,
        float(inten['mu']), float(inten['sigma']) ** 2,
        float(lid['pi']), float(lid['kappa']), float(lid['gamma']),
        float(shp['rho']), means, variances,
    )


def load_classes(path: str) -> ClassSet:
    """Parse classes.yaml into a validated ClassSet ('other' appended last)."""
    with open(path, 'r') as fh:
        cfg = yaml.safe_load(fh)

    _require(isinstance(cfg, dict), 'top level must be a mapping')
    for key in ('global', 'classes', 'other'):
        _require(key in cfg, f'missing top-level "{key}" block')

    class_blocks = cfg['classes']
    _require(isinstance(class_blocks, list) and len(class_blocks) >= 1,
             '"classes" must be a non-empty list')

    names = []
    for blk in class_blocks:
        _require('name' in blk, 'every class needs a "name"')
        names.append(str(blk['name']))
    _require(len(set(names)) == len(names), f'duplicate class names in {names}')
    _require(OTHER not in names,
             f'"{OTHER}" is reserved for the catch-all block — rename that class')

    rows = [_class_row(n, blk) for n, blk in zip(names, class_blocks)]
    rows.append(_class_row(OTHER, cfg['other']))
    names.append(OTHER)

    (nu, omega2, mu, sigma2, pi_l, kappa, gamma_rate, rho,
     shape_mean, shape_var) = (np.array(col) for col in zip(*rows))

    g = cfg['global']
    factors_cfg = cfg.get('factors', {})
    factors_enabled = {f: bool(factors_cfg.get(f, True)) for f in FACTOR_NAMES}

    aperture = float(g.get('elev_aperture_deg', 20.0))
    _require(aperture > 0.0, 'global.elev_aperture_deg must be > 0')
    other_weight = float(g.get('other_weight', 1.0))
    _require(other_weight > 0.0, 'global.other_weight must be > 0')

    return ClassSet(
        names=tuple(names),
        nu=nu.astype(np.float64),
        omega2=omega2.astype(np.float64),
        mu=mu.astype(np.float64),
        sigma2=sigma2.astype(np.float64),
        pi_l=pi_l.astype(np.float64),
        kappa=kappa.astype(np.float64),
        gamma_rate=gamma_rate.astype(np.float64),
        rho=rho.astype(np.float64),
        shape_mean=shape_mean.astype(np.float64),
        shape_var=shape_var.astype(np.float64),
        lambda_lift=float(g.get('lambda_lift', 0.0)),
        tilt_rad=float(np.deg2rad(float(g.get('sonar_tilt_deg', 0.0)))),
        aperture_rad=float(np.deg2rad(aperture)),
        other_weight=other_weight,
        factors_enabled=factors_enabled,
    )
