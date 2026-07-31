"""
classes — the a-priori class inventory, loaded and validated from classes.yaml.

This is the whole "model": a class c is fully specified by

    theta_c = ( l_c, u_c, tau_c;  mu_c, sigma_c;  pi_lidar_c, zeta_c;
                rho_c, e_bar_c, E_c, w_bar_c, W_c )                      (eq 35)

with zeta_c the clearance parameters (a rate gamma_c for surface-piercing
classes, or a soft interval for classes beneath cover). Every one of them is
assigned a priori from acoustic and geometric reasoning. NOTHING here is ever
fitted, updated, or learned at runtime (I4) — ``load_classes`` is called once
at node start and the returned :class:`ClassSet` is frozen.

ASSIGNABILITY (I8) drives the file format. A parameter is admissible only if it
is a physical quantity you can write down or count at a calibration target, so
the YAML asks for exactly those:

  * height intervals and clearances are METRES,
  * amplitude levels are RAW COUNTS off the sonar image,
  * ``pi_lidar`` and ``rho`` are FRACTIONS OF RETURNS — assign the complement
    ("what fraction of wall returns lack validated overhead support?"),
  * region shape is given as a MEDIAN LENGTH plus a SPREAD FACTOR ("a pillar is
    about 0.5 m across, within a factor of 2"), which this loader converts to
    the log-Gaussian parameters the factor consumes:
        e_bar_c = log(median_m),  E_c = log(spread_factor).

Two classes are separated ONLY by the channels in which their parameters
differ (I7) — a channel whose values coincide cancels from their pairwise
comparison. Copy shared channels verbatim when splitting a class and put the
distinction in the one channel whose physics actually differs. Do not invent
fake differences in a channel that is physically identical.

There is NO reserved catch-all class. The inventory is exactly the list in the
YAML; if you want an "unexplained" sink, define a deliberately broad class and
name it whatever you like.
"""

from dataclasses import dataclass, field

import numpy as np
import yaml

FACTOR_NAMES = ('position', 'amplitude', 'lidar', 'shape')

#: Accepted values of ``ablation.shape_dedup_mode``.
DEDUP_MODES = ('per_voxel', 'inv_nr')




@dataclass(frozen=True)
class ClassSet:
    """The frozen a-priori inventory: one array entry per class, in YAML order.

    Every array has length C = number of classes; index c refers to
    ``names[c]`` throughout the package (the class axis is this order,
    everywhere, including the published point fields and the census CSV).
    """

    names: tuple                # (C,) class names, in file order

    # ── position factor (eq 24) ──────────────────────────────────────────────
    l: np.ndarray               # l_c   height occupancy lower edge [m]
    u: np.ndarray               # u_c   ... upper edge [m], u_c >= l_c
    tau: np.ndarray             # tau_c intrinsic edge softness [m]

    # ── amplitude factor ─────────────────────────────────────────────────────
    mu: np.ndarray              # mu_c    raw amplitude level [counts]
    sigma: np.ndarray           # sigma_c amplitude spread [counts]

    # ── LiDAR-support factor (eqs 28-29) ─────────────────────────────────────
    pi_lidar: np.ndarray        # pi^l_c  P(validated overhead support | c), in (0,1)
    clr_is_exp: np.ndarray      # bool: True = surface-piercing (exponential clearance)
    clr_gamma: np.ndarray       # gamma_c exponential rate [1/m] (exp classes)
    clr_l: np.ndarray           # lg_c    clearance soft-box lower edge [m] (box classes)
    clr_u: np.ndarray           # ug_c    ... upper edge [m]
    clr_sigma: np.ndarray       # sg_c    ... edge softness [m]

    # ── morphology factor (eqs 32-33) ────────────────────────────────────────
    rho: np.ndarray             # rho_c   P(return forms a bright region | c), in (0,1)
    e_bar: np.ndarray           # e_bar_c mean of log bearing extent [log m]
    e_std: np.ndarray           # E_c     std of log bearing extent
    w_bar: np.ndarray           # w_bar_c mean of log range thickness [log m]
    w_std: np.ndarray           # W_c     std of log range thickness

    # ── global constants: sensor / water / site, shared by every class ───────
    tilt_rad: float             # theta      fixed mechanical downtilt [rad]
    aperture_rad: float         # Delta_phi  unresolved vertical aperture [rad]
    beamwidth_rad: float        # angular WIDTH of one beam [rad] (deconvolved)
    beam_spacing_rad: float     # angular STEP between adjacent beams [rad]
    eps_xy: float               # eps_xy     overhead-column radius [m]
    k_min: int                  # k_min      overhead points required to validate


    factors_enabled: dict = field(default_factory=dict)   # factor name -> bool
    ablation: dict = field(default_factory=dict)          # flag name -> value

    @property
    def n_classes(self) -> int:
        return len(self.names)

    def index(self, name: str) -> int:
        """Column of ``name`` in the class axis. Raises if it is not defined."""
        return self.names.index(name)



def _require(cond: bool, msg: str):
    if not cond:
        raise ValueError(f'classes.yaml: {msg}')


def _log_shape(name: str, channel: str, blk: dict):
    """(mean, std) of a log-Gaussian length from ``median_m`` + ``spread_factor``.

    The YAML states a length you can measure and a multiplicative tolerance,
    because both lengths vary across classes by FACTORS, not increments.
    """
    _require('median_m' in blk and 'spread_factor' in blk,
             f'class "{name}": shape.{channel} needs median_m and spread_factor')
    median = float(blk['median_m'])
    spread = float(blk['spread_factor'])
    _require(median > 0.0, f'class "{name}": shape.{channel}.median_m must be > 0 m')
    _require(spread > 1.0,
             f'class "{name}": shape.{channel}.spread_factor must be > 1 '
             '(it is a multiplicative tolerance: 2.0 means "within a factor of two")')
    return float(np.log(median)), float(np.log(spread))


def _class_row(name: str, blk: dict):
    """Validate one class block and flatten it into the parameter tuple."""
    for section in FACTOR_NAMES:
        _require(section in blk, f'class "{name}" is missing the "{section}" block')

    pos, amp, lid, shp = (blk['position'], blk['amplitude'],
                          blk['lidar'], blk['shape'])

    # ── position ─────────────────────────────────────────────────────────────
    l_c, u_c, tau_c = float(pos['l']), float(pos['u']), float(pos['tau'])
    _require(u_c >= l_c, f'class "{name}": position.u must be >= position.l '
                         '(u == l is the legitimate zero-extent / seabed member)')
    _require(tau_c > 0.0, f'class "{name}": position.tau must be > 0 m')

    # ── amplitude ────────────────────────────────────────────────────────────
    mu_c, sigma_c = float(amp['mu']), float(amp['sigma'])
    _require(mu_c > 0.0, f'class "{name}": amplitude.mu must be > 0 counts')
    _require(sigma_c > 0.0, f'class "{name}": amplitude.sigma must be > 0 counts')

    # ── LiDAR support ────────────────────────────────────────────────────────
    pi_c = float(lid['pi'])
    _require(0.0 < pi_c < 1.0,
             f'class "{name}": lidar.pi must be strictly inside (0, 1) — it is a '
             'FRACTION OF RETURNS, not a property of the object. pi = 1 makes a '
             'single coverage gap contribute log(0) = -inf and permanently vetoes '
             'the correct class in that voxel.')
    clr = lid.get('clearance', {})
    kind = str(clr.get('kind', 'exp')).lower()
    _require(kind in ('exp', 'box'),
             f'class "{name}": lidar.clearance.kind must be "exp" '
             '(surface-piercing) or "box" (sits beneath cover)')
    if kind == 'exp':
        gamma = float(clr['gamma'])
        _require(gamma > 0.0, f'class "{name}": lidar.clearance.gamma must be > 0')
        clr_l = clr_u = 0.0
        clr_s = 1.0
    else:
        gamma = 1.0
        clr_l, clr_u = float(clr['l']), float(clr['u'])
        clr_s = float(clr['sigma'])
        _require(clr_u >= clr_l,
                 f'class "{name}": lidar.clearance.u must be >= .l')
        _require(clr_s > 0.0, f'class "{name}": lidar.clearance.sigma must be > 0')

    # ── morphology ───────────────────────────────────────────────────────────
    rho_c = float(shp['rho'])
    _require(0.0 < rho_c < 1.0,
             f'class "{name}": shape.rho must be strictly inside (0, 1) — a '
             'fraction of returns; rho = 1 vetoes the class on any bare return.')
    _require('extent' in shp and 'thickness' in shp,
             f'class "{name}": shape needs both "extent" (e_R) and '
             '"thickness" (w_R)')
    e_bar, e_std = _log_shape(name, 'extent', shp['extent'])
    w_bar, w_std = _log_shape(name, 'thickness', shp['thickness'])

    return (l_c, u_c, tau_c,
            mu_c, sigma_c,
            pi_c, kind == 'exp', gamma, clr_l, clr_u, clr_s,
            rho_c, e_bar, e_std, w_bar, w_std)


def load_classes(path: str) -> ClassSet:
    """Parse and validate classes.yaml into a frozen :class:`ClassSet`."""
    with open(path, 'r') as fh:
        cfg = yaml.safe_load(fh)

    _require(isinstance(cfg, dict), 'top level must be a mapping')
    for key in ('global', 'classes'):
        _require(key in cfg, f'missing top-level "{key}" block')

    blocks = cfg['classes']
    _require(isinstance(blocks, list) and len(blocks) >= 2,
             '"classes" must be a list of at least two classes — a single-class '
             'inventory has no responsibility to compute')

    names = []
    for blk in blocks:
        _require('name' in blk, 'every class needs a "name"')
        names.append(str(blk['name']))
    _require(len(set(names)) == len(names), f'duplicate class names in {names}')

    rows = [_class_row(n, blk) for n, blk in zip(names, blocks)]
    cols = list(zip(*rows))

    def arr(i, dtype=np.float64):
        return np.array(cols[i], dtype=dtype)

    # ── globals: sensor geometry, water, site calibration ────────────────────
    g = cfg['global']
    aperture = float(g.get('elev_aperture_deg', 20.0))
    beamwidth = float(g.get('beamwidth_deg', 1.0))
    # Defaults to the beamwidth so an inventory written before this parameter
    # existed keeps its old behaviour rather than silently changing extents.
    spacing = float(g.get('beam_spacing_deg', beamwidth))
    _require(aperture > 0.0, 'global.elev_aperture_deg must be > 0')
    _require(beamwidth > 0.0, 'global.beamwidth_deg must be > 0')
    _require(spacing > 0.0, 'global.beam_spacing_deg must be > 0')
    _require(spacing <= beamwidth,
             f'global.beam_spacing_deg ({spacing}) must be <= beamwidth_deg '
             f'({beamwidth}): beams that step further apart than they are wide '
             'would leave gaps in the fan, which no multibeam does. Check you '
             'have not swapped the two.')

    lid_g = g.get('lidar', {}) or {}
    eps_xy = float(lid_g.get('eps_xy', 0.30))
    k_min = int(lid_g.get('k_min', 3))
    _require(eps_xy > 0.0, 'global.lidar.eps_xy must be > 0 m')
    _require(k_min >= 1, 'global.lidar.k_min must be >= 1 — the count is a '
                         'validation gate against spray/birds/noise, so a single '
                         'stray point must not be able to assert structure')

    mu, sigma = arr(3), arr(4)

    factors_cfg = cfg.get('factors', {}) or {}
    factors_enabled = {f: bool(factors_cfg.get(f, True)) for f in FACTOR_NAMES}

    abl_cfg = cfg.get('ablation', {}) or {}
    dedup = str(abl_cfg.get('shape_dedup_mode', 'per_voxel')).lower()
    _require(dedup in DEDUP_MODES,
             f'ablation.shape_dedup_mode must be one of {DEDUP_MODES}')
    ablation = {
        'position_gaussian': bool(abl_cfg.get('position_gaussian', False)),
        'lidar_legacy_gap_ref': bool(abl_cfg.get('lidar_legacy_gap_ref', False)),
        'shape_dedup_mode': dedup,
        'spatial_prior': bool(abl_cfg.get('spatial_prior', True)),
    }

    return ClassSet(
        names=tuple(names),
        l=arr(0), u=arr(1), tau=arr(2),
        mu=mu, sigma=sigma,
        pi_lidar=arr(5), clr_is_exp=arr(6, bool), clr_gamma=arr(7),
        clr_l=arr(8), clr_u=arr(9), clr_sigma=arr(10),
        rho=arr(11), e_bar=arr(12), e_std=arr(13),
        w_bar=arr(14), w_std=arr(15),
        tilt_rad=float(np.deg2rad(float(g.get('sonar_tilt_deg', 0.0)))),
        aperture_rad=float(np.deg2rad(aperture)),
        beamwidth_rad=float(np.deg2rad(beamwidth)),
        beam_spacing_rad=float(np.deg2rad(spacing)),
        eps_xy=eps_xy,
        k_min=k_min,
        factors_enabled=factors_enabled,
        ablation=ablation,
    )
