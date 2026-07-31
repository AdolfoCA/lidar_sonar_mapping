"""
argus — pure-NumPy implementation of the ARGUS class-conditional likelihood and
the Dirichlet voxel map.

    L_{i,c} = p(p_i|c) · p(A_i|c) · p(l_i|c) · p(m_i|c)

One module per concern:

    classes.py          the a-priori class inventory, loaded from classes.yaml
    soft_interval.py    B(x; l, u, sig) — the soft-box family (eq 23), shared by
                        the position factor and the "under cover" clearance
    geometry.py         s_i, rho_var_i (eqs 21-22) and h_i (eq 18)
    bottom_grid.py      rolling z_bed_hat(x, y) — the local bed fit behind h_i
    region_descriptor.py e_R (beam-corrected bearing extent), w_R (thickness)
    factor_position.py  p(p_i|c) = B(h_i; l_c, u_c, sig_ic)          (eq 24)
    factor_amplitude.py p(A_i|c) = N(A_i; mu_c, sigma_c^2)  raw counts
    factor_lidar.py     p(l_i|c) — coverage-gated hurdle             (eqs 28-29)
    factor_shape.py     p(m_i|c) — log-Gaussian hurdle, deduped      (eqs 32-33)
    likelihood.py       sums the factor log-matrices; responsibilities (eq 9)
    voxel_map.py        Dirichlet map, (t-1) snapshot, spatial prediction (eq 16),
                        MAP labels (eq 15), per-class voxel census

DESIGN RULES (all modules in this subpackage):

  * NO ROS IMPORTS. Everything here runs on plain arrays, so each factor can be
    unit-tested against hand-computed values and replayed on logged data with
    no node spinning.
  * LOG SPACE ONLY (I5). Factors return (N, C) log-likelihood matrices; the
    product of eq 34 is a sum here. Four multiplied densities underflow
    float64; log-sums do not. Results are always finite — never NaN, never
    -inf — so no class can be permanently vetoed by one numerical edge case.
  * ABSTENTION IS A ZERO ROW. A factor that cannot be evaluated for a return
    (its gating channel is NaN, or the factor is disabled in config)
    contributes zeros — the multiplicative identity — so the remaining factors
    still combine correctly and no return is ever dropped.
  * NO LEARNING (I4). Class parameters are read from config and never mutated.
    The only online accumulation anywhere is the Dirichlet over voxels.
"""

from sonar_classification.argus.classes import ClassSet, load_classes  # noqa: F401
