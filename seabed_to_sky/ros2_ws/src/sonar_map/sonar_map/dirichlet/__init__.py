"""
dirichlet — pure-NumPy implementation of the paper's class-conditional
likelihood (eq 24) over a FIXED, config-defined class set.

    L_{i,c} = p(p_i|c) · p(I_i|p_i,c) · p(ℓ_i|c) · p(m_i|c)          (eq 24)

One module per factor, plus the shared geometry and the assembler:

    classes.py          class definitions loaded from classes.yaml (Table I)
    geometry.py         shared per-return quantities: h_i, ϱ_i  (eqs 25, 29–30)
    factor_position.py  p(p_i|c)     = N(h_i; ν_c, ω²_c + ϱ_i)        (eq 31)
    factor_intensity.py p(I_i|p_i,c) = N(I_i; μ_c + λh_i, σ²_c + λ²ϱ_i) (eq 34)
    factor_lidar.py     p(ℓ_i|c)     hurdle Bernoulli×Pois>0×Exp      (eq 37)
    factor_shape.py     p(m_i|c)     hurdle + de-duplicated Gaussians (eq 42)
    likelihood.py       sums the factor log-matrices into log L (N, K+1)
    bottom_grid.py      rolling ẑ_bed(x, y) bottom reference for h_i

DESIGN RULES (all modules):
  * NO ROS imports — everything here runs offline on plain arrays, so each
    factor can be unit-tested against hand-computed values and replayed on
    logged data without spinning a node.
  * LOG space only — factors return (N, K+1) log-likelihood matrices; the
    product of eq 24 is a sum here. Products of four small densities underflow
    float64; log-sums do not.
  * ABSTENTION — a factor that cannot be evaluated for a return (its channel
    is NaN, e.g. no LiDAR cloud yet, or the factor is disabled in the config)
    contributes a row of ZEROS: the remaining factors still combine correctly.
  * The class axis is always length K+1 with the catch-all "other" LAST,
    matching ClassSet.names.
"""

from sonar_map.dirichlet.classes import ClassSet, load_classes  # noqa: F401
