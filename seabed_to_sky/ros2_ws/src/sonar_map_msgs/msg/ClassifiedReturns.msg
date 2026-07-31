# One batch of sonar leading-edge returns with the raw measurement channels of
# the paper's measurement model  y_i = (p_i, I_i, ℓ_i, m_i)  (eq 9) — one
# message per scan.
#
# This message does NOT carry a hard class label. Classification is the soft
# Dirichlet responsibility r_{i,c} ∝ L_{i,c}·π_c(N_v) (eq 16), assembled per
# voxel in voxel_mapper from the four factors of the class-conditional
# likelihood (eq 24):
#     L_{i,c} = p(p_i|c) · p(I_i|p_i,c) · p(ℓ_i|c) · p(m_i|c)
# The channels below are exactly the inputs those factors consume.
#
# Per-return data is stored as parallel arrays (all of length N) rather than an
# array of sub-messages: this avoids per-element serialization overhead and is
# still self-describing through the named fields. All positions are in the map
# frame (odom, ENU, z up). Every return is a real leading-edge detection;
# zero-intensity / non-detection beams are dropped upstream (the paper's
# categorical has no FREE class).

std_msgs/Header header

# ── Scan-level seabed-intensity context (from seabed_estimator) ──────────────
# Informational / telemetry only in the fixed-class eq-24 pipeline: class
# intensity means μ_c are ABSOLUTE values from classes.yaml, not offsets.
float64 mu_i           # seabed backscatter intensity estimate  μ_I
float64 sigma_i        # seabed backscatter intensity std  σ_I (= sqrt(var_i))

# ── Position & intensity channels  p_i, I_i  (eqs 25, 31, 34) ────────────────
float32[] x             # return position x [m, odom]
float32[] y             # return position y [m, odom]
float32[] z             # return position z [m, odom] — BORESIGHT estimate z_w(0)
                        # (eq 26 at ϕ=0); the elevation ambiguity enters the
                        # factors through ϱ_i computed from slant_range (eq 30)
float32[] intensity     # backscatter intensity  I_i
float32[] slant_range   # r_i: range from the sonar to the return [m]
float32[] z_bed         # ẑ_bed(x_i, y_i): local bottom estimate at the return's
                        # XY [m, odom] (eq 25 needs h_i = z_i − ẑ_bed). NaN when
                        # the bottom reference is not yet available there.

# ── LiDAR-support channels  ℓ_i = (k_i, g_i)  (eqs 35–36) ────────────────────
# Overhead-column query around the RETURN's XY (radius ε_xy): does the return
# lie beneath structure seen by the surface LiDAR?  s^ℓ_i = 1[k_i > 0] is
# derived downstream. Feed the hurdle factor p(ℓ_i|c) (eq 37).
float32[] over_count    # k_i = |K_i|: # LiDAR points in the vertical column
                        # above the return (XY disc of radius ε_xy)
float32[] over_gap      # g_i = min_{q∈K_i} z_q − z_i: vertical gap to the
                        # lowest overhead LiDAR point [m]. NaN when k_i = 0
                        # (hurdle: magnitude modelled only when present).

# ── Legacy LiDAR channels (kept for the debug observers; NOT used by eq 24) ──
float32[] lidar_dist    # d_i: XY distance from the return to the nearest LiDAR
                        # point q* (horizontal match — see return_classifier)
float32[] lidar_count   # # LiDAR points within radius ε_L of q* (XY disc)

# ── Shape / morphology channels  m_i  (eqs 39–40) ────────────────────────────
# Computed in image domain by the leading-edge extractor (region growing from
# the leading edge into the bright pixels behind it). A return either belongs
# to exactly one grown region (s^m_i = 1, carrying that region's descriptor
# m_R, de-duplicated by the 1/N_R power in eq 42) or is bare seabed (s^m_i = 0,
# descriptor channels NaN). Feed the hurdle factor p(m_i|c) (eq 42).
float32[] shape_flag      # s^m_i ∈ {0, 1}: return belongs to a grown region
float32[] region_size     # N_R: # returns in the region (0 when s^m_i = 0)
float32[] region_logarea  # a_R: log physical area of the region [log m²]
float32[] region_solidity # ς_R ∈ (0, 1): region area / convex-hull area
                          # (raw; the factor applies the logit, eq 41)
float32[] region_shadow   # ξ_R: shadow signature of the region (composite of
                          # the depth/length/sharpness of the acoustic shadow
                          # trailing the region in range)
float32[] region_texture  # τ_R: internal intensity variance of the region
