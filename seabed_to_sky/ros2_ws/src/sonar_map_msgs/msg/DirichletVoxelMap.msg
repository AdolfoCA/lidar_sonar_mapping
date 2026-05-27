# A full snapshot of the active Dirichlet voxel store.
#
# Published by voxel_mapper; swept by seabed_surface, which applies the
# promotion-readiness check. Stored as parallel arrays (all of length N).
# Voxel centres are recoverable from voxel_key via the 21-bit-per-axis
# unpacking (matches sonar_map_ned_semantic) — no need to ship them.

std_msgs/Header header

int64[]   voxel_key          # packed 21-bit-per-axis voxel hash keys
float32[] w_free             # Dirichlet evidence weight, FREE
float32[] w_sb               # Dirichlet evidence weight, SEABED
float32[] w_obj              # Dirichlet evidence weight, OBJECT
float32[] w_str              # Dirichlet evidence weight, STRUCTURE
float32[] weight             # total accumulated Dirichlet weight W
float32[] alpha_str_prior    # per-voxel LiDAR-derived STRUCTURE prior added to
                             # alpha_0 in pi_str's numerator and the shared
                             # denominator: pi_str = (alpha_0 + alpha_str_prior
                             # + w_str) / (4*alpha_0 + alpha_str_prior + W).
