# One batch of semantically classified sonar returns — one message per scan.
#
# Per-return data is stored as parallel arrays (all of length N) rather than an
# array of sub-messages: this avoids per-element serialization overhead and is
# still self-describing through the named fields and the label constants below.
#
# All positions are in the map frame (odom, ENU, z up). The scan-level scalars
# carry the estimator-derived context that was used to classify this batch, so
# a downstream consumer needs no other input to interpret the returns.

std_msgs/Header header

# ── Class labels (values of the per-return `label` field) ────────────────────
uint8 FREE=0
uint8 SEABED=1
uint8 OBJECT=2
uint8 STRUCTURE=3

# ── Scan-level context used for this classification ──────────────────────────
float64 mu_i           # live seabed-intensity estimate used by voxel_mapper to
                       # compute soft Dirichlet responsibilities per return.

# ── Per-return parallel arrays (each of length N) ────────────────────────────
float32[] x             # return position x [m, odom]
float32[] y             # return position y [m, odom]
float32[] z             # return position z [m, odom]
float32[] intensity     # raw return intensity
float32[] slant_range   # range from the sonar to the return [m]
uint8[]   label         # assigned hard class (FREE vs non-FREE drives routing;
                        # actual class evidence is via soft responsibilities)
float32[] d_factor      # geometric LiDAR distance gate, in [0, 1]:
                        #   d_factor = exp(-d / d_scale)
                        # where d = horizontal distance from the return to the
                        # nearest LiDAR point q*. 1 when at LiDAR; sharp decay.
float32[] count_factor  # geometric LiDAR cluster density gate, in [0, 1):
                        #   count_factor = 1 - exp(-alpha_count * count)
                        # where count = # LiDAR pts within r_lidar of q*.
                        # Saturates with cluster density.