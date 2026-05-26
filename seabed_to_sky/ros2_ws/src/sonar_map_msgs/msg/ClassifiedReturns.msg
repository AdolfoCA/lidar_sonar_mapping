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
float64 seabed_depth   # mu_h used [m, positive down from the z=0 datum] — unused
                       # downstream, kept for backward compatibility (always 0).
float64 tau_struct     # structure intensity threshold = lambda_struct * mu_i
float64 tau_obj        # object    intensity threshold = lambda_obj    * mu_i
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
float32[] lidar_support # geometric LiDAR-support score:
                        #   v = (count of LiDAR pts within R of nearest LiDAR
                        #        point q*) * exp(-d / d_scale),
                        # where d = horizontal distance from the return to q*.
                        # Continuous in [0, +inf); 0 when no LiDAR is available.
