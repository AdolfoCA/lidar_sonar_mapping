# Per-scan debug telemetry of the eq-24 class-conditional likelihood pipeline —
# one message per processed scan, for real-time debugging in Foxglove /
# PlotJuggler of WHICH factor drives the class assignments.
#
# All per-class arrays are parallel to class_names (length n_classes, "other"
# last). The per-factor matrix is flattened row-major:
#     mean_log_lik[f * n_classes + c] = mean over the scan's returns of the
#     log-likelihood of factor f under class c.
# A factor disabled in classes.yaml still occupies its row (all zeros) so the
# row indexing is stable across configurations.
#
# Foxglove: plot `.resp_share[c]` to watch how the scan's responsibility mass
# splits across classes, and `.mean_log_lik[f*K̃+c]` to see which factor is
# pulling class c up or down.

std_msgs/Header header

uint32    n_returns          # returns in this scan
uint8     n_classes          # K+1 (matches DirichletVoxelMap)
string[]  class_names        # length n_classes
string[]  factor_names       # length n_factors: [position, intensity, lidar, shape]

float32[] mean_log_lik       # flattened (n_factors × n_classes) — see above
float32[] resp_share         # (n_classes,) mean responsibility r̄_c over the scan
float32[] map_share          # (n_classes,) fraction of scan returns whose
                             # argmax_c(L·π) is class c (hard-assignment view)
