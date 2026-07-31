# A full snapshot of the Dirichlet voxel store, over the PREDEFINED class set
# C = {classes.yaml classes..., "other"} — K+1 categories, "other" always last.
#
# Published by voxel_mapper. Stored as parallel arrays. Voxel centres are
# recoverable from voxel_key via the 21-bit-per-axis unpacking (matches
# voxel_mapper) — no need to ship them.
#
# The per-voxel Dirichlet soft counts are flattened row-major:
#     w[i * n_classes + c]  =  m_{v_i, c}
# so voxel i's full count vector is  w[i*K̃ : (i+1)*K̃]  with K̃ = n_classes.

std_msgs/Header header

uint8     n_classes          # K+1 (predefined classes + trailing "other")
string[]  class_names        # length n_classes; class_names[c] labels column c

int64[]   voxel_key          # packed 21-bit-per-axis voxel hash keys (length N)
float32[] w                  # flattened (N × n_classes) Dirichlet soft counts
float32[] weight             # total accumulated evidence  W = M_v = Σ_c m_{v,c}  (length N)
