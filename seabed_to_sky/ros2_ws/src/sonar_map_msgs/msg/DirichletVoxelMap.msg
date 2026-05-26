# A full snapshot of the active Dirichlet voxel store.
#
# Published by voxel_mapper; swept by seabed_surface, which applies the
# promotion-readiness check. Stored as parallel arrays (all of length N) plus a
# flattened ragged ring-buffer block, so the consumer can vectorise the sweep.

std_msgs/Header header

int64[]   voxel_key          # packed 21-bit-per-axis voxel hash keys
float64[] center_x           # world-frame voxel centre x [m, odom]
float64[] center_y           # world-frame voxel centre y [m, odom]
float64[] center_z           # world-frame voxel centre z [m, odom]
float32[] w_free             # Dirichlet evidence weight, FREE
float32[] w_sb               # Dirichlet evidence weight, SEABED
float32[] w_obj              # Dirichlet evidence weight, OBJECT
float32[] w_str              # Dirichlet evidence weight, STRUCTURE
float32[] weight             # total accumulated Dirichlet weight W
float32[] mean_slant_range   # mean slant range of contributing returns [m]

# Ragged ring buffers, flattened: voxel i owns the ring_count[i] values of
# ring_depths_flat that start at offset sum(ring_count[0:i]).
int32[]   ring_count
float32[] ring_depths_flat
