# Keys of voxels that seabed_surface has promoted on its periodic sweep.
#
# Sent back to voxel_mapper, which removes them from the active Dirichlet store
# and retires their locations so returns that later fall there are dropped.
# Promotion is irreversible.
#
# seabed_surface re-sends any promoted key still present in the latest snapshot,
# so the feedback self-heals if a message is dropped.

std_msgs/Header header

int64[] keys
