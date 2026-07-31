# Live snapshot of the online-learned class inventory in learned_voxel_mapper —
# one message per scan, for real-time debugging of class FORMATION and LEARNING
# in Foxglove/PlotJuggler. All per-class arrays are parallel (length n_classes)
# and ordered by class_id (append-only), so array index == class index is stable
# over time and can be plotted as mu[0], mu[1], ... .
#
# Foxglove: add a Plot panel and series `.n_classes` (watch classes appear),
# `.mu[i]` (each class's learned intensity mean), `.sigma[i]` (its std). The
# intensity Gaussian of class i is N(mu[i], sigma[i]^2); evidence[i] is the
# effective number of returns behind it (the NIG pseudo-count κ).

std_msgs/Header header

int32     n_classes           # current size of the inventory

int32[]   class_id            # stable id (birth order)
uint8[]   semantic_tag        # 0=SEABED, 1=OBJECT, 2=STRUCTURE (by brightness this scan)
float32[] mu                  # learned intensity mean  μ̂_c   (updated every scan)
float32[] sigma               # learned intensity std   σ̂_c = sqrt(b/a)
float32[] evidence            # κ_c — effective # returns behind the class
