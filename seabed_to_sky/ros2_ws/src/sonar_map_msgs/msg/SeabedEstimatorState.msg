# State of the seabed_estimator node — published once per sonar scan.
#
# A single scalar Kalman filter tracks the typical seabed backscatter intensity
# (mu_i). The depth half of the estimator was removed: bathymetry now emerges
# from the per-voxel Dirichlet aggregation in voxel_mapper.

std_msgs/Header header

float64 mu_i    # estimated seabed intensity       [sonar intensity units]
float64 var_i   # variance of the seabed intensity [intensity^2]
