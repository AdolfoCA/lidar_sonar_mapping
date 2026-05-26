# State of the seabed_estimator node — published once per sonar scan.
#
# Two independent scalar Kalman filters track the typical seabed depth and the
# typical seabed backscatter intensity. Depth is positive-downward, measured
# from the z = 0 odom datum (the surface plane the USV operates on).

std_msgs/Header header

float64 mu_h    # estimated seabed depth          [m, positive down]
float64 var_h   # variance of the seabed depth    [m^2]
float64 mu_i    # estimated seabed intensity       [sonar intensity units]
float64 var_i   # variance of the seabed intensity [intensity^2]
