import numpy as np
from filters.filter import GNSSIMUKalmanFilter  # Assuming EKF is the class name

# Initialize EKF
kf = GNSSIMUKalmanFilter(rate=10.0)

# Define control input from IMU for a couple of cycles
# Assuming ax, ay, az are in m/s^2 and yaw is in rad
control_inputs = np.array([
    [1.0, 0.0],  # ax, ay, az, dyaw
    [0.8, 0.01],
    [0.7, 0.01],
    [0.62, -0.01]
])

# Define measurement (x, y, z) and covariance matrix
measurement = np.array([0.1, 0.1, 0.57])
R = np.eye(3) * 0.01  # Assuming measurement noise is 0.01 for all dimensions

# Perform prediction and update steps for each control input
for u in control_inputs:
    # Perform prediction step
    kf.predict(u)

    # Perform update step
    kf.update(measurement, R)

    x_p, cov_p = kf.get_state()

    # Print results
    print("Control input:", u)
    print("Predicted state:", x_p)
    # print("Updated state:", ekf.state)