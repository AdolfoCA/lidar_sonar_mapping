import numpy as np

class GNSSIMUKalmanFilter:
    """
    Extended Kalman Filter for GNSS-IMU fusion.
    Treats the GNSS as the measurement and the IMU as the control input.

    The state vector is defined as:
    [easting, northing, d_easting, d_northing, b_ax, b_ay] in gps frame

    The control input vector is defined as:
    [ax, ay] in base_link frame
    """

    N_STATES = 6
    G = 9.83

    def __init__(self, rate: float = 10.0, heading: float = 0.0) -> None:
        """
        Args:
            rate (float, optional): Sample rate of the filter. Defaults to 10.0.
            heading (float, optional): Initial heading angle in radians. Defaults to 0.0.
        """
        self.x = np.zeros((self.N_STATES))           # easting, northing, d_easting, d_northing, b_ax, b_ay
        self.P = np.eye(self.N_STATES) * 0.001   # Model uncertainty
        self.P[0,0] = 0.001 # we are certain about the initial position
        self.P[1,1] = 0.001 # we are certain about the initial position
        self.Q = np.eye(self.N_STATES) * 0.005  # Process noise
        self.K = None
        self.y = None

        self.SAMPLE_RATE = rate
        self.dt = 1.0 / self.SAMPLE_RATE

        self.heading = self._wrap_angle(heading)
        self.B = np.array([
            [0, 0],
            [0, 0],
            [self.dt, 0],
            [0, self.dt],
            [0, 0],
            [0, 0],
        ])
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0]
        ])


    def predict(self, u: np.ndarray) -> None:
        """
        Prediction step of the Kalman Filter

        Args:
            u (np.ndarray): Control input vector [a_x, a_y] (body frame)
        """
        F = self._F(self.heading)
        u_bias = self.x[4:6]
        x_pred = F @ self.x + self.B @ (u - u_bias)
        P_pred = F @ self.P @ F.T + self.Q

        self.x = x_pred
        self.P = P_pred

    def update(self, m: np.ndarray, R: np.ndarray) -> None:
        """
        Update step of the Kalman Filter.

        Args:
            m (np.ndarray): Measurement vector [easting (m), northing (m), heading (rad)]
            R (np.ndarray): Measurement covariance matrix [3x3]
        """
        meas = m[:2]
        R_meas = R[:2, :2]
        R_meas = R_meas * 10e1
        y = meas - self.H @ self.x
        self.y = y

        S = self.H @ self.P @ self.H.T + R_meas
        self.K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + self.K @ y
        I = np.eye(self.N_STATES)
        self.P = (I - self.K @ self.H) @ self.P
        
        self.heading = self._wrap_angle(m[2])


    def get_state(self) -> np.ndarray:
        """
        Get the current state vector

        Returns:
            np.ndarray: Current state vector [easting, northing]
            np.ndarray: Current state covariance matrix [2x2]
        """
        output_state = np.array([
            self.x[0],  
            self.x[1],
        ])

        output_cov = np.array([
            [self.P[0, 0], 0.0],
            [0.0, self.P[1, 1]],
        ])

        return output_state, output_cov
    
    def _F(self, theta: float) -> np.ndarray:
        """
        State transition matrix

        Returns:
            np.ndarray: State transition matrix
        """
        dt = self.dt
        return np.array([
            [1, 0, np.cos(theta) * dt, -np.sin(theta) * dt, 0, 0],
            [0, 1, np.sin(theta) * dt,  np.cos(theta) * dt, 0, 0],
            [0, 0, 1, 0, -dt, 0],
            [0, 0, 0, 1, 0, -dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

    def _wrap_angle(self, angle: float) -> float:
        """
        Wrap angle to [-pi, pi]

        Args:
            angle (float): Angle in radians

        Returns:
            float: Wrapped angle in radians
        """
        return (angle + np.pi) % (2*np.pi) - np.pi
    


