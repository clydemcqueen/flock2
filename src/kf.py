import numpy as np


class KalmanFilter(object):

    def __init__(self, state_dim, measurement_dim):
        if state_dim < 1:
            raise ValueError('state_dim must be 1 or greater')
        if measurement_dim < 1:
            raise ValueError('measurement_dim must be 1 or greater')

        self.state_dim = state_dim                              # Number of state variables
        self.measurement_dim = measurement_dim                  # Number of measurement variables
        self.x = np.zeros((state_dim, 1))                       # State mean
        self.P = np.eye(state_dim)                              # State covariance
        self.Q = np.eye(state_dim)                              # Process covariance
        self.F = np.eye(state_dim)                              # State transition matrix
        self.H = np.zeros((measurement_dim, state_dim))         # Measurement function

        # Computed by update, here for inspection
        self.K = np.zeros((state_dim, measurement_dim))         # Kalman gain
        self.y = np.zeros((measurement_dim, 1))                 # Residual
        self.S = np.zeros((measurement_dim, measurement_dim))   # System uncertainty

    def check_shapes(self):
        if self.x.shape != (self.state_dim, 1):
            raise ValueError('x shape is wrong')
        if self.P.shape != (self.state_dim, self.state_dim):
            raise ValueError('P shape is wrong')
        if self.Q.shape != (self.state_dim, self.state_dim):
            raise ValueError('Q shape is wrong')
        if self.F.shape != (self.state_dim, self.state_dim):
            raise ValueError('F shape is wrong')
        if self.H.shape != (self.measurement_dim, self.state_dim):
            raise ValueError('H shape is wrong')
        if self.K.shape != (self.state_dim, self.measurement_dim):
            raise ValueError('K shape is wrong')
        if self.y.shape != (self.measurement_dim, 1):
            raise ValueError('y shape is wrong')
        if self.S.shape != (self.measurement_dim, self.measurement_dim):
            raise ValueError('S shape is wrong')

    def predict(self):
        self.check_shapes()
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(self.F, self.P).dot(self.F.T) + self.Q
        return self.x, self.P

    def update(self, z: np.ndarray, R: np.ndarray):
        self.check_shapes()
        if z.shape != (self.measurement_dim, 1):
            raise ValueError('z shape is wrong')
        if R.shape != (self.measurement_dim, self.measurement_dim):
            raise ValueError('R shape is wrong')

        PHT = np.dot(self.P, self.H.T)
        self.S = np.dot(self.H, PHT) + R
        self.K = np.dot(PHT, np.linalg.inv(self.S))
        self.y = z - np.dot(self.H, self.x)
        self.x = self.x + np.dot(self.K, self.y)
        self.P = self.P - np.dot(self.K, self.H).dot(self.P)  # Should be P = (I - KH)P, but might be unstable?
        return self.x, self.P
