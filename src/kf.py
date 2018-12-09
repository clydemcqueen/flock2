import numpy as np


class KalmanFilter(object):

    def __init__(self, state_dim, measurement_dim):
        if state_dim < 1:
            raise ValueError('state_dim must be 1 or greater')
        if measurement_dim < 1:
            raise ValueError('measurement_dim must be 1 or greater')

        self._state_dim = state_dim                             # Number of state variables
        self._measurement_dim = measurement_dim                 # Number of measurement variables
        self._x = np.zeros((state_dim, 1))                      # State mean
        self._P = np.eye(state_dim)                             # State covariance
        self._Q = np.eye(state_dim)                             # Process covariance
        self._F = np.eye(state_dim)                             # State transition matrix
        self._H = np.zeros((measurement_dim, state_dim))        # Measurement function
        self._K = np.zeros((state_dim, measurement_dim))        # Kalman gain
        self._y = np.zeros((measurement_dim, 1))                # Residual
        self._S = np.zeros((measurement_dim, measurement_dim))  # System uncertainty

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, x):
        if x.shape != (self._state_dim, 1):
            raise ValueError('x shape is wrong')
        self._x = x

    @property
    def P(self):
        return self._P

    @P.setter
    def P(self, P):
        if P.shape != (self._state_dim, self._state_dim):
            raise ValueError('P shape is wrong')
        self._P = P

    @property
    def Q(self):
        return self._Q

    @Q.setter
    def Q(self, Q):
        if Q.shape != (self._state_dim, self._state_dim):
            raise ValueError('Q shape is wrong')
        self._Q = Q

    @property
    def F(self):
        return self._F

    @F.setter
    def F(self, F):
        if F.shape != (self._state_dim, self._state_dim):
            raise ValueError('F shape is wrong')
        self._F = F

    @property
    def H(self):
        return self._H

    @ H.setter
    def H(self, H):
        if H.shape != (self._measurement_dim, self._state_dim):
            raise ValueError('H shape is wrong')
        self._H = H

    @property
    def K(self):
        return self._K

    @ K.setter
    def K(self, K):
        if K.shape != (self._state_dim, self._measurement_dim):
            raise ValueError('K shape is wrong')
        self._K = K

    @property
    def y(self):
        return self._y

    @ y.setter
    def y(self, y):
        if y.shape != (self._measurement_dim, 1):
            raise ValueError('y shape is wrong')
        self._y = y

    @property
    def S(self):
        return self._S

    @ S.setter
    def S(self, S):
        if S.shape != (self._measurement_dim, self._measurement_dim):
            raise ValueError('S shape is wrong')
        self._S = S

    def predict(self):
        self._x = np.dot(self._F, self._x)
        self._P = np.dot(self._F, self._P).dot(self._F.T) + self._Q
        return self._x, self._P

    def update(self, z: np.ndarray, R: np.ndarray):
        if z.shape != (self._measurement_dim, 1):
            raise ValueError('z shape is wrong')
        if R.shape != (self._measurement_dim, self._measurement_dim):
            raise ValueError('R shape is wrong')

        PHT = np.dot(self._P, self._H.T)
        self._S = np.dot(self._H, PHT) + R
        self._K = np.dot(PHT, np.linalg.inv(self._S))
        self._y = z - np.dot(self._H, self._x)
        self._x = self._x + np.dot(self._K, self._y)
        self._P = self._P - np.dot(self._K, self._H).dot(self._P)  # Should be P = (I - KH)P, but might be unstable?
        return self._x, self._P
