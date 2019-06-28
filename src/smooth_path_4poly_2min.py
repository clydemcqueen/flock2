#!/usr/bin/env python

import numpy as np


class SmoothPath4Poly2Min:

    def __init__(self, ys, ts, v0, vn):
        self.ys = np.array(ys)
        self.ts = np.array(ts)

        self.tbeg = self.ts[0]
        self.tend = self.ts[self.ts.shape[0] - 1]

        self.ps = self.generate_smooth_trajectory(v0, vn)
        self.p_dots = self.ps @ np.array([[0, 0, 0],
                                          [1, 0, 0],
                                          [0, 2, 0],
                                          [0, 0, 3]])
        self.p_dotdots = self.ps @ np.array([[0, 0],
                                             [0, 0],
                                             [2, 0],
                                             [0, 6]])

        self.last_t_idx = 0

    def generate_smooth_trajectory(self, v0, vn):
        n = self.ys.shape[0]

        # create the taus
        taus = self.ts[1:] - self.ts[:-1]

        # create the Q matrix
        Q = np.zeros([4 * (n - 1), 4 * (n - 1)])
        for i in range(n - 1):
            tau = taus[i]
            ib = i * 4 + 2
            Q[ib:ib + 2, ib:ib + 2] = np.array([[4 * tau, 6 * tau ** 2], [6 * tau ** 2, 12 * tau ** 3]])

        # create the A and b matrices
        A = np.zeros([4 * (n - 1), 4 * (n - 1)])
        b = np.zeros(4 * (n - 1))

        # add the start and end velocity constraints
        A[0, 1] = 1
        b[0] = v0
        tau = taus[n - 2]
        A[1, 4 * (n - 2):4 * (n - 2) + 4] = np.array([0, 1, 2 * tau, 3 * tau ** 2])
        b[1] = vn

        rb = 2

        # add the position constraints
        for i in range(n - 1):
            tau = taus[i]
            A[rb:rb + 2, i * 4:i * 4 + 4] = np.array([[1, 0, 0, 0], [1, tau, tau ** 2, tau ** 3]])
            b[rb:rb + 2] = self.ys[i:i + 2]
            rb += 2

        # add the velocity continuity constraints
        for i in range(n - 2):
            tau = taus[i]
            A[rb:rb + 1, i * 4:i * 4 + 8] = np.array([0, 1, 2 * tau, 3 * tau ** 2, 0, -1, 0, 0])
            rb += 1

        # add the unknown velocities constraints
        for i in range(1, n - 1):
            A[rb:rb + 1, i * 4:i * 4 + 4] = np.array([0, 1, 0, 0])
            rb += 1

        # compute R
        Ainv = np.linalg.inv(A)
        R = Ainv.T @ Q @ Ainv

        # partition R
        ip = 4 * (n - 1) - (n - 2)
        Rvv = R[ip:, ip:]
        Rvc = R[ip:, 0:ip]
        bc = b[0:ip]

        # solve for bv
        bv = -np.linalg.inv(Rvv) @ Rvc @ bc

        # plug the optimum bv into b and compute the polynomial coefficients
        b[ip:] = bv
        ps = Ainv @ b

        return ps.reshape(-1, 4)

    def pick_pt(self, t):
        if self.ts[self.last_t_idx] <= t <= self.ts[self.last_t_idx + 1]:
            return self.last_t_idx, t - self.ts[self.last_t_idx]

        if self.ts[self.last_t_idx] > t:
            while self.last_t_idx > 0:
                self.last_t_idx -= 1
                if self.ts[self.last_t_idx] <= t:
                    return self.last_t_idx, t - self.ts[self.last_t_idx]
            return 0, -1.0

        while self.last_t_idx < self.ts.shape[0] - 2:
            self.last_t_idx += 1
            if t <= self.ts[self.last_t_idx + 1]:
                return self.last_t_idx, t - self.ts[self.last_t_idx]
        return 0, -1.0

    def calc_pt_y(self, pt):
        t = pt[1]
        if t < 0.:
            return 0.0
        return self.ps[pt[0], :] @ np.array([1, t, t ** 2, t ** 3])

    def calc_pt_y_dot(self, pt):
        t = pt[1]
        if t < 0.:
            return 0.0
        return self.p_dots[pt[0], :] @ np.array([1, t, t ** 2])

    def calc_pt_y_dotdot(self, pt):
        t = pt[1]
        if t < 0.:
            return 0.0
        return self.p_dotdots[pt[0], :] @ np.array([1, t])

    def calc_y(self, t):
        return self.calc_pt_y(self.pick_pt(t))

    def calc_y_dot(self, t):
        return self.calc_pt_y_dot(self.pick_pt(t))

    def calc_y_dotdot(self, t):
        return self.calc_pt_y_dotdot(self.pick_pt(t))


class Path3d:
    def __init__(self, ts, waypoints):
        wps = np.array(waypoints)
        self.x = SmoothPath4Poly2Min(wps[:, 0], ts, 0.0, 0.0)
        self.y = SmoothPath4Poly2Min(wps[:, 1], ts, 0.0, 0.0)
        self.z = SmoothPath4Poly2Min(wps[:, 2], ts, 0.0, 0.0)

    def calc_y_and_y_dot(self, t):
        pt = self.x.pick_pt(t)
        return \
            np.array([self.x.calc_pt_y(pt), self.y.calc_pt_y(pt), self.z.calc_pt_y(pt)]), \
            np.array([self.x.calc_pt_y_dot(pt), self.y.calc_pt_y_dot(pt), self.z.calc_pt_y_dot(pt)])
