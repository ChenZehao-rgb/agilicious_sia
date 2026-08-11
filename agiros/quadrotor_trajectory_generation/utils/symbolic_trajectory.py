import casadi as cs
import numpy as np

from .time_mapping import time_mapping_poly_coeffs


class SymbolicTrajectory:
    def __init__(self, duration, t_speedup):
        # define position trajectory symbolically
        self.t = cs.MX.sym("t")
        self.polynomial_time_mapping(duration, t_speedup)

        #################################################################################
        # describe your trajectory here as a function of t_adj
        #################################################################################
        # sphere trajectory rotating around x-axis
        # radius_x = 5.0
        # radius_y = 3.5
        # radius_z = 2.5
        # # fast config
        # freq_slow = 0.009
        # freq_fast = 0.33
        # # slow config
        # # freq_slow = 0.02
        # # freq_fast = 0.12
        # self.pos_x = 3.0 + radius_x * (
        #             cs.sin(2.0 * cs.pi * freq_fast * self.t_adj) * cs.cos(2.0 * cs.pi * freq_slow * self.t_adj))
        # self.pos_y = 1.0 + radius_y * (cs.cos(2.0 * cs.pi * freq_fast * self.t_adj))
        # self.pos_z = 3.5 + radius_z * (
        #             cs.sin(2.0 * cs.pi * freq_fast * self.t_adj) * cs.sin(2.0 * cs.pi * freq_slow * self.t_adj))

        # circle
        radius = 1.5
        freq = 0.45
        self.pos_x = 1.5 + radius * cs.sin(2.0 * cs.pi * freq * self.t_adj)
        self.pos_y = 0.0 + radius * cs.cos(2.0 * cs.pi * freq * self.t_adj)
        self.pos_z = 1.0

    def polynomial_time_mapping(self, duration, t_speedup):
        #################################################################################
        # time mapping, ensures that trajectories start and end in hover
        #################################################################################
        # We use piecewise polynomial functions to represent time during speedup,
        # coast and slowdown.
        x = cs.MX(cs.DM([0.0, t_speedup, duration - t_speedup, duration]))
        continuity_order = 4
        start_coeffs = time_mapping_poly_coeffs(continuity_order)

        coeffs = np.zeros((2 * continuity_order + 1, 3))
        # speedup phase
        coeffs[:, 0] = np.flip(start_coeffs, 0)
        # coasting phase
        coeffs[1, 1] = 1.0
        # slowdown phase
        coeffs[:, 2] = -np.flip(start_coeffs, 0)

        biases_numpy = np.array([0.0, t_speedup / 2.0, duration - t_speedup])
        t_biases_numpy = np.array([0.0, t_speedup, duration])
        t_dirs_numpy = np.array([1.0, 1.0, -1.0])

        poly_coeffs = cs.MX(cs.DM(coeffs))
        n = poly_coeffs.shape[0]
        biases = cs.MX(cs.DM(biases_numpy))
        t_biases = cs.MX(cs.DM(t_biases_numpy))
        t_dirs = cs.MX(cs.DM(t_dirs_numpy))

        L = cs.low(x, self.t)

        coeff = poly_coeffs[:, L]
        bias = biases[L]
        t_bias = t_biases[L]
        t_dir = t_dirs[L]

        self.t_adj = cs.dot(coeff, (t_dir * (self.t - t_bias) / t_speedup) ** cs.DM(range(n))) * t_speedup + bias
