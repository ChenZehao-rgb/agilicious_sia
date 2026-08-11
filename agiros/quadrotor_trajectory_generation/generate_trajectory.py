import argparse
import time

import casadi as cs
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.interpolate import UnivariateSpline
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import ExpSineSquared

from config.settings import Settings
from utils.nan_helper import nan_helper
from utils.quadrotor import Quad
from utils.symbolic_trajectory import SymbolicTrajectory
from utils.time_mapping import time_mapping
from utils.trajectory import Trajectory
from utils.visualization import draw_trajectory


class TrajectoryGenerator:
    def __init__(self, config):
        self.config = config
        self.quad = Quad(self.config)

    def generate(self):
        start_time = time.time()

        timestr = time.strftime("%Y%m%d-%H%M%S")
        output_fn = "generated_trajectories/" + self.config.traj_type + "_" + timestr + ".csv"

        print("Computing [%s] trajectory!" % self.config.traj_type)
        if self.config.traj_type == 'symbolic':
            trajectory = self.compute_symbolic_trajectory()
        elif self.config.traj_type == 'random':
            trajectory = self.compute_random_trajectory()
        else:
            print("Unknown trajectory type.")
            exit()

        if trajectory.check_integrity():
            if self.config.generate_plots:
                draw_trajectory(trajectory)
            self.export(trajectory, output_fn)
        else:
            print("Trajectory check failed.")
            exit()
        print("Trajectory generation took [%.3f] seconds." % (time.time() - start_time))

    def export(self, trajectory, output_fn):
        '''
        Save trajectory to csv
        '''
        take_every_nth = int(self.config.traj_dt / self.config.dt_gen)

        # save trajectory to csv
        df_traj = pd.DataFrame()
        df_traj['t'] = trajectory.t[::take_every_nth]
        df_traj['p_x'] = trajectory.pos[::take_every_nth, 0]
        df_traj['p_y'] = trajectory.pos[::take_every_nth, 1]
        df_traj['p_z'] = trajectory.pos[::take_every_nth, 2]

        df_traj['q_w'] = trajectory.att[::take_every_nth, 0]
        df_traj['q_x'] = trajectory.att[::take_every_nth, 1]
        df_traj['q_y'] = trajectory.att[::take_every_nth, 2]
        df_traj['q_z'] = trajectory.att[::take_every_nth, 3]

        df_traj['v_x'] = trajectory.vel[::take_every_nth, 0]
        df_traj['v_y'] = trajectory.vel[::take_every_nth, 1]
        df_traj['v_z'] = trajectory.vel[::take_every_nth, 2]

        df_traj['w_x'] = trajectory.omega[::take_every_nth, 0]
        df_traj['w_y'] = trajectory.omega[::take_every_nth, 1]
        df_traj['w_z'] = trajectory.omega[::take_every_nth, 2]

        df_traj['a_lin_x'] = trajectory.acc[::take_every_nth, 0]
        df_traj['a_lin_y'] = trajectory.acc[::take_every_nth, 1]
        df_traj['a_lin_z'] = trajectory.acc[::take_every_nth, 2]

        df_traj['a_rot_x'] = trajectory.acc_rot[::take_every_nth, 0]
        df_traj['a_rot_y'] = trajectory.acc_rot[::take_every_nth, 1]
        df_traj['a_rot_z'] = trajectory.acc_rot[::take_every_nth, 2]

        df_traj['u_1'] = trajectory.inputs[::take_every_nth, 0]
        df_traj['u_2'] = trajectory.inputs[::take_every_nth, 1]
        df_traj['u_3'] = trajectory.inputs[::take_every_nth, 2]
        df_traj['u_4'] = trajectory.inputs[::take_every_nth, 3]

        df_traj['jerk_x'] = trajectory.jerk[::take_every_nth, 0]
        df_traj['jerk_y'] = trajectory.jerk[::take_every_nth, 1]
        df_traj['jerk_z'] = trajectory.jerk[::take_every_nth, 2]

        df_traj['snap_x'] = trajectory.snap[::take_every_nth, 0]
        df_traj['snap_y'] = trajectory.snap[::take_every_nth, 1]
        df_traj['snap_z'] = trajectory.snap[::take_every_nth, 2]

        print("Saving trajectory to [%s]." % output_fn)
        df_traj.to_csv(output_fn, index=False)

    def compute_symbolic_trajectory(self):
        dt = self.config.dt_gen

        # define position trajectory symbolically
        symbolic_traj = SymbolicTrajectory(self.config.traj_duration, self.config.t_speedup)

        pos = cs.vertcat(symbolic_traj.pos_x, symbolic_traj.pos_y, symbolic_traj.pos_z)
        vel = cs.jacobian(pos, symbolic_traj.t)
        acc = cs.jacobian(vel, symbolic_traj.t)
        jerk = cs.jacobian(acc, symbolic_traj.t)
        snap = cs.jacobian(jerk, symbolic_traj.t)

        t_vec, dt = np.linspace(0.0, self.config.traj_duration, int(self.config.traj_duration / dt), endpoint=False,
                                retstep=True)

        f_t_adj = cs.Function('t_adj', [symbolic_traj.t], [symbolic_traj.t_adj])
        f_pos = cs.Function('f_pos', [symbolic_traj.t], [pos])
        f_vel = cs.Function('f_vel', [symbolic_traj.t], [vel])
        f_acc = cs.Function('f_acc', [symbolic_traj.t], [acc])
        f_jerk = cs.Function('f_jerk', [symbolic_traj.t], [jerk])
        f_snap = cs.Function('f_snap', [symbolic_traj.t], [snap])

        pos_list = []
        vel_list = []
        alin_list = []
        jerk_list = []
        snap_list = []
        t_adj_list = []
        for t_curr in t_vec:
            t_adj_list.append(f_t_adj(t_curr).full().squeeze())
            pos_list.append(f_pos(t_curr).full().squeeze())
            vel_list.append(f_vel(t_curr).full().squeeze())
            alin_list.append(f_acc(t_curr).full().squeeze())
            jerk_list.append(f_jerk(t_curr).full().squeeze())
            snap_list.append(f_snap(t_curr).full().squeeze())

        trajectory = Trajectory()
        trajectory.t = np.array(t_vec)
        trajectory.pos = np.array(pos_list)
        trajectory.vel = np.array(vel_list)
        trajectory.acc = np.array(alin_list)
        trajectory.jerk = np.array(jerk_list)
        trajectory.snap = np.array(snap_list)

        # casadi derivative results in NaN values at the boundaries
        nans, vx = nan_helper(trajectory.vel[:, 0])
        trajectory.vel[nans, 0] = np.interp(vx(nans), vx(~nans), trajectory.vel[~nans, 0])
        nans, vy = nan_helper(trajectory.vel[:, 1])
        trajectory.vel[nans, 1] = np.interp(vy(nans), vy(~nans), trajectory.vel[~nans, 1])
        nans, vz = nan_helper(trajectory.vel[:, 2])
        trajectory.vel[nans, 2] = np.interp(vz(nans), vz(~nans), trajectory.vel[~nans, 2])

        nans, ax = nan_helper(trajectory.acc[:, 0])
        trajectory.acc[nans, 0] = np.interp(ax(nans), ax(~nans), trajectory.acc[~nans, 0])
        nans, ay = nan_helper(trajectory.acc[:, 1])
        trajectory.acc[nans, 1] = np.interp(ay(nans), ay(~nans), trajectory.acc[~nans, 1])
        nans, az = nan_helper(trajectory.acc[:, 2])
        trajectory.acc[nans, 2] = np.interp(az(nans), az(~nans), trajectory.acc[~nans, 2])

        nans, jx = nan_helper(trajectory.jerk[:, 0])
        trajectory.jerk[nans, 0] = np.interp(jx(nans), jx(~nans), trajectory.jerk[~nans, 0])
        nans, jy = nan_helper(trajectory.jerk[:, 1])
        trajectory.jerk[nans, 1] = np.interp(jy(nans), jy(~nans), trajectory.jerk[~nans, 1])
        nans, jz = nan_helper(trajectory.jerk[:, 2])
        trajectory.jerk[nans, 2] = np.interp(jz(nans), jz(~nans), trajectory.jerk[~nans, 2])

        nans, sx = nan_helper(trajectory.snap[:, 0])
        trajectory.snap[nans, 0] = np.interp(sx(nans), sx(~nans), trajectory.snap[~nans, 0])
        nans, sy = nan_helper(trajectory.snap[:, 1])
        trajectory.snap[nans, 1] = np.interp(sy(nans), sy(~nans), trajectory.snap[~nans, 1])
        nans, sz = nan_helper(trajectory.snap[:, 2])
        trajectory.snap[nans, 2] = np.interp(sz(nans), sz(~nans), trajectory.snap[~nans, 2])

        if self.config.debug:
            t_adj_np = np.array(t_adj_list)
            plt.plot(trajectory.t)
            plt.plot(t_adj_np)
            plt.plot(trajectory.pos[:, 0])
            plt.plot(trajectory.vel[:, 0])
            plt.plot(trajectory.acc[:, 0])
            plt.plot(trajectory.pos[:, 1])
            plt.plot(trajectory.vel[:, 1])
            plt.plot(trajectory.acc[:, 1])
            plt.plot(trajectory.pos[:, 2])
            plt.plot(trajectory.vel[:, 2])
            plt.plot(trajectory.acc[:, 2])
            plt.show()

        trajectory.compute_full_traj(self.quad, self.config.yaw_mode, self.config.desired_yaw)

        return trajectory

    def compute_random_trajectory(self):
        if self.config.seed == -1:
            self.config.seed = np.random.randint(0, 9999999)

        # kernel to generate functions that repeat exactly
        print("seed is: %d" % self.config.seed)
        np.random.seed(self.config.seed)

        kernel_x = ExpSineSquared(length_scale=np.random.uniform(self.config.rand_min_length_scale,
                                                                 self.config.rand_max_length_scale),
                                  periodicity=np.random.uniform(self.config.rand_min_period,
                                                                self.config.rand_max_period))
        kernel_y = ExpSineSquared(length_scale=np.random.uniform(self.config.rand_min_length_scale,
                                                                 self.config.rand_max_length_scale),
                                  periodicity=np.random.uniform(self.config.rand_min_period,
                                                                self.config.rand_max_period))
        kernel_z = ExpSineSquared(length_scale=np.random.uniform(self.config.rand_min_length_scale,
                                                                 self.config.rand_max_length_scale),
                                  periodicity=np.random.uniform(self.config.rand_min_period,
                                                                self.config.rand_max_period))

        for i_kernel in range(self.config.rand_num_kernels):
            kernel_x += ExpSineSquared(length_scale=np.random.uniform(self.config.rand_min_length_scale,
                                                                      self.config.rand_max_length_scale),
                                       periodicity=np.random.uniform(self.config.rand_min_period,
                                                                     self.config.rand_max_period))
            kernel_y += ExpSineSquared(length_scale=np.random.uniform(self.config.rand_min_length_scale,
                                                                      self.config.rand_max_length_scale),
                                       periodicity=np.random.uniform(self.config.rand_min_period,
                                                                     self.config.rand_max_period))
            kernel_z += ExpSineSquared(length_scale=np.random.uniform(self.config.rand_min_length_scale,
                                                                      self.config.rand_max_length_scale),
                                       periodicity=np.random.uniform(self.config.rand_min_period,
                                                                     self.config.rand_max_period))

        gp_x = GaussianProcessRegressor(kernel=kernel_x)
        gp_y = GaussianProcessRegressor(kernel=kernel_y)
        gp_z = GaussianProcessRegressor(kernel=kernel_z)
        t_coarse = np.linspace(0.0, self.config.traj_duration, int(self.config.traj_duration / 0.1), endpoint=False)
        t_vec, dt = np.linspace(0.0, self.config.traj_duration, int(self.config.traj_duration / self.config.dt_gen),
                                endpoint=False, retstep=True)

        # manipulate time vector to get smooth start and end
        startup_time = time_mapping(self.config.t_speedup, dt)

        t_vec_cropped = t_vec[(self.config.t_speedup / 2.0 <= t_vec) & (
                t_vec < self.config.traj_duration - 3.0 * self.config.t_speedup / 2.0)]
        scaled_time = np.concatenate([startup_time, t_vec_cropped,
                                      t_vec_cropped[-1] + self.config.t_speedup / 2.0 - np.flip(startup_time)])[:,
                      np.newaxis]

        print("sampling x...")
        x_sampled = gp_x.sample_y(t_coarse[:, np.newaxis], 1, random_state=self.config.seed)
        print("sampling y...")
        y_sampled = gp_y.sample_y(t_coarse[:, np.newaxis], 1, random_state=self.config.seed + 1)
        print("sampling z...")
        z_sampled = gp_z.sample_y(t_coarse[:, np.newaxis], 1, random_state=self.config.seed + 2)

        pos_sampled = np.concatenate([x_sampled, y_sampled, z_sampled], axis=1)
        # scale to arena bounds
        max_traj = np.max(pos_sampled, axis=0)
        min_traj = np.min(pos_sampled, axis=0)
        pos_centered = pos_sampled - (max_traj + min_traj) / 2.0
        pos_scaled = pos_centered * (self.config.bound_max - self.config.bound_min) / (max_traj - min_traj)
        pos_coarse = pos_scaled + (self.config.bound_max + self.config.bound_min) / 2.0

        if self.config.debug:
            plt.plot(pos_coarse[:, 0], label="x")
            plt.plot(pos_coarse[:, 1], label="y")
            plt.plot(pos_coarse[:, 2], label="z")
            plt.legend()
            plt.show()

        spl_x = UnivariateSpline(t_coarse, pos_coarse[:, 0], k=5)
        spl_y = UnivariateSpline(t_coarse, pos_coarse[:, 1], k=5)
        spl_z = UnivariateSpline(t_coarse, pos_coarse[:, 2], k=5)

        trajectory = Trajectory()
        trajectory.t = t_vec
        trajectory.pos = np.concatenate([spl_x(scaled_time),
                                         spl_y(scaled_time),
                                         spl_z(scaled_time)], axis=1)

        # compute derivatives via numeric differentiation
        trajectory.vel = np.gradient(trajectory.pos, axis=0) / dt
        trajectory.acc = np.gradient(trajectory.vel, axis=0) / dt
        trajectory.jerk = np.gradient(trajectory.acc, axis=0) / dt
        trajectory.snap = np.gradient(trajectory.jerk, axis=0) / dt

        trajectory.compute_full_traj(self.quad, self.config.yaw_mode, self.config.desired_yaw)

        return trajectory


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate a quadrotor trajectory.')
    parser.add_argument('--settings_file', help='Path to configuration file.', required=True)

    args = parser.parse_args()
    settings_filepath = args.settings_file

    settings = Settings(settings_filepath)

    traj_generator = TrajectoryGenerator(settings)

    for i in range(settings.num_traj):
        traj_generator.generate()
