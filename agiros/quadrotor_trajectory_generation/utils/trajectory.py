import numpy as np

from .q_funcs import q_dot_q, quaternion_inverse, v_dot_q, euler_to_quaternion, unit_quat


class Trajectory:
    def __init__(self):
        # Time since start
        self.t = None

        # Linear states and derivatives
        self.pos = None
        self.vel = None
        self.acc = None
        self.jerk = None
        self.snap = None

        # Angular states and derivatives
        self.att = None
        self.omega = None  # expressed in bodyframe
        self.acc_rot = None

        # Single rotor thrust inputs
        self.inputs = None

    def compute_full_traj(self, quad, yaw_mode, desired_yaw):
        print("Using yaw mode [%s]." % yaw_mode)
        len_traj = self.t.shape[0]
        dt = np.mean(np.diff(self.t))

        # Compute thrust direction
        gravity = 9.81
        thrust_np = self.acc + np.tile(np.array([[0, 0, 1]]), (len_traj, 1)) * gravity
        # Compute body axes
        z_b = thrust_np / np.sqrt(np.sum(thrust_np ** 2, 1))[:, np.newaxis]
        # computing the quaternion based on two vectors:
        # https://math.stackexchange.com/questions/2251214/calculate-quaternions-from-two-directional-vectors
        e_z = np.array([[0.0, 0.0, 1.0]])
        q_w = 1.0 + np.sum(e_z * z_b, axis=1)
        q_xyz = np.cross(e_z, z_b)
        att_np = 0.5 * np.concatenate([np.expand_dims(q_w, axis=1), q_xyz], axis=1)
        att_np = att_np / np.sqrt(np.sum(att_np ** 2, 1))[:, np.newaxis]

        if yaw_mode == "constant":
            yaw_d = desired_yaw * np.ones([self.pos.shape[0]])
            q_yaw = euler_to_quaternion(0.0, 0.0, yaw_d)
            q_yaw = np.transpose(q_yaw)
            att_new = np.full_like(att_np, 0)
            for i in range(0, len_traj):
                att_new[i, :] = q_dot_q(att_np[i, :], q_yaw[i, :])
                att_new[i, :] = unit_quat(att_new[i, :])
            att_np = att_new
        if yaw_mode == "velocity":
            vel_body = np.full_like(self.vel, 0)
            for i in range(0, len_traj):
                att_np_inverse = quaternion_inverse(att_np[i, :])
                vel_body[i, :] = v_dot_q(self.vel[i, :], att_np_inverse)
            yaw_d = np.arctan2(vel_body[:, 1], vel_body[:, 0])
            q_yaw = euler_to_quaternion(0.0, 0.0, yaw_d)
            q_yaw = np.transpose(q_yaw)
            att_new = np.full_like(att_np, 0)
            for i in range(0, len_traj):
                att_new[i, :] = q_dot_q(att_np[i, :], q_yaw[i, :])
                att_new[i, :] = unit_quat(att_new[i, :])
            att_np = att_new
        if yaw_mode == "acceleration":
            acc_body = np.full_like(self.acc, 0)
            for i in range(0, len_traj):
                att_np_inverse = quaternion_inverse(att_np[i, :])
                acc_body[i, :] = v_dot_q(self.acc[i, :], att_np_inverse)
            yaw_d = np.arctan2(acc_body[:, 1], acc_body[:, 0])
            q_yaw = euler_to_quaternion(0.0, 0.0, yaw_d)
            q_yaw = np.transpose(q_yaw)
            att_new = np.full_like(att_np, 0)
            for i in range(0, len_traj):
                att_new[i, :] = q_dot_q(att_np[i, :], q_yaw[i, :])
                att_new[i, :] = unit_quat(att_new[i, :])
            att_np = att_new

        # remove quaternion flips
        for i in range(1, len_traj):
            if np.linalg.norm(att_np[i - 1, :] - att_np[i, :]) > np.linalg.norm(
                    att_np[i - 1, :] + att_np[i, :]):
                att_np[i, :] = -1.0 * att_np[i, :]

        self.omega = np.zeros_like(self.pos)
        f_t = np.zeros((len_traj, 1))

        # Use numerical differentiation of quaternions
        q_dot = np.gradient(att_np, axis=0) / dt
        w_int = np.zeros((len_traj, 3))
        for i in range(len_traj):
            w_int[i, :] = 2.0 * q_dot_q(quaternion_inverse(att_np[i, :]), q_dot[i])[1:]
            f_t[i, 0] = quad.mass * z_b[i].dot(thrust_np[i, :].T)
        self.omega[:, 0] = w_int[:, 0]
        self.omega[:, 1] = w_int[:, 1]
        self.omega[:, 2] = w_int[:, 2]

        n_iter_yaw_refinement = 20
        if yaw_mode == "min_yaw_rate":
            for iter_yaw_refinement in range(n_iter_yaw_refinement):
                print("Maximum yawrate before refinement %d / %d: %.6f" % (
                    iter_yaw_refinement, n_iter_yaw_refinement, np.max(np.abs(self.omega[:, 2]))))
                q_new = att_np
                yaw_corr_acc = 0.0
                for i in range(1, len_traj):
                    yaw_corr = -self.omega[i, 2] * dt
                    yaw_corr_acc += yaw_corr
                    q_corr = np.array([np.cos(yaw_corr_acc / 2.0), 0.0, 0.0, np.sin(yaw_corr_acc / 2.0)])
                    q_new[i, :] = q_dot_q(att_np[i, :], q_corr)
                    w_int[i, :] = 2.0 * q_dot_q(quaternion_inverse(att_np[i, :]), q_dot[i])[1:]

                q_new_dot = np.gradient(q_new, axis=0) / dt
                for i in range(1, len_traj):
                    w_int[i, :] = 2.0 * q_dot_q(quaternion_inverse(q_new[i, :]), q_new_dot[i])[1:]

                att_np = q_new
                self.omega[:, 0] = w_int[:, 0]
                self.omega[:, 1] = w_int[:, 1]
                self.omega[:, 2] = w_int[:, 2]
                print("Maximum yawrate after refinement: %.3f" % np.max(np.abs(self.omega[:, 2])))
                if np.max(np.abs(self.omega[:, 2])) < 0.005:
                    break

        self.att = att_np
        self.acc_rot = np.gradient(self.omega, axis=0) / dt

        self.inputs = np.zeros((self.pos.shape[0], 4))

        # Compute inputs
        rate_x_Jrate = np.array([(quad.J[2] - quad.J[1]) * self.omega[:, 2] * self.omega[:, 1],
                                 (quad.J[0] - quad.J[2]) * self.omega[:, 0] * self.omega[:, 2],
                                 (quad.J[1] - quad.J[0]) * self.omega[:, 1] * self.omega[:, 0]]).T

        tau = self.acc_rot * quad.J[np.newaxis, :] + rate_x_Jrate
        b = np.concatenate((tau, f_t), axis=-1)
        a_mat = np.concatenate((quad.y_f[np.newaxis, :],
                                -quad.x_f[np.newaxis, :],
                                quad.z_l_tau[np.newaxis, :],
                                np.ones_like(quad.z_l_tau)[np.newaxis, :]), 0)

        for i in range(len_traj):
            self.inputs[i, :] = np.linalg.solve(a_mat, b[i, :])

    def fix_start_end(self):
        '''
        Numeric differentiation introduces artifacts at the boundaries.
        Replace the first and last few samples to fix this.
        :return:
        '''

        replace_n_samples = 5

        # Start
        self.pos[:replace_n_samples, :] = self.pos[replace_n_samples, :]
        self.vel[:replace_n_samples, :] = self.vel[replace_n_samples, :]
        self.acc[:replace_n_samples, :] = self.acc[replace_n_samples, :]
        self.jerk[:replace_n_samples, :] = self.jerk[replace_n_samples, :]
        self.snap[:replace_n_samples, :] = self.snap[replace_n_samples, :]
        self.att[:replace_n_samples, :] = self.att[replace_n_samples, :]
        self.omega[:replace_n_samples, :] = self.omega[replace_n_samples, :]
        self.acc_rot[:replace_n_samples, :] = self.acc_rot[replace_n_samples, :]

        self.inputs[:replace_n_samples, :] = self.inputs[replace_n_samples, :]

        # End
        self.pos[-replace_n_samples:, :] = self.pos[-replace_n_samples, :]
        self.vel[-replace_n_samples:, :] = self.vel[-replace_n_samples, :]
        self.acc[-replace_n_samples:, :] = self.acc[-replace_n_samples, :]
        self.jerk[-replace_n_samples:, :] = self.jerk[-replace_n_samples, :]
        self.snap[-replace_n_samples:, :] = self.snap[-replace_n_samples, :]
        self.att[-replace_n_samples:, :] = self.att[-replace_n_samples, :]
        self.omega[-replace_n_samples:, :] = self.omega[-replace_n_samples, :]
        self.acc_rot[-replace_n_samples:, :] = self.acc_rot[-replace_n_samples, :]

        self.inputs[-replace_n_samples:, :] = self.inputs[-replace_n_samples, :]

    def as_array(self):
        return np.concatenate([self.pos, self.att,
                               self.vel, self.omega,
                               self.acc, self.acc_rot,
                               self.inputs,
                               self.jerk, self.snap], axis=1)

    def check_integrity(self):
        print("Checking trajectory integrity...")

        dt = np.expand_dims(np.gradient(self.t, axis=0), axis=1)

        errors = np.zeros((dt.shape[0], 3))

        gravity = 9.81

        numeric_velocity = np.gradient(self.pos, axis=0) / dt
        numeric_thrust = np.gradient(self.vel, axis=0) / dt + np.array([0.0, 0.0, gravity])
        numeric_q_diff = np.gradient(self.att, axis=0) / dt

        for i in range(dt.shape[0]):
            # 1) check if velocity is consistent with position
            errors[i, 0] = np.linalg.norm(numeric_velocity[i, :] - self.vel[i, :])
            if not np.allclose(self.vel[i, :], numeric_velocity[i, :], atol=0.05, rtol=0.05):
                print("inconsistent linear velocity at i = %d" % i)
                print(numeric_velocity[i, :])
                print(self.vel[i, :])
                return False

            # 2) check if attitude is consistent with acceleration
            if np.abs(np.linalg.norm(self.att[i, :]) - 1.0) > 1e-6:
                print("quaternion does not have unit norm at i = %d" % i)
                print(self.att[i, :])
                print(np.linalg.norm(self.att[i, :]))
                return False

            body_z_axis = numeric_thrust[i, :] / np.linalg.norm(numeric_thrust[i, :])
            e_z = np.array([0.0, 0.0, 1.0])
            q_w = 1.0 + np.dot(e_z, body_z_axis)
            q_xyz = np.cross(e_z, body_z_axis)
            numeric_attitude = 0.5 * np.array([q_w] + q_xyz.tolist())
            numeric_attitude = numeric_attitude / np.linalg.norm(numeric_attitude)
            # the two attitudes can only differ in yaw --> check x,y component
            q_diff = q_dot_q(quaternion_inverse(self.att[i, :]), numeric_attitude)
            errors[i, 1] = np.linalg.norm(q_diff[1:3])
            if not np.allclose(q_diff[1:3], np.zeros(2, ), atol=0.05, rtol=0.05):
                print("Attitude and acceleration do not match at i = %d" % i)
                print(self.att[i, :])
                print(numeric_attitude)
                print(q_diff)
                return False

            # 3) check if bodyrates agree with attitude difference
            numeric_bodyrates = 2.0 * q_dot_q(quaternion_inverse(self.att[i, :]),
                                              numeric_q_diff[i, :])[1:]
            errors[i, 2] = np.linalg.norm(numeric_bodyrates - self.omega[i, :])
            if not np.allclose(numeric_bodyrates, self.omega[i, :], atol=0.05, rtol=0.05):
                print("inconsistent angular velocity at i = %d" % i)
                print(numeric_bodyrates)
                print(self.omega[i, :])
                return False

        print("Trajectory check successful")
        print("Maximum linear velocity error: %.5f" % np.max(errors[:, 0]))
        print("Maximum attitude error: %.5f" % np.max(errors[:, 1]))
        print("Maximum angular velocity error: %.5f" % np.max(errors[:, 2]))

        return True
