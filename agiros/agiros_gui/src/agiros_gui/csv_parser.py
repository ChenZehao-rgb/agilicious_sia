#!/usr/bin/env python

import csv


# Parser class
class CsvParser:
    # Protocol specific parameters:
    # N_ELEMENTS_PER_ROW = 39  # t, pos_xyz, att_wxyz, vel_xyz, omega_xyz, linacc_xyz, angacc_xyz, rotor_thrusts
    LABELS_PRESENT = True  # If true, there is a row with labels
    LINE_OFFSETS = [0,
                    0]  # Number of lines that don't matter to us. First number is before label and second number after label, if there is a label

    t = []
    p = []
    q = []
    v = []
    w = []
    a_lin = []
    a_rot = []
    u = []
    mu = []
    nu = []
    tau = []
    s = []
    j = []

    def __init__(self, filename):
        self.load_csv(filename)

    def load_csv(self, filename):
        self.clear_previous()
        assert type(filename) == str
        with open(filename, 'r') as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=',')
            # Skip first lines before label
            for i in range(0, self.LINE_OFFSETS[0]):
                next(csv_reader)

            # Save label and skip line offsets after label
            if self.LABELS_PRESENT:
                labels = next(csv_reader)
                labels = [s.strip() for s in labels]
                self.N_ELEMENTS_PER_ROW = len(
                    labels)  # t, pos_xyz, att_wxyz, vel_xyz, omega_xyz, linacc_xyz, angacc_xyz, rotor_thrusts

                assert len(labels) == self.N_ELEMENTS_PER_ROW
                for i in range(0, self.LINE_OFFSETS[1]):
                    next(csv_reader)


            # Start parsing real data
            line_count = 0
            for row in csv_reader:
                if (len(row)) != self.N_ELEMENTS_PER_ROW:
                    message = "Error parsing .csv file in line {}. Format not correct.".format(line_count + 1)
                    raise Exception(message)

                row_to_float = [float(el) for el in row]
                self.append_row_to_fields(row_to_float, labels)
                line_count = line_count + 1

    def append_row_to_fields(self, row, labels):
        assert len(row) == self.N_ELEMENTS_PER_ROW

        l_pos   = []
        l_quat  = []
        l_vel   = []
        l_rate  = []
        l_acc   = []
        l_alpha = []
        l_u     = []
        l_tau   = []
        l_nu    = []
        l_mu    = []
        l_jerk  = []
        l_snap  = []
        
        for label in labels:
            if label.startswith("p_"):
                l_pos.append(labels.index(label))
            if label.startswith("q_"):
                l_quat.append(labels.index(label))
            if label.startswith("v_"):
                l_vel.append(labels.index(label))
            if label.startswith("w_"):
                l_rate.append(labels.index(label))
            if label.startswith("a_lin_"):
                l_acc.append(labels.index(label))
            if label.startswith("a_rot_"):
                l_alpha.append(labels.index(label))
            if label.startswith("u_"):
                l_u.append(labels.index(label))
            if label.startswith("tau_"):
                l_tau.append(labels.index(label))
            if label.startswith("nu_"):
                l_nu.append(labels.index(label))
            if label.startswith("mu_"):
                l_mu.append(labels.index(label))
            if label.startswith("jerk_"):
                l_jerk.append(labels.index(label))
            if label.startswith("snap_"):
                l_snap.append(labels.index(label))
                
        self.t.append(row[0])
        self.p.append([row[i] for i in l_pos])
        self.q.append([row[i] for i in l_quat])
        self.v.append([row[i] for i in l_vel])
        self.w.append([row[i] for i in l_rate])
        self.a_lin.append([row[i] for i in l_acc])
        self.a_rot.append([row[i] for i in l_alpha])
        self.u.append([row[i] for i in l_u])     
        self.mu.append([row[i] for i in l_mu])
        self.nu.append([row[i] for i in l_nu])
        self.tau.append([row[i] for i in l_tau])
        self.j.append([row[i] for i in l_jerk])
        self.s.append([row[i] for i in l_snap])

    def clear_previous(self):
        self.t[:] = []
        self.p[:] = []
        self.q[:] = []
        self.v[:] = []
        self.w[:] = []
        self.a_lin[:] = []
        self.a_rot[:] = []
        self.u[:] = []
        self.mu[:] = []
        self.nu[:] = []
        self.tau[:] = []
        self.j[:] = []
        self.s[:] = []
