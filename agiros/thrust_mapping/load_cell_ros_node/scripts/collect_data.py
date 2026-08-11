#!/usr/bin/python3

import datetime
import os
import sys
import time

import agiros_msgs.msg as agiros_msgs
import geometry_msgs.msg as geometry_msgs
import numpy as np
import rospy
import std_msgs.msg as std_msgs


class AcquireThrustMapping:
    def __init__(self, data_dir, is_single_rotor_thrust, cycles, min_cmd=-0.95, max_cmd=1.0, zero_cmd=-1.0,
                 num_cmd_steps=10,
                 step_duration=4.0):

        rospy.init_node('acquire_thrust_map')
        self.num_cmd_steps = num_cmd_steps
        self._zero_command = zero_cmd
        self.data_dir = data_dir
        self.is_single_rotor_thrust = is_single_rotor_thrust
        self.cycles = cycles
        self.min_cmd = min_cmd
        self.max_cmd = max_cmd
        self._cmds = None
        self._duration = step_duration

        self._raw_data = []
        self._raw_cmds = []
        self._raw_voltage = []

        self.load_cell_data = None
        self.command_data = None
        self.voltage_data = None

        self._rate = rospy.Rate(50)

        # Command publishers
        self._bias_pub = rospy.Publisher('/load_cell/set_bias', std_msgs.Empty, queue_size=1)
        self._start_pub = rospy.Publisher('/load_cell/start_streaming', std_msgs.Empty, queue_size=1)
        self._stop_pub = rospy.Publisher('/load_cell/stop_streaming', std_msgs.Empty, queue_size=1)
        self._cmd_pub = rospy.Publisher('/command', agiros_msgs.Command, queue_size=1)
        self._arm_pub = rospy.Publisher('/arm', std_msgs.Bool, queue_size=1)

        # Data acquisition
        self._load_sub = rospy.Subscriber("/load_cell/data", geometry_msgs.WrenchStamped, self.loadcell_callback)
        self._voltage_sub = rospy.Subscriber("/voltage", std_msgs.Float32, self.voltage_callback)

    def run(self):
        self.create_thrust_trajectory()
        # Wait some time to get things running; otherwise FMU is not armed
        time.sleep(1)
        self.arm_fmu()

        rospy.logout("Wait after arming to enable quad to be responsive...")
        time.sleep(5)
        rospy.logout("Wait ended...")

        self.set_load_cell_bias()

        rospy.logout("Start thrust mapping acquisition...")

        # Start stream, this will fill the raw_data buffer through the load cell subscription
        # Empty raw buffers before start
        self.reset_buffer()
        self.start_streaming()

        # Do steps
        last_applied_cmd = self._cmds[0]
        for cmd in self._cmds:
            rospy.logout("Send command: %f", cmd)
            init_time = rospy.Time.now()
            iter_idx = 0

            while (rospy.Time.now() - init_time).to_sec() < self._duration and \
                    not rospy.is_shutdown():
                self.send_cmd(cmd)
                if iter_idx > 25:
                    # to avoid measuring transient effects, we wait a bit after each step
                    self.log_data()
                last_applied_cmd = cmd
                self._rate.sleep()
                iter_idx += 1

        # Ramp down motor otherwise active breaking messes up PSU
        rospy.logout("Ramping down motor from: cmd=%f to cmd=%f", last_applied_cmd, self._zero_command)
        break_duration = 1.0
        start_breaking_time = rospy.get_time()
        while not rospy.is_shutdown():
            cmd = (self._zero_command - last_applied_cmd) / break_duration * (
                    rospy.get_time() - start_breaking_time) + last_applied_cmd
            if cmd <= self._zero_command:
                # Stop motor completely by sending off command
                self.send_cmd(self._zero_command)
                break
            self.send_cmd(cmd)
            self._rate.sleep()

        self.stop_streaming()
        self.send_zero_cmd(1.0)  # Stop motor

        if not len(self._raw_data) > 0:
            rospy.logerr("No loadcell data captured; check connection!")
            sys.exit(1)

        # Finished run, save data
        self.write_data_to_disk()

        rospy.logout("Finished run")

    def create_thrust_trajectory(self):
        thrust_up = np.linspace(self.min_cmd, self.max_cmd, self.num_cmd_steps)
        thrust_down = np.flip(thrust_up)[1:-1]

        self._cmds = np.tile(np.concatenate([thrust_up, thrust_down]), self.cycles)
        # append zero command at the end
        self._cmds = np.insert(self._cmds, 0, self._zero_command)
        total_duration = self._duration * len(self._cmds)
        print("Thrust mapping will take %.1f seconds" % total_duration)

    def loadcell_callback(self, msg):
        self.load_cell_data = msg

    def voltage_callback(self, msg):
        self.voltage_data = msg

    def log_data(self):
        if self.load_cell_data is None or self.command_data is None or self.voltage_data is None:
            print("Not saving data since not all sensors have been received")
            print(self.load_cell_data)
            print(self.command_data)
            print(self.voltage_data)
            return
        self._raw_data.append(self.load_cell_data)
        self._raw_cmds.append(self.command_data)
        self._raw_voltage.append(self.voltage_data)

    def reset_buffer(self):
        self._raw_data = []
        self._raw_cmds = []
        self._raw_voltage = []

    def arm_fmu(self):
        arm_msg = std_msgs.Bool()
        arm_msg.data = True
        self._arm_pub.publish(arm_msg)

    def set_load_cell_bias(self):
        self._bias_pub.publish(std_msgs.Empty())
        rospy.sleep(2.0)

    def send_zero_cmd(self, duration):
        init_time = rospy.Time.now()
        while (rospy.Time.now() - init_time).to_sec() < duration and not rospy.is_shutdown():
            self.send_cmd(self._zero_command)
            self._rate.sleep()

    def send_cmd(self, cmd):
        cmd_msg = agiros_msgs.Command()
        cmd_msg.is_single_rotor_thrust = self.is_single_rotor_thrust

        if cmd_msg.is_single_rotor_thrust:
            cmd_msg.thrusts = [cmd, cmd, cmd, cmd]
        else:
            cmd_msg.collective_thrust = cmd

        self.command_data = cmd_msg
        self._cmd_pub.publish(cmd_msg)

    def start_streaming(self):
        self._start_pub.publish(std_msgs.Empty())

    def stop_streaming(self):
        self._stop_pub.publish(std_msgs.Empty())

    def write_data_to_disk(self):
        length = len(self._raw_data)
        assert len(self._raw_data) == len(self._raw_cmds) == len(self._raw_voltage)

        force_data = np.zeros((length, 3))
        force_data[:, 0] = np.array([msg.wrench.force.x for msg in self._raw_data[0:length]])
        force_data[:, 1] = np.array([msg.wrench.force.y for msg in self._raw_data[0:length]])
        force_data[:, 2] = np.array([msg.wrench.force.z for msg in self._raw_data[0:length]])

        torque_data = np.zeros((length, 3))
        torque_data[:, 0] = np.array([msg.wrench.torque.x for msg in self._raw_data[0:length]])
        torque_data[:, 1] = np.array([msg.wrench.torque.y for msg in self._raw_data[0:length]])
        torque_data[:, 2] = np.array([msg.wrench.torque.z for msg in self._raw_data[0:length]])

        cmd_data = np.zeros((length, 1))
        if self.is_single_rotor_thrust:
            cmd_data[:, 0] = np.array([msg.thrusts[0] for msg in self._raw_cmds])
        else:
            cmd_data[:, 0] = np.array([msg.collective_thrust for msg in self._raw_cmds])
        
        voltage_data = np.zeros((length, 1))
        voltage_data[:, 0] = np.array([msg.data for msg in self._raw_voltage])

        # Save to file
        current_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        curr_dir = os.path.join(self.data_dir, current_time)
        os.makedirs(curr_dir)
        np.savetxt(os.path.join(curr_dir, 'force_data.csv'), force_data, delimiter=',')
        np.savetxt(os.path.join(curr_dir, 'torque_data.csv'), torque_data, delimiter=',')
        np.savetxt(os.path.join(curr_dir, 'cmd_data.csv'), cmd_data, delimiter=',')
        np.savetxt(os.path.join(curr_dir, 'voltage_data.csv'), voltage_data, delimiter=',')

        print("Saved data to [%s]" % curr_dir)


if __name__ == "__main__":
    data_dir = rospy.get_param('/calculate_thrust_mapping/data_dir')
    is_single_rotor_thrust = rospy.get_param('/calculate_thrust_mapping/single_rotor_thrust')
    cycles = int(rospy.get_param('/calculate_thrust_mapping/cycles'))
    min_cmd = float(rospy.get_param('/calculate_thrust_mapping/min_cmd'))
    max_cmd = float(rospy.get_param('/calculate_thrust_mapping/max_cmd'))
    zero_cmd = float(rospy.get_param('/calculate_thrust_mapping/zero_cmd'))
    num_cmd_steps = int(rospy.get_param('/calculate_thrust_mapping/num_cmd_steps'))
    step_duration = int(rospy.get_param('/calculate_thrust_mapping/step_duration'))

    rospy.logout("")
    app = AcquireThrustMapping(data_dir, is_single_rotor_thrust, cycles, min_cmd, max_cmd, zero_cmd, num_cmd_steps,
                               step_duration)
    app.run()
