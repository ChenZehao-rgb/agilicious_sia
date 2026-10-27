#!/usr/bin/env python3
"""Synthetic ROS delivery and output-PTY regressions; no physical devices.

Uses the repository's existing process/PTY harness. AGI_ROS2_TEST_INSTALL selects
an isolated local build. Timing distributions are workstation measurements only.
"""
import copy
import json
import math
import os
from pathlib import Path
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'test'))
import rclpy
from agi_ros2.msg import ComputationStatus, ControlCommand, FusedState, Health, OutputStatus, OutputTiming
from test_node_pipeline import CLOCK_ID, Harness
from test_shadow_pipeline import ShadowHarness

ROOT = Path(__file__).resolve().parents[2]


def distribution(values):
    values = sorted(v for v in values if math.isfinite(v))
    def percentile(p):
        k = (len(values)-1)*p
        lo, hi = int(k), min(int(k)+1, len(values)-1)
        return values[lo] + (values[hi]-values[lo])*(k-lo)
    return dict(n=len(values), **{k: percentile(p) for k,p in [('p50',.5),('p95',.95),('p99',.99),('max',1.)]}) if values else {}


class DeliveryHarness(ShadowHarness):
    def __init__(self, output=False):
        super().__init__()
        self.output = output
        self.age = .00374
        self.duplicate = self.stopped = self.invalid = False
        self.generation = 1
        self.fc_session = 'synthetic-source'
        self.sample_shift = self.ros_shift = 0.
        self.sequence = 0
        self.next_state = 0.
        self.last_state = None
        self.input_states = []
        for topic, cls in [('control_command', ControlCommand), ('computation_status', ComputationStatus),
                           ('output_status', OutputStatus), ('output_timing', OutputTiming)]:
            self.subscribe(topic, cls)
        common = dict(runtime_config=str(ROOT/'agi_ros2/config/hardware.yaml'), shadow_only=False)
        if output:
            self.start_node('command_output_node', device=os.ttyname(self.slave), **common,
                            **{f'msp.{name}.enabled':False for name in ('attitude','rc','status','analog','battery','gps')})
        else:
            self.start_node(os.environ.get('FC_CONTROL_EXECUTABLE', 'control_node'), **common)
        self.driver = self.drive

    def drive(self):
        Harness.publish_inputs(self, False, False)
        now = time.monotonic()
        if not self.output:
            status = OutputStatus()
            status.header.stamp = self.stamp()
            status.clock_id = CLOCK_ID
            status.steady_time = now
            status.session_start = 1.
            status.transport_healthy = status.thrust_mapping_ready = True
            self.publisher('output_status', OutputStatus).publish(status)
        if now < self.next_state or self.stopped:
            return
        self.next_state = max(self.next_state+.01, now+.001)
        if self.duplicate and self.last_state:
            state = copy.deepcopy(self.last_state)
            state.published_steady_time = now
        else:
            state = FusedState()
            state.header.stamp = self.stamp()
            ns = state.header.stamp.sec*10**9+state.header.stamp.nanosec-int((self.age-self.ros_shift)*1e9)
            state.header.stamp.sec, state.header.stamp.nanosec = divmod(ns,10**9)
            state.header.frame_id = 'odom'
            state.clock_id = CLOCK_ID
            state.state_source = state.navigation_source = 'fc_fused'
            state.navigation_session = self.fc_session
            state.fc_time_usec = int((now-self.age)*1e6)
            state.reset_counter = self.generation
            state.position.z = 1.
            state.orientation.w = 1.
            state.fc_state_valid = not self.invalid
            state.clock_aligned = state.heading_valid = True
            state.fc_state_sample_steady_time = now-self.age+self.sample_shift
            state.fc_state_receive_steady_time = now
            state.published_steady_time = time.monotonic()
            # 50 Hz health epoch, carried without manufacturing a new health sample.
            state.fc_health_sample_steady_time = math.floor(now/.02)*.02-.004
            state.fc_health_receive_steady_time = math.floor(now/.02)*.02
            state.fc_xy_observation_time = state.fc_z_observation_time = now-.05
            state.fc_flags = 0xc7
            state.position_variance = [.01]*3
            state.velocity_variance = [.01]*3
            self.last_state = state
        self.publisher('fused_state', FusedState).publish(state)
        self.input_states.append(state)
        if self.output:
            command = ControlCommand()
            command.header.stamp = self.stamp()
            command.header.frame_id = 'base_link'
            command.clock_id = CLOCK_ID
            self.sequence += 1
            command.sequence = self.sequence
            command.total_thrust = 7.
            command.permit_override = self.auto
            command.control_session_start = 1.
            command.published_steady_time = time.monotonic()
            e = command.evidence
            e.now = e.command_time = e.rc_time = e.status_time = now
            e.solve_seconds = .001
            for field in ('command_valid','controller_warm','receiver_valid','rc_link','armed',
                          'heading_valid','accuracy_ok','clock_aligned','config_verified','thrust_mapping_ready',
                          'geofence_ok','msp_healthy','fc_valid'):
                setattr(e,field,True)
            e.auto_switch = self.auto
            e.kill = False
            e.fc_session = state.navigation_session
            e.fc_generation = state.reset_counter
            e.fc_sample_time = state.fc_state_sample_steady_time
            e.fc_receive_time = state.fc_state_receive_steady_time
            e.fc_publish_time = state.published_steady_time
            for a,b in [('fc_health_sample_time','fc_health_sample_steady_time'),
                        ('fc_health_receive_time','fc_health_receive_steady_time'),
                        ('fc_xy_observation_time','fc_xy_observation_time'),('fc_z_observation_time','fc_z_observation_time'),
                        ('fc_flags','fc_flags'),('fc_position_variance','position_variance'),('fc_velocity_variance','velocity_variance')]:
                setattr(e,a,getattr(state,b))
            self.publisher('control_command',ControlCommand).publish(command)

    def computations(self):
        return [c for c in self.received['computation_status'] if math.isfinite(c.solve_seconds)]


class DeliveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init()
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()

    def test_new_sample_delivery_and_faults(self):
        h = DeliveryHarness()
        reports = {}
        try:
            h.run(1.5)
            self.assertTrue(h.computations())
            for age in (.00374,.00524,.00734):
                h.age = age
                h.run(.1)
                offset = len(h.received['control_command'])
                h.run(1.2)
                commands = h.received['control_command'][offset:]
                computed = [c for c in commands if math.isfinite(c.evidence.solve_seconds)]
                self.assertGreater(len(computed),90)
                identities=[(c.evidence.fc_session,c.evidence.fc_sample_time) for c in computed]
                self.assertEqual(len(identities),len(set(identities)))
                gaps=[b.control_start_steady_time-a.control_start_steady_time for a,b in zip(computed,computed[1:])]
                self.assertTrue(all(gap>=.00799 for gap in gaps))
                reports[str(age)] = dict(computations=len(computed), notifications=len(commands),
                    control_gap_ms=distribution([v*1000 for v in gaps]),
                    age_before_ms=distribution([(c.control_start_steady_time-c.evidence.fc_sample_time)*1000 for c in computed]),
                    age_after_ms=distribution([(c.control_end_steady_time-c.evidence.fc_sample_time)*1000 for c in computed]),
                    adapter_to_callback_ms=distribution([(c.state_received_steady_time-c.state_published_steady_time)*1000 for c in computed]),
                    callback_to_start_ms=distribution([(c.control_start_steady_time-c.state_received_steady_time)*1000 for c in computed]),
                    compute_ms=distribution([c.evidence.solve_seconds*1000 for c in computed]),
                    warm_resets=h.received['computation_status'][-1].warm_resets,
                    post_deadline=sum(c.control_end_steady_time-c.evidence.fc_sample_time>.015 for c in computed))
            h.age = .00374
            h.run(.8)
            self.assertEqual(h.received['computation_status'][-1].warm_cycles,50)
            h.auto = True
            h.run(.12)
            self.assertTrue(any(c.permit_override for c in h.received['control_command'][-8:]))
            for field,value in [('duplicate',True),('stopped',True),('invalid',True),('age',.020),
                                ('sample_shift',.1),('ros_shift',1.),('kill',True),('armed',False)]:
                previous=getattr(h,field)
                setattr(h,field,value)
                h.run(.12)
                c=h.received['computation_status'][-1]
                self.assertFalse(c.controller_success,field)
                self.assertEqual(c.warm_cycles,0,field)
                self.assertFalse(c.trajectory_active,field)
                setattr(h,field,previous)
                h.run(.7)
                self.assertFalse(h.received['computation_status'][-1].trajectory_active,field)
                h.auto=False
                h.run(.1)
                h.auto=True
                h.run(.1)
            # No invalid message is delivered: next valid message carries the new generation.
            h.generation+=1
            h.run(.12)
            self.assertFalse(h.received['computation_status'][-1].trajectory_active)
            h.fc_session='rebooted-fc'
            h.run(.12)
            self.assertFalse(h.received['computation_status'][-1].trajectory_active)
            self.assertFalse(any(code==200 for _,code,_ in h.codes))
        finally:
            Path(os.environ.get('FC_DELIVERY_REPORT','/tmp/fc-delivery.json')).write_text(json.dumps(reports,indent=2))
            h.close()

    def test_output_pty_independently_expires_without_invalid_message(self):
        h = DeliveryHarness(output=True)
        try:
            h.run(1.)
            h.auto=True
            h.run(.15)
            self.assertTrue(any(code==200 for _,code,_ in h.codes), str(h.received['output_status'][-1]))
            # Freeze both state and commands, without sending an invalid event.
            h.stopped=True
            sample=h.last_state.fc_state_sample_steady_time
            h.run(.040)
            self.assertFalse(h.received['output_status'][-1].override_active)
            revoked=[s for s in h.received['output_status'] if s.steady_time>sample+.025 and not s.override_active]
            self.assertTrue(revoked)
            first=min(revoked,key=lambda s:s.steady_time)
            self.assertGreater(first.fc_sample_age_at_check,.025)
            self.assertLess(first.steady_time-sample,h.age+.025)
            writes=[t for t,code,_ in h.codes if code==200]
            self.assertLessEqual(max(writes),sample+.025)
            h.stopped=False
            h.run(.12)
            self.assertFalse(h.received['output_status'][-1].override_active)
            h.auto=False
            h.run(.08)
            h.auto=True
            h.run(.08)
            self.assertTrue(h.received['output_status'][-1].override_active)
            Path(os.environ.get('FC_OUTPUT_REPORT','/tmp/fc-output.json')).write_text(json.dumps({
                'revocation_sample_age_ms':1000*(first.steady_time-sample),
                'gate_sample_age_ms':1000*first.fc_sample_age_at_check,
                'reason':first.reason,'pty_msp200_frames':len(writes),'real_serial_devices':0},indent=2))
        finally: h.close()


if __name__=='__main__': unittest.main()
