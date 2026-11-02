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
import signal
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'test'))
import rclpy
from agi_ros2.msg import GoalStatus, GeographicGoal, NavigationOrigin, ComputationStatus, ControlCommand, FusedState, Health, OutputStatus, OutputTiming
from test_node_pipeline import CLOCK_ID, Harness
from test_shadow_pipeline import ShadowHarness
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
from test_fc_fused_pipeline import FcHarness
import struct

ROOT = Path(__file__).resolve().parents[2]


def distribution(values):
    values = sorted(v for v in values if math.isfinite(v))
    def percentile(p):
        k = (len(values)-1)*p
        lo, hi = int(k), min(int(k)+1, len(values)-1)
        return values[lo] + (values[hi]-values[lo])*(k-lo)
    return dict(n=len(values), **{k: percentile(p) for k,p in [('p50',.5),('p95',.95),('p99',.99),('max',1.)]}) if values else {}


class DeliveryHarness(ShadowHarness):
    def __init__(self, output=False, shadow=False):
        super().__init__()
        self.output = output
        self.age = .00374
        self.duplicate = self.stopped = self.invalid = False
        self.generation = 1
        self.fc_session = 'synthetic-source'
        self.fc_reset = 0
        self.sample_shift = self.ros_shift = 0.
        self.sequence = 0
        self.next_state = 0.
        self.last_state = None
        self.input_states = []
        self.position = (0., 0., 1.)
        self.velocity = (0., 0., 0.)
        for topic, cls in [('control_command', ControlCommand), ('computation_status', ComputationStatus),
                           ('output_status', OutputStatus), ('output_timing', OutputTiming)]:
            self.subscribe(topic, cls)
        common = dict(runtime_config=str(ROOT/'agi_ros2/config/hardware.yaml'), shadow_only=shadow)
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
            state.fc_reset_counter = self.fc_reset
            state.position.x, state.position.y, state.position.z = self.position
            state.velocity.x, state.velocity.y, state.velocity.z = self.velocity
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
            # Command creation follows state publication, as in the real pipeline.
            e.now = e.command_time = command.published_steady_time
            e.rc_time = e.status_time = now
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

    def online_goal_scenario(self, shadow, geographic=False):
        h = DeliveryHarness(shadow=shadow)
        self.addCleanup(h.close)
        h.subscribe('reference', Odometry)
        h.subscribe('goal_status', GoalStatus)
        publisher = h.publisher('go_to_position', PointStamped)
        geo_publisher = h.publisher('go_to_geographic_position', GeographicGoal)
        origin_publisher = h.node.create_publisher(NavigationOrigin, 'navigation/origin',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        origin = NavigationOrigin()
        origin.header.frame_id = 'odom'
        origin.session_id = h.fc_session
        origin.altitude_reference = 'msl'
        origin.xy_valid = origin.z_valid = True
        origin.longitude_scale = .75
        origin.altitude = 123.
        origin.fc_time_usec = 1

        def send(position, frame='odom'):
            goal = PointStamped()  # Zero stamp is intentionally valid.
            goal.header.frame_id = frame
            goal.point.x, goal.point.y, goal.point.z = position
            if geographic and frame == 'odom':
                geo = GeographicGoal()
                geo.latitude = position[1] / 111319.49424743652
                geo.longitude = position[0] / (111319.49424743652 * .75)
                geo.altitude_msl = 123. + position[2]
                geo_publisher.publish(geo)
            else:
                publisher.publish(goal)
            h.run(.08)

        def log_text():
            for stream in h.logs:
                stream.flush()
            return '\n'.join(path.read_text() for path in h.path.glob('control_node*.log'))

        def reference():
            p = h.received['reference'][-1].pose.pose.position
            return (p.x, p.y, p.z)

        h.run(1.5)
        if geographic:
            send((.5, 0., 1.))
            self.assertIn('FC origin session/reset/time mismatch', log_text())
            self.assertEqual(h.received['goal_status'][-1].request_state, GoalStatus.REJECTED)
            self.assertIn('session/reset/time mismatch', h.received['goal_status'][-1].request_reason)
            origin_publisher.publish(origin)
            h.run(.08)
            # A late old session and old time must not overwrite the valid origin.
            old = copy.deepcopy(origin)
            old.session_id = 'retired-session'
            old.altitude = -1000.
            origin_publisher.publish(old)
            h.run(.04)
        send((.5, 0., 1.))
        self.assertIn('go_to_position rejected: AUTO', log_text())
        h.auto = True
        h.run(.12)
        self.assertTrue(h.received['computation_status'][-1].trajectory_active)
        self.assertTrue(h.received['computation_status'][-1].hover_stable)
        self.assertFalse(h.received['goal_status'][-1].active_valid)
        start = reference()
        for point, frame, reason in [((.5, 0., 1.), 'map', 'frame_id must be odom'),
                                      ((math.nan, 0., 1.), 'odom', 'nonfinite position'),
                                      ((51., 0., 1.), 'odom', 'outside configured geofence')]:
            send(point, frame)
            self.assertIn('invalid WGS84/MSL target' if geographic and math.isnan(point[0]) else reason, log_text())
            self.assertEqual(reference(), start)
        h.velocity = (.3, 0., 0.)
        h.run(.05)
        self.assertFalse(h.received['computation_status'][-1].hover_stable)
        send((.5, 0., 1.))
        self.assertIn('not stable: speed', log_text())
        h.velocity = (0., 0., 0.)
        h.run(.05)
        send((.5, 0., 1.))
        self.assertEqual(log_text().count('go_to_position accepted:'), 1)
        active_sequence = h.received['goal_status'][-1].active_request_sequence
        self.assertEqual(h.received['goal_status'][-1].request_state, GoalStatus.ACCEPTED)
        self.assertEqual(h.received['goal_status'][-1].active_phase, GoalStatus.MOVING)
        h.run(.65)
        send((0., .5, 1.))
        self.assertIn('rejected: trajectory moving', log_text())
        self.assertEqual(h.received['goal_status'][-1].request_state, GoalStatus.REJECTED)
        self.assertTrue(h.received['goal_status'][-1].active_valid)
        self.assertEqual(h.received['goal_status'][-1].active_request_sequence, active_sequence)
        h.run(.9)
        self.assertEqual(reference(), (.5, 0., 1.))
        self.assertIn('segment ended; holding target', log_text())
        self.assertEqual(h.received['goal_status'][-1].active_phase, GoalStatus.SETTLING)
        send((0., .5, 1.))
        self.assertIn('not stable: position error', log_text())
        self.assertEqual(log_text().count('go_to_position accepted:'), 1)
        h.position = (.5, 0., 1.)
        h.run(.05)
        self.assertIn('stable hover', h.received['computation_status'][-1].reason)
        self.assertEqual(h.received['goal_status'][-1].active_phase, GoalStatus.STABLE)
        self.assertTrue(h.received['computation_status'][-1].hover_stable)
        late = []
        late_sub = h.node.create_subscription(GoalStatus, 'goal_status', late.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        h.run(.05)
        self.assertTrue(late)
        self.assertEqual(late[-1].active_request_sequence, active_sequence)
        h.node.destroy_subscription(late_sub)
        send((.5, .5, 1.))
        self.assertEqual(log_text().count('go_to_position accepted:'), 2)
        # Existing yaw-first behavior: pi/2 turn takes 1.875*(pi/2)/.5 s,
        # followed by the 1.5 s translation. Do not mistake the turn for a stall.
        h.run(7.6)
        for actual, expected in zip(reference(), (.5, .5, 1.)):
            self.assertAlmostEqual(actual, expected, places=12)
        self.assertEqual(h.received['computation_status'][-1].warm_cycles, 50)
        self.assertTrue(h.received['computation_status'][-1].trajectory_active)
        # Deliver a queued goal alongside a new coordinate generation. Callback order
        # must not let it survive the reset and replay on the next AUTO edge.
        h.position = (.5, .5, 1.)
        h.run(.05)
        h.processes[0].send_signal(signal.SIGSTOP)
        try:
            goal = PointStamped()
            goal.header.frame_id = 'odom'
            goal.point.x, goal.point.y, goal.point.z = (.5, 1., 1.)
            if geographic:
                geo = GeographicGoal(latitude=1. / 111319.49424743652,
                                     longitude=.5 / (111319.49424743652 * .75), altitude_msl=124.)
                geo_publisher.publish(geo)
            else:
                publisher.publish(goal)
            h.generation += 1
            h.fc_reset += 256
            h.run(.04)
        finally:
            h.processes[0].send_signal(signal.SIGCONT)
        h.run(.1)
        self.assertFalse(h.received['computation_status'][-1].trajectory_active)
        self.assertFalse(h.received['goal_status'][-1].active_valid)
        self.assertEqual(h.received['goal_status'][-1].request_state, GoalStatus.REJECTED)
        send((10., 0., 1.))
        h.run(.7)
        self.assertFalse(h.received['computation_status'][-1].trajectory_active)
        h.auto = False
        h.run(.1)
        h.auto = True
        h.run(.1)
        self.assertTrue(h.received['computation_status'][-1].trajectory_active)
        self.assertEqual(reference(), h.position)
        self.assertEqual(log_text().count('go_to_position accepted:'), 2)
        if shadow:
            self.assertFalse(any(c.permit_override for c in h.received['control_command']))
        else:
            self.assertTrue(h.received['control_command'][-1].permit_override)
        accepted = [c for c in h.computations() if 'go_to_position accepted:' in c.reason]
        self.assertEqual(len(accepted), 2)
        print('online goal planning+MPC timing:', [c.solve_seconds for c in accepted], flush=True)
        self.assertFalse(any(code == 200 for _, code, _ in h.codes))

    def test_online_goals_on_fc_control_thread(self):
        self.online_goal_scenario(False)

    def test_geographic_goals_share_local_chain_and_reset_cleanup(self):
        self.online_goal_scenario(True, geographic=True)

    def test_pause_priority_idle_sequence_and_reset(self):
        h = DeliveryHarness(shadow=False)
        self.addCleanup(h.close)
        h.subscribe('reference', Odometry)
        h.subscribe('goal_status', GoalStatus)
        goals = h.publisher('go_to_position', PointStamped)
        pauses = h.publisher('pause_hover', Empty)
        h.run(1.5)
        h.auto = True
        h.run(.1)
        self.assertTrue(h.received['computation_status'][-1].hover_stable)
        goal = PointStamped()
        goal.header.frame_id = 'odom'
        goal.point.x, goal.point.z = 8., 1.
        def hold_after_computed_sample():
            h.next_state = 0.
            h.drive()
            h.stopped = True
            until = time.monotonic() + .012
            # A just-published sample may still be coalesced by the 8 ms scheduler.
            # Wait for its actual computation before asserting idle semantics.
            while h.received['computation_status'][-1].fc_time_usec != h.last_state.fc_time_usec and time.monotonic() < until:
                h.run(.0005)
            self.assertEqual(h.received['computation_status'][-1].fc_time_usec, h.last_state.fc_time_usec)
            self.assertTrue(h.received['computation_status'][-1].controller_success)

        # Supply one fresh FC sample, then exercise callbacks during healthy idle.
        # The hold lasts less than the existing 25 ms watchdog; all gates stay on.
        hold_after_computed_sample()
        try:
            goals.publish(goal)
            h.run(.003)
            self.assertEqual(h.received['goal_status'][-1].request_state, GoalStatus.PENDING)
            pauses.publish(Empty())
            h.run(.003)
            pending = h.received['goal_status'][-1]
            self.assertEqual(pending.request_source, 'pause_hover')
            self.assertEqual(pending.request_state, GoalStatus.PENDING)
            sequence = pending.request_sequence
            goals.publish(goal)
            h.run(.003)
            rejected = h.received['goal_status'][-1]
            self.assertEqual(rejected.request_state, GoalStatus.REJECTED)
            self.assertEqual(rejected.request_reason, 'pause_hover pending')
            self.assertFalse(rejected.active_valid)
        finally:
            h.stopped = False
        h.run(.08)
        status = h.received['goal_status'][-1]
        self.assertTrue(status.pause_active)
        self.assertTrue(status.active_valid)
        self.assertEqual(status.active_request_sequence, sequence)
        self.assertEqual(status.request_sequence, sequence+1)
        self.assertEqual(status.request_state, GoalStatus.REJECTED)
        self.assertEqual(status.active_phase, GoalStatus.STABLE)
        self.assertEqual((status.active_enu.x, status.active_enu.y, status.active_enu.z), h.position)
        self.assertTrue(h.received['control_command'][-1].permit_override)
        pauses.publish(Empty())
        h.run(.05)
        status = h.received['goal_status'][-1]
        self.assertEqual(status.request_state, GoalStatus.ACCEPTED)
        self.assertEqual(status.active_request_sequence, sequence)
        goals.publish(goal)
        h.run(.5)
        status = h.received['goal_status'][-1]
        self.assertFalse(status.pause_active)
        self.assertEqual(status.active_phase, GoalStatus.MOVING)
        previous_revision = status.revision
        pauses.publish(Empty())
        h.run(.05)
        status = h.received['goal_status'][-1]
        self.assertTrue(status.pause_active)
        self.assertEqual(status.request_state, GoalStatus.ACCEPTED)
        self.assertEqual(status.active_phase, GoalStatus.MOVING)
        self.assertGreater(status.revision, previous_revision)
        self.assertNotEqual(status.active_enu.x, goal.point.x)
        stop = (status.active_enu.x, status.active_enu.y, status.active_enu.z)
        active_sequence = status.active_request_sequence
        goals.publish(goal)
        h.run(.05)
        self.assertEqual(h.received['goal_status'][-1].request_state, GoalStatus.REJECTED)
        self.assertTrue(h.received['goal_status'][-1].pause_active)
        pauses.publish(Empty())
        h.run(2.)
        status = h.received['goal_status'][-1]
        self.assertEqual(status.active_request_sequence, active_sequence)
        self.assertEqual((status.active_enu.x, status.active_enu.y, status.active_enu.z), stop)
        # A stop can be within 0.2 m already; explicitly make actual speed unstable.
        h.velocity = (.3, 0., 0.)
        h.run(.05)
        self.assertEqual(h.received['goal_status'][-1].active_phase, GoalStatus.SETTLING)
        h.position, h.velocity = stop, (0., 0., 0.)
        h.run(.05)
        self.assertEqual(h.received['goal_status'][-1].active_phase, GoalStatus.STABLE)
        goals.publish(goal)
        h.run(.05)
        self.assertFalse(h.received['goal_status'][-1].pause_active)
        self.assertEqual(h.received['goal_status'][-1].request_state, GoalStatus.ACCEPTED)
        # Pending pause cannot cross an FC reset, including after a new AUTO edge.
        hold_after_computed_sample()
        pauses.publish(Empty())
        h.run(.003)
        self.assertEqual(h.received['goal_status'][-1].request_state, GoalStatus.PENDING)
        h.generation += 1
        h.fc_reset += 1
        h.next_state = 0.
        h.stopped = False
        h.run(.08)
        self.assertEqual(h.received['goal_status'][-1].request_state, GoalStatus.REJECTED)
        self.assertFalse(h.received['goal_status'][-1].pause_active)
        h.run(.7)
        h.auto = False
        h.run(.08)
        h.auto = True
        h.run(.08)
        self.assertTrue(h.received['computation_status'][-1].trajectory_active)
        self.assertFalse(h.received['goal_status'][-1].active_valid)

    def test_origin_receiver_does_not_touch_health(self):
        h = FcHarness()
        self.addCleanup(h.close)
        h.wait_warm()
        received = []
        sub = h.node.create_subscription(NavigationOrigin, 'navigation/origin', received.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.addCleanup(h.node.destroy_subscription, sub)

        def send(stamp=None, reset=None, scale=.75, kind=0xBF02):
            payload = struct.pack('<BBHIQiiff', 1, 32, 3, h.reset_count if reset is None else reset,
                                  h.last_state if stamp is None else stamp, 0, 0, scale, 12300.)
            h.peer.v2_extension_send(0, 0, 0, kind, list(payload + bytes(249-32)))
            h.run(.08)

        send()
        self.assertTrue(received[-1].xy_valid and received[-1].z_valid)
        self.assertEqual(received[-1].altitude, 123.)
        self.assertEqual(received[-1].longitude_scale, .75)
        original = copy.deepcopy(received[-1])
        send(stamp=original.fc_time_usec-1, scale=.5)
        self.assertEqual(received[-1].longitude_scale, .75)
        generation = h.latest('fused_state').reset_counter
        send(kind=0xBF03)
        send(scale=math.nan)
        self.assertFalse(received[-1].xy_valid)
        self.assertEqual(h.latest('fused_state').reset_counter, generation)
        self.assertEqual(h.latest('computation_status').warm_cycles, 50)
        send()
        # A late subscriber gets the retained origin without waiting for another packet.
        late = []
        late_sub = h.node.create_subscription(NavigationOrigin, 'navigation/origin', late.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.addCleanup(h.node.destroy_subscription, late_sub)
        h.run(.15)
        self.assertTrue(late and late[-1].xy_valid)
        old = copy.deepcopy(received[-1])
        h.reset_count += 256
        h.run(.1)
        self.assertFalse(received[-1].xy_valid)
        send(stamp=old.fc_time_usec, reset=old.fc_reset_counter)
        self.assertFalse(received[-1].xy_valid)
        send()
        self.assertEqual(received[-1].fc_reset_counter, 256)
        health_time = h.latest('sensors/fc/estimator_status').fc_time_usec
        h.health_enabled = False
        h.run(.1)
        send()
        self.assertEqual(h.latest('sensors/fc/estimator_status').fc_time_usec, health_time)
        self.assertFalse(h.latest('fused_state').fc_state_valid)  # Origin cannot renew a health lease.
        h.reconnect()
        h.run(.8)
        self.assertFalse(received[-1].xy_valid)
        self.assertNotEqual(received[-1].session_id, old.session_id)
        self.assertEqual(received[-1].header.frame_id, 'odom')
        self.assertEqual(received[-1].altitude_reference, 'msl')

    def test_online_goals_in_shadow(self):
        self.online_goal_scenario(True)

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
