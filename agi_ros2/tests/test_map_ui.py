#!/usr/bin/env python3
"""Local synthetic DDS/HTTP/map tests; run in an isolated ROS domain, no hardware."""
import copy
import json
import math
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from concurrent.futures import ThreadPoolExecutor
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import map_ui
import ground_station_telemetry as relay
import rclpy
from std_msgs.msg import Empty
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rosidl_runtime_py.convert import message_to_ordereddict
from agi_ros2.msg import FusedState, GeographicGoal, GoalStatus, NavigationOrigin, Health, Authority, ComputationStatus


def mapping():
    origin = NavigationOrigin()
    origin.header.frame_id = 'odom'
    origin.session_id = 'remote-receiver'
    origin.altitude_reference = 'msl'
    origin.xy_valid = origin.z_valid = True
    origin.longitude_scale = .75
    origin.fc_reset_counter = 0x12345678
    origin.fc_time_usec = 4300000000
    state = FusedState()
    state.header.frame_id = 'odom'
    state.state_source = 'fc_fused'
    state.fc_state_valid = True
    state.clock_id = 'a-different-host-boot-id'
    state.navigation_session = origin.session_id
    state.fc_reset_counter = origin.fc_reset_counter
    state.fc_time_usec = origin.fc_time_usec + 1
    state.reset_counter = 42
    state.orientation.w = 1.
    return state, origin


class DisplayTests(unittest.TestCase):
    def test_tuning_patch_types_ranges_and_duplicate_keys(self):
        for group, values in [
                ('trajectory', {'go_to_pose_max_vel': True}),
                ('trajectory', {'go_to_pose_max_vel': 0}),
                ('mpc', {'Q_pos_x': -1}), ('mpc', {'Q_vel': math.nan}),
                ('mpc', {'R_body_rates': [1, 2]}), ('mpc', {'R_body_rates': [1, True, 3]}),
                ('mpc', {'R_body_rates_x': 1}), ('mpc', {'exp_decay': 0}),
                ('trajectory', {'Q_pos_x': 1})]:
            with self.assertRaises(ValueError): map_ui.tuning_patch({'group': group, 'values': values})
        values = map_ui.tuning_patch({'group': 'mpc', 'values': {'Q_pos_x': 0, 'exp_decay': 1.01, 'R_body_rates': [1, 2, 3]}})
        self.assertEqual(values[2].value.double_array_value.tolist(), [1., 2., 3.])
        with self.assertRaises(ValueError):
            json.loads('{"values":{"Q_pos_x":1,"Q_pos_x":2}}', object_pairs_hook=map_ui.unique_json)

    def test_unavailable_position_reports_recorded_fc_reason(self):
        # User SITL bag 2026-11-01 18:49: GPS fix exists, but disarmed
        # FC reports flags=174, no XY fusion and no valid horizontal origin.
        state = dict(state_source='fc_fused', fc_state_valid=False, fc_flags=174,
                     readiness_reason='FC XY/Z/fusion/snapshot unavailable')
        reason = map_ui.mapping_error(state, None)
        self.assertIn('FC 未解锁', reason)
        self.assertIn('XY 融合未启用', reason)
        self.assertIn(state['readiness_reason'], reason)
        self.assertIn('ground_station_telemetry', map_ui.mapping_error(None, None))
        state['fc_flags'] = 0
        state['readiness_reason'] = 'MAVLink TCP connection failed'
        reason = map_ui.mapping_error(state, None)
        self.assertIn('MAVLink TCP connection failed', reason)
        self.assertNotIn('FC 未解锁', reason)

    def test_projection_matches_cpp_forward_anchors(self):
        # Same independent inputs/results exercised by fc_geographic_test.cpp.
        state, origin = mapping()
        origin.altitude = 123.
        o, s = map(message_to_ordereddict, (origin, state))
        self.assertIsNone(map_ui.mapping_error(s, o))
        self.assertEqual(map_ui.METERS_PER_DEGREE, 111319.49424743652)
        p = map_ui.geographic_position(dict(x=-166.97924137115479, y=111.31949424743652, z=4.), o)
        self.assertAlmostEqual(p['latitude'], .001, places=13)
        self.assertAlmostEqual(p['longitude'], -.002, places=11)
        self.assertEqual(p['altitude_msl'], 127.)
        o['longitude'] = 179.999
        p = map_ui.geographic_position(dict(x=166.97924137115479, y=0., z=0.), o)
        self.assertAlmostEqual(p['longitude'], -179.999, places=11)
        for field, value in [('fc_reset_counter', o['fc_reset_counter'] + 256), ('session_id', 'old'),
                             ('fc_time_usec', state.fc_time_usec + 1), ('xy_valid', False), ('z_valid', False),
                             ('altitude_reference', 'ellipsoid'), ('longitude_scale', 0.), ('latitude', math.nan)]:
            bad = dict(o, **{field: value})
            self.assertIsNotNone(map_ui.mapping_error(s, bad), field)
        zero_state, zero_origin = mapping()
        zero_origin.longitude_scale = 1.
        zero_o = message_to_ordereddict(zero_origin)
        self.assertIsNone(map_ui.mapping_error(message_to_ordereddict(zero_state), zero_o))
        self.assertEqual(map_ui.geographic_position(dict(x=0., y=0., z=0.), zero_o)['altitude_msl'], 0.)
        self.assertEqual(map_ui.heading_degrees(dict(x=0., y=0., z=0., w=1.)), 90.)
        self.assertAlmostEqual(map_ui.heading_degrees(dict(x=0., y=0., z=math.sqrt(.5), w=math.sqrt(.5))), 0., places=10)
        self.assertIsNone(map_ui.heading_degrees(dict(x=1e308, y=1e308, z=1e308, w=1e308)))
        self.assertIsNone(map_ui.geographic_position(dict(x=math.nan, y=0., z=0.), o))
        self.assertEqual(json.dumps(map_ui.clean_json({'v': [math.nan, math.inf, -math.inf]}), allow_nan=False),
                         '{"v": [null, null, null]}')

    def test_mbtiles_rows_mime_missing_and_readonly(self):
        import sqlite3
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'test.mbtiles'
            with sqlite3.connect(path) as db:
                db.executescript('CREATE TABLE metadata(name TEXT,value TEXT); '
                                 'CREATE TABLE tiles(zoom_level INTEGER,tile_column INTEGER,tile_row INTEGER,tile_data BLOB);')
                db.execute('INSERT INTO metadata VALUES (?,?)', ('format', 'png'))
                for x, data in enumerate([b'\x89PNG\r\n\x1a\nNORTH', b'\xff\xd8\xffJPEG', b'RIFF0000WEBPwebp']):
                    db.execute('INSERT INTO tiles VALUES (2,?,3,?)', (x, data))
                db.execute('INSERT INTO tiles VALUES (2,0,0,?)', (b'\x89PNG\r\n\x1a\nSOUTH',))
            tiles = map_ui.MBTiles(path)
            self.assertTrue(tiles.info['available'])
            self.assertEqual(tiles.tile(2, 0, 0), (b'\x89PNG\r\n\x1a\nNORTH', 'image/png'))
            self.assertTrue(tiles.tile(2, 0, 3)[0].endswith(b'SOUTH'))
            self.assertEqual(tiles.tile(2, 1, 0)[1], 'image/jpeg')
            self.assertEqual(tiles.tile(2, 2, 0)[1], 'image/webp')
            self.assertIsNone(tiles.tile(2, 3, 0))
            with self.assertRaises(ValueError):
                tiles.tile(2, 4, 0)
            with tiles.connect() as db, self.assertRaises(sqlite3.OperationalError):
                db.execute('DELETE FROM tiles')
            with sqlite3.connect(path) as db:
                db.execute("UPDATE metadata SET value='pbf'")
            self.assertFalse(map_ui.MBTiles(path).info['available'])
            self.assertFalse(map_ui.MBTiles(Path(tmp) / 'missing').info['available'])


class DDSTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.relay = relay.GroundStationTelemetry()
        self.ui = map_ui.MapUI()
        self.source = rclpy.create_node('map_ui_test_source')
        self.executor = SingleThreadedExecutor()
        for node in (self.relay, self.ui, self.source):
            self.executor.add_node(node)
        self.received = {topic: [] for topic in ('fused_state', 'health', 'authority')}
        self.pubs = {}
        for topic, cls in [('fused_state', FusedState), ('health', Health), ('authority', Authority)]:
            self.pubs[topic] = self.source.create_publisher(cls, topic, relay.LATEST)
            self.source.create_subscription(cls, 'ground_station/' + topic,
                                            lambda msg, key=topic: self.received[key].append((time.monotonic(), msg)), relay.LATEST)
        self.origin_pub = self.source.create_publisher(NavigationOrigin, 'navigation/origin', relay.RETAINED)
        self.goal_pub = self.source.create_publisher(GoalStatus, 'goal_status', relay.RETAINED)
        self.commands = []
        self.source.create_subscription(GeographicGoal, 'go_to_geographic_position', self.commands.append, 1)
        self.run_for(.4)

    def tearDown(self):
        self.executor.shutdown()
        for node in (self.ui, self.relay, self.source):
            node.destroy_node()

    def test_tuning_http_rejects_invalid_without_sending_and_reports_timeout(self):
        server = map_ui.make_server(self.ui, map_ui.MBTiles(None), '127.0.0.1', 0)
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        base = f'http://127.0.0.1:{server.server_port}'
        def post(body, origin=None):
            headers = {'Content-Type':'application/json'}
            if origin: headers['Origin'] = origin
            return urllib.request.urlopen(urllib.request.Request(base+'/api/tuning', body.encode(), headers))
        try:
            with patch.object(self.ui, 'tuning_call') as remote:
                for body in ['{"group":"mpc","values":{"Q_pos_x":1,"Q_pos_x":2}}',
                             '{"group":"mpc","values":{"Q_pos_x":true}}',
                             '{"group":"mpc","values":{"R_body_rates":[1,2]}}',
                             '{"group":"mpc","values":{"Q_vel":NaN}}',
                             '{"group":"trajectory","values":{"Q_pos_x":1}}']:
                    with self.assertRaises(urllib.error.HTTPError) as error: post(body)
                    self.assertEqual(error.exception.code, 400)
                remote.assert_not_called()
            body = '{"group":"trajectory","values":{"go_to_pose_max_vel":1.5}}'
            with self.assertRaises(urllib.error.HTTPError) as error: post(body, 'http://unrelated.test')
            self.assertEqual(error.exception.code, 403)
            with patch.object(self.ui, 'tuning_call', side_effect=TimeoutError('结果未确认')) as remote:
                with self.assertRaises(urllib.error.HTTPError) as error: post(body)
                self.assertEqual(error.exception.code, 503)
                self.assertEqual(json.load(error.exception)['status'], 'unconfirmed')
                self.assertEqual(remote.call_count, 1)
        finally:
            server.shutdown(); server.server_close(); thread.join()

    def test_tuning_service_bridge_confirmation_and_readback_failure(self):
        from rcl_interfaces.srv import GetParameters, SetParametersAtomically
        installed = {'go_to_pose_max_vel': 2., 'go_to_pose_max_acc': 1., 'mode': 'sitl',
                     'controller_type': 'GEO', 'clock_id': 'fixture', 'control_session_start': 10.}
        calls = []
        def get(request, response):
            response.values = [Parameter(name, value=installed.get(name)).get_parameter_value() for name in request.names]
            return response
        def set_values(request, response):
            calls.append(request)
            if request.parameters[0].name.startswith('Q_'):
                response.result.reason = 'GEO'; return response
            for item in request.parameters: installed[item.name] = Parameter.from_parameter_msg(item).value
            response.result.successful = True
            return response
        self.source.create_service(GetParameters, 'flight_control/get_runtime_parameters', get)
        self.source.create_service(SetParametersAtomically, 'flight_control/set_runtime_parameters', set_values)
        self.run_for(.5)
        with ThreadPoolExecutor() as worker:
            def call(function, *args):
                future = worker.submit(function, *args)
                until = time.monotonic() + 4
                while not future.done() and time.monotonic() < until: self.run_for(.01)
                return future.result(timeout=1)
            current = call(self.ui.get_tuning)
            self.assertEqual(current['controller_type'], 'GEO')
            self.assertIsNone(current['mpc']['Q_pos_x'])
            result = call(self.ui.set_tuning, {'group':'trajectory', 'values':{'go_to_pose_max_vel':1.5}})
            self.assertEqual(result['status'], 'applied')
            self.assertEqual(result['current']['trajectory'], {'go_to_pose_max_vel':1.5, 'go_to_pose_max_acc':1.})
            result = call(self.ui.set_tuning, {'group':'mpc', 'values':{'Q_pos_x':2}})
            self.assertEqual(result['status'], 'rejected')
            with patch.object(self.ui, 'get_tuning', side_effect=TimeoutError):
                result = call(self.ui.set_tuning, {'group':'trajectory', 'values':{'go_to_pose_max_acc':.8}})
                self.assertEqual(result['status'], 'applied')
                self.assertIn('回读未完成', result['message'])
            self.assertEqual(len(calls), 3)

    def run_for(self, duration, publish=None):
        end, next_source, count = time.monotonic() + duration, time.monotonic(), 0
        while time.monotonic() < end:
            now = time.monotonic()
            if publish and now >= next_source:
                publish(count)
                count += 1
                next_source += .01
            self.executor.spin_once(timeout_sec=.001)
        return count

    def test_rates_atomic_events_stop_and_identity(self):
        state, origin = mapping()
        health = Health(battery_voltage=16.)
        authority = Authority(receiver_valid=True, armed=True, auto_switch=True, rc_link=True)
        def publish(index):
            state.fc_state_sequence += 1
            state.header.stamp.nanosec = state.fc_state_sequence
            health.battery_voltage = 16. + (index % 7) * .01
            health.reason = str(index)  # Text, voltage, header changes must not bypass throttling.
            for key, msg in [('fused_state', state), ('health', health), ('authority', authority)]:
                self.pubs[key].publish(msg)
        self.origin_pub.publish(origin)
        self.run_for(.3, publish)
        rates = []
        for fast, slow in [(20., 2.), (10., 1.)]:
            result = self.relay.set_parameters_atomically([Parameter('fast_rate_hz', value=fast), Parameter('slow_rate_hz', value=slow)])
            self.assertTrue(result.successful)
            self.run_for(.2, publish)
            for rows in self.received.values():
                rows.clear()
            before = time.monotonic()
            count = self.run_for(3.1, publish)
            elapsed = time.monotonic() - before
            measured = {key: len(rows)/elapsed for key, rows in self.received.items()}
            rates.append(dict(requested=[fast, slow], source_hz=count/elapsed, measured=measured))
            self.assertAlmostEqual(measured['fused_state'], fast, delta=1.0)
            self.assertAlmostEqual(measured['health'], slow, delta=.5)
            self.assertAlmostEqual(measured['authority'], slow, delta=.5)
            self.assertAlmostEqual(count/elapsed, 100., delta=1.)
        old_timers = dict(self.relay.timers_by_rate)
        result = self.relay.set_parameters_atomically([Parameter('fast_rate_hz', value=30.), Parameter('slow_rate_hz', value=math.nan)])
        self.assertFalse(result.successful)
        self.assertEqual(old_timers, self.relay.timers_by_rate)
        self.assertEqual(self.relay.get_parameter('fast_rate_hz').value, 10.)
        authority.kill = True
        start = time.monotonic()
        self.pubs['authority'].publish(authority)
        self.run_for(.1)
        self.assertTrue(self.received['authority'][-1][1].kill)
        self.assertLess(self.received['authority'][-1][0] - start, .1)
        self.run_for(1.2)  # Drain final latest payloads; then no source means no repeats.
        counts = {key: len(rows) for key, rows in self.received.items()}
        self.run_for(1.2)
        self.assertEqual(counts, {key: len(rows) for key, rows in self.received.items()})
        snap = self.ui.snapshot()
        self.assertGreater(snap['topics']['fused_state']['age_s'], 1.)
        self.assertIsNotNone(snap['position'])  # A different host boot ID is allowed.
        identity = snap['identity']
        # The UI still rejects a future origin if it actually receives it.
        # Relay deduplication must not relax this timestamp validity check.
        future_origin = copy.deepcopy(origin)
        future_origin.fc_time_usec = state.fc_time_usec + 1
        self.ui.receive('navigation/origin', future_origin)
        self.assertIsNone(self.ui.snapshot()['position'])
        self.assertEqual(self.ui.snapshot()['identity'], identity)
        forwarded = self.received['fused_state'][-1][1]
        self.assertEqual(forwarded.header, state.header)
        self.assertEqual(forwarded.clock_id, state.clock_id)
        changed = copy.deepcopy(state)
        changed.fc_reset_counter += 256
        self.pubs['fused_state'].publish(changed)
        self.run_for(.2)
        self.assertIsNone(self.ui.snapshot()['position'])
        print('RELAY_RATES ' + json.dumps(rates), flush=True)

    def test_unchanged_origin_refresh_does_not_overtake_position(self):
        state, origin = mapping()
        self.origin_pub.publish(origin)
        self.pubs['fused_state'].publish(state)
        self.run_for(.2)
        initial = self.ui.snapshot()
        self.assertIsNotNone(initial['position'])
        # A real FC origin refresh arrives before the next 20 Hz position.
        refreshed = copy.deepcopy(origin)
        refreshed.header.stamp.sec += 1
        refreshed.fc_time_usec = state.fc_time_usec + 30000
        self.origin_pub.publish(refreshed)
        self.run_for(.1)
        current = self.ui.snapshot()
        self.assertIsNotNone(current['position'])
        self.assertEqual(current['topics']['navigation/origin']['version'],
                         initial['topics']['navigation/origin']['version'])
        self.assertEqual(current['topics']['navigation/origin']['data'], message_to_ordereddict(origin))
        # A late-joining UI must receive that actual retained snapshot too.
        late = map_ui.MapUI()
        self.executor.add_node(late)
        try:
            self.run_for(.2)
            self.assertEqual(late.snapshot()['topics']['navigation/origin']['data'], message_to_ordereddict(origin))
        finally:
            self.executor.remove_node(late)
            late.destroy_node()
        # Every mapping/validity/identity change still forwards immediately.
        for field, value in [('latitude', .001), ('longitude', .002), ('altitude', 3.),
                             ('longitude_scale', .7), ('xy_valid', False), ('z_valid', False),
                             ('session_id', 'new-session'), ('fc_reset_counter', origin.fc_reset_counter+1),
                             ('altitude_reference', 'ellipsoid'), ('fc_time_usec', 0)]:
            changed = copy.deepcopy(refreshed)
            setattr(changed, field, value)
            self.origin_pub.publish(changed)
            self.run_for(.1)
            snap = self.ui.snapshot()
            self.assertEqual(snap['topics']['navigation/origin']['data'], message_to_ordereddict(changed), field)
            self.assertIsNone(snap['position'], field)
            self.origin_pub.publish(origin)
            self.run_for(.1)
            self.assertIsNotNone(self.ui.snapshot()['position'], field)

    def test_starting_worker_does_not_block_telemetry(self):
        gate = threading.Event()
        heartbeat = []
        from diagnostic_msgs.msg import DiagnosticArray
        self.source.create_subscription(DiagnosticArray, 'ground_station/runtime/hardware/status',
                                        heartbeat.append, relay.RUNTIME_QOS)
        state, _ = mapping()
        timer = self.source.create_timer(.01, lambda: self.pubs['fused_state'].publish(state))
        with patch.object(self.relay.runtime, 'run', side_effect=lambda: gate.wait(5)):
            try:
                self.relay.runtime.start()
                self.run_for(1.2)
                self.assertEqual(self.relay.runtime.snapshot()['state'], 'starting')
                self.assertGreaterEqual(len(self.received['fused_state']), 20)
                self.assertGreaterEqual(len(heartbeat), 2)
            finally:
                gate.set()
                self.relay.runtime.worker.join()
                self.source.destroy_timer(timer)

    def test_runtime_heartbeat_role_http_and_no_aircraft_dependency(self):
        from diagnostic_msgs.msg import DiagnosticArray
        rows, starts = [], []
        self.source.create_subscription(DiagnosticArray, 'ground_station/runtime/hardware/status',
                                        lambda m: rows.append((time.monotonic(), m)), relay.RUNTIME_QOS)
        self.source.create_subscription(Empty, 'ground_station/runtime/hardware/start', starts.append, relay.RUNTIME_QOS)
        # Never start hardware: this test only exercises dispatch into the fixed launcher.
        with patch.object(self.relay.runtime, 'start') as start:
            self.run_for(1.6)
            self.assertGreaterEqual(len(rows), 3)
            self.assertIsNone(self.ui.snapshot()['position'])
            wrong = self.source.create_publisher(Empty, 'ground_station/runtime/simulation/start', relay.RUNTIME_QOS)
            self.run_for(.15)
            wrong.publish(Empty())
            self.run_for(.15)
            self.assertEqual(start.call_count, 0)
            self.assertEqual(rows[-1][1].status[0].name, 'summary')
            result = self.relay.set_parameters([Parameter('runtime_mode', value='simulation')])
            self.assertFalse(result[0].successful)
            self.assertEqual(self.relay.get_parameter('runtime_mode').value, 'hardware')
            before = len(rows)
            self.relay.set_parameters([Parameter('slow_rate_hz', value=10.)])
            self.run_for(.55)
            self.assertGreaterEqual(len(rows) - before, 5)
            server = map_ui.make_server(self.ui, map_ui.MBTiles(None), '127.0.0.1', 0)
            thread = threading.Thread(target=server.serve_forever)
            thread.start()
            base = f'http://127.0.0.1:{server.server_port}'
            def post(payload, origin=None):
                headers = {'Content-Type': 'application/json'}
                if origin: headers['Origin'] = origin
                with urllib.request.urlopen(urllib.request.Request(base + '/api/runtime/start',
                                             json.dumps(payload).encode(), headers)) as response:
                    return response.status, json.load(response)
            try:
                for payload, origin, code in [({'mode':'other'}, None, 400), ({'mode':'simulation'}, None, 400),
                        ({'mode':'hardware', 'argv':[]}, None, 400), ({'mode':[]}, None, 400),
                        ({'mode':'hardware'}, 'http://unrelated.example', 403)]:
                    with self.assertRaises(urllib.error.HTTPError) as error: post(payload, origin)
                    self.assertEqual(error.exception.code, code)
                status, result = post({'mode': 'hardware'})
                self.assertEqual(status, 202)
                self.assertEqual(result['message'], '启动请求已发送，等待执行端反馈')
                self.run_for(.15)
                self.assertEqual(start.call_count, 1)
                self.assertEqual(len(starts), 1)
                late = []
                self.source.create_subscription(Empty, 'ground_station/runtime/hardware/start', late.append, relay.RUNTIME_QOS)
                self.run_for(.15)
                self.assertEqual(late, [])
                with ThreadPoolExecutor(max_workers=8) as pool:
                    replies = list(pool.map(lambda _: post({'mode':'hardware'}), range(8)))
                self.assertTrue(all(reply[0] == 202 for reply in replies))
                self.run_for(.15)
                count = start.call_count
                for _ in range(3): urllib.request.urlopen(base + '/api/state').close()
                self.run_for(.15)
                self.assertEqual(start.call_count, count)
                original = self.ui.runtime_publishers['hardware']
                try:
                    for exception in (TimeoutError, RuntimeError):
                        with patch.object(original, 'publish', side_effect=exception('synthetic error')):
                            with self.assertRaises(urllib.error.HTTPError) as error: post({'mode':'hardware'})
                            body = json.load(error.exception)
                            self.assertEqual(error.exception.code, 503)
                            self.assertIn('启动结果未确认', body['error'])
                            self.assertNotIn('地图', body['error'])
                finally:
                    self.ui.runtime_publishers['hardware'] = original
                # Immediate event is independent of the heartbeat timer.
                self.relay.set_parameters([Parameter('slow_rate_hz', value=.01)])
                self.relay.runtime.update(state='failed', reason='synthetic startup failure')
                self.run_for(.1)
                item = self.ui.snapshot()['topics']['runtime/hardware/status']
                self.assertEqual(item['data']['status'][0]['message'], 'synthetic startup failure')
                self.ui.latest['runtime/hardware/status']['received'] -= 4
                with self.assertRaises(urllib.error.HTTPError) as error: post({'mode':'hardware'})
                self.assertIn('服务未连接', json.load(error.exception)['error'])
            finally:
                server.shutdown(); server.server_close(); thread.join()

    def test_runtime_stop_http_role_and_volatile(self):
        stops = []
        self.source.create_subscription(Empty, 'ground_station/runtime/hardware/stop', stops.append, relay.RUNTIME_QOS)
        with patch.object(self.relay.runtime, 'stop') as stop:
            wrong = self.source.create_publisher(Empty, 'ground_station/runtime/simulation/stop', relay.RUNTIME_QOS)
            self.run_for(.6)
            wrong.publish(Empty())
            self.run_for(.1)
            self.assertEqual(stop.call_count, 0)
            server = map_ui.make_server(self.ui, map_ui.MBTiles(None), '127.0.0.1', 0)
            thread = threading.Thread(target=server.serve_forever)
            thread.start()
            base = f'http://127.0.0.1:{server.server_port}'
            def post(payload, origin=None):
                headers = {'Content-Type': 'application/json'}
                if origin: headers['Origin'] = origin
                with urllib.request.urlopen(urllib.request.Request(base + '/api/runtime/stop',
                                             json.dumps(payload).encode(), headers)) as response:
                    return response.status, json.load(response)
            try:
                for payload, origin, code in [({'mode':'other'}, None, 400), ({'mode':'simulation'}, None, 400),
                        ({'mode':'hardware', 'pid':1}, None, 400), ({'mode':'hardware'}, 'http://unrelated.example', 403)]:
                    with self.assertRaises(urllib.error.HTTPError) as error: post(payload, origin)
                    self.assertEqual(error.exception.code, code)
                status, result = post({'mode':'hardware'})
                self.assertEqual(status, 202)
                self.assertIn('结束任务请求已发送', result['message'])
                self.run_for(.15)
                self.assertEqual(stop.call_count, 1)
                self.assertEqual(len(stops), 1)
                late = []
                self.source.create_subscription(Empty, 'ground_station/runtime/hardware/stop', late.append, relay.RUNTIME_QOS)
                self.run_for(.15)
                self.assertEqual(late, [])
                for exception in (TimeoutError, RuntimeError):
                    with patch.object(self.ui.runtime_stop_publishers['hardware'], 'publish', side_effect=exception('synthetic')):
                        with self.assertRaises(urllib.error.HTTPError) as error: post({'mode':'hardware'})
                        self.assertEqual(error.exception.code, 503)
                        self.assertIn('结束任务结果未确认', json.load(error.exception)['error'])
                for _ in range(3): urllib.request.urlopen(base + '/api/state').close()
                self.run_for(.1)
                self.assertEqual(stop.call_count, 1)
                self.ui.latest['runtime/hardware/status']['received'] -= 4
                with self.assertRaises(urllib.error.HTTPError) as error: post({'mode':'hardware'})
                self.assertIn('服务未连接', json.load(error.exception)['error'])
            finally:
                server.shutdown(); server.server_close(); thread.join()

    def test_hardware_shutdown_http_and_volatile_without_executing_shutdown(self):
        shutdowns = []
        self.source.create_subscription(Empty, 'ground_station/runtime/hardware/shutdown', shutdowns.append, relay.RUNTIME_QOS)
        # All hardware shutdown requests stay inside this mock; never call sudo.
        with patch.object(self.relay.runtime, 'shutdown') as shutdown:
            self.run_for(.6)
            server = map_ui.make_server(self.ui, map_ui.MBTiles(None), '127.0.0.1', 0)
            thread = threading.Thread(target=server.serve_forever)
            thread.start()
            base = f'http://127.0.0.1:{server.server_port}'
            def post(payload, origin=None):
                headers = {'Content-Type': 'application/json'}
                if origin: headers['Origin'] = origin
                with urllib.request.urlopen(urllib.request.Request(base + '/api/runtime/shutdown',
                                             json.dumps(payload).encode(), headers)) as response:
                    return response.status, json.load(response)
            try:
                for payload, origin, code in [({'mode':'simulation'}, None, 400), ({'mode':'other'}, None, 400),
                        ({'mode':'hardware', 'command':'shutdown now'}, None, 400), ({'mode':'hardware', 'host':'remote'}, None, 400),
                        ({'mode':'hardware'}, 'http://unrelated.example', 403)]:
                    with self.assertRaises(urllib.error.HTTPError) as error: post(payload, origin)
                    self.assertEqual(error.exception.code, code)
                status, result = post({'mode':'hardware'})
                self.assertEqual(status, 202)
                self.assertEqual(result['message'], '关机请求已发送，等待执行端反馈')
                self.run_for(.15)
                self.assertEqual(shutdown.call_count, 1)
                self.assertEqual(len(shutdowns), 1)
                late = []
                self.source.create_subscription(Empty, 'ground_station/runtime/hardware/shutdown', late.append, relay.RUNTIME_QOS)
                wrong = self.source.create_publisher(Empty, 'ground_station/runtime/simulation/shutdown', relay.RUNTIME_QOS)
                self.run_for(.15)
                wrong.publish(Empty())
                self.run_for(.1)
                self.assertEqual(late, [])
                self.assertEqual(shutdown.call_count, 1)
                for exception in (TimeoutError, RuntimeError):
                    with patch.object(self.ui.runtime_shutdown_publisher, 'publish', side_effect=exception('synthetic')):
                        with self.assertRaises(urllib.error.HTTPError) as error: post({'mode':'hardware'})
                        self.assertEqual(error.exception.code, 503)
                        self.assertIn('关机结果未确认', json.load(error.exception)['error'])
                self.ui.latest['runtime/hardware/status']['received'] -= 4
                with self.assertRaises(urllib.error.HTTPError) as error: post({'mode':'hardware'})
                self.assertIn('服务未连接', json.load(error.exception)['error'])
                self.assertEqual(shutdown.call_count, 1)
            finally:
                server.shutdown(); server.server_close(); thread.join()

    def test_pause_once_empty_volatile_and_immediate_retained_feedback(self):
        pauses = []
        self.source.create_subscription(Empty, 'pause_hover', pauses.append, 1)
        self.run_for(.2)
        server = map_ui.make_server(self.ui, map_ui.MBTiles(None), '127.0.0.1', 0)
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        base = f'http://127.0.0.1:{server.server_port}'
        try:
            def post(payload, origin=None):
                headers = {'Content-Type': 'application/json'}
                if origin:
                    headers['Origin'] = origin
                return urllib.request.urlopen(urllib.request.Request(base + '/api/pause', json.dumps(payload).encode(), headers))
            for payload, origin, code in [({'extra': 1}, None, 400), ({}, 'http://unrelated.example', 403)]:
                with self.assertRaises(urllib.error.HTTPError) as error:
                    post(payload, origin)
                self.assertEqual(error.exception.code, code)
            with post({}) as response:
                self.assertEqual(response.status, 202)
                self.assertEqual(json.load(response)['message'], '暂停请求已发送，等待控制端处理')
            self.run_for(.1)
            self.assertEqual(len(pauses), 1)
            late = []
            self.source.create_subscription(Empty, 'pause_hover', late.append, 1)
            for _ in range(3):
                urllib.request.urlopen(base + '/api/state').close()
            self.run_for(.2)
            self.assertEqual(len(pauses), 1)
            self.assertEqual(late, [])
            # Long slow timer proves retained pause changes are forwarded as events.
            self.relay.set_parameters([Parameter('slow_rate_hz', value=.01)])
            goal = GoalStatus(pause_active=True, active_valid=True, active_phase=GoalStatus.MOVING,
                              active_request_sequence=1, revision=1)
            self.goal_pub.publish(goal)
            self.run_for(.1)
            self.assertTrue(self.ui.snapshot()['topics']['goal_status']['data']['pause_active'])
            goal.active_enu.x = 3.
            goal.revision = 2
            self.goal_pub.publish(goal)
            self.run_for(.1)
            self.assertEqual(self.ui.snapshot()['topics']['goal_status']['data']['active_enu']['x'], 3.)
            original = self.ui.pause_publisher
            class BrokenPublisher:
                def __init__(self, exception):
                    self.exception = exception
                def publish(self, message):
                    raise self.exception('synthetic failure')
            try:
                for exception in (RuntimeError, TimeoutError):
                    self.ui.pause_publisher = BrokenPublisher(exception)
                    with self.assertRaises(urllib.error.HTTPError) as error:
                        post({})
                    self.assertEqual(error.exception.code, 503)
                    self.assertIn('未确认', json.load(error.exception)['error'])
            finally:
                self.ui.pause_publisher = original
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

    def test_late_snapshot_http_validation_and_no_replay(self):
        state, origin = mapping()
        goal = GoalStatus(clock_id=state.clock_id, control_session_start=25., request_sequence=7,
                          request_state=GoalStatus.REJECTED, request_reason='B rejected', active_valid=True,
                          active_request_sequence=6, active_phase=GoalStatus.STABLE,
                          active_fc_session=state.navigation_session, active_fc_reset_counter=state.fc_reset_counter,
                          active_generation=state.reset_counter)
        computation_pub = self.source.create_publisher(ComputationStatus, 'computation_status', relay.LATEST)
        self.origin_pub.publish(origin)
        self.goal_pub.publish(goal)
        self.run_for(.2)
        # Late UI subscribes after retained source and relay publications.
        self.executor.remove_node(self.ui)
        self.ui.destroy_node()
        self.ui = map_ui.MapUI()
        self.executor.add_node(self.ui)
        self.run_for(.4)
        self.pubs['fused_state'].publish(state)
        computation_pub.publish(ComputationStatus(control_session_start=25.))
        self.run_for(.2)
        snap = self.ui.snapshot()
        self.assertEqual(snap['topics']['goal_status']['data']['request_sequence'], 7)
        self.assertIsNotNone(snap['active_position'])
        self.assertEqual(snap['active_position']['altitude_msl'], 0.)
        server = map_ui.make_server(self.ui, map_ui.MBTiles(None), '127.0.0.1', 0)
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        base = f'http://127.0.0.1:{server.server_port}'
        try:
            for _ in range(4):
                with urllib.request.urlopen(base + '/api/state') as response:
                    self.assertEqual(json.load(response)['topics']['goal_status']['data']['active_request_sequence'], 6)
            with urllib.request.urlopen(base + '/') as response:
                self.assertIn(b'/leaflet/leaflet.js', response.read())
            for bad in [{'latitude': math.nan, 'longitude': 0., 'altitude_msl': 0.},
                        {'latitude': True, 'longitude': 0., 'altitude_msl': 0.},
                        {'latitude': 91., 'longitude': 0., 'altitude_msl': 0.}, {'extra': 3}]:
                request = urllib.request.Request(base + '/api/goal', json.dumps(bad).encode(), {'Content-Type': 'application/json'})
                with self.assertRaises(urllib.error.HTTPError) as error:
                    urllib.request.urlopen(request)
                self.assertEqual(error.exception.code, 400)
            payload = dict(latitude=0., longitude=0., altitude_msl=0.)
            request = urllib.request.Request(base + '/api/goal', json.dumps(payload).encode(), {'Content-Type': 'application/json'})
            with urllib.request.urlopen(request) as response:
                self.assertEqual(response.status, 202)
                self.assertEqual(json.load(response)['status'], 'sent')
            self.run_for(.2)
            self.assertEqual(len(self.commands), 1)
            # New late command subscriber must not receive a retained command.
            late_commands = []
            self.source.create_subscription(GeographicGoal, 'go_to_geographic_position', late_commands.append, 1)
            for _ in range(3):
                urllib.request.urlopen(base + '/api/state').close()
            self.run_for(.3)
            self.assertEqual(len(self.commands), 1)
            self.assertEqual(late_commands, [])
        finally:
            server.shutdown()
            server.server_close()
            thread.join()
        self.run_for(.1)
        self.assertEqual(len(self.commands), 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
