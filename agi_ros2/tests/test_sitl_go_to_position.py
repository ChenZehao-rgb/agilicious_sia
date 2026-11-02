#!/usr/bin/env python3
"""Explicit README SITL workflow using the real sim_rc and ros2 topic pub CLI.

Source ROS and install/agi_ros2/local_setup.bash first. Run only with unused
SITL ports and an isolated ROS_DOMAIN_ID/GZ_PARTITION. Never opens hardware.
The default retains ROS EKF. --state-source fc_fused --yaw-chain also validates
signed yaw pulses and three consecutive goals with the real FC estimator.
All timing gates remain on; CSV, rosbag and result.json are retained.
"""
import argparse
import socket
import urllib.request
import csv
import json
import math
import os
from pathlib import Path
import re
import signal
import shlex
import subprocess
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import SetParameters
from agi_ros2.msg import (NavigationOrigin, FcEstimatedState, FcEstimatorStatus, Authority,
                          ComputationStatus, ControlCommand, FusedState, OutputStatus, GoalStatus)
from nav_msgs.msg import Odometry


ROOT = Path(__file__).resolve().parents[2]
TARGET = (5.0, 0.0, 2.0)


def xyz(value):
    return [value.x, value.y, value.z]


def norm(value):
    return math.sqrt(sum(x*x for x in value))


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def angle_error(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--state-source', choices=['ros_ekf', 'fc_fused'], default='ros_ekf')
    parser.add_argument('--yaw-chain', action='store_true')
    parser.add_argument('--geographic', action='store_true', help='Two WGS84/MSL goals from the actual FC origin')
    parser.add_argument('--map-ui', action='store_true', help='Send geographic goals through the local UI HTTP server')
    parser.add_argument('--pause-hover', action='store_true', help='Browser goal/pause clicks in translation and yaw; requires Playwright/Chrome')
    parser.add_argument('--runtime-tuning', action='store_true', help='Apply trajectory/MPC patches during flight through HTTP')
    args = parser.parse_args()
    if args.runtime_tuning and not (args.map_ui and args.geographic and args.state_source == 'fc_fused'):
        parser.error('--runtime-tuning requires --map-ui --geographic --state-source fc_fused')
    if args.pause_hover and not (args.map_ui and args.geographic):
        parser.error('--pause-hover requires --map-ui --geographic --state-source fc_fused')
    if args.map_ui and not args.geographic:
        parser.error('--map-ui requires --geographic')
    with socket.socket() as port_socket:
        port_socket.bind(('127.0.0.1', 0))
        ui_port = port_socket.getsockname()[1]
    ui_url = f'http://127.0.0.1:{ui_port}'

    def http(path, payload=None):
        request = urllib.request.Request(ui_url + path, None if payload is None else json.dumps(payload).encode(),
                                         {'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=6) as response:
            return json.load(response)

    fc_state = args.state_source == 'fc_fused'
    if args.geographic and not fc_state:
        parser.error('--geographic requires --state-source fc_fused')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    config = (ROOT / 'agi_ros2/config/simulation.yaml').read_text()
    config, count = re.subn(r'(?m)^  trajectory:.*$', "  trajectory: ''", config)
    assert count == 1
    # Preserve this ROS EKF/truth-time regression when the default profile selects FC state.
    config = re.sub(r"(?m)^  source: (fc_fused|ros_ekf)$", "  source: " + args.state_source, config)
    runtime = out / 'simulation.yaml'
    runtime.write_text(config)
    rclpy.init()
    node = rclpy.create_node('online_goal_validation', parameter_overrides=[Parameter('use_sim_time', value=not fc_state)])
    latest, rows, events, processes, streams, subscriptions = {}, {}, [], [], [], []
    result = {'passed': False, 'state_source': args.state_source, 'yaw_chain': args.yaw_chain,
              'geographic': args.geographic, 'map_ui': args.map_ui, 'pause_hover': args.pause_hover, 'runtime_config': str(runtime), 'goals': [], 'yaw_pulses': []}
    truth_offset = [0., 0., 0.]

    def sim_time():
        return node.get_clock().now().nanoseconds * 1e-9

    def event(kind, **values):
        row = dict(event=kind, sim_time=sim_time(), wall=time.monotonic(), **values)
        events.append(row)
        print(json.dumps(row), flush=True)

    def receive(topic, msg):
        latest[topic] = msg
        row = dict(stamp=msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9, wall=time.monotonic())
        if topic == 'navigation/origin':
            for key in ('session_id', 'latitude', 'longitude', 'altitude', 'altitude_reference',
                        'xy_valid', 'z_valid', 'longitude_scale', 'fc_reset_counter', 'fc_time_usec'):
                row[key] = getattr(msg, key)
        elif topic == 'goal_status':
            for key in ('request_sequence', 'request_state', 'request_reason', 'active_valid',
                        'active_request_sequence', 'active_phase', 'active_reason', 'pause_active'):
                row[key] = getattr(msg, key)
            row['active_enu'] = xyz(msg.active_enu)
        elif topic.startswith('sensors/fc/'):
            row.update(fc_time_usec=msg.fc_time_usec, valid=msg.valid)
        elif topic in ('reference', 'ground_truth'):
            row.update(position=xyz(msg.pose.pose.position), velocity=xyz(msg.twist.twist.linear),
                       yaw=yaw(msg.pose.pose.orientation), rates=xyz(msg.twist.twist.angular))
        elif topic == 'fused_state':
            row.update(position=xyz(msg.position), velocity=xyz(msg.velocity), ready=msg.estimator_ready,
                       reason=msg.readiness_reason, navigation_rejections=msg.navigation_rejections,
                       yaw=yaw(msg.orientation), rates=xyz(msg.body_rates), fc_flags=msg.fc_flags)
        elif topic == 'computation_status':
            for key in ('warm_cycles', 'trajectory_active', 'hover_stable', 'reference_elapsed', 'solve_seconds', 'cycle_seconds',
                        'state_age', 'navigation_age', 'imu_age', 'reason', 'controller_type', 'controller_success',
                        'control_session_start', 'warm_resets'):
                row[key] = getattr(msg, key)
        elif topic == 'control_command':
            row.update(permit_override=msg.permit_override, sequence=msg.sequence, total_thrust=msg.total_thrust,
                       body_rates=xyz(msg.body_rates), mode=msg.mode, evidence_now=msg.evidence.now,
                       imu_time=msg.evidence.imu_time, imu_sample_time=msg.evidence.imu_sample_time,
                       control_start_steady_time=msg.control_start_steady_time)
        elif topic == 'output_status':
            for key in ('override_active', 'reason', 'last_fault', 'fault_count', 'command_age', 'write_seconds'):
                row[key] = getattr(msg, key)
        else:
            row.update(armed=msg.armed, auto_switch=msg.auto_switch, kill=msg.kill,
                       manual_aetr=[int(x) for x in msg.manual_aetr])
        rows[topic].append(row)

    for topic, cls in [('fused_state', FusedState), ('reference', Odometry), ('ground_truth', Odometry),
                       ('computation_status', ComputationStatus), ('control_command', ControlCommand),
                       ('output_status', OutputStatus), ('authority', Authority)]:
        rows[topic] = []
        subscriptions.append(node.create_subscription(cls, topic, lambda m, t=topic: receive(t, m), qos_profile_sensor_data))
    for topic, cls in [('goal_status', GoalStatus), ('navigation/origin', NavigationOrigin), ('sensors/fc/estimated_state', FcEstimatedState),
                       ('sensors/fc/estimator_status', FcEstimatorStatus)]:
        rows[topic] = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic in ('navigation/origin', 'goal_status') else qos_profile_sensor_data
        subscriptions.append(node.create_subscription(cls, topic, lambda m, t=topic: receive(t, m), qos))

    client = node.create_client(SetParameters, '/sim_rc/set_parameters')

    def spin():
        rclpy.spin_once(node, timeout_sec=.002)

    def wait_for(predicate, timeout, label):
        start, report = time.monotonic(), time.monotonic()
        while time.monotonic() - start < timeout:
            spin()
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('runtime exited: ' + str([p.poll() for p in processes]))
            if predicate():
                return
            if time.monotonic() - report > 10:
                c, f = latest.get('computation_status'), latest.get('fused_state')
                event('waiting', phase=label, reason=c.reason if c else None,
                      position=xyz(f.position) if f else None, velocity=xyz(f.velocity) if f else None,
                      fc_yaw_deg=math.degrees(yaw(f.orientation)) if f else None,
                      truth_yaw_deg=math.degrees(yaw(latest['ground_truth'].pose.pose.orientation))
                      if 'ground_truth' in latest else None)
                report = time.monotonic()
        raise RuntimeError('timeout: ' + label)

    def delay(seconds):
        end = sim_time() + seconds
        wait_for(lambda: sim_time() >= end, max(20, seconds*5), 'delay')

    def param(name, value):
        request = SetParameters.Request(parameters=[Parameter(name, value=value).to_parameter_msg()])
        future = client.call_async(request)
        wait_for(future.done, 10, 'set ' + name)
        assert all(r.successful for r in future.result().results), future.result()
        event('parameter', name=name, value=value)

    def active():
        c, o = latest.get('computation_status'), latest.get('output_status')
        return c and o and c.trajectory_active and o.override_active

    def stable(target=None):
        f, ref = latest.get('fused_state'), latest.get('reference')
        if not active() or f is None or ref is None:
            return False
        expected = target if target is not None else xyz(ref.pose.pose.position)
        estimator_stable = norm(xyz(f.velocity)) <= .2 and norm(
            a-b for a, b in zip(xyz(f.position), expected)) <= .2
        if target is None:
            return estimator_stable
        truth = latest['ground_truth']
        actual_target = [a+b for a,b in zip(target, truth_offset)]
        return estimator_stable and norm(xyz(truth.twist.twist.linear)) <= .2 and norm(
            a-b for a,b in zip(xyz(truth.pose.pose.position), actual_target)) <= .2

    def stable_window(target, seconds, timeout, require_active=False):
        since = None
        def check():
            nonlocal since
            if require_active and not active():
                raise RuntimeError('AUTO revoked: ' + latest['computation_status'].reason + '; output: ' +
                                   latest['output_status'].reason)
            if stable(target):
                if since is None:
                    since = sim_time()
                return sim_time() - since >= seconds
            since = None
            return False
        wait_for(check, timeout, 'stable hover')

    try:
        commands = [
            ('sim', ['python3', 'betaflight_sitl/run.py', '--no-gazebo', '--no-build',
                     '--runtime-config', str(runtime), '--duration', '420']),
            ('flight', ['./agi_ros2/scripts/launch.sh', 'runtime_config:=' + str(runtime),
                        'bag_output:=' + str(out / 'bag')]),
            ('sim_rc', ['ros2', 'run', 'agi_ros2', 'sim_rc.py', '--ros-args', '-p', 'use_sim_time:=' + str(not fc_state).lower()])]
        if args.map_ui:
            commands += [('relay', ['ros2', 'run', 'agi_ros2', 'ground_station_telemetry.py']),
                         ('map_ui', ['ros2', 'run', 'agi_ros2', 'map_ui.py', '--port', str(ui_port)])]
        for name, command in commands:
            event('launch', name=name, command=command)
            stream = (out / (name + '.log')).open('w')
            streams.append(stream)
            processes.append(subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        wait_for(client.service_is_ready, 30, 'sim_rc service')
        param('kill', False)
        wait_for(lambda: 'ground_truth' in latest, 60, 'Gazebo started')
        delay(10)
        if fc_state:
            # FC horizontal fusion becomes valid after arming.
            param('armed', True)
        wait_for(lambda: 'computation_status' in latest and latest['computation_status'].warm_cycles == 50,
                 90, 'AUTO_STANDBY warmup')
        event('warm_ready')
        if not fc_state:
            param('armed', True)
        delay(2)
        param('throttle', 1420)
        wait_for(lambda: latest['fused_state'].position.z >= 2., 40, 'manual takeoff')
        if args.yaw_chain:
            for rc_yaw, expected_sign in [(1460, 1), (1540, -1)]:
                start_yaw = yaw(latest['ground_truth'].pose.pose.orientation)
                param('yaw', rc_yaw)
                delay(1.5)
                delta = angle_error(yaw(latest['ground_truth'].pose.pose.orientation), start_yaw)
                summary = dict(rc_yaw=rc_yaw, truth_yaw_delta=delta)
                for topic in ('ground_truth', 'fused_state'):
                    end_wall = time.monotonic()
                    samples = [r for r in rows[topic] if end_wall-.5 <= r['wall'] <= end_wall]
                    assert samples
                    mean_rate = sum(r['rates'][2] for r in samples)/len(samples)
                    summary[topic + '_mean_wz'] = mean_rate
                    assert expected_sign*mean_rate > .02, summary
                assert expected_sign*delta > .02, summary
                result['yaw_pulses'].append(summary)
                event('yaw_pulse_verified', **summary)
            param('yaw', 1500)
        wait_for(lambda: latest['fused_state'].position.z >= (8.5 if fc_state else 2.), 40, 'takeoff height')
        param('auto_switch', True)
        wait_for(active, 5, 'AUTO capture')
        event('auto_active', position=xyz(latest['fused_state'].position))
        auto_start = time.monotonic()
        stable_window(None, 30 if args.yaw_chain else 1, 75, require_active=True)
        wait_for(lambda: latest['computation_status'].hover_stable, 5, 'initial AUTO hover feedback')
        if args.map_ui:
            def initial_hover_visible():
                snapshot = http('/api/state')
                computation = snapshot['topics'].get('computation_status', {}).get('data', {})
                goal = snapshot['topics'].get('goal_status', {}).get('data', {})
                return computation.get('hover_stable') and goal.get('active_valid') is False
            wait_for(initial_hover_visible, 5, 'initial AUTO hover HTTP feedback without an online goal')
        event('pre_goal_stable', position=xyz(latest['fused_state'].position),
              reference=xyz(latest['reference'].pose.pose.position),
              hover_stable=latest['computation_status'].hover_stable)
        if fc_state:
            truth_offset = [a-b for a,b in zip(xyz(latest['ground_truth'].pose.pose.position),
                                              xyz(latest['fused_state'].position))]
        result['truth_odom_offset'] = truth_offset
        targets = [TARGET, (5., 5., 2.), (0., 0., 1.)] if args.yaw_chain else [TARGET]
        geographic_goals = []
        if args.geographic:
            def origin_ready():
                o, f = latest.get('navigation/origin'), latest['fused_state']
                return o and o.xy_valid and o.z_valid and o.session_id == f.navigation_session and \
                    o.fc_reset_counter == f.fc_reset_counter and o.fc_time_usec <= f.fc_time_usec
            wait_for(origin_ready, 5, 'current FC geographic origin')
            origin = latest['navigation/origin']
            result['origin'] = rows['navigation/origin'][-1]
            # Independent native-projection anchors (degE7 deltas), not an inverse
            # call to the new converter and not Gazebo's world origin.
            for lat_units, lon_units in [(100, 500), (500, -200)]:
                geographic_goals.append(dict(latitude=origin.latitude+lat_units*1e-7,
                    longitude=origin.longitude+lon_units*1e-7, altitude_msl=origin.altitude+2.))
            targets = [(500*1.1131949424743652*.01*origin.longitude_scale, 100*1.1131949424743652*.01, 2.),
                       (-200*1.1131949424743652*.01*origin.longitude_scale, 500*1.1131949424743652*.01, 2.)]
        if args.runtime_tuning:
            targets = [(16., 0., 2.), (28., 0., 2.)]
            geographic_goals = [dict(latitude=origin.latitude+y/111319.49424743652,
                                    longitude=origin.longitude+x/(111319.49424743652*origin.longitude_scale),
                                    altitude_msl=origin.altitude+z) for x,y,z in targets]
            initial_tuning = http('/api/tuning')
            assert initial_tuning['controller_type'] == 'MPC'
            result['runtime_tuning'] = dict(initial=initial_tuning)
        if args.pause_hover:
            from playwright.sync_api import sync_playwright
            with sync_playwright() as playwright:
                browser = playwright.chromium.launch(executable_path='/opt/google/chrome/chrome', headless=True,
                                                       args=['--no-sandbox'])
                page = browser.new_page(viewport={'width': 1440, 'height': 960})
                # Test exercises the UI; map downloads/external resources are irrelevant.
                page.route('**/*', lambda route: route.continue_() if route.request.url.startswith(ui_url) else route.abort())
                browser_commands = []
                page.on('request', lambda request: browser_commands.append(request.url)
                        if request.method == 'POST' else None)
                page.goto(ui_url)
                def click_goal(point):
                    geo = dict(latitude=origin.latitude+point[1]/111319.49424743652,
                               longitude=origin.longitude+point[0]/(111319.49424743652*origin.longitude_scale),
                               altitude_msl=origin.altitude+point[2])
                    for field, key in [('latitude','latitude'), ('longitude','longitude'), ('altitude','altitude_msl')]:
                        page.locator('#'+field).fill(str(geo[key]))
                    sequence = latest['goal_status'].request_sequence
                    page.locator('#execute').click()
                    wait_for(lambda: latest['goal_status'].request_sequence > sequence and
                             latest['goal_status'].request_state == GoalStatus.ACCEPTED, 5, 'browser goal accepted')
                    assert not latest['goal_status'].pause_active
                    return geo
                pause_results = []
                for mode in ('translation', 'yaw'):
                    start = xyz(latest['reference'].pose.pose.position)
                    heading = yaw(latest['reference'].pose.pose.orientation)
                    direction = heading + (1.5 if mode == 'yaw' else 0.)
                    target = [start[0]+12*math.cos(direction), start[1]+12*math.sin(direction), start[2]]
                    goal_time = time.monotonic()
                    geo = click_goal(target)
                    if mode == 'translation':
                        wait_for(lambda: norm(xyz(latest['reference'].twist.twist.linear)) > .6 and
                                 norm(xyz(latest['fused_state'].velocity)) > .4, 30, 'translation before pause')
                    else:
                        wait_for(lambda: abs(latest['reference'].twist.twist.angular.z) > .2, 20, 'yaw before pause')
                    before_speed = norm(xyz(latest['fused_state'].velocity))
                    before_rate = latest['reference'].twist.twist.angular.z
                    before_position = xyz(latest['reference'].pose.pose.position)
                    pause_sequence = latest['goal_status'].request_sequence + 1
                    pause_time = time.monotonic()
                    page.locator('#pause').click()
                    wait_for(lambda: latest['goal_status'].request_sequence == pause_sequence and
                             latest['goal_status'].request_state == GoalStatus.ACCEPTED and
                             latest['goal_status'].pause_active, 5, 'browser pause accepted')
                    stopped = xyz(latest['goal_status'].active_enu)
                    assert latest['goal_status'].active_phase == GoalStatus.MOVING
                    if mode == 'translation':
                        assert norm(a-b for a,b in zip(stopped, before_position)) > .1
                    stable_window(stopped, 3, 30, require_active=True)
                    wait_for(lambda: latest['goal_status'].active_phase == GoalStatus.STABLE, 3, 'pause stable phase')
                    # Hold past the original goal's yaw+translation completion time.
                    original_duration = 12 + (1.875*1.5/.5 if mode == 'yaw' else 1.5)
                    delay(max(3, goal_time + original_duration + 2 - time.monotonic()))
                    assert latest['goal_status'].pause_active and stable(stopped)
                    assert norm(a-b for a,b in zip(xyz(latest['reference'].pose.pose.position), stopped)) < 1e-6
                    assert norm(xyz(latest['reference'].twist.twist.linear)) == 0
                    assert norm(xyz(latest['reference'].twist.twist.angular)) == 0
                    for field, key in [('latitude','latitude'), ('longitude','longitude'), ('altitude','altitude_msl')]:
                        assert float(page.locator('#'+field).input_value()) == geo[key]
                    # Refresh displays retained pause feedback without publishing a command.
                    count = len(browser_commands)
                    page.reload()
                    page.wait_for_function("document.getElementById('phase').textContent === '已暂停，稳定悬停'")
                    assert len(browser_commands) == count
                    page.screenshot(path=str(out / ('pause_'+mode+'.png')), full_page=True)
                    end_wall = time.monotonic()
                    summary = dict(mode=mode, pause_wall=pause_time, stop=stopped, click_reference=before_position,
                                   speed_before=before_speed, yaw_rate_before=before_rate,
                                   speed_after=norm(xyz(latest['fused_state'].velocity)),
                                   original_goal_end_waited=True)
                    for topic in ('fused_state', 'ground_truth'):
                        window = [r for r in rows[topic] if end_wall-2 <= r['wall'] <= end_wall]
                        expected = [a+b for a,b in zip(stopped,truth_offset)] if topic == 'ground_truth' else stopped
                        summary[topic] = dict(position_error_max=max(norm(a-b for a,b in zip(r['position'], expected)) for r in window),
                                              speed_max=max(norm(r['velocity']) for r in window))
                        assert summary[topic]['position_error_max'] <= .2 and summary[topic]['speed_max'] <= .2
                    event('browser_pause_verified', **summary)
                    pause_results.append(summary)
                    # An explicit new UI goal in the same AUTO session must work.
                    next_target = [stopped[0]+.5*math.cos(direction), stopped[1]+.5*math.sin(direction), stopped[2]]
                    click_goal(next_target)
                    stable_window(next_target, 2, 40, require_active=True)
                assert browser_commands.count(ui_url+'/api/pause') == 2
                assert browser_commands.count(ui_url+'/api/goal') == 4
                result['pauses'] = pause_results
                result['browser_commands'] = browser_commands
                browser.close()
        for index, target in enumerate(targets):
            start_ref = xyz(latest['reference'].pose.pose.position)
            start_heading = yaw(latest['reference'].pose.pose.orientation)
            horizontal = [target[i]-start_ref[i] for i in range(2)]
            end_heading = math.atan2(horizontal[1], horizontal[0]) if norm(horizontal) > 1e-6 else start_heading
            goal_start = time.monotonic()
            previous_sequence = latest['goal_status'].request_sequence
            goal_stream = (out / f'goal_{index}_cli.log').open('w')
            streams.append(goal_stream)
            goal = '{header: {frame_id: odom}, point: {x: %s, y: %s, z: %s}}' % target
            # Both recorder and controller must be discovered before --once.
            goal_command = ['ros2', 'topic', 'pub', '--once', '-w', '2', '/go_to_position',
                            'geometry_msgs/msg/PointStamped', goal]
            if args.geographic:
                goal = json.dumps(geographic_goals[index])
                goal_command = ['ros2', 'topic', 'pub', '--once', '-w', '2', '/go_to_geographic_position',
                                'agi_ros2/msg/GeographicGoal', goal]
            if args.map_ui:
                response = http('/api/goal', geographic_goals[index])
                assert response['status'] == 'sent', response
                event('goal_http', target=geographic_goals[index], expected_enu=target, response=response)
            else:
                event('goal_cli', command=goal_command, expected_enu=target)
                result.setdefault('publish_commands', []).append(shlex.join(goal_command))
                cli_env = os.environ.copy()
                # Avoid NumPy worker startup disturbing FC state freshness.
                cli_env.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
                cli = subprocess.Popen(goal_command, stdout=goal_stream, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=cli_env)
                try:
                    wait_for(lambda: cli.poll() is not None, 15, 'goal CLI')
                    assert cli.returncode == 0
                finally:
                    if cli.poll() is None:
                        os.killpg(cli.pid, signal.SIGINT)
                        cli.wait(timeout=5)
            # Acceptance is a retained control-owned state. A single best-effort
            # computation event can be lost while the HTTP caller is blocked.
            def goal_accepted():
                status = latest['goal_status']
                if status.request_sequence <= previous_sequence:
                    return False
                if status.request_state == GoalStatus.REJECTED:
                    raise RuntimeError('goal rejected: ' + status.request_reason)
                return status.request_state == GoalStatus.ACCEPTED and status.active_valid
            wait_for(goal_accepted, 5, 'retained goal acceptance')
            accepted = next(r for r in reversed(rows['goal_status']) if r['request_sequence'] > previous_sequence
                            and r['request_state'] == GoalStatus.ACCEPTED)
            event('goal_accepted', target=target, stamp=accepted['stamp'])
            accepted_sequence = latest['goal_status'].active_request_sequence
            if args.map_ui:
                # A valid geographic request outside the existing fence must not erase A.
                rejected = dict(geographic_goals[index], altitude_msl=geographic_goals[index]['altitude_msl'] + 10000.)
                http('/api/goal', rejected)
                wait_for(lambda: latest['goal_status'].request_state == GoalStatus.REJECTED, 3, 'B rejection')
                assert latest['goal_status'].active_request_sequence == accepted_sequence
                assert latest['goal_status'].active_valid
                event('rejected_B_preserved_A', active_sequence=accepted_sequence)
            if args.runtime_tuning and index == 0:
                wait_for(lambda: norm(xyz(latest['reference'].twist.twist.linear)) > .3, 25, 'A moving before tuning')
                before = http('/api/tuning')
                tuning_wall = time.monotonic()
                trajectory_result = http('/api/tuning', dict(group='trajectory', values={'go_to_pose_max_vel':1.2,
                                                                                       'go_to_pose_max_acc':.8}))
                assert trajectory_result['status'] == 'applied', trajectory_result
                mpc_result = http('/api/tuning', dict(group='mpc', values={'Q_pos_x':before['mpc']['Q_pos_x']*1.02}))
                assert mpc_result['status'] == 'applied', mpc_result
                assert mpc_result['current']['control_session_start'] == before['control_session_start']
                assert mpc_result['current']['mpc']['Q_pos_x'] == before['mpc']['Q_pos_x']*1.02
                assert mpc_result['current']['trajectory'] == {'go_to_pose_max_vel':1.2,'go_to_pose_max_acc':.8}
                event('runtime_parameters_applied_while_moving', trajectory=trajectory_result, mpc=mpc_result,
                      active_sequence=accepted_sequence)
                delay(.5)
                assert latest['goal_status'].active_request_sequence == accepted_sequence
                assert latest['computation_status'].trajectory_active and latest['computation_status'].warm_cycles == 50
                window = [r for r in rows['computation_status'] if tuning_wall-.2 <= r['wall'] <= tuning_wall+.5]
                commands_window = [r for r in rows['control_command'] if tuning_wall-.2 <= r['control_start_steady_time'] <= tuning_wall+.5]
                assert window and commands_window
                assert all(r['controller_success'] and r['trajectory_active'] for r in window)
                assert all(r['permit_override'] for r in commands_window)
                times = [r['control_start_steady_time'] for r in commands_window]
                result['runtime_tuning'].update(applied_wall=tuning_wall, trajectory=trajectory_result, mpc=mpc_result,
                    cycle_interval_max_s=max(b-a for a,b in zip(times,times[1:])),
                    solve_max_s=max(r['solve_seconds'] for r in window),
                    cycle_max_s=max(r['cycle_seconds'] for r in window),
                    warm_resets_before=window[0]['warm_resets'], warm_resets_after=window[-1]['warm_resets'])
            stable_window(target, 15 if args.yaw_chain or args.geographic else 3, 90, require_active=True)
            wait_for(lambda: latest['goal_status'].active_phase == GoalStatus.STABLE, 3, 'structured stable phase')
            if args.map_ui:
                delay(.2)
                snapshot = http('/api/state')
                sequence = snapshot['topics']['goal_status']['data']['request_sequence']
                assert snapshot['active_position'] is not None, snapshot
                for _ in range(3):
                    assert http('/api/state')['topics']['goal_status']['data']['request_sequence'] == sequence
                observed = {r['active_phase'] for r in rows['goal_status']
                            if r['active_request_sequence'] == accepted_sequence}
                # If tracking is already stable at the segment end, no settling tick is required.
                assert {GoalStatus.MOVING, GoalStatus.STABLE} <= observed, observed
                event('ui_snapshot_restored', snapshot=snapshot, phases=sorted(observed))

            event('target_stable', target=target, position=xyz(latest['fused_state'].position),
                  truth=xyz(latest['ground_truth'].pose.pose.position))
            end_wall = events[-1]['wall']
            if args.geographic:
                assert norm(a-b for a,b in zip(xyz(latest['reference'].pose.pose.position), target)) < 1e-6
            else:
                assert xyz(latest['reference'].pose.pose.position) == list(target)
            goal_result = dict(target=target, accepted=accepted)
            reference_rows = [r for r in rows['reference'] if accepted['stamp']+.02 <= r['stamp']
                              and goal_start <= r['wall'] <= end_wall]
            assert reference_rows
            if args.runtime_tuning:
                peak = max(norm(r['velocity']) for r in reference_rows)
                expected_peak = initial_tuning['trajectory']['go_to_pose_max_vel'] if index == 0 else 1.2
                assert abs(peak-expected_peak) < .001, (peak, expected_peak)
                goal_result['reference_peak_speed'] = peak
                goal_result['trajectory_parameters_used'] = 'original A' if index == 0 else 'updated B'
            turn_rows = [r for r in reference_rows if abs(angle_error(r['yaw'], end_heading)) > 1e-4]
            assert all(norm(a-b for a,b in zip(r['position'], start_ref)) < 1e-6 for r in turn_rows), \
                'translation started before reference turn finished'
            assert all(norm(r['velocity']) < 1e-6 for r in turn_rows), 'nonzero translation during turn'
            assert max(abs(r['rates'][2]) for r in reference_rows) <= .500001, 'yaw reference exceeds configured rate'
            goal_result['heading'] = dict(start_rad=start_heading, end_rad=end_heading, turn_samples=len(turn_rows),
                                          reference_yaw_rate_max=max(abs(r['rates'][2]) for r in reference_rows))
            for topic in ('fused_state', 'ground_truth'):
                # Truth has simulation stamps, FC has host stamps: use common receive time.
                window = [x for x in rows[topic] if end_wall-3 <= x['wall'] <= end_wall]
                expected = [a+b for a,b in zip(target, truth_offset)] if topic == 'ground_truth' else target
                errors = [norm(a-b for a,b in zip(x['position'], expected)) for x in window]
                speeds = [norm(x['velocity']) for x in window]
                heading_errors = [abs(angle_error(x['yaw'], end_heading)) for x in window]
                assert window
                goal_result[topic] = dict(position_error_max=max(errors), speed_max=max(speeds),
                                         heading_error_max_deg=math.degrees(max(heading_errors)))
                assert max(errors) <= .2 and max(speeds) <= .2, topic + ' not stably at target'
                assert max(heading_errors) <= math.radians(3), topic + ' not holding terminal heading'
            result['goals'].append(goal_result)
        end_wall = time.monotonic()
        active_rows = [x for x in rows['computation_status'] if auto_start+.1 <= x['wall'] <= end_wall]
        output_rows = [x for x in rows['output_status'] if auto_start+.1 <= x['wall'] <= end_wall]
        assert active_rows and all(x['trajectory_active'] and x['controller_success'] for x in active_rows)
        assert output_rows and all(x['override_active'] for x in output_rows)
        result['auto_seconds'] = end_wall-auto_start
        result['max_solve_seconds'] = max(x['solve_seconds'] for x in active_rows)
        result['max_cycle_seconds'] = max(x['cycle_seconds'] for x in active_rows)
        if args.yaw_chain:
            # Match by receive time only across the two clock domains; reject loose pairs.
            truth = rows['ground_truth']
            j, errors, reference_errors = 0, [], []
            heading = rows['reference'][-1]['yaw']
            for state in rows['fused_state']:
                if state['wall'] < auto_start or state['wall'] > end_wall:
                    continue
                while j+1 < len(truth) and abs(truth[j+1]['wall']-state['wall']) < abs(truth[j]['wall']-state['wall']):
                    j += 1
                if abs(truth[j]['wall']-state['wall']) < .025:
                    errors.append(abs(angle_error(state['yaw'], truth[j]['yaw'])))
                if state['wall'] > end_wall-10:
                    reference_errors.append(abs(angle_error(state['yaw'], heading)))
            assert len(errors) > .9*len(active_rows), 'insufficient synchronized yaw evidence'
            result['yaw_error_max_deg'] = math.degrees(max(errors))
            result['final_heading_error_max_deg'] = math.degrees(max(reference_errors))
            assert max(errors) < math.radians(3), result
            assert max(reference_errors) < math.radians(3), result
            if fc_state:
                flags = [state['fc_flags'] for state in rows['fused_state']
                         if auto_start+.1 <= state['wall'] <= end_wall]
                result['fc_flags'] = sorted(set(flags))
                assert flags and all(flag & 12 == 12 for flag in flags), 'FC magnetic heading not valid/fresh'
        if fc_state:
            result['fc_stream_rates_hz'] = {}
            for topic in ('sensors/fc/estimated_state', 'sensors/fc/estimator_status'):
                samples = [r for r in rows[topic] if auto_start+.1 <= r['wall'] <= end_wall and r['valid']]
                assert len(samples) > 2
                result['fc_stream_rates_hz'][topic] = (len(samples)-1)*1e6/(samples[-1]['fc_time_usec']-samples[0]['fc_time_usec'])
            result['fc_state_age_max'] = max(x['state_age'] for x in active_rows)
        param('kill', True)
        wait_for(lambda: not latest['output_status'].override_active, 5, 'KILL output revoked')
        wait_for(lambda: not latest['goal_status'].active_valid, 3, 'goal cleared on KILL')
        event('kill_verified')
        result['passed'] = True
    except Exception as exc:
        result['failure'] = str(exc)
        event('failure', reason=str(exc))
    finally:
        if client.service_is_ready():
            try:
                param('kill', True)
                delay(.2)
            except Exception:
                pass
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=10)
        for stream in streams:
            stream.close()
        for topic, data in rows.items():
            if data:
                with (out / (topic.replace('/', '_') + '.csv')).open('w') as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(data[0]))
                    writer.writeheader()
                    writer.writerows({k: json.dumps(v) if isinstance(v, list) else v for k,v in row.items()} for row in data)
        result['events'] = events
        result['counts'] = {key: len(value) for key, value in rows.items()}
        (out / 'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
