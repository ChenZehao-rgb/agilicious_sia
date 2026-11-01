#!/usr/bin/env python3
"""Explicit README SITL workflow using the real sim_rc and ros2 topic pub CLI.

Source ROS and install/agi_ros2/local_setup.bash first. Run only with unused
SITL ports and an isolated ROS_DOMAIN_ID/GZ_PARTITION. Never opens hardware.
The default retains ROS EKF. --state-source fc_fused --yaw-chain also validates
signed yaw pulses and three consecutive goals with the real FC estimator.
All timing gates remain on; CSV, rosbag and result.json are retained.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from agi_ros2.msg import Authority, ComputationStatus, ControlCommand, FusedState, OutputStatus
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
    args = parser.parse_args()
    fc_state = args.state_source == 'fc_fused'
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
              'runtime_config': str(runtime), 'goals': [], 'yaw_pulses': []}
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
        if topic in ('reference', 'ground_truth'):
            row.update(position=xyz(msg.pose.pose.position), velocity=xyz(msg.twist.twist.linear),
                       yaw=yaw(msg.pose.pose.orientation), rates=xyz(msg.twist.twist.angular))
        elif topic == 'fused_state':
            row.update(position=xyz(msg.position), velocity=xyz(msg.velocity), ready=msg.estimator_ready,
                       reason=msg.readiness_reason, navigation_rejections=msg.navigation_rejections,
                       yaw=yaw(msg.orientation), rates=xyz(msg.body_rates), fc_flags=msg.fc_flags)
        elif topic == 'computation_status':
            for key in ('warm_cycles', 'trajectory_active', 'reference_elapsed', 'solve_seconds', 'cycle_seconds',
                        'state_age', 'navigation_age', 'imu_age', 'reason', 'controller_type', 'controller_success'):
                row[key] = getattr(msg, key)
        elif topic == 'control_command':
            row.update(permit_override=msg.permit_override, sequence=msg.sequence, total_thrust=msg.total_thrust,
                       body_rates=xyz(msg.body_rates), mode=msg.mode, evidence_now=msg.evidence.now,
                       imu_time=msg.evidence.imu_time, imu_sample_time=msg.evidence.imu_sample_time)
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
        event('pre_goal_stable', position=xyz(latest['fused_state'].position),
              reference=xyz(latest['reference'].pose.pose.position))
        if fc_state:
            truth_offset = [a-b for a,b in zip(xyz(latest['ground_truth'].pose.pose.position),
                                              xyz(latest['fused_state'].position))]
        result['truth_odom_offset'] = truth_offset
        targets = [TARGET, (5., 5., 2.), (0., 0., 1.)] if args.yaw_chain else [TARGET]
        for index, target in enumerate(targets):
            goal_start = time.monotonic()
            goal_stream = (out / f'goal_{index}_cli.log').open('w')
            streams.append(goal_stream)
            goal = '{header: {frame_id: odom}, point: {x: %s, y: %s, z: %s}}' % target
            # Both recorder and controller must be discovered before --once.
            goal_command = ['ros2', 'topic', 'pub', '--once', '-w', '2', '/go_to_position',
                            'geometry_msgs/msg/PointStamped', goal]
            event('goal_cli', command=goal_command)
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
            def accepted_rows():
                return [x for x in rows['computation_status'] if x['wall'] >= goal_start
                        and 'go_to_position accepted:' in x['reason']]
            wait_for(lambda: bool(accepted_rows()), 5, 'goal acceptance')
            accepted = accepted_rows()[0]
            event('goal_accepted', target=target, stamp=accepted['stamp'])
            stable_window(target, 15 if args.yaw_chain else 3, 90, require_active=True)
            event('target_stable', target=target, position=xyz(latest['fused_state'].position),
                  truth=xyz(latest['ground_truth'].pose.pose.position))
            end_wall = events[-1]['wall']
            assert xyz(latest['reference'].pose.pose.position) == list(target)
            goal_result = dict(target=target, accepted=accepted)
            for topic in ('fused_state', 'ground_truth'):
                # Truth has simulation stamps, FC has host stamps: use common receive time.
                window = [x for x in rows[topic] if end_wall-3 <= x['wall'] <= end_wall]
                expected = [a+b for a,b in zip(target, truth_offset)] if topic == 'ground_truth' else target
                errors = [norm(a-b for a,b in zip(x['position'], expected)) for x in window]
                speeds = [norm(x['velocity']) for x in window]
                assert window
                goal_result[topic] = dict(position_error_max=max(errors), speed_max=max(speeds))
                assert max(errors) <= .2 and max(speeds) <= .2, topic + ' not stably at target'
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
        param('kill', True)
        wait_for(lambda: not latest['output_status'].override_active, 5, 'KILL output revoked')
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
                with (out / (topic + '.csv')).open('w') as stream:
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
