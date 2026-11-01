#!/usr/bin/env python3
"""Explicit README SITL workflow using the real sim_rc and ros2 topic pub CLI.

Source ROS and install/agi_ros2/local_setup.bash first. Run only with unused
SITL ports and an isolated ROS_DOMAIN_ID/GZ_PARTITION. Never opens hardware.
This regression retains the ROS EKF sensor path and an empty trajectory; timing gates remain on.
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
GOAL = '{header: {frame_id: odom}, point: {x: 5.0, y: 0.0, z: 2.0}}'


def xyz(value):
    return [value.x, value.y, value.z]


def norm(value):
    return math.sqrt(sum(x*x for x in value))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    config = (ROOT / 'agi_ros2/config/simulation.yaml').read_text()
    config, count = re.subn(r'(?m)^  trajectory:.*$', "  trajectory: ''", config)
    assert count == 1
    # Preserve this ROS EKF/truth-time regression when the default profile selects FC state.
    config = re.sub(r"(?m)^  source: fc_fused$", "  source: ros_ekf", config)
    runtime = out / 'simulation.yaml'
    runtime.write_text(config)
    rclpy.init()
    node = rclpy.create_node('online_goal_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    latest, rows, events, processes, streams, subscriptions = {}, {}, [], [], [], []
    result = {'passed': False, 'target': TARGET, 'runtime_config': str(runtime)}

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
            row.update(position=xyz(msg.pose.pose.position), velocity=xyz(msg.twist.twist.linear))
        elif topic == 'fused_state':
            row.update(position=xyz(msg.position), velocity=xyz(msg.velocity), ready=msg.estimator_ready,
                       reason=msg.readiness_reason, navigation_rejections=msg.navigation_rejections)
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
                      position=xyz(f.position) if f else None, velocity=xyz(f.velocity) if f else None)
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
        return norm(xyz(f.velocity)) <= .2 and norm(
            a-b for a, b in zip(xyz(f.position), expected)) <= .2

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
                     '--runtime-config', str(runtime), '--duration', '240']),
            ('flight', ['./agi_ros2/scripts/launch.sh', 'runtime_config:=' + str(runtime),
                        'bag_output:=' + str(out / 'bag')]),
            ('sim_rc', ['ros2', 'run', 'agi_ros2', 'sim_rc.py', '--ros-args', '-p', 'use_sim_time:=true'])]
        for name, command in commands:
            event('launch', name=name, command=command)
            stream = (out / (name + '.log')).open('w')
            streams.append(stream)
            processes.append(subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        wait_for(client.service_is_ready, 30, 'sim_rc service')
        param('kill', False)
        wait_for(lambda: 'computation_status' in latest and latest['computation_status'].warm_cycles == 50
                 and sim_time() >= 8, 90, 'AUTO_STANDBY warmup')
        event('warm_ready')
        param('armed', True)
        delay(2)
        param('throttle', 1420)
        wait_for(lambda: latest['fused_state'].position.z >= 1.5, 40, 'manual takeoff')
        param('auto_switch', True)
        wait_for(active, 5, 'AUTO capture')
        event('auto_active', position=xyz(latest['fused_state'].position))
        stable_window(None, 1, 45, require_active=True)
        event('pre_goal_stable', position=xyz(latest['fused_state'].position),
              reference=xyz(latest['reference'].pose.pose.position))
        goal_stream = (out / 'goal_cli.log').open('w')
        streams.append(goal_stream)
        goal_command = ['ros2', 'topic', 'pub', '--once', '/go_to_position', 'geometry_msgs/msg/PointStamped', GOAL]
        event('goal_cli', command=goal_command)
        cli = subprocess.Popen(goal_command, stdout=goal_stream, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wait_for(lambda: cli.poll() is not None, 15, 'goal CLI')
            assert cli.returncode == 0
        finally:
            if cli.poll() is None:
                os.killpg(cli.pid, signal.SIGINT)
                cli.wait(timeout=5)
        wait_for(lambda: any('go_to_position accepted:' in x['reason'] for x in rows['computation_status']),
                 5, 'goal acceptance')
        accepted = next(x for x in rows['computation_status'] if 'go_to_position accepted:' in x['reason'])
        result['accepted'] = accepted
        event('goal_accepted', stamp=accepted['stamp'], solve_seconds=accepted['solve_seconds'])
        stable_window(TARGET, 3, 80, require_active=True)
        event('target_stable', position=xyz(latest['fused_state'].position),
              truth=xyz(latest['ground_truth'].pose.pose.position))
        end = rows['computation_status'][-1]['stamp']
        start = accepted['stamp']
        result['tracking_interval'] = [start, end]
        active_rows = [x for x in rows['computation_status'] if start <= x['stamp'] <= end]
        output_rows = [x for x in rows['output_status'] if start <= x['stamp'] <= end]
        assert all(x['trajectory_active'] and x['controller_success'] for x in active_rows)
        assert output_rows and all(x['override_active'] for x in output_rows)
        assert xyz(latest['reference'].pose.pose.position) == list(TARGET)
        result['final_reference'] = xyz(latest['reference'].pose.pose.position)
        for topic in ('fused_state', 'ground_truth'):
            window = [x for x in rows[topic] if end-3 <= x['stamp'] <= end]
            errors = [norm(a-b for a,b in zip(x['position'], TARGET)) for x in window]
            speeds = [norm(x['velocity']) for x in window]
            assert window
            result[topic] = dict(samples=len(window), final_position=window[-1]['position'],
                                 position_rmse=math.sqrt(sum(x*x for x in errors)/len(errors)),
                                 position_error_max=max(errors), speed_max=max(speeds))
            assert max(errors) <= .2 and max(speeds) <= .2, topic + ' not stably at target'
        result['max_solve_seconds'] = max(x['solve_seconds'] for x in active_rows)
        result['max_cycle_seconds'] = max(x['cycle_seconds'] for x in active_rows)
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
