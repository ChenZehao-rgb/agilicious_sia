#!/usr/bin/env python3
"""Run the three original CSVs through Gazebo, Betaflight and the ROS 2 graph.

No time scaling, trajectory numeric-data changes or relaxed flight timing are used.
The receiver emulator first raises the vehicle manually, then gives one AUTO
edge. Reference tracking is scored against Gazebo ground truth at matching
simulation timestamps. Every run saves raw topic CSVs, tracking.csv and JSON.

After sourcing ROS and the workspace, invoke explicitly (never runs on import):
  ROS_DOMAIN_ID=95 ROS_LOCALHOST_ONLY=1 GZ_PARTITION=agi_trajectory_validation \
    python3 agi_ros2/test/test_sitl_trajectories.py --output /tmp/agi_trajectories
Use --describe-only to inspect the exact datasets without starting processes.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
DATASETS = ROOT / 'miscellaneous/datasets/ref_trajs/open_source'
TRAJECTORIES = ('aggressive_50mps.csv', 'HELIX_FWD20_50mps.csv', 'CPC33_Z1.csv')
GRAVITY = 9.8066
CSV_HEADER = 't,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,w_x,w_y,w_z'
# Match the standalone SITL application's startup sequence: Betaflight's five-second
# boot grace must expire with ARM low before the switch can authorize an arm attempt.
DISARMED_SETUP_SECONDS = 6.
PREARM_SETUP_SECONDS = 2.


def describe_trajectory(path, target_mass):
    """Interpret the existing 30-column data format, including its short header."""
    with path.open() as stream:
        if stream.readline().rstrip('\r\n') != CSV_HEADER:
            raise ValueError('Expected the supported legacy short CSV header: ' + str(path))
    data = np.loadtxt(path, delimiter=',', skiprows=1)
    if data.ndim != 2 or data.shape[1] != 30 or len(data) < 2 or not np.isfinite(data).all():
        raise ValueError('Expected at least two finite 30-column trajectory rows: ' + str(path))
    dt = np.diff(data[:, 0])
    if np.any(dt <= 0):
        raise ValueError('Trajectory timestamps must increase')
    force = data[:, 14:17] + [0., 0., GRAVITY]
    specific_force = np.linalg.norm(force, axis=1)
    rotor_sum = data[:, 20:24].sum(axis=1)
    source_mass = float(rotor_sum @ specific_force / (specific_force @ specific_force))
    if not math.isfinite(source_mass) or source_mass <= 0:
        raise ValueError('Cannot estimate trajectory source mass')
    quaternion = data[:, 4:8]
    norms = np.linalg.norm(quaternion, axis=1)
    if np.any(norms < 1e-9):
        raise ValueError('Zero reference quaternion')
    quaternion = quaternion / norms[:, None]
    vertical = 1 - 2 * (quaternion[:, 1]**2 + quaternion[:, 2]**2)
    relative = data[:, 1:4] - data[0, 1:4]
    speed = np.linalg.norm(data[:, 8:11], axis=1)
    thrust = rotor_sum / source_mass * target_mass
    return dict(path=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                rows=len(data), duration_seconds=float(data[-1, 0] - data[0, 0]),
                source_dt_min_seconds=float(dt.min()), source_dt_max_seconds=float(dt.max()),
                start_position_m=data[0, 1:4].tolist(), end_position_m=data[-1, 1:4].tolist(),
                relative_position_min_m=relative.min(axis=0).tolist(), relative_position_max_m=relative.max(axis=0).tolist(),
                speed_max_mps=float(speed.max()), speed_start_mps=float(speed[0]), speed_end_mps=float(speed[-1]),
                acceleration_max_mps2=float(np.linalg.norm(data[:, 14:17], axis=1).max()),
                specific_force_min_mps2=float(specific_force.min()), specific_force_max_mps2=float(specific_force.max()),
                body_rates_abs_max_radps=np.abs(data[:, 11:14]).max(axis=0).tolist(),
                tilt_max_degrees=float(np.rad2deg(np.arccos(np.clip(vertical, -1, 1))).max()),
                inverted_reference_rows=int((vertical < 0).sum()), source_mass_estimate_kg=source_mass,
                source_thrust_min_n=float(rotor_sum.min()), source_thrust_max_n=float(rotor_sum.max()),
                source_force_fit_rmse_n=float(np.sqrt(np.mean((rotor_sum - source_mass * specific_force)**2))),
                target_mass_kg=target_mass, target_total_thrust_min_n=float(thrust.min()),
                target_total_thrust_max_n=float(thrust.max()), original_quaternion_norm_max_error=float(np.max(np.abs(norms - 1))))


def manual_throttle(position, velocity, target, bridge):
    """Receiver-side altitude conditioning only; the AUTO trajectory is unchanged."""
    acceleration = min(1.5 * GRAVITY, max(.6 * GRAVITY, GRAVITY + 2.5 * (target - position) - 2.0 * velocity))
    idle = bridge['motor_idle']
    hover_motor = idle + (1 - idle) * bridge['hover_throttle']
    normalized = (hover_motor * math.sqrt(acceleration / GRAVITY) - idle) / (1 - idle)
    return round(bridge['min_check'] + (2000 - bridge['min_check']) * min(1., max(0., normalized)))


def receiver_inputs(stage, position, velocity, target, bridge):
    throttle = manual_throttle(position, velocity, target, bridge) if stage == 'manual' else 1000
    return dict(armed=stage in ('prearm', 'manual', 'auto'), auto_switch=stage == 'auto',
                kill=stage == 'kill', manual_aetr=[1500, 1500, throttle, 1500])


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def vector(value):
    return [value.x, value.y, value.z]


def odometry_row(message, body_velocity=False):
    quaternion = message.pose.pose.orientation
    q = np.array([quaternion.w, quaternion.x, quaternion.y, quaternion.z])
    velocity = np.array(vector(message.twist.twist.linear))
    if body_velocity:
        cross = 2 * np.cross(q[1:], velocity)
        velocity += q[0] * cross + np.cross(q[1:], cross)
    return dict(stamp_ns=stamp_ns(message), position=vector(message.pose.pose.position),
                velocity=velocity.tolist(), quaternion=q.tolist())


def write_csv(path, rows):
    if not rows:
        path.write_text('')
        return
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json.dumps(value) if isinstance(value, (list, dict)) else value
                             for key, value in json_ready(row).items()})


def json_ready(value):
    if isinstance(value, np.ndarray):
        value = value.tolist()
    if isinstance(value, dict):
        return {key: json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_ready(item) for item in value]
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def save_results(directory, summary, topic_rows):
    """Persist the outcome before optional large CSV logs, retaining logging failures."""
    summary = json_ready(summary)
    result = directory / 'result.json'
    result.write_text(json.dumps(summary, indent=2, allow_nan=False))
    logging_errors = []
    for name, rows in topic_rows.items():
        try:
            write_csv(directory / (name + '.csv'), rows)
        except Exception as error:
            logging_errors.append(name + ': ' + type(error).__name__ + ': ' + str(error))
    if logging_errors:
        summary['logging_errors'] = logging_errors
        summary['passed'] = False
        summary['acceptance_failures'].append('Some raw CSV logs could not be saved; see logging_errors')
        result.write_text(json.dumps(summary, indent=2, allow_nan=False))
    return summary


def metrics(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if len(values) == 0:
        return dict(count=0, mean=None, rms=None, p95=None, p99=None, maximum=None)
    return dict(count=len(values), mean=float(values.mean()), rms=float(np.sqrt(np.mean(values**2))),
                p95=float(np.percentile(values, 95)), p99=float(np.percentile(values, 99)), maximum=float(values.max()))


def score_tracking(references, truth, computations, duration):
    """Associate using the reported state age, and score at the original reference time.

    TimeSampler uses state.t while ComputationStatus stamps the control cycle.
    Their known difference is state_age; it is not an estimated tracking delay.
    One microsecond only accommodates floating timestamp reconstruction.
    """
    active = {}
    for row in computations:
        age = row.get('state_age', 0.)
        if (row['trajectory_active'] and math.isfinite(age) and 0 <= age <= .010 and
                0 <= row['reference_elapsed'] - age <= duration):
            active[row['stamp_ns']] = row
    expected = sorted((stamp - round(row.get('state_age', 0.) * 1e9), stamp) for stamp, row in active.items())
    expected_times = np.array([item[0] for item in expected], dtype=np.int64)
    matches = []
    used = set()
    for ref in references:
        index = int(np.searchsorted(expected_times, ref['stamp_ns']))
        candidates = [i for i in (index - 1, index) if 0 <= i < len(expected)]
        if not candidates:
            continue
        closest = min(candidates, key=lambda i: abs(int(expected_times[i]) - ref['stamp_ns']))
        difference = ref['stamp_ns'] - int(expected_times[closest])
        if abs(difference) > 1000 or closest in used:
            continue
        used.add(closest)
        matches.append((ref, active[expected[closest][1]], difference))
    truth = sorted({row['stamp_ns']: row for row in truth}.values(), key=lambda row: row['stamp_ns'])
    tracking = []
    if len(truth) >= 2:
        times = np.array([row['stamp_ns'] for row in truth], dtype=np.int64)
        positions = np.array([row['position'] for row in truth])
        velocities = np.array([row['velocity'] for row in truth])
        for ref, computation, alignment_error in matches:
            index = int(np.searchsorted(times, ref['stamp_ns'], side='right'))
            if index == 0 or index >= len(times):
                continue
            gap = int(times[index] - times[index - 1])
            if gap > 20_000_000:
                continue
            blend = (ref['stamp_ns'] - int(times[index - 1])) / gap
            position = positions[index - 1] + blend * (positions[index] - positions[index - 1])
            velocity = velocities[index - 1] + blend * (velocities[index] - velocities[index - 1])
            if not all(np.isfinite(value).all() for value in (position, velocity, ref['position'], ref['velocity'])):
                continue
            error = position - ref['position']
            reference_elapsed = computation['reference_elapsed'] + (ref['stamp_ns'] - computation['stamp_ns']) * 1e-9
            tracking.append(dict(stamp_ns=ref['stamp_ns'], computation_stamp_ns=computation['stamp_ns'],
                                 reference_elapsed=reference_elapsed, association_error_ns=alignment_error,
                                 reference_position=ref['position'], truth_position=position.tolist(),
                                 reference_velocity=ref['velocity'], truth_velocity=velocity.tolist(),
                                 position_error=error.tolist(), position_error_norm=float(np.linalg.norm(error)),
                                 velocity_error_norm=float(np.linalg.norm(velocity - ref['velocity'])),
                                 truth_interpolation_gap_seconds=gap * 1e-9))
    elapsed = [row['reference_elapsed'] - row.get('state_age', 0.) for row in active.values()]
    ordered_stamps = sorted(active)
    summary = dict(active_reference_samples=len(matches), scored_samples=len(tracking),
                   scoring_fraction=len(tracking) / len(matches) if matches else 0.,
                   active_control_sample_coverage=min(1., len(active) * .01 / duration),
                   reference_publication_fraction=len(matches) / len(active) if active else 0.,
                   reference_association='reference.stamp = computation.stamp - reported state_age',
                   association_tolerance_ns=1000,
                   association_error_max_ns=max((abs(error) for _, _, error in matches), default=None),
                   associated_state_age_seconds=metrics([row.get('state_age', 0.) for _, row, _ in matches]),
                   reference_elapsed_max=max(elapsed, default=0.),
                   coverage_fraction=min(1., max(elapsed, default=0.) / duration),
                   largest_active_sample_gap_seconds=max(np.diff(ordered_stamps), default=0) * 1e-9,
                   actual_speed_max_mps=max((float(np.linalg.norm(row['truth_velocity'])) for row in tracking), default=None),
                   reference_speed_max_mps=max((float(np.linalg.norm(row['reference_velocity'])) for row in tracking), default=None),
                   position_error_m=metrics([row['position_error_norm'] for row in tracking]),
                   velocity_error_mps=metrics([row['velocity_error_norm'] for row in tracking]))
    return summary, tracking


def evaluate_acceptance(summary):
    tracking = summary['tracking']
    rmse = tracking['position_error_m']['rms']
    checks = {
        'Full original trajectory and terminal hold completed': summary['trajectory_finished'],
        'KILL released output': summary['final_output_inactive'],
        'Selected controller used throughout': summary['controller_matches'],
        'Original reference time coverage at least 99 percent': tracking['coverage_fraction'] >= .99,
        'Active control sample gap at most 25 ms': tracking['largest_active_sample_gap_seconds'] <= .025,
        '100 Hz active sample coverage at least 99 percent': tracking['active_control_sample_coverage'] >= .99,
        'Reference publication coverage at least 99 percent': tracking['reference_publication_fraction'] >= .99,
        'Ground truth scoring coverage at least 99 percent': tracking['scoring_fraction'] >= .99,
        'Ground truth position RMSE within configured limit': rmse is not None and rmse <= summary['rmse_limit_m'],
        'Raw logs saved successfully': not summary.get('logging_errors'),
    }
    summary['acceptance_failures'] = [name for name, success in checks.items() if not success]
    summary['passed'] = summary['first_failure'] is None and not summary['acceptance_failures']


def read_topic_csv(path):
    def decode(value):
        if value == '':
            return None
        if value in ('True', 'False'):
            return value == 'True'
        if value.startswith(('[', '{')):
            return json.loads(value)
        try:
            return int(value)
        except ValueError:
            try:
                return float(value)
            except ValueError:
                return value
    with path.open() as stream:
        return [{key: decode(value) for key, value in row.items()} for row in csv.DictReader(stream)]


def rescore_directory(directory):
    """Recompute offline association only; retain recorded flight facts and the original outcome."""
    result = directory / 'result.json'
    original = result.read_bytes()
    summary = json.loads(original)
    topics = {name: read_topic_csv(directory / (name + '.csv'))
              for name in ('reference', 'ground_truth', 'fused_state', 'computation_status')}
    duration = summary['dataset']['duration_seconds']
    summary['tracking'], truth_rows = score_tracking(topics['reference'], topics['ground_truth'], topics['computation_status'], duration)
    summary['fused_tracking'], fused_rows = score_tracking(topics['reference'], topics['fused_state'], topics['computation_status'], duration)
    evaluate_acceptance(summary)
    backup = directory / 'result.before_rescore.json'
    if not backup.exists():
        backup.write_bytes(original)
    summary['offline_rescore'] = dict(original_result=str(backup),
                                     scoring_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                                     explanation='Use reported state_age to associate reference with its control cycle; '
                                                 'truth remains interpolated at the unchanged reference timestamp')
    summary = save_results(directory, summary, {'tracking': truth_rows, 'fused_tracking': fused_rows})
    suite_path = directory.parent / 'summary.json'
    if suite_path.is_file():
        suite = json.loads(suite_path.read_text())
        name = Path(summary['dataset']['path']).name
        if name in suite:
            suite[name] = summary
            suite_path.write_text(json.dumps(suite, indent=2, allow_nan=False))
    return summary


def stop_processes(processes):
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    for process in reversed(processes):
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)


def wait_for_free_ports(timeout=10.):
    deadline = time.monotonic() + timeout
    while True:
        probes = []
        busy = None
        try:
            for kind, port in ((socket.SOCK_STREAM, 5761), (socket.SOCK_DGRAM, 9002),
                               (socket.SOCK_DGRAM, 9003), (socket.SOCK_DGRAM, 9004)):
                probe = socket.socket(socket.AF_INET, kind)
                probes.append(probe)
                if kind == socket.SOCK_STREAM:
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    probe.bind(('127.0.0.1', port))
                except OSError as error:
                    busy = f'port {port}: {error}'
                    break
        finally:
            for probe in probes:
                probe.close()
        if busy is None:
            return
        if time.monotonic() >= deadline:
            raise RuntimeError('Simulator ports did not become free; no run was started: ' + busy)
        time.sleep(.1)


def run_one(path, profile_path, profile, options, directory):
    import rclpy
    from agi_ros2.msg import Authority, ComputationStatus, ControlCommand, FusedState, OutputStatus
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter

    directory.mkdir(parents=True, exist_ok=False)
    dataset = describe_trajectory(path, profile['pilot']['quadrotor']['mass'])
    profile_digest = hashlib.sha256(profile_path.read_bytes()).hexdigest()
    duration = dataset['duration_seconds']
    capture_height = max(options.capture_height, 1.5 - dataset['relative_position_min_m'][2])
    rclpy.init()
    node = rclpy.create_node('full_trajectory_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    observations = {topic: [] for topic in ('fused_state', 'reference', 'ground_truth', 'control_command',
                                           'output_status', 'computation_status', 'authority')}
    latest = {}
    subscriptions = []
    processes = []
    streams = []
    stage = 'disarmed'
    failure = None
    auto_stamp = None
    disarmed_stamp = None
    prearm_stamp = None
    manual_stamp = None
    stable_since = None
    ever_active = False
    max_elapsed = 0.
    output_fault_baseline = None
    finished = False
    killed_at = None

    def receive(topic, message):
        latest[topic] = message
        if topic in ('reference', 'ground_truth'):
            row = odometry_row(message, body_velocity=topic == 'ground_truth')
        elif topic == 'fused_state':
            row = dict(stamp_ns=stamp_ns(message), position=vector(message.position), velocity=vector(message.velocity),
                       initialized=message.initialized, reset_counter=message.reset_counter,
                       readiness_reason=message.readiness_reason)
        elif topic == 'control_command':
            row = dict(stamp_ns=stamp_ns(message), permit_override=message.permit_override, mode=message.mode,
                       total_thrust=message.total_thrust, body_rates=vector(message.body_rates),
                       sequence=message.sequence, solve_seconds=message.evidence.solve_seconds)
        elif topic == 'output_status':
            row = dict(stamp_ns=stamp_ns(message), override_active=message.override_active,
                       transport_healthy=message.transport_healthy, fault_count=message.fault_count,
                       reason=message.reason, last_fault=message.last_fault, command_age=message.command_age)
        else:
            row = dict(stamp_ns=stamp_ns(message), trajectory_active=message.trajectory_active,
                       reference_elapsed=message.reference_elapsed, controller_type=message.controller_type,
                       controller_success=message.controller_success, warm_cycles=message.warm_cycles,
                       solve_seconds=message.solve_seconds, cycle_seconds=message.cycle_seconds,
                       imu_age=message.imu_age, navigation_age=message.navigation_age,
                       state_age=message.state_age, reason=message.reason)
        observations[topic].append(row)

    for topic, message_type in (('fused_state', FusedState), ('control_command', ControlCommand),
                                ('output_status', OutputStatus), ('computation_status', ComputationStatus),
                                ('reference', Odometry), ('ground_truth', Odometry)):
        subscriptions.append(node.create_subscription(message_type, topic, lambda message, name=topic: receive(name, message), 500))
    publisher = node.create_publisher(Authority, 'authority', 1)
    commands = [
        ('sim', ['python3', str(ROOT / 'betaflight_sitl/run.py'), '--no-gazebo', '--no-build', '--ros2',
                 '--runtime-config', str(profile_path), '--controller', options.controller.lower()]),
        ('flight', ['ros2', 'launch', str(ROOT / 'agi_ros2/launch/flight.launch.py'), 'record_bag:=false',
                    'runtime_config:=' + str(profile_path), 'controller:=' + options.controller,
                    'trajectory:=' + str(path)]),
    ]
    started = time.monotonic()
    wall_limit = options.wall_timeout or max(180., 8 * duration + 120.)
    environment = dict(os.environ)
    environment['GZ_PARTITION'] = environment.get('GZ_PARTITION', 'agi_trajectory') + '_' + path.stem + '_' + str(time.monotonic_ns())
    summary = {}
    try:
        wait_for_free_ports()
        for name, command in commands:
            stream = (directory / (name + '.log')).open('w')
            streams.append(stream)
            processes.append(subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                                               start_new_session=True, env=environment))
        last_rc = 0.
        last_progress = 0.
        previous_stage = None
        while True:
            wall = time.monotonic()
            sim = node.get_clock().now().nanoseconds * 1e-9
            if any(process.poll() is not None for process in processes):
                failure = failure or 'A simulator/flight process exited before the test ended'
                break
            if wall - started > wall_limit and stage != 'kill':
                failure = failure or 'Test wall deadline exceeded (flight timing constraints were not changed)'
                stage, killed_at = 'kill', wall
            computation = latest.get('computation_status')
            state = latest.get('fused_state')
            output = latest.get('output_status')
            if (stage == 'disarmed' and computation and computation.warm_cycles >= 50 and state and
                    disarmed_stamp is not None and sim - disarmed_stamp >= DISARMED_SETUP_SECONDS):
                stage, prearm_stamp = 'prearm', sim
            if stage == 'prearm' and sim - prearm_stamp >= PREARM_SETUP_SECONDS:
                stage, manual_stamp = 'manual', sim
            if stage == 'manual' and state:
                steady = abs(state.position.z - capture_height) < .15 and abs(state.velocity.z) < .2
                stable_since = stable_since if steady and stable_since is not None else sim if steady else None
                if stable_since is not None and sim - stable_since >= 1. and computation.warm_cycles >= 50:
                    stage, auto_stamp = 'auto', sim
                    output_fault_baseline = output.fault_count if output else 0
                elif sim - manual_stamp > 25.:
                    failure = 'Manual altitude conditioning did not settle before AUTO'
                    stage, killed_at = 'kill', wall
            if stage == 'auto' and computation and stamp_ns(computation) * 1e-9 >= auto_stamp:
                max_elapsed = max(max_elapsed, computation.reference_elapsed)
                if computation.trajectory_active:
                    ever_active = True
                elif ever_active:
                    failure = 'AUTO/reference revoked: ' + computation.reason
                    stage, killed_at = 'kill', wall
                elif sim - auto_stamp > 1.:
                    failure = 'AUTO edge did not authorize: ' + computation.reason
                    stage, killed_at = 'kill', wall
                if output and output.fault_count > output_fault_baseline:
                    failure = failure or 'Output fault: ' + (output.last_fault or output.reason)
                    stage, killed_at = 'kill', wall
                if max_elapsed >= duration + options.terminal_hold:
                    finished = True
                    stage, killed_at = 'kill', wall
            if stage == 'kill' and wall - killed_at >= .6:
                break
            if stage != previous_stage or wall - last_progress >= 5.:
                print(json.dumps(json_ready(dict(trajectory=path.name, stage=stage, simulation_seconds=sim,
                                                 reference_elapsed=max_elapsed, duration_seconds=duration,
                                                 height_m=state.position.z if state else None,
                                                 warm_cycles=computation.warm_cycles if computation else 0,
                                                 reason=failure or (computation.reason if computation else 'Waiting for computation status')))),
                      flush=True)
                last_progress, previous_stage = wall, stage
            if wall - last_rc >= .02:
                inputs = receiver_inputs(stage, state.position.z if state else 0., state.velocity.z if state else 0.,
                                         capture_height, profile['bridge'])
                authority = Authority()
                authority.header.stamp = node.get_clock().now().to_msg()
                for key, value in inputs.items():
                    setattr(authority, key, value)
                authority.rc_link = True
                publisher.publish(authority)
                if stage == 'disarmed' and sim > 0 and disarmed_stamp is None:
                    disarmed_stamp = sim
                observations['authority'].append(dict(stamp_ns=stamp_ns(authority), stage=stage,
                                                       armed=authority.armed, auto_switch=authority.auto_switch,
                                                       kill=authority.kill, manual_aetr=list(authority.manual_aetr)))
                last_rc = wall
            rclpy.spin_once(node, timeout_sec=.001)
    except Exception as error:
        failure = failure or type(error).__name__ + ': ' + str(error)
    finally:
        process_codes = [process.poll() for process in processes]
        try:
            stop_processes(processes)
        except Exception as error:
            failure = failure or 'Process cleanup failed: ' + str(error)
        for stream in streams:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()
        tracking, rows = score_tracking(observations['reference'], observations['ground_truth'],
                                       observations['computation_status'], duration)
        fused_tracking, fused_rows = score_tracking(observations['reference'], observations['fused_state'],
                                                   observations['computation_status'], duration)
        active_computations = [row for row in observations['computation_status'] if row['trajectory_active']]
        auto_computations = [row for row in observations['computation_status']
                             if auto_stamp is not None and row['stamp_ns'] * 1e-9 >= auto_stamp]
        outputs = observations['output_status']
        startup_faults = {row['fault_count']: row for row in outputs if row['fault_count'] and
                          (auto_stamp is None or row['stamp_ns'] * 1e-9 < auto_stamp)}
        final_inactive = bool(outputs) and not outputs[-1]['override_active']
        controller_matches = bool(active_computations) and all(row['controller_type'] == options.controller for row in active_computations)
        summary = dict(dataset=dataset, controller=options.controller, time_scale=1.0, delay_test=False,
                       runtime_config=str(profile_path), runtime_config_sha256=profile_digest,
                       gazebo_partition=environment['GZ_PARTITION'], output_fault_baseline_at_auto=output_fault_baseline,
                       startup_output_faults=list(startup_faults.values()),
                       launched_commands=commands, capture_height_m=capture_height, auto_stamp_seconds=auto_stamp,
                       disarmed_stamp_seconds=disarmed_stamp, prearm_stamp_seconds=prearm_stamp,
                       manual_stamp_seconds=manual_stamp,
                       disarmed_setup_seconds=DISARMED_SETUP_SECONDS, prearm_setup_seconds=PREARM_SETUP_SECONDS,
                       trajectory_finished=finished, active_reference_elapsed_max=max_elapsed,
                       first_failure=failure, final_output_inactive=final_inactive, controller_matches=controller_matches,
                       counts={key: len(value) for key, value in observations.items()}, tracking=tracking,
                       fused_tracking=fused_tracking,
                       solve_seconds=metrics([row['solve_seconds'] for row in auto_computations]),
                       cycle_seconds=metrics([row['cycle_seconds'] for row in auto_computations]),
                       solve_budget_exceedances=sum(row['solve_seconds'] > .008 for row in auto_computations),
                       cycle_period_exceedances=sum(row['cycle_seconds'] > .010 for row in auto_computations),
                       last_computation=observations['computation_status'][-1] if observations['computation_status'] else None,
                       last_output=outputs[-1] if outputs else None, process_codes_before_cleanup=process_codes,
                       wall_seconds=time.monotonic() - started, rmse_limit_m=options.rmse_limit)
        evaluate_acceptance(summary)
        summary = save_results(directory, summary, {**observations, 'tracking': rows, 'fused_tracking': fused_rows})
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectories', nargs='*', type=Path, help='default: the exact three requested original CSVs')
    parser.add_argument('--runtime-config', type=Path, default=ROOT / 'agi_ros2/config/simulation.yaml')
    parser.add_argument('--controller', type=str.upper, choices=('MPC', 'GEO'), default='MPC')
    parser.add_argument('--output', type=Path, default=Path('/tmp/agi_sitl_trajectories'))
    parser.add_argument('--describe-only', action='store_true')
    parser.add_argument('--rescore', nargs='+', type=Path, help='completed run directories to score again without ROS or simulation')
    parser.add_argument('--capture-height', type=float, default=3.)
    parser.add_argument('--terminal-hold', type=float, default=1.)
    parser.add_argument('--wall-timeout', type=float, default=0., help='test wall deadline only; 0 uses an automatic deadline')
    parser.add_argument('--rmse-limit', type=float, default=1., help='ground-truth position RMSE acceptance limit in metres')
    options = parser.parse_args()
    if options.rescore:
        results = [rescore_directory(path.expanduser().resolve()) for path in options.rescore]
        print(json.dumps([dict(trajectory=Path(result['dataset']['path']).name, passed=result['passed'],
                               failure=result['first_failure'], acceptance_failures=result['acceptance_failures'],
                               tracking=result['tracking']) for result in results], indent=2))
        return 0 if all(result['passed'] for result in results) else 1
    for name in ('capture_height', 'terminal_hold', 'wall_timeout', 'rmse_limit'):
        value = getattr(options, name)
        if not math.isfinite(value) or value < 0 or (name in ('capture_height', 'rmse_limit') and value == 0):
            parser.error(name + ' must be finite and ' + ('positive' if name in ('capture_height', 'rmse_limit') else 'nonnegative'))
    profile_path = options.runtime_config.expanduser().resolve()
    profile = yaml.safe_load(profile_path.read_text())
    if profile['mode'] != 'sitl' or profile['flight']['sitl_delay_test']:
        parser.error('requires the simulation profile with sitl_delay_test=false')
    trajectories = [path.expanduser().resolve() for path in options.trajectories] or [DATASETS / name for name in TRAJECTORIES]
    if options.describe_only:
        print(json.dumps([describe_trajectory(path, profile['pilot']['quadrotor']['mass']) for path in trajectories], indent=2))
        return 0
    options.output.mkdir(parents=True, exist_ok=True)
    results = {}
    for path in trajectories:
        directory = options.output / (path.stem + '_' + time.strftime('%Y%m%d_%H%M%S'))
        result = run_one(path, profile_path, profile, options, directory)
        results[path.name] = result
        (options.output / 'summary.json').write_text(json.dumps(results, indent=2, allow_nan=False))
        print(json.dumps(dict(trajectory=path.name, passed=result['passed'], result=str(directory / 'result.json'),
                             failure=result['first_failure'], acceptance_failures=result['acceptance_failures'],
                             tracking=result['tracking']), indent=2), flush=True)
    return 0 if all(result['passed'] for result in results.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
