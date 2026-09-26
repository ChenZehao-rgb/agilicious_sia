#!/usr/bin/env python3
"""Run conditional, segmented native EKF experiments. No ROS or control output."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def euler_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]


def rotate_world_yaw(quat, angle):
    w, x, y, z = quat
    c, s = math.cos(angle/2), math.sin(angle/2)
    result = np.array([c*w-s*z, c*x-s*y, c*y+s*x, c*z+s*w])
    return (result/np.linalg.norm(result)).tolist()


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def save_rows(path, rows):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def stats(values):
    data = np.asarray(values, dtype=float)
    data = data[np.isfinite(data)]
    if not len(data):
        return {'count':0}
    return {'count':len(data), 'median':float(np.median(data)), 'p95':float(np.percentile(data, 95)),
            'maximum':float(data.max()), 'rms':float(np.sqrt(np.mean(data**2)))}


def experiment(name, height_method, seed_duration, nis, imu, inputs, output, scratch, runner, profile):
    nav = np.genfromtxt(inputs/f'nav_{height_method}.csv', delimiter=',', skip_header=1)
    audit = json.loads((inputs/'input_audit.json').read_text())
    seed = audit['seeds'][str(float(seed_duration))]
    cuts = np.flatnonzero(np.diff(imu[:, 0]) > .025)+1
    imu_segments = np.split(imu, cuts)
    assert len(imu_segments) == 2
    all_states, all_updates, segments, seeds = [], [], [], []
    last_state = None
    case_dir = scratch/name
    case_dir.mkdir(parents=True)
    for epoch, samples in enumerate(imu_segments):
        gps = nav[nav[:, 1] == epoch]
        minimum = seed_duration if epoch == 0 else samples[0, 0]
        gps = gps[gps[:, 0] >= minimum]
        first = gps[0]
        t, yaw = first[0], first[8]
        if epoch == 0:
            roll, pitch = seed['roll'], seed['pitch']
            bg, ba = seed['gyro_mean'], [0., 0., 0.]
            quat = euler_quaternion(roll, pitch, yaw)
            seed_method = f'{seed_duration}s ground IMU mean, first available GPS state'
        else:
            assert last_state is not None
            q = [float(last_state[k]) for k in ['qw','qx','qy','qz']]
            previous_yaw = float(last_state['yaw'])
            quat = rotate_world_yaw(q, yaw-previous_yaw)
            bg = [float(last_state[k]) for k in ['bgx','bgy','bgz']]
            ba = [float(last_state[k]) for k in ['bax','bay','baz']]
            seed_method = ('Explicit reseed: previous segment tilt/bias held; yaw and p/v from current GPS. '
                           'No propagation over missing IMU, no valid stationary restart claim.')
        initial = ['S', t, epoch, *first[2:8], *quat, *bg, *ba, *first[9:15]]
        event_file = case_dir/f'epoch{epoch}.csv'
        events = [(r[0], 1, ['I', *r]) for r in samples if r[0] >= t]
        events.extend((r[0], 0, ['G', r[0], *r[2:]]) for r in gps[1:])
        events.sort(key=lambda x:(x[0], x[1]))
        with event_file.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(initial)
            writer.writerows(event[2] for event in events)
        prefix = case_dir/f'epoch{epoch}'
        process = subprocess.run([str(runner), str(event_file), str(prefix), str(nis), '.025', str(profile)],
                                 capture_output=True, text=True)
        (case_dir/f'epoch{epoch}.log').write_text(process.stdout+process.stderr)
        if process.returncode:
            raise RuntimeError(f'{name} epoch {epoch}: {process.stderr}')
        states = read_rows(Path(str(prefix)+'_states.csv'))
        updates = read_rows(Path(str(prefix)+'_updates.csv'))
        last_state = states[-1]
        # Store all navigation updates and approximately 50 Hz inertial states.
        previous_bin = None
        for row in states:
            current_bin = math.floor(float(row['t'])*50)
            if row['source'] != 'imu' or current_bin != previous_bin:
                all_states.append(row)
                previous_bin = current_bin
        all_updates.extend(updates)
        segments.extend(read_rows(Path(str(prefix)+'_segments.csv')))
        seeds.append({'epoch':epoch, 'time_s':t, 'method':seed_method, 'quaternion_wxyz':quat,
                      'gyro_bias':bg, 'acceleration_bias':ba})
    save_rows(output/f'{name}_states.csv', all_states)
    save_rows(output/f'{name}_updates.csv', all_updates)
    save_rows(output/f'{name}_segments.csv', segments)
    segment_metrics = []
    for epoch in range(2):
        rows = [r for r in all_updates if int(float(r['segment'])) == epoch]
        accepted = [r for r in rows if r['status'] == 'accepted']
        values = lambda fields: np.array([[float(r[k]) for k in fields] for r in rows])
        post = values(['post_px','post_py','post_pz'])
        gps = values(['gps_px','gps_py','gps_pz'])
        velocity = values(['post_vx','post_vy','post_vz'])-values(['gps_vx','gps_vy','gps_vz'])
        residual = post-gps
        segment_metrics.append({'epoch':epoch, 'updates':len(rows), 'accepted':len(accepted),
            'rejected':sum(r['status']=='rejected' for r in rows),
            'not_evaluated':sum(r['status'] not in ('accepted','rejected') for r in rows),
            'nis':stats([float(r['nis']) for r in rows]),
            'horizontal_input_residual_m':stats(np.linalg.norm(residual[:, :2], axis=1)),
            'vertical_input_residual_m':stats(abs(residual[:, 2])),
            'velocity_input_residual_m_s':stats(np.linalg.norm(velocity, axis=1)),
            'input_residual_caveat':'Same GPS was fused; these are not independent accuracy errors.'})
    return {'name':name, 'height_method':height_method, 'seed_duration_s':seed_duration,
            'nis_limit':None if nis == 'inf' else float(nis), 'seeds':seeds, 'segments':segment_metrics,
            'stored_state_samples':len(all_states), 'segment_events':segments}


def run(inputs, output, runner, profile):
    profile = profile.resolve()
    output.mkdir(parents=True, exist_ok=True)
    imu = np.genfromtxt(inputs/'imu.csv', delimiter=',', skip_header=1)
    results = []
    cases = [('linear_2s', 'linear', 2., '24.322'), ('hold_2s', 'hold', 2., '24.322'),
             ('linear_1s', 'linear', 1., '24.322'), ('linear_2s_ungated', 'linear', 2., 'inf')]
    with tempfile.TemporaryDirectory(prefix='agi-offline-ekf-') as directory:
        for name, height, seed, nis in cases:
            results.append(experiment(name, height, seed, nis, imu, inputs, output, Path(directory), runner, profile))
            print(name, [(x['accepted'],x['updates']) for x in results[-1]['segments']], flush=True)
    profile_name = str(profile.relative_to(ROOT)) if ROOT in profile.parents else str(profile)
    report = {'filter':'production agi::EkfImu from rebuilt libagilib',
              'profile':profile_name, 'profile_sha256':hashlib.sha256(profile.read_bytes()).hexdigest(),
              'state_output':'all GPS updates plus approximately 50 Hz IMU predictions',
              'cases':results, 'strict_online_result':'No initialization in recorded bag; offline seeds are conditional experiments.',
              'limitations':['No independent localization truth.', 'Original full-rate GPS altitude not recorded.',
                             'Short startup seed and in-flight reseed are offline assumptions.',
                             'All finite covariance is conditional on the chosen model/noise; it is not proven accuracy.']}
    (output/'results.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', type=Path, default=HERE/'inputs')
    parser.add_argument('--output', type=Path, default=HERE/'results')
    parser.add_argument('--runner', type=Path, default=HERE/'offline_ekf')
    parser.add_argument('--profile', type=Path, help='Defaults to the snapshot in --inputs')
    args = parser.parse_args()
    run(args.inputs, args.output, args.runner, args.profile or args.inputs/'hardware_snapshot.yaml')
