#!/usr/bin/env python3
"""Build real EKF revisions and compare height fusion using prepared bag CSVs.

The input directory must contain initial/imu/nav/baro/references/recorded.csv and
the extraction manifest. This replays estimator mathematics, not DDS scheduling
or reference acquisition. Output files and source snapshots live under analysis/.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import time

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
FILES = ('ekf_imu.cpp', 'ekf_imu.hpp', 'ekf_imu_params.cpp', 'ekf_imu_params.hpp')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_variant(output, revision):
    includes = output / 'include/agilib/estimator/ekf_imu'
    includes.mkdir(parents=True, exist_ok=True)
    source_hashes = {}
    header = ''
    for name in FILES:
        folder = 'include/agilib' if name.endswith('.hpp') else 'src'
        relative = Path('agilib') / folder / 'estimator/ekf_imu' / name
        source = ((ROOT / relative).read_text() if revision == 'worktree' else
                  subprocess.check_output(['git', 'show', f'{revision}:{relative}'], cwd=ROOT, text=True))
        source_hashes[str(relative)] = hashlib.sha256(source.encode()).hexdigest()
        if name == 'ekf_imu.hpp':
            header = source
        source = re.sub(r'\bEkfImu\b', 'ReplayEkfImu', source)
        source = re.sub(r'\bEkfImuParameters\b', 'ReplayEkfImuParameters', source)
        destination = includes / name if name.endswith('.hpp') else output / name
        destination.write_text(source)
    library = ROOT / 'build/agi_ros2/agilib'
    flags = ['-std=c++17', '-O2', '-DNDEBUG', '-march=native', '-fno-finite-math-only',
             '-DEIGEN_DONT_PARALLELIZE', '-DEIGEN_STACK_ALLOCATION_LIMIT=1048576',
             '-I' + str(output / 'include'), '-I' + str(ROOT / 'agilib/include'),
             '-isystem', '/usr/include/eigen3']
    if 'NavigationMeasurementMode' in header:
        flags.append('-DREPLAY_HORIZONTAL_MODE=1')
    if 'addNavigation(' in header:
        flags.append('-DREPLAY_WEIGHTED_MODE=1')
    for stem in ('ekf_imu', 'ekf_imu_params'):
        subprocess.run(['g++', *flags, '-c', str(output / f'{stem}.cpp'),
                        '-o', str(output / f'{stem}.o')], check=True)
    binary = output / 'replay'
    subprocess.run(['g++', *flags, '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-Wno-unused-parameter',
                    str(Path(__file__).with_suffix('.cpp')), str(output / 'ekf_imu.o'),
                    str(output / 'ekf_imu_params.o'), '-L' + str(library),
                    '-Wl,-rpath,' + str(library),
                    '-Wl,-rpath-link,' + str(ROOT / 'agilib/externals/acados-src/lib'),
                    '-lagilib', '-pthread', '-o', str(binary)], check=True)
    manifest = dict(revision=revision, source_sha256=source_hashes,
                    source_changes='Only class/type names renamed to avoid linking the library EKF.',
                    replay_source_sha256=digest(Path(__file__).with_suffix('.cpp')),
                    library_sha256=digest(library / 'libagilib.so'))
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return binary, manifest


def summarize(trace, recorded):
    frame = pd.read_csv(trace, float_precision='round_trip')
    later = frame.loc[frame.t_rel >= 30]
    if frame.empty or later.empty:
        raise ValueError('Replay needs initialized samples and at least 30 seconds of data')
    result = dict(rows=len(frame), z_range_m=float(frame.z.max() - frame.z.min()),
                  z_final_m=float(frame.z.iloc[-1]),
                  vz_rms_after_30s_m_s=float(np.sqrt(np.mean(later.vz**2))),
                  x_range_m=float(frame.x.max() - frame.x.min()),
                  y_range_m=float(frame.y.max() - frame.y.min()),
                  xy_speed_rms_after_30s_m_s=float(np.sqrt(np.mean(later.vx**2 + later.vy**2))),
                  bias_final_m=float(frame.bias.iloc[-1]),
                  bias_variance_final_m2=float(frame.bias_variance.iloc[-1]),
                  absolute_height_variance_final_m2=float(frame.pvarz.iloc[-1]),
                  relative_height_variance_final_m2=(float(frame.relative_height_variance.iloc[-1])
                                                     if np.isfinite(frame.relative_height_variance.iloc[-1]) else None),
                  imu_failures=int((frame.imu_ok == 0).sum()), get_failures=int((frame.get_ok == 0).sum()),
                  nonfinite_states=int((~np.isfinite(frame[['x', 'y', 'z', 'vx', 'vy', 'vz', 'pvarz']])).any(axis=1).sum()))
    for key in ('nav_accepted', 'nav_rejected', 'baro_accepted', 'baro_rejected',
                'height_accepted', 'height_rejected', 'vz_accepted', 'vz_rejected'):
        result[key] = int(frame[key].iloc[-1])
    for prefix in ('nav', 'baro', 'height', 'vz'):
        count = result[prefix + '_accepted'] + result[prefix + '_rejected']
        result[prefix + '_rejection_rate'] = result[prefix + '_rejected'] / count if count else None
    matches = frame.merge(recorded, on='t', suffixes=('', '_recorded'))
    if len(matches) != len(frame):
        raise ValueError('Replay did not match all recorded initialized timestamps')
    result['recorded_z_max_difference_m'] = float((matches.z - matches.z_recorded).abs().max())
    return result, frame


def plot_cases(frames, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True, constrained_layout=True)
    for name in ('original_continuous', 'baro_primary_continuous', 'weighted_continuous'):
        frame = frames[name]
        axes[0].plot(frame.t_rel.to_numpy(), frame.z.to_numpy(), label=name.replace('_continuous', ''), linewidth=1)
        axes[1].plot(frame.t_rel.to_numpy(), frame.vz.to_numpy(), label=name.replace('_continuous', ''), linewidth=.7, alpha=.8)
    axes[0].set_ylabel('Estimated z (m)')
    axes[0].set_title('Stationary bag: native EKF replay with the same continuous pressure reference')
    axes[0].legend()
    axes[1].set_ylabel('Estimated vertical velocity (m/s)')
    axes[1].set_xlabel('Seconds since recorded EKF initialization')
    for axis in axes:
        axis.grid(alpha=.2)
    for extension in ('png', 'pdf'):
        figure.savefig(output / f'comparison.{extension}', dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=ROOT / 'agi_ros2/analysis/baro_weighted_20260930/inputs')
    parser.add_argument('--output', type=Path, default=ROOT / 'agi_ros2/analysis/baro_weighted_20260930')
    parser.add_argument('--original-ref', default='6e2394b')
    parser.add_argument('--baro-primary-ref', default='bbae240')
    parser.add_argument('--skip-build', action='store_true', help='Reuse and verify source manifests before running')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    extraction = json.loads((args.input / 'manifest.json').read_text())
    for filename, expected in extraction['input_sha256'].items():
        if digest(args.input / filename) != expected:
            raise ValueError('Input hash mismatch: ' + filename)
    builds = {}
    for name, revision in [('original', args.original_ref), ('baro_primary', args.baro_primary_ref), ('weighted', 'worktree')]:
        output = args.output / 'build' / name
        if args.skip_build:
            manifest = json.loads((output / 'manifest.json').read_text())
            if manifest['revision'] != revision or manifest['replay_source_sha256'] != digest(Path(__file__).with_suffix('.cpp')):
                raise ValueError('Stale replay build: ' + name)
            for filename, expected in manifest['source_sha256'].items():
                source = ((ROOT / filename).read_bytes() if revision == 'worktree' else
                          subprocess.check_output(['git', 'show', f'{revision}:{filename}'], cwd=ROOT))
                if hashlib.sha256(source).hexdigest() != expected:
                    raise ValueError('Stale estimator build: ' + filename)
            if manifest['library_sha256'] != digest(ROOT / 'build/agi_ros2/agilib/libagilib.so'):
                raise ValueError('Changed linked library; rebuild replay')
            builds[name] = (output / 'replay', manifest)
        else:
            builds[name] = build_variant(output, revision)
    cases = [
        ('original_recorded', 'original', 'full3d', 'recorded', .01),
        ('original_continuous', 'original', 'full3d', 'continuous', .01),
        ('baro_primary_continuous', 'baro_primary', 'baro-primary', 'continuous', 0),
        ('weighted_continuous', 'weighted', 'weighted', 'continuous', 0),
    ]
    recorded = pd.read_csv(args.input / 'recorded.csv', float_precision='round_trip')
    summary = dict(input_manifest=extraction, cases={}, limits=dict(z_range_m=1.1, vz_rms_ratio=1.1),
                   limitations=[
                       'Native EKF replay, not full ROS/DDS scheduling or reference acquisition.',
                       'Continuous-reference cases share the first reference and omit two recorded reference resets.',
                       'Initialization uses the recorded initialized state, derived gyro bias and zero acceleration bias.',
                       'Stationarity is operator-reported; no surveyed height or dynamic flight truth.',
                       'Host wall time includes CSV output and is not a CM5 real-time certification.'])
    frames = {}
    for name, source, mode, reference, random_walk in cases:
        trace = args.output / f'{name}.csv'
        command = [str(builds[source][0]), '--input', str(args.input), '--output', str(trace),
                   '--nav-mode', mode, '--reference-mode', reference, '--baro-q', str(random_walk)]
        started = time.monotonic()
        with (args.output / f'{name}.log').open('w') as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        elapsed = time.monotonic() - started
        metrics, frames[name] = summarize(trace, recorded)
        summary['cases'][name] = dict(command=command, build=builds[source][1], metrics=metrics,
                                     replay_wall_seconds=elapsed, trace_sha256=digest(trace))
        print(name, json.dumps(metrics), flush=True)
    new = summary['cases']['weighted_continuous']['metrics']
    old = summary['cases']['baro_primary_continuous']['metrics']
    ratio = new['vz_rms_after_30s_m_s'] / old['vz_rms_after_30s_m_s']
    checks = dict(height_range=new['z_range_m'] <= 1.1, vertical_speed=ratio <= 1.1,
                  original_reproduced=summary['cases']['original_recorded']['metrics']['recorded_z_max_difference_m'] < .0001,
                  valid_traces=all(not case['metrics'][key] for case in summary['cases'].values()
                                   for key in ('imu_failures', 'get_failures', 'nonfinite_states')))
    summary['acceptance'] = dict(checks=checks, passed=all(checks.values()), vertical_speed_ratio=ratio)
    (args.output / 'comparison.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    plot_cases(frames, args.output)
    print(json.dumps(summary['acceptance'], indent=2))
    if not all(checks.values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
