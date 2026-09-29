#!/usr/bin/env python3
"""Compute source-backed stability and GNSS/EKF consistency, without true-position claims.

Run extract.py first. All times are relative to metadata bag start. Position
spread across a moving recording is NOT a ground-truth positioning error.
"""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
BLUE, GOLD, ORANGE = '#245c91', '#a97919', '#c25228'


def stats(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {'finite_n': 0}
    return dict(finite_n=len(a), min=float(a.min()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), max=float(a.max()),
                mean=float(a.mean()), std=float(a.std()))


def transitions(d, field):
    return d.loc[d[field].ne(d[field].shift()), ['bag_s', 'stamp_s', field]].to_dict('records')


def position_summary(d):
    a = d[['position.' + x for x in 'xyz']].to_numpy()
    if not len(a):
        return {'n': 0}
    centered = a - a.mean(axis=0)
    return dict(n=len(a), interval_s=d.bag_s.iloc[[0, -1]].tolist(),
                min_xyz_m=a.min(axis=0).tolist(), max_xyz_m=a.max(axis=0).tolist(),
                span_xyz_m=np.ptp(a, axis=0).tolist(),
                end_minus_start_xyz_m=(a[-1] - a[0]).tolist(),
                horizontal_radius_about_mean_m=stats(np.linalg.norm(centered[:, :2], axis=1)),
                max_horizontal_distance_from_first_m=float(np.linalg.norm(a[:, :2] - a[0, :2], axis=1).max()),
                std_xyz_m=a.std(axis=0).tolist())


def summarize(path):
    data = {k: pd.read_csv(path / (k + '.csv.gz')) for k in
            ['fused_state', 'sensors_imu', 'sensors_navigation', 'sensors_local_navigation', 'authority']}
    f, i, n, l, a = (data[k] for k in data)
    l = l[l.observation_valid].copy()
    n['speed'] = np.linalg.norm(n[['velocity.' + x for x in 'xyz']], axis=1)
    valid = f[f.initialized].copy()
    summary = {'manifest': json.loads((path / 'manifest.json').read_text()),
               'fused_counts': {k: {str(v): int(c) for v, c in f[k].value_counts().items()}
                                for k in ['initialized', 'imu_ready', 'navigation_valid', 'navigation_ready',
                                          'estimator_ready', 'readiness_reason', 'reset_counter']},
               'fused_transitions': {k: transitions(f, k) for k in ['initialized', 'reset_counter']},
               'authority_transitions': {k: transitions(a, k) for k in ['armed', 'rc_link']},
               'gnss_reported_accuracy': {k: stats(n[k]) for k in
                                           ['horizontal_accuracy', 'vertical_accuracy', 'velocity_accuracy']},
               'fix_type_counts': {str(v): int(c) for v, c in n.fix_type.value_counts().items()},
               'clock_aligned_counts': {str(v): int(c) for v, c in n.clock_aligned.value_counts().items()},
               'gnss_speed_m_s': stats(n.speed), 'navigation_whole_recording': position_summary(l),
               'fused_initialized_whole_recording': position_summary(valid), 'streams': {}, 'windows': {}}
    summary['fused_initialized_whole_recording']['meaning'] = 'Whole-recording extent across resets, not continuous drift or truth error'
    summary['fused_by_reset_counter'] = {str(k): position_summary(v) for k, v in valid.groupby('reset_counter')}
    for name, frame in [('imu', i), ('navigation', n), ('local_navigation', l), ('fused', f)]:
        t = frame['header.stamp_ns'].to_numpy()
        summary['streams'][name] = {'n': len(frame), 'duplicate_header_n': int(pd.Series(t).duplicated().sum()),
                                   'nonpositive_header_gap_n': int((np.diff(t) <= 0).sum()),
                                   'header_gap_s': stats(np.diff(t) * 1e-9),
                                   'bag_age_s': stats((frame.log_ns - frame['header.stamp_ns']) * 1e-9),
                                   'hz': float((len(t) - 1) * 1e9 / (t[-1] - t[0]))}
    for lo, hi in [(0, 10), (0, 60), (0, 120), (6, 60), (10, 60), (60, 120), (120, 210), (219.7, 227.61)]:
        subset = i[(i.bag_s >= lo) & (i.bag_s < hi)]
        if subset.empty:
            continue
        acc = subset[['linear_acceleration.' + x for x in 'xyz']].to_numpy()
        gyro = subset[['angular_velocity.' + x for x in 'xyz']].to_numpy()
        summary['windows'][f'{lo}_{hi}'] = {
            'meaning': ('Operator confirmed approximately first 120 seconds stationary' if '161859' in path.name and hi <= 120
                        else 'Diagnostic interval; do not interpret moving-recording spread as position error'),
            'imu_n': len(subset), 'acc_mean_xyz': acc.mean(axis=0).tolist(),
            'acc_mean_norm': float(np.linalg.norm(acc.mean(axis=0))),
            'acc_std_xyz': acc.std(axis=0).tolist(), 'gyro_std_xyz': gyro.std(axis=0).tolist(),
            'navigation': position_summary(l[(l.bag_s >= lo) & (l.bag_s < hi)]),
            'fused': position_summary(valid[(valid.bag_s >= lo) & (valid.bag_s < hi)])}
    # Interpolate the high-rate fused state to GNSS acquisition time, never across resets.
    consistency = []
    for _, fs in valid.groupby('reset_counter'):
        fs = fs.sort_values('stamp_s').drop_duplicates('header.stamp_ns')
        nav = l[(l.stamp_s >= fs.stamp_s.min()) & (l.stamp_s <= fs.stamp_s.max())].copy()
        if not len(nav):
            continue
        for axis in 'xyz':
            nav['fused.' + axis] = np.interp(nav.stamp_s, fs.stamp_s, fs['position.' + axis])
            nav['residual.' + axis] = nav['fused.' + axis] - nav['position.' + axis]
        nav['horizontal_residual_m'] = np.linalg.norm(nav[['residual.x', 'residual.y']], axis=1)
        nav['vertical_absolute_residual_m'] = np.abs(nav['residual.z'])
        consistency.append(nav)
    if consistency:
        c = pd.concat(consistency)
        c.to_csv(path / 'aligned_consistency.csv.gz', index=False)
        summary['gnss_fused_consistency'] = {}
        for name, subset in [('all', c), ('before_210s', c[c.bag_s < 210]), ('last_rejection_run', c[(c.bag_s >= 219.7) & (c.bag_s < 227.61)])]:
            summary['gnss_fused_consistency'][name] = {k: stats(subset[k]) for k in
                                                      ['horizontal_residual_m', 'vertical_absolute_residual_m', 'residual.z']}
        # Invert the estimator's published acceleration equation using exact integer time matches.
        m = valid.merge(i, on='header.stamp_ns', suffixes=('_f', '_i'), validate='one_to_one')
        quat = m[['orientation.' + x + '_f' for x in 'xyzw']].to_numpy()
        quat = quat / np.linalg.norm(quat, axis=1)[:, None]
        world = m[['acceleration.' + x for x in 'xyz']].to_numpy() - np.array([0, 0, -9.8066])
        acc = m[['linear_acceleration.' + x for x in 'xyz']].to_numpy()
        q_inverse_vector = -quat[:, :3]
        body = world + 2 * np.cross(q_inverse_vector, np.cross(q_inverse_vector, world) + quat[:, 3, None] * world)
        bias = acc - body
        b = pd.DataFrame({'bag_s': m.bag_s_f, **{'ba_' + x: bias[:, k] for k, x in enumerate('xyz')}})
        b.iloc[::25].to_csv(path / 'reconstructed_acceleration_bias_20hz.csv', index=False)
        summary['reconstructed_bias'] = {'exact_matched_n': len(m), 'intervals': {}}
        for lo, hi in [(5.79, 6), (10, 20), (30, 60), (60, 120), (180, 210), (219.7, 227.61)]:
            subset = b[(b.bag_s >= lo) & (b.bag_s < hi)]
            summary['reconstructed_bias']['intervals'][f'{lo}_{hi}'] = {k: stats(subset[k]) for k in ['ba_x', 'ba_y', 'ba_z']}
    (path / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False))
    return data, summary


def plot(datasets):
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False, 'axes.grid': True,
                         'grid.alpha': .18, 'figure.facecolor': 'white'})
    fig, axs = plt.subplots(4, 2, figsize=(14, 13), constrained_layout=True)
    for col, (label, (d, summary)) in enumerate(datasets.items()):
        f, n, imu = d['fused_state'].copy(), d['sensors_local_navigation'], d['sensors_imu']
        n = n[n.observation_valid]
        f.loc[~f.initialized, ['position.' + x for x in 'xyz']] = np.nan
        for row, axis in enumerate('xyz'):
            ax = axs[row, col]
            ax.plot(n.stamp_s.to_numpy(), n['position.' + axis].to_numpy(), color=GOLD, lw=1.2, label='GNSS local observation')
            ax.plot(f.stamp_s.to_numpy()[::10], f['position.' + axis].to_numpy()[::10], color=BLUE, lw=1.2, label='Initialized EKF')
            ax.set_ylabel(f'{axis.upper()} (m, ENU)')
            ax.set_title(f'{label}  |  {axis.upper()} position')
            if row == 0:
                ax.legend(loc='upper left', fontsize=9)
            ax.set_xlim(0, 250)
            ax.set_xlabel('Acquisition time since bag start (s)')
        ax = axs[3, col]
        ax.step(f.stamp_s.to_numpy(), f.initialized.astype(int).to_numpy(), where='post', color=BLUE, label='EKF initialized')
        a = d['authority']
        ax.step(a.bag_s.to_numpy(), a.armed.astype(int).to_numpy(), where='post', color=ORANGE, ls='--', label='Authority armed (receipt time)')
        ax.set_ylim(-.1, 1.4)
        ax.set_yticks([0, 1], ['false', 'true'])
        ax.set_xlim(0, 250)
        ax.set_xlabel('Time since bag start (s)')
        ax.set_title(f'{label}  |  Initialization and ARM')
        ax.legend(loc='upper left', fontsize=9)
    for row in range(3):
        limits = [axs[row, col].get_ylim() for col in range(2)]
        common = min(x[0] for x in limits), max(x[1] for x in limits)
        for col in range(2):
            axs[row, col].set_ylim(common)
    fig.suptitle('2026-09-26 hardware diagnostics: source GNSS, fused position and initialization\n'
                 'Operator: initially stationary, later motion within ~2 m outdoors. No position ground truth.', fontsize=14)
    fig.savefig(HERE / 'position_and_initialization.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    datasets = {path.name.split('_')[2]: summarize(path) for path in sorted(HERE.glob('hardware_20260926_*')) if path.is_dir()}
    plot(datasets)
    print('Validated summaries and position_and_initialization.png written.')
