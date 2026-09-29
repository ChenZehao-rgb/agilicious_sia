#!/usr/bin/env python3
"""Quantify navigation-associated position corrections without confusing motion.

An adjacent-state correction proxy subtracts previous v*dt + a*dt^2/2 from
the position increment. It is not the exact EKF innovation or Kalman gain.
GNSS acceptance is identified by an advancing rtk_stamp inside one initialized
reset segment, not navigation_accepted_updates (which also has accuracy gates).
"""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / 'hardware_diagnostic_20260926/hardware_20260926_161859_702449'
BLUE, GOLD, ORANGE = '#245c91', '#a97919', '#c25228'


def stats(values):
    v = np.asarray(values, dtype=float)
    v = v[np.isfinite(v)]
    if not len(v):
        return {'n': 0}
    return {'n': len(v), 'median': float(np.median(v)), 'p95': float(np.quantile(v, .95)),
            'max': float(v.max()), 'min': float(v.min()), 'mean': float(v.mean())}


def position_stats(frame):
    p = frame[['position.' + axis for axis in 'xyz']].to_numpy()
    return {'n': len(frame), 'receipt_interval_s': frame.bag_s.iloc[[0, -1]].tolist(),
            'span_xyz_m': np.ptp(p, axis=0).tolist(), 'end_minus_start_xyz_m': (p[-1] - p[0]).tolist(),
            'max_horizontal_from_first_m': float(np.linalg.norm(p[:, :2] - p[0, :2], axis=1).max()),
            'mean_xyz_m': p.mean(axis=0).tolist()}


def main():
    f = pd.read_csv(SOURCE / 'fused_state.csv.gz').sort_values('log_ns', kind='stable').reset_index(drop=True)
    local = pd.read_csv(SOURCE / 'sensors_local_navigation.csv.gz')
    local = local[local.observation_valid].copy()
    dt = f['header.stamp_ns'].diff().to_numpy() * 1e-9
    eligible = (f.initialized & f.initialized.shift(fill_value=False) &
                f.reset_counter.eq(f.reset_counter.shift()) & (dt > 0) & (dt <= .01))
    updated = f.rtk_stamp_ns.gt(f.rtk_stamp_ns.shift()) & eligible
    rows = f[['bag_s', 'stamp_s', 'header.stamp_ns', 'rtk_stamp_ns', 'reset_counter',
              'navigation_innovation_squared', 'navigation_rejections']].copy()
    rows['dt_s'] = dt
    rows['eligible'] = eligible
    rows['navigation_update'] = updated
    rows['measurement_age_s'] = (f['header.stamp_ns'] - f.rtk_stamp_ns) * 1e-9
    for axis in 'xyz':
        increment = f['position.' + axis].diff()
        prediction = f['velocity.' + axis].shift() * dt + .5 * f['acceleration.' + axis].shift() * dt**2
        rows['position_step_' + axis + '_m'] = increment
        rows['correction_' + axis + '_m'] = increment - prediction
    rows['horizontal_correction_m'] = np.linalg.norm(rows[['correction_x_m', 'correction_y_m']], axis=1)
    rows['vertical_correction_abs_m'] = rows.correction_z_m.abs()
    rows['correction_3d_m'] = np.linalg.norm(rows[['correction_' + a + '_m' for a in 'xyz']], axis=1)
    first100 = rows[(rows.bag_s >= 0) & (rows.bag_s < 100) & rows.eligible]
    fields = ['horizontal_correction_m', 'vertical_correction_abs_m', 'correction_3d_m', 'measurement_age_s']
    result = {'source': str(SOURCE), 'method': __doc__, 'windows': {}}
    for lo, hi in [(0, 100), (6, 10), (10, 30), (30, 60), (60, 100), (100, 210), (210, 219.7)]:
        d = rows[(rows.bag_s >= lo) & (rows.bag_s < hi) & rows.eligible]
        result['windows'][f'{lo}_{hi}'] = {
            'accepted_navigation_update_rows': int(d.navigation_update.sum()),
            'update': {key: stats(d.loc[d.navigation_update, key]) for key in fields},
            'no_update': {key: stats(d.loc[~d.navigation_update, key]) for key in fields[:3]},
        }
    fs = f[(f.bag_s >= 6) & (f.bag_s < 100) & f.initialized]
    ls = local[(local.bag_s >= 6) & (local.bag_s < 100)]
    result['common_static_window_6_100'] = {'fused': position_stats(fs), 'local_navigation': position_stats(ls)}
    result['first100_largest_corrections'] = first100.nlargest(12, 'correction_3d_m').to_dict('records')
    updates = rows[rows.navigation_update].merge(
        local[['header.stamp_ns', 'position.x', 'position.y', 'position.z', 'position_variance.0',
               'position_variance.1', 'position_variance.2', 'velocity_variance.0', 'heading_variance']],
        left_on='rtk_stamp_ns', right_on='header.stamp_ns', how='left', validate='one_to_one', suffixes=('', '_nav'))
    result['accepted_updates_matched_to_local'] = {'updates': len(updates),
                                                  'matched': int(updates['header.stamp_ns_nav'].notna().sum())}
    assert updates['header.stamp_ns_nav'].notna().all()
    updates.to_csv(HERE / 'navigation_corrections.csv.gz', index=False)
    first100[first100.navigation_update].iloc[::50].head(20).to_csv(HERE / 'source_preview.csv', index=False)
    (HERE / 'jump_results.json').write_text(json.dumps(result, indent=2, allow_nan=False))

    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.grid': True, 'grid.alpha': .18, 'figure.facecolor': 'white'})
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, constrained_layout=True)
    for ax, axis, name in zip(axes, 'xyz', ['East', 'North', 'Up (relative MSL)']):
        nav = local[local.bag_s.between(0, 100, inclusive='left')]
        state = f[f.initialized & f.bag_s.between(0, 100, inclusive='left')]
        ax.plot(nav.stamp_s.to_numpy(), nav['position.' + axis].to_numpy(), color=GOLD, lw=1.3, label='GNSS local observation')
        ax.plot(state.stamp_s.to_numpy()[::5], state['position.' + axis].to_numpy()[::5],
                color=BLUE, lw=1.25, label='EKF output')
        ax.set_ylabel(name + ' (m)')
    axes[0].legend(loc='lower left', ncol=2)
    axes[-1].set_xlim(0, 100)
    axes[-1].set_xlabel('Acquisition time relative to bag start (s)')
    fig.suptitle('First 100 s: operator-confirmed stationary interval\n'
                 'Fixed ENU origin; no surveyed position/height truth. Uninitialized EKF samples hidden.', fontsize=13)
    fig.savefig(HERE / 'static_positions.png', dpi=170)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    up = first100[first100.navigation_update]
    axes[0, 0].plot(up.bag_s.to_numpy(), up.horizontal_correction_m.to_numpy() * 100, '.', ms=2.4, color=BLUE, label='Horizontal')
    axes[0, 0].plot(up.bag_s.to_numpy(), up.vertical_correction_abs_m.to_numpy() * 100, '.', ms=2.4, color=ORANGE, label='Vertical absolute')
    axes[0, 0].set(title='Navigation-associated position corrections', ylabel='Correction proxy (cm)', xlabel='Bag receipt time (s)', xlim=(0, 100))
    axes[0, 0].legend()
    zoom = f[f.initialized & f.stamp_s.between(90, 91)]
    zu = up[up.stamp_s.between(90, 91)]
    axes[0, 1].plot(zoom.stamp_s.to_numpy(), zoom['position.z'].to_numpy(), color=BLUE, lw=1.1)
    for x in zu.stamp_s:
        axes[0, 1].axvline(x, color=GOLD, alpha=.45, lw=.8)
    axes[0, 1].set(title='One-second zoom; gold lines mark accepted updates', ylabel='EKF Up position (m)', xlabel='Acquisition time (s)', xlim=(90, 91))
    tail = f[f.bag_s.between(210, 230)].copy()
    axes[1, 0].plot(tail.bag_s.to_numpy(), tail.navigation_innovation_squared.to_numpy(), color=ORANGE, lw=1.2)
    axes[1, 0].axhline(24.322, color='#555555', ls='--', lw=1, label='Current-source gate: 24.322')
    axes[1, 0].set(title='Later interval: rejected navigation updates', ylabel='Combined navigation NIS', xlabel='Bag receipt time (s)', xlim=(210, 230))
    axes[1, 0].legend(fontsize=8)
    axes[1, 1].plot(tail.bag_s.to_numpy(), ((tail.log_ns - tail['header.stamp_ns']) * 1e-6).to_numpy(), color=BLUE, lw=1.1, label='Acquisition-to-record delay')
    axes[1, 1].plot(tail.bag_s.to_numpy(), ((tail.published_steady_time - tail.imu_receive_time) * 1000).to_numpy(),
                    color=ORANGE, lw=1.1, label='Callback-to-publish interval')
    axes[1, 1].set(title='Later interval: fusion timing', ylabel='Delay (ms)', xlabel='Bag receipt time (s)', xlim=(210, 230))
    axes[1, 1].legend(fontsize=8)
    fig.suptitle('GNSS correction steps and the separate late-recording failure\n'
                 'Correction proxy removes short-step inertial motion; later motion is not assumed stationary.', fontsize=13)
    fig.savefig(HERE / 'corrections_and_failure.png', dpi=170)
    plt.close(fig)
    print(json.dumps(result['windows']['0_100'], indent=2))
    print(json.dumps(result['common_static_window_6_100'], indent=2))


if __name__ == '__main__':
    main()
