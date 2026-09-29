#!/usr/bin/env python3
"""Read-only static-bag analysis; receipt time is discontinuous at the clock step.

The user reports the vehicle was stationary. Position drift is relative, not
absolute accuracy. Runtime parameters are not fully recorded in this bag.
"""
from pathlib import Path
import json
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
SOURCE = HERE / 'hardware_20260929_112215_827325'


def quantiles(values):
    v = np.asarray(values, dtype=float)
    v = v[np.isfinite(v)]
    return dict(zip(['min', 'median', 'p95', 'max'], np.quantile(v, [0, .5, .95, 1]).tolist()))


def positionStats(d):
    p = d[['position.' + x for x in 'xyz']].to_numpy()
    v = d[['velocity.' + x for x in 'xyz']].to_numpy()
    return {'n': len(d), 'acquisition_interval_s': d.stamp_s.iloc[[0, -1]].tolist(),
            'axis_span_m': np.ptp(p, axis=0).tolist(),
            'end_minus_start_m': (p[-1] - p[0]).tolist(),
            'horizontal_max_from_first_m': float(np.linalg.norm(p[:, :2] - p[0, :2], axis=1).max()),
            'speed_mps': quantiles(np.linalg.norm(v, axis=1))}


def main():
    frames = {n: pd.read_csv(SOURCE / (n + '.csv.gz')) for n in
              ['fused_state', 'sensors_local_navigation', 'sensors_navigation',
               'sensors_imu', 'authority', 'msp_decoded_state', 'parameter_events']}
    f, n, imu = (frames[x] for x in ['fused_state', 'sensors_local_navigation', 'sensors_imu'])
    initialized = f[f.initialized]
    updates = initialized[initialized.rtk_stamp_ns.diff() > 0]
    # Integer timestamp subtraction before conversion to float.
    wall_delta = f.log_ns.diff() * 1e-9
    steady_delta = f.published_steady_time.diff()
    jump_idx = (wall_delta - steady_delta).idxmax()
    jump = f.loc[jump_idx]
    first, last = initialized.stamp_s.iloc[[0, -1]]
    valid = n[n.observation_valid]
    same_nav = valid[valid.stamp_s.between(first, last)]
    all_imu_acc = imu[['linear_acceleration.' + a for a in 'xyz']].to_numpy()
    pre = imu[imu.log_ns < jump.log_ns]
    pre_acc = pre[['linear_acceleration.' + a for a in 'xyz']].to_numpy()
    pre_gyr = pre[['angular_velocity.' + a for a in 'xyz']].to_numpy()
    all_gyr = imu[['angular_velocity.' + a for a in 'xyz']].to_numpy()
    max_gyr_idx = np.linalg.norm(all_gyr, axis=1).argmax()
    cov_cols = [k + '_variance.' + str(i) for k in ['position', 'velocity'] for i in range(3)]
    report = {
        'scope': __doc__,
        'bag_manifest': json.loads((SOURCE / 'manifest.json').read_text()),
        'initialized_count': len(initialized),
        'initialized_acquisition_interval_s': [first, last],
        'initialized_duration_s': last - first,
        'accepted_updates_excluding_initialization': len(updates),
        'rejections_max': int(f.navigation_rejections.max()),
        'accepted_update_nis': quantiles(updates.navigation_innovation_squared),
        'clock_step': {
            'last_pre_step_receipt_s': float(f.loc[jump_idx - 1, 'bag_s']),
            'first_post_step_receipt_s': float(jump.bag_s),
            'receipt_delta_s': float(wall_delta.loc[jump_idx]),
            'steady_delta_s': float(steady_delta.loc[jump_idx]),
            'estimated_positive_step_s': float((wall_delta - steady_delta).loc[jump_idx]),
            'method': 'Adjacent fused-state receipt time delta minus publisher steady-time delta; includes small DDS scheduling error.',
            'physical_fused_stream_duration_s': float(f.published_steady_time.iloc[-1] - f.published_steady_time.iloc[0]),
        },
        'all_valid_navigation': positionStats(valid),
        'same_acquisition_window': {'fused': positionStats(initialized), 'navigation': positionStats(same_nav)},
        'navigation_horizontal_diameter_m': float(max(np.linalg.norm(valid[['position.x', 'position.y']].to_numpy() - row, axis=1).max() for row in valid[['position.x', 'position.y']].to_numpy())),
        'observed_measurement_variances': {c: sorted(valid[c].unique().tolist()) for c in cov_cols + ['heading_variance']},
        'receiver_reported_accuracy': {c: quantiles(valid[c]) for c in ['horizontal_accuracy', 'vertical_accuracy', 'velocity_accuracy']},
        'pre_clock_jump_imu': {'samples': len(pre), 'mean_acceleration_xyz_mps2': pre_acc.mean(axis=0).tolist(),
            'mean_acceleration_norm_mps2': float(np.linalg.norm(pre_acc, axis=1).mean()),
            'acceleration_std_xyz_mps2': pre_acc.std(axis=0).tolist(),
            'gyro_mean_xyz_radps': pre_gyr.mean(axis=0).tolist(),
            'gyro_std_xyz_radps': pre_gyr.std(axis=0).tolist()},
        'later_sensor_disturbance': {'max_gyro_norm_radps': float(np.linalg.norm(all_gyr, axis=1).max()),
            'max_gyro_bag_s': float(imu.iloc[max_gyr_idx].bag_s),
            'acceleration_norm_mps2': quantiles(np.linalg.norm(all_imu_acc, axis=1)),
            'note': 'Reported stationary; sensor rotation/vibration/anomaly cannot be assigned a cause or translated into position motion from this bag.'},
        'all_authority_armed_values': sorted(frames['authority'].armed.unique().tolist()),
        'recorded_parameter_nodes': sorted(frames['parameter_events'].node.unique().tolist()),
        'satellite_count_limit': 'No satellite count field in recorded Navigation/LocalNavigation or MAVLink diagnostic schemas; approximately 17 is operator-reported.',
        'runtime_config_limit': 'rosout records /home/ubuntu/agilicious_sia/agi_ros2/config/hardware.yaml, without full content or deployed binary hash.',
    }
    (HERE / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    columns = ['bag_s', 'stamp_s', 'published_steady_time', 'initialized', 'reset_counter',
               'navigation_innovation_squared', 'navigation_rejections', 'readiness_reason']
    changes = (f.initialized.ne(f.initialized.shift()) | f.reset_counter.ne(f.reset_counter.shift()) |
               f.readiness_reason.ne(f.readiness_reason.shift()))
    f.loc[changes, columns].to_csv(HERE / 'fusion_transitions.csv', index=False)
    updates[['bag_s', 'stamp_s', 'rtk_stamp_ns', 'navigation_innovation_squared']].to_csv(HERE / 'accepted_updates.csv', index=False)
    assert len(updates) == 317 and int(f.navigation_rejections.max()) == 0
    assert updates.navigation_innovation_squared.max() < 1
    assert 33.39 < report['clock_step']['estimated_positive_step_s'] < 33.42
    assert not frames['authority'].armed.any()
    print(json.dumps({k: v for k, v in report.items() if k not in ['bag_manifest', 'scope']}, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
