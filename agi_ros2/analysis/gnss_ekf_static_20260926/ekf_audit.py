#!/usr/bin/env python3
"""Read-only EKF audit of the first hardware bag's extracted observations.

Uses only the user-confirmed first 100 seconds for stationary statistics.
Later events are state-machine diagnostics, not stationary accuracy measures.
No replay/tuning is performed and current config is not a bag parameter dump.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
SOURCE = ROOT / 'agi_ros2/analysis/hardware_diagnostic_20260926/hardware_20260926_161859_702449'


def quantiles(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    return dict(zip(('min', 'median', 'p95', 'max'),
                    map(float, np.quantile(a, [0, .5, .95, 1])))) if len(a) else None


def record(row):
    names = ['bag_s', 'stamp_s', 'reset_counter', 'initialized',
             'navigation_valid', 'navigation_rejections',
             'navigation_innovation_squared', 'rtk_stamp_ns',
             'covariance_stamp_ns', 'position.x', 'position.y', 'position.z',
             'velocity.x', 'velocity.y', 'velocity.z', 'readiness_reason']
    return row[names].to_dict()


def clean(x):
    if isinstance(x, dict):
        return {str(k): clean(v) for k, v in x.items()}
    if isinstance(x, list):
        return [clean(v) for v in x]
    if isinstance(x, np.generic):
        x = x.item()
    return None if isinstance(x, float) and not np.isfinite(x) else x


def main():
    f = pd.read_csv(SOURCE / 'fused_state.csv.gz')
    n = pd.read_csv(SOURCE / 'sensors_local_navigation.csv.gz')
    imu = pd.read_csv(SOURCE / 'sensors_imu.csv.gz')
    parameters = pd.read_csv(SOURCE / 'parameter_events.csv.gz')
    static = f[(f.bag_s >= 0) & (f.bag_s < 100) & f.initialized].copy()
    # Only once per newly accepted posterior: no 500 Hz duplicate weighting.
    updates = static[static.covariance_stamp_ns.diff() > 0]
    rejected = f[f.navigation_rejections.diff() > 0]
    reset_index = f.index[f.readiness_reason.eq('EKF prediction failed; reinitialization required')][0]
    reset, previous = f.loc[reset_index], f.loc[reset_index - 1]
    accepts = f[(f.covariance_stamp_ns.diff() > 0) & f.initialized & (f.index < reset_index)]
    last_accepted = accepts.iloc[-1]
    episode = f.loc[last_accepted.name:reset_index - 1]
    source_episode = n[n.bag_s.between(last_accepted.bag_s, reset.bag_s)]
    covariance_columns = [f'{kind}_variance.{i}' for kind in ('position', 'velocity') for i in range(3)]
    columns = ['reset_counter', 'navigation_rejections', 'navigation_valid',
               'navigation_ready', 'estimator_ready', 'accuracy_known',
               'accuracy_ok', 'navigation_accepted_updates', 'readiness_reason']
    at_end = static.iloc[-1]
    # Frozen scalar F=0,G=1 case of the source's covariance injection formula.
    # Distinguish sample variance from continuous PSD; no physical unit choice
    # or replacement config value is inferred from this algebraic example.
    steps = [0.002, 0.001, 0.0002, 0.0001, 0.00005]
    report = {
        'source_directory': str(SOURCE.relative_to(ROOT)),
        'source_sha256': {name: hashlib.sha256((SOURCE / name).read_bytes()).hexdigest()
                          for name in ['fused_state.csv.gz', 'sensors_local_navigation.csv.gz',
                                       'sensors_imu.csv.gz', 'parameter_events.csv.gz']},
        'scope': 'Static statistics: 0 <= bag receipt time < 100 seconds; initialized states only. No absolute position truth.',
        'static_initialized_count': len(static),
        'static_first_initialized': record(static.iloc[0]),
        'static_last_initialized': record(at_end),
        'static_accepted_posterior_advances': len(updates),
        'static_flags': {c: static[c].value_counts().to_dict() for c in columns},
        'static_nis_once_per_posterior_update': quantiles(updates.navigation_innovation_squared),
        'static_covariance_stddev_once_per_posterior_update': {
            c.replace('variance', 'stddev'): quantiles(np.sqrt(updates[c])) for c in covariance_columns},
        'static_final_stddev': {
            c.replace('variance', 'stddev'): float(np.sqrt(at_end[c])) for c in covariance_columns},
        'static_navigation_invalid_records': [record(row) for _, row in static[~static.navigation_valid].iterrows()],
        'first_navigation_rejection': record(rejected.iloc[0]),
        'last_navigation_accepted_before_reset': record(last_accepted),
        'last_finite_state_before_reset': record(previous),
        'prediction_reset': record(reset),
        'consecutive_rejection_count_after_last_accept': int(((rejected.index > last_accepted.name) &
                                                             (rejected.index < reset_index)).sum()),
        'queue_coverage': {
            'posterior_to_reset_seconds': float((int(reset['header.stamp_ns']) - int(previous.rtk_stamp_ns)) * 1e-9),
            'imu_samples_after_posterior_through_reset': int(((imu['header.stamp_ns'] > previous.rtk_stamp_ns) &
                                                          (imu['header.stamp_ns'] <= reset['header.stamp_ns'])).sum()),
            'current_source_max_queue_size': 4096,
        },
        'rejection_episode': {
            'distinct_covariance_stamps': int(episode.covariance_stamp_ns.nunique()),
            'distinct_covariance_values': int(episode[covariance_columns].drop_duplicates().shape[0]),
            'source_navigation_messages': len(source_episode),
            'source_observation_valid_all': bool(source_episode.observation_valid.all()),
            'source_clock_aligned_all': bool(source_episode.clock_aligned.all()),
            'source_receipt_delay_s': quantiles(source_episode.bag_s - source_episode.stamp_s),
            'fused_receipt_delay_s': quantiles(episode.bag_s - episode.stamp_s),
        },
        'recorded_parameter_event_nodes': parameters.node.value_counts().to_dict(),
        'parameter_limit': 'No state_fusion parameter event exists in this extracted parameter_events topic. Current hardware.yaml was subsequently changed and is not the recording parameter snapshot.',
        'current_source_scalar_covariance_injection_example': [
            {'substep_s': dt, 'acceleration_variance_parameter': .1,
             'one_second_velocity_variance_injected': dt * .1,
             'one_second_bias_variance_injected_for_Q_1e_9': dt * 1e-9}
            for dt in steps],
        'noise_scaling_interpretation': 'Source adds dt^2*(G R_imu G^T + Q) per <=100us integration substep. For a frozen scalar model total injection in T seconds is T*dt*parameter. Thus it depends on numerical subdivision. If R_imu is 500Hz sample variance, a 2ms held sample split into 20 independent 100us increments injects approximately 1/20 of its correlated sample-noise covariance. If Q denotes continuous-time PSD, discretization should scale with dt. Units and intended convention must be established before changing values. This is current-source analysis, not proof of the exact firmware/binary used for the bag.',
    }
    out = HERE / 'ekf_audit_metrics.json'
    out.write_text(json.dumps(clean(report), ensure_ascii=False, indent=2) + '\n')
    print(out)


if __name__ == '__main__':
    main()
