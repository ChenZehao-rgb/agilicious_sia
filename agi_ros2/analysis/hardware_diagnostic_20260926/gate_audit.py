#!/usr/bin/env python3
"""Audit first recording's GNSS rejection episode and EKF IMU history coverage.

Inputs are the schema-decoded CSVs produced by extract.py. All diagnostic times
are reported both as bag receipt time and source stamp where relevant. Biases
are algebraically reconstructed from published acceleration/rates and exact
timestamp-matched IMU, using EkfImu::vectorToState; they are not measured truth.
"""
import json
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parent
BAG = ROOT / 'hardware_20260926_161859_702449'


def stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not values.size:
        return None
    return dict(zip(('min', 'median', 'p95', 'max'),
                    map(float, np.quantile(values, [0, .5, .95, 1]))))


def clean(value):
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, list):
        return [clean(v) for v in value]
    if isinstance(value, (np.integer, np.floating, np.bool_)):
        value = value.item()
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def record(row):
    keys = ['bag_s', 'stamp_s', 'header.stamp_ns', 'rtk_stamp_ns',
            'reset_counter', 'initialized', 'navigation_valid',
            'navigation_rejections', 'navigation_innovation_squared',
            'position.x', 'position.y', 'position.z',
            'velocity.x', 'velocity.y', 'velocity.z', 'readiness_reason']
    return clean(row[keys].to_dict())


def main():
    fused = pd.read_csv(BAG / 'fused_state.csv.gz')
    imu = pd.read_csv(BAG / 'sensors_imu.csv.gz')
    nav = pd.read_csv(BAG / 'sensors_local_navigation.csv.gz')
    raw = pd.read_csv(BAG / 'sensors_navigation.csv.gz')
    manifest = json.loads((BAG / 'manifest.json').read_text())
    reset_index = fused.index[fused.readiness_reason.eq(
        'EKF prediction failed; reinitialization required')][0]
    reset = fused.loc[reset_index]
    previous = fused.loc[reset_index - 1]
    posterior_ns = int(previous.rtk_stamp_ns)
    reset_ns = int(reset['header.stamp_ns'])
    rejected = fused[(fused.navigation_rejections.diff() > 0) &
                     (fused.index < reset_index)]
    accepted = fused[(fused.rtk_stamp_ns.diff() > 0) & fused.initialized &
                     (fused.index < reset_index)]
    last_accepted = accepted.iloc[-1]
    final_episode = rejected[rejected.index > last_accepted.name]
    # Exact timestamp identity avoids an arbitrary nearest-neighbour match.
    joined = fused[fused.initialized].merge(imu, on='header.stamp_ns',
                                           suffixes=('', '_imu'), validate='one_to_one')
    q = joined[['orientation.x', 'orientation.y', 'orientation.z', 'orientation.w']].to_numpy()
    q /= np.linalg.norm(q, axis=1)[:, None]
    x, y, z, w = q.T
    rotation = np.empty((len(q), 3, 3))
    rotation[:, 0, :] = np.array([1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)]).T
    rotation[:, 1, :] = np.array([2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)]).T
    rotation[:, 2, :] = np.array([2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]).T
    specific_force = joined[['linear_acceleration.x', 'linear_acceleration.y', 'linear_acceleration.z']].to_numpy()
    world_force = joined[['acceleration.x', 'acceleration.y', 'acceleration.z']].to_numpy() + [0, 0, 9.8066]
    acceleration_bias = specific_force - np.einsum('nji,nj->ni', rotation, world_force)
    gyro_bias = joined[['angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z']].to_numpy() - joined[['body_rates.x', 'body_rates.y', 'body_rates.z']].to_numpy()
    roll = np.degrees(np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
    pitch = np.degrees(np.arcsin(np.clip(2*(w*y-z*x), -1, 1)))
    yaw = np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    for axis, name in enumerate('xyz'):
        joined['reconstructed_ba.' + name] = acceleration_bias[:, axis]
        joined['reconstructed_bw.' + name] = gyro_bias[:, axis]
    joined['roll_deg'], joined['pitch_deg'] = roll, pitch
    source_nav = nav[nav.observation_valid].sort_values('stamp_s')
    source_heading = np.interp(joined.stamp_s, source_nav.stamp_s,
                               np.unwrap(source_nav.heading))
    joined['heading_disagreement_deg'] = np.degrees(np.arctan2(
        np.sin(yaw-source_heading), np.cos(yaw-source_heading)))
    for prefix in ('position', 'velocity'):
        for axis in 'xyz':
            joined[prefix+'_disagreement.'+axis] = joined[prefix+'.'+axis] - np.interp(
                joined.stamp_s, source_nav.stamp_s, source_nav[prefix+'.'+axis])
    windows = []
    for lo, hi in [(6, 30), (30, 60), (60, 120), (120, 180), (180, 210),
                   (210, 217), (217, 219.5), (219.5, 224), (224, 227.608), (244.8, 247.7)]:
        f = joined[joined.stamp_s.between(lo, hi)]
        n = nav[nav.stamp_s.between(lo, hi)]
        r = raw[raw.stamp_s.between(lo, hi)]
        wnorm = np.linalg.norm(f[['angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z']], axis=1)
        anorm = np.linalg.norm(f[['linear_acceleration.x', 'linear_acceleration.y', 'linear_acceleration.z']], axis=1)
        item = dict(source_interval_s=[lo, hi], fused_samples=len(f), local_navigation_samples=len(n),
                    observation_valid_count=int(n.observation_valid.sum()),
                    heading_valid_count=int(n.heading_valid.sum()),
                    clock_aligned_count=int(n.clock_aligned.sum()),
                    fix_type_counts=n.fix_type.value_counts().to_dict(),
                    local_navigation_receipt_age_s=stats(n.bag_s-n.stamp_s),
                    fused_receipt_age_s=stats(f.bag_s-f.stamp_s),
                    imu_gyro_norm_rad_s=stats(wnorm), imu_acceleration_norm_m_s2=stats(anorm),
                    fused_roll_deg=stats(f.roll_deg), fused_pitch_deg=stats(f.pitch_deg),
                    heading_disagreement_deg=stats(f.heading_disagreement_deg),
                    receiver_horizontal_accuracy_m=stats(r.horizontal_accuracy),
                    receiver_vertical_accuracy_m=stats(r.vertical_accuracy),
                    receiver_velocity_accuracy_m_s=stats(r.velocity_accuracy))
        for prefix in ('position', 'velocity', 'reconstructed_ba', 'reconstructed_bw',
                       'position_disagreement', 'velocity_disagreement'):
            item[prefix] = {axis: stats(f[prefix+'.'+axis]) for axis in 'xyz'}
        item['gnss_position_m'] = {axis: stats(n['position.'+axis]) for axis in 'xyz'}
        windows.append(item)
    continuous_nav = nav[nav.bag_s.between(float(last_accepted.bag_s), float(reset.bag_s))]
    episode_fused = fused.loc[last_accepted.name:reset_index-1]
    # Interpolation is for position disagreement, not for reproducing EKF timing.
    difference = np.column_stack([
        episode_fused['position.'+a] - np.interp(episode_fused.stamp_s,
            source_nav.stamp_s, source_nav['position.'+a]) for a in 'xyz'])
    report = dict(
        bag=manifest['bag'],
        timing_note='bag_s: recorder receipt; stamp_s: sensor/source time minus recorder start. They diverge during EKF rejection backlog.',
        state_transitions=[record(row) for _, row in fused[
            fused.initialized.ne(fused.initialized.shift()) |
            fused.navigation_valid.ne(fused.navigation_valid.shift()) |
            fused.reset_counter.ne(fused.reset_counter.shift())].iterrows()],
        first_rejection=record(rejected.iloc[0]), last_accepted_before_reset=record(last_accepted),
        last_finite_before_reset=record(previous), prediction_reset=record(reset),
        rejected_updates_before_reset=len(rejected),
        uninterrupted_rejections_after_last_accept=len(final_episode),
        rejection_events=[record(row) for _, row in rejected.iterrows()],
        queue_coverage=dict(max_queue_size=4096, last_posterior_ns=posterior_ns,
            reset_imu_stamp_ns=reset_ns, elapsed_source_s=(reset_ns-posterior_ns)*1e-9,
            imu_samples_strictly_after_posterior_through_reset=int(((imu['header.stamp_ns']>posterior_ns)&(imu['header.stamp_ns']<=reset_ns)).sum()),
            fused_callbacks_strictly_after_posterior_through_reset=int(((fused['header.stamp_ns']>posterior_ns)&(fused['header.stamp_ns']<=reset_ns)).sum()),
            source_evidence=['agilib/include/agilib/estimator/ekf_imu/ekf_imu.hpp:100',
                'agilib/src/estimator/ekf_imu/ekf_imu.cpp:128-159',
                'agilib/src/estimator/ekf_imu/ekf_imu.cpp:212-216',
                'agilib/src/estimator/ekf_imu/ekf_imu.cpp:383-388',
                'agi_ros2/src/state_fusion_node.cpp:324-327']),
        source_during_last_rejection_episode=dict(samples=len(continuous_nav),
            all_observation_valid=bool(continuous_nav.observation_valid.all()),
            all_heading_valid=bool(continuous_nav.heading_valid.all()),
            all_clock_aligned=bool(continuous_nav.clock_aligned.all()),
            fix_type_counts=continuous_nav.fix_type.value_counts().to_dict(),
            receipt_age_s=stats(continuous_nav.bag_s-continuous_nav.stamp_s),
            source_sample_gap_s=stats(continuous_nav.stamp_s.diff()),
            fused_receipt_age_s=stats(episode_fused.bag_s-episode_fused.stamp_s),
            position_disagreement_m={a:stats(difference[:,j]) for j,a in enumerate('xyz')}),
        matched_fused_imu_samples=len(joined), windows=windows,
        interpretation=dict(
            established='After the last accepted GNSS update, 4096 further IMU samples cover 8.202381056 s. This exactly hits the current EKF history guard and matches the prediction-failed reset. Source GNSS continued publishing valid observations while accepted navigation aged out.',
            inference='Each rejected GNSS update restarts covariance propagation from the frozen posterior; growing fused message latency during the episode is consistent with the increasing replay cost, but CPU profiling was not recorded.',
            limitation='The stored NIS is joint position/velocity/heading residual; it cannot uniquely identify the triggering axis without offline EKF replay. Algebraically reconstructed biases are estimator internal values, not independent calibration estimates.'))
    (ROOT/'gate_audit.json').write_text(json.dumps(clean(report), ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(clean({k:report[k] for k in ('first_rejection','last_accepted_before_reset',
        'last_finite_before_reset','prediction_reset','rejected_updates_before_reset',
        'uninterrupted_rejections_after_last_accept','queue_coverage',
        'source_during_last_rejection_episode')}),ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
