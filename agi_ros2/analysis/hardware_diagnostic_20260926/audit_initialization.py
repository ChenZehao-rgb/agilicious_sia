#!/usr/bin/env python3
"""Audit recorded startup gates and reproduce the IMU initialization windows.

Inputs are extract.py's lossless numeric CSV tables. This is diagnostic analysis;
it changes no runtime files. Execute from any directory with Python/numpy/pandas.
Raw rolling windows follow imu_initialization.h's retain-one-boundary-sample
rule. Observed collection runs use FusedState.readiness_reason, so they reflect
the online first-stage gate rather than a speculative cross-DDS arrival replay.
"""

import json
from pathlib import Path

import numpy as np
import pandas as pd


BASE = Path(__file__).resolve().parent
G = 9.8066
COLLECTING = "Collecting stationary IMU samples; checking gravity, gyro bias and noise"
PARAMS = dict(duration_s=3.0, minimum_samples=1000, max_gyro_bias_rad_s=.15,
              max_gyro_axis_std_rad_s=.02, max_accel_axis_std_m_s2=.2,
              gravity_tolerance_m_s2=.7, navigation_max_speed_m_s=.3,
              max_imu_gap_s=.025)


def stats(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return dict(n=0)
    return dict(n=int(len(a)), minimum=float(a.min()), median=float(np.median(a)),
                p95=float(np.quantile(a, .95)), maximum=float(a.max()))


def records(frame):
    return json.loads(frame.to_json(orient="records", double_precision=12))


def runs(frame, keys):
    group = frame[keys].ne(frame[keys].shift()).any(axis=1).cumsum()
    output = []
    for _, part in frame.groupby(group, sort=False):
        row = {k: part.iloc[0][k] for k in keys}
        row.update(start_bag_s=float(part.bag_s.iloc[0]), end_bag_s=float(part.bag_s.iloc[-1]),
                   start_stamp_s=float(part.stamp_s.iloc[0]), end_stamp_s=float(part.stamp_s.iloc[-1]),
                   samples=len(part), stamp_span_s=float(part.stamp_s.iloc[-1] - part.stamp_s.iloc[0]))
        output.append(row)
    return pd.DataFrame(output)


def rolling(imu):
    t = imu.stamp_s.to_numpy()
    # Runtime removes first sample while sample[1].t <= current.t - 3 seconds.
    left = np.maximum(0, np.searchsorted(t, t - PARAMS['duration_s'], side="right") - 1)
    starts = np.maximum.accumulate(np.r_[0, np.where((np.diff(t) <= 0) | (np.diff(t) > .025),
                                                   np.arange(1, len(t)), 0)])
    left = np.maximum(left, starts)
    count = np.arange(len(t)) - left + 1
    result = pd.DataFrame(dict(bag_s=imu.bag_s, stamp_s=t, left=left,
                               samples=count, duration_s=t - t[left]))
    for prefix, column in (("acc", "linear_acceleration"), ("gyro", "angular_velocity")):
        value = imu[[f"{column}.{axis}" for axis in "xyz"]].to_numpy()
        summed = np.vstack((np.zeros(3), value.cumsum(axis=0)))
        squared = np.vstack((np.zeros(3), (value * value).cumsum(axis=0)))
        mean = (summed[1:] - summed[left]) / count[:, None]
        std = np.sqrt(np.maximum(0, (squared[1:] - squared[left]) / count[:, None] - mean * mean))
        result[f"{prefix}_mean_norm"] = np.linalg.norm(mean, axis=1)
        result[f"{prefix}_max_axis_std"] = std.max(axis=1)
        for j, axis in enumerate("xyz"):
            result[f"{prefix}_mean_{axis}"] = mean[:, j]
            result[f"{prefix}_std_{axis}"] = std[:, j]
    result["gravity_residual"] = np.abs(result.acc_mean_norm - G)
    result["full_window"] = (result.samples >= 1000) & (result.duration_s >= 3.)
    result["gyro_bias_pass"] = result.gyro_mean_norm <= .15
    result["gyro_noise_pass"] = result.gyro_max_axis_std <= .02
    result["accel_noise_pass"] = result.acc_max_axis_std <= .2
    result["gravity_pass"] = result.gravity_residual <= .7
    result["all_imu_pass"] = result[["full_window", "gyro_bias_pass", "gyro_noise_pass",
                                     "accel_noise_pass", "gravity_pass"]].all(axis=1)
    return result


def summarize_windows(frame):
    full = frame[frame.full_window]
    return dict(full_windows=len(full), all_imu_pass=int(full.all_imu_pass.sum()),
                failing_windows={k: int((~full[k]).sum()) for k in
                                 ["gyro_bias_pass", "gyro_noise_pass", "accel_noise_pass", "gravity_pass"]},
                metrics={k: stats(full[k]) for k in ["acc_mean_norm", "gravity_residual", "acc_max_axis_std",
                                                     "gyro_mean_norm", "gyro_max_axis_std"]})


def analyze(path):
    imu = pd.read_csv(path / 'sensors_imu.csv.gz')
    nav = pd.read_csv(path / 'sensors_local_navigation.csv.gz')
    authority = pd.read_csv(path / 'authority.csv.gz')
    fused = pd.read_csv(path / 'fused_state.csv.gz')
    windows = rolling(imu)
    # Independent direct computation checks the cumulative-sum implementation.
    validation_errors = []
    for k in np.linspace(1502, len(imu) - 1, 31, dtype=int):
        row = windows.iloc[k]
        part = imu.iloc[int(row.left):k + 1]
        acc = part[[f'linear_acceleration.{axis}' for axis in 'xyz']].to_numpy()
        gyro = part[[f'angular_velocity.{axis}' for axis in 'xyz']].to_numpy()
        validation_errors.extend([abs(np.linalg.norm(acc.mean(axis=0)) - row.acc_mean_norm),
                                  abs(acc.std(axis=0).max() - row.acc_max_axis_std),
                                  abs(gyro.std(axis=0).max() - row.gyro_max_axis_std)])
    assert max(validation_errors) < 1e-7
    windows.to_csv(BASE / f"{path.name}_imu_windows.csv.gz", index=False, compression='gzip')
    nav["speed"] = np.linalg.norm(nav[[f"velocity.{x}" for x in "xyz"]], axis=1)
    authority_runs = runs(authority, ['armed', 'rc_link'])
    fused_runs = runs(fused, ['initialized', 'reset_counter'])
    reason_runs = runs(fused, ['initialized', 'reset_counter', 'readiness_reason'])
    collections = reason_runs[(~reason_runs.initialized) & (reason_runs.readiness_reason == COLLECTING)]
    collection_detail = []
    for _, row in collections.iterrows():
        part = imu[(imu.stamp_s >= row.start_stamp_s - 1e-6) & (imu.stamp_s <= row.end_stamp_s + 1e-6)]
        if part.empty:
            continue
        acc = part[[f"linear_acceleration.{x}" for x in "xyz"]].to_numpy()
        gyro = part[[f"angular_velocity.{x}" for x in "xyz"]].to_numpy()
        # When collection lasts >3s the full rolling metrics are summarized separately.
        collection_detail.append(dict(start_bag_s=float(row.start_bag_s), end_bag_s=float(row.end_bag_s),
                                      start_stamp_s=float(row.start_stamp_s), end_stamp_s=float(row.end_stamp_s),
                                      span_s=float(row.stamp_span_s), recorded_imu_samples=len(part),
                                      acceleration_mean_norm=float(np.linalg.norm(acc.mean(axis=0))),
                                      gravity_residual=float(abs(np.linalg.norm(acc.mean(axis=0)) - G)),
                                      acceleration_max_axis_std=float(acc.std(axis=0).max()),
                                      gyro_mean_norm=float(np.linalg.norm(gyro.mean(axis=0))),
                                      gyro_max_axis_std=float(gyro.std(axis=0).max())))
    gaps = imu.assign(gap_s=imu.stamp_s.diff())
    reset_events = fused.loc[fused.reset_counter.ne(fused.reset_counter.shift()),
                            ['bag_s', 'stamp_s', 'initialized', 'reset_counter', 'readiness_reason']]
    failure_evidence = []
    for k in fused.index[fused.readiness_reason == 'EKF prediction failed; reinitialization required']:
        if k == 0:
            continue
        row, previous = fused.loc[k], fused.loc[k - 1]
        last_fix = int(previous.rtk_stamp_ns)
        count = ((imu['header.stamp_ns'] > last_fix) &
                 (imu['header.stamp_ns'] <= int(row['header.stamp_ns']))).sum()
        failure_evidence.append(dict(bag_s=float(row.bag_s), stamp_s=float(row.stamp_s),
                                     previous_accepted_fix_stamp_ns=last_fix,
                                     elapsed_since_accepted_fix_s=(int(row['header.stamp_ns']) - last_fix) * 1e-9,
                                     imu_samples_after_accepted_fix=int(count), ekf_imu_queue_capacity=4096,
                                     navigation_rejections_before_reset=int(previous.navigation_rejections),
                                     code_mechanism='EkfImu::propagatePrior returns false when queue length is 4096 and oldest IMU is newer than posterior time.'))
    successful = fused.loc[fused.initialized & ~fused.initialized.shift(fill_value=False)]
    successes = []
    for _, row in successful.iterrows():
        at = windows.iloc[np.abs(windows.stamp_s.to_numpy() - row.stamp_s).argmin()]
        successes.append(dict(bag_s=float(row.bag_s), stamp_s=float(row.stamp_s),
                              reset_counter=int(row.reset_counter),
                              raw_window=records(at.to_frame().T)[0]))
    # Approximate bag-order prerequisites, explicitly not exact subscription callback order.
    joined = pd.merge_asof(imu[['bag_s', 'stamp_s']].sort_values('bag_s'),
                           authority[['bag_s', 'stamp_s', 'armed', 'rc_link']].sort_values('bag_s'),
                           on='bag_s', direction='backward', suffixes=('', '_authority'))
    joined = pd.merge_asof(joined, nav[['bag_s', 'stamp_s', 'speed']].rename(columns={'stamp_s': 'stamp_s_nav'}),
                           on='bag_s', direction='backward')
    joined['preconditions_proxy'] = ((joined.armed == False) & (joined.rc_link == True) &
                                     (joined.speed <= .3) & (joined.stamp_s - joined.stamp_s_nav <= .3) &
                                     (joined.stamp_s - joined.stamp_s_nav >= -.01) &
                                     (joined.bag_s - joined.stamp_s_authority <= .1))
    intervals = dict(initial_first_10s=summarize_windows(windows[windows.stamp_s <= 10]),
                     initial_first_30s=summarize_windows(windows[windows.stamp_s <= 30]),
                     all_recording=summarize_windows(windows))
    if '161859' in path.name:
        intervals['user_confirmed_static_first_120s'] = summarize_windows(windows[windows.stamp_s <= 120])
        intervals['before_late_reset_215_to_228s'] = summarize_windows(windows[(windows.stamp_s >= 215) & (windows.stamp_s <= 228)])
        intervals['after_late_reset_228_to_248s'] = summarize_windows(windows[windows.stamp_s >= 228])
    return dict(bag=path.name, sample_counts={k: len(v) for k, v in
                 [('imu', imu), ('local_navigation', nav), ('authority', authority), ('fused', fused)]},
                authority_runs=records(authority_runs), fused_runs=records(fused_runs),
                reset_events=records(reset_events), successful_initializations=successes,
                prediction_failure_evidence=failure_evidence,
                rolling_validation=dict(direct_windows_checked=31, max_absolute_metric_difference=max(validation_errors)),
                observed_collection_runs=collection_detail,
                longest_observed_collection_span_s=float(collections.stamp_span_s.max()),
                imu_nonpositive_or_over_25ms_gaps=records(gaps.loc[(gaps.gap_s <= 0) | (gaps.gap_s > .025),
                                                                  ['bag_s', 'stamp_s', 'gap_s']]),
                imu_interval_s=stats(gaps.gap_s),
                initial_navigation_speed_exceedances=records(nav.loc[(nav.bag_s <= 12) & (nav.speed > .3),
                                                                      ['bag_s', 'stamp_s', 'speed']]),
                raw_imu_window_summaries=intervals,
                preconditions_bag_order_proxy_samples=int(joined.preconditions_proxy.sum()))


def main():
    result = dict(methodology={
        'units': 'seconds from bag start; accelerations m/s^2; angular rates rad/s',
        'thresholds': PARAMS,
        'rolling_window_rule': 'Keep the most recent sample at/before t-3s and every later sample; reset across gaps >25ms or nonpositive time.',
        'observed_collection': 'Contiguous uninitialized FusedState rows whose recorded reason is Collecting stationary IMU samples; first-stage failures clear online buffer.',
        'collection_metrics': 'Metrics over each contiguous recorded collecting segment, not a rolling 3s window when segment exceeds 3s.',
        'raw_windows': 'Ignore navigation/authority resets to isolate IMU-only physical gates; this does not claim online eligibility.',
        'limitations': ['Bag reception order differs from subscriber callback order. The preconditions proxy is approximate and excludes actual monotonic receipt ages.',
                       'User confirms approximately first 120s of first bag stationary; exact later movement timestamps and external ground truth unavailable.',
                       'Low IMU noise alone does not prove no translation. Armed is a software startup exclusion even when motors spin without vehicle motion.',
                       'A few initial IMU/fused messages were not recorded; observed segments reflect available rows.']},
                  bags=[analyze(p) for p in sorted(BASE.glob('hardware_*')) if p.is_dir()])
    (BASE / 'initialization_audit.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    for bag in result['bags']:
        print(bag['bag'], 'initialized:', len(bag['successful_initializations']),
              'longest collecting segment:', round(bag['longest_observed_collection_span_s'], 6))


if __name__ == '__main__':
    main()
