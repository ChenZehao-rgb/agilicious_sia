#!/usr/bin/env python3
"""Inspect GNSS quality; all primary windows use receipt time from bag metadata start.

This companion consumes previously extracted per-message CSVs. The owning task
independently checks them against MCAP. No source files or tuning are changed.
"""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SOURCE = ROOT / 'agi_ros2/analysis/hardware_diagnostic_20260926/hardware_20260926_161859_702449'
BAG = ROOT / 'bags/hardware_20260926_161859_702449'
sys.path.insert(0, str(ROOT / 'agi_ros2/scripts'))
from shadow_support import local_position


def stats(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {'n': 0}
    return dict(n=len(a), min=float(a.min()), p05=float(np.quantile(a, .05)),
                median=float(np.median(a)), p95=float(np.quantile(a, .95)),
                max=float(a.max()), mean=float(a.mean()), std=float(a.std()))


def counts(frame, key):
    return {str(k): int(v) for k, v in frame[key].value_counts(dropna=False).items()}


def horizontal_diameter(position):
    return float(np.sqrt(max(np.max(np.sum((chunk[:, None, :2] - position[None, :, :2]) ** 2, axis=2))
                             for chunk in np.array_split(position, max(1, len(position) // 256)))))


def endpoint_windows(frame, lower):
    beginning = frame[frame.valid_navigation & frame.bag_s.between(lower, lower + 10, inclusive='left')]
    end = frame[frame.valid_navigation & frame.bag_s.between(90, 100, inclusive='left')]
    keys = ['east_m', 'north_m', 'up_m']
    delta = end[keys].mean() - beginning[keys].mean()
    return dict(method=f'Mean over receipt-time [90,100) minus mean over [{lower},{lower + 10}) seconds',
                start_rows=len(beginning), end_rows=len(end), start_mean_enu_m=beginning[keys].mean().tolist(),
                end_mean_enu_m=end[keys].mean().tolist(), displacement_enu_m=delta.tolist(),
                horizontal_displacement_m=float(np.linalg.norm(delta.iloc[:2])),
                heading_change_deg=float(np.angle(np.exp(1j * end.heading).mean() / np.exp(1j * beginning.heading).mean()) * 180 / np.pi))


def summary(frame):
    d = frame[frame.valid_navigation].copy()
    p = d[['east_m', 'north_m', 'up_m']].to_numpy()
    v = d[['velocity.' + a for a in 'xyz']].to_numpy()
    t = d.stamp_s.to_numpy()
    delta_t = np.diff(t)
    valid_step = (delta_t > 0) & (delta_t <= .3) & (d.source_session.iloc[1:].to_numpy() == d.source_session.iloc[:-1].to_numpy())
    position_delta = np.diff(p, axis=0)
    velocity_integral = .5 * (v[:-1] + v[1:]) * delta_t[:, None]
    position_delta_supported = position_delta[valid_step].sum(axis=0)
    integrated_velocity = velocity_integral[valid_step].sum(axis=0)
    heading = np.unwrap(d.heading.to_numpy()) * 180 / np.pi
    return dict(
        received_rows=len(frame), valid_rows=len(d), invalid_rows=int((~frame.valid_navigation).sum()),
        receipt_interval_s=d.bag_s.iloc[[0, -1]].tolist(),
        measurement_interval_s=d.stamp_s.iloc[[0, -1]].tolist(),
        fix_type_counts=counts(frame, 'fix_type'), heading_valid_counts=counts(frame, 'heading_valid'),
        clock_aligned_counts=counts(frame, 'clock_aligned'), source_session_counts=counts(frame, 'source_session'),
        coordinate_min_enu_m=p.min(axis=0).tolist(), coordinate_max_enu_m=p.max(axis=0).tolist(),
        axis_ranges_enu_m=np.ptp(p, axis=0).tolist(),
        horizontal_diameter_m=horizontal_diameter(p),
        first_to_last_enu_m=(p[-1] - p[0]).tolist(),
        first_to_last_horizontal_m=float(np.linalg.norm(p[-1, :2] - p[0, :2])),
        horizontal_excursion_from_first_m=stats(np.linalg.norm(p[:, :2] - p[0, :2], axis=1)),
        velocity_enu_m_s={a: stats(v[:, k]) for k, a in enumerate('enu')},
        speed_3d_m_s=stats(np.linalg.norm(v, axis=1)),
        speed_horizontal_m_s=stats(np.linalg.norm(v[:, :2], axis=1)),
        speed_over_point3_count=int((np.linalg.norm(v, axis=1) > .3).sum()),
        accuracy={k: stats(d[k]) for k in ['horizontal_accuracy', 'vertical_accuracy', 'velocity_accuracy']},
        heading_degrees_unwrapped=stats(heading), heading_first_to_last_deg=float(heading[-1] - heading[0]),
        heading_range_deg=float(np.ptp(heading)),
        header_gaps_s=stats(delta_t), header_gaps_over_point3=int((delta_t > .3).sum()),
        duplicate_header_count=int(d['header.stamp_ns'].duplicated().sum()),
        nonpositive_header_gaps=int((delta_t <= 0).sum()),
        bag_receipt_age_s=stats((d.log_ns - d['header.stamp_ns']) * 1e-9),
        receipt_age_over_point3_count=int(((d.log_ns - d['header.stamp_ns']) > 300_000_000).sum()),
        header_before_bag_start_count=int((d.stamp_s < 0).sum()),
        measurement_rate_hz=float((len(d) - 1) / (t[-1] - t[0])),
        velocity_consistency=dict(
            method='Trapezoidal ENU velocity integral using source header time; skip >0.3 s, nonpositive, or session-crossing intervals.',
            supported_duration_s=float(delta_t[valid_step].sum()),
            omitted_duration_s=float(delta_t[~valid_step].sum()),
            supported_position_displacement_enu_m=position_delta_supported.tolist(),
            integrated_velocity_enu_m=integrated_velocity.tolist(),
            displacement_minus_velocity_integral_enu_m=(position_delta_supported - integrated_velocity).tolist(),
            per_frame_finite_difference_minus_mean_velocity_enu_m_s={
                a: stats((position_delta[valid_step] / delta_t[valid_step, None] - .5 * (v[:-1] + v[1:])[valid_step])[:, k])
                for k, a in enumerate('enu')
            },
        ),
    )


def main():
    metadata = yaml.safe_load((BAG / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    start_ns = metadata['starting_time']['nanoseconds_since_epoch']
    manifest = json.loads((SOURCE / 'manifest.json').read_text())
    assert manifest['start_ns'] == start_ns
    n = pd.read_csv(SOURCE / 'sensors_navigation.csv.gz')
    local = pd.read_csv(SOURCE / 'sensors_local_navigation.csv.gz')
    origin = pd.read_csv(SOURCE / 'navigation_origin.csv.gz')
    diagnostic = pd.read_csv(SOURCE / 'sensors_mavlink_status.csv.gz')
    assert np.max(np.abs(n.bag_s - (n.log_ns - start_ns) * 1e-9)) < 1e-10
    assert np.max(np.abs(n.stamp_s - (n['header.stamp_ns'] - start_ns) * 1e-9)) < 1e-10
    for key in ['latitude', 'longitude', 'altitude', 'altitude_reference', 'session_id']:
        assert origin[key].nunique() == 1, key
    o = tuple(origin.iloc[0][['latitude', 'longitude', 'altitude']])
    basis = origin.altitude_reference.iloc[0]
    n['valid_navigation'] = (n.fix_type.between(3, 6) & n.clock_aligned & n.heading_valid &
                             np.isfinite(n[['latitude', 'longitude', 'altitude', 'heading',
                                            'velocity.x', 'velocity.y', 'velocity.z']]).all(axis=1))
    coordinates = np.array([local_position(o, tuple(row), basis) if valid else [np.nan] * 3
                            for row, valid in zip(n[['latitude', 'longitude', 'altitude']].to_numpy(), n.valid_navigation)])
    for k, key in enumerate(['east_m', 'north_m', 'up_m']):
        n[key] = coordinates[:, k]
    # Exact integer stamps validate reconstruction against every paired valid local output.
    matched = n[n.valid_navigation].merge(local[local.observation_valid], on='header.stamp_ns', suffixes=('_raw', '_local'), validate='one_to_one')
    reconstruction_error = np.array([matched[c] - matched['position.' + a] for c, a in zip(['east_m', 'north_m', 'up_m'], 'xyz')]).T
    local_selected = local[local.observation_valid & local.bag_s.between(6, 100, inclusive='left')].copy()
    common = n.set_index('header.stamp_ns').loc[local_selected['header.stamp_ns']].reset_index()
    common['bag_s'] = local_selected.bag_s.to_numpy()
    common['log_ns'] = local_selected.log_ns.to_numpy()
    for key, axis in zip(['east_m', 'north_m', 'up_m'], 'xyz'):
        common[key] = local_selected['position.' + axis].to_numpy()
    result = dict(
        provenance=dict(bag=str(BAG), extracted_source=str(SOURCE), metadata_start_ns=start_ns,
                        duration_s=metadata['duration']['nanoseconds'] * 1e-9,
                        primary_window='0 <= bag receipt time minus metadata starting_time < 100 s',
                        valid_rows_definition='Finite position/velocity/heading with fix 3..6, heading_valid and clock_aligned; does not apply receipt freshness or accuracy gates.',
                        stationarity='User-confirmed first 100 seconds; no survey position/height truth supplied.',
                        source_files_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
                            [SOURCE / 'sensors_navigation.csv.gz', SOURCE / 'sensors_local_navigation.csv.gz',
                             SOURCE / 'navigation_origin.csv.gz', SOURCE / 'sensors_mavlink_status.csv.gz', BAG / 'metadata.yaml']},
                        reconstruction_matched_local_rows=len(matched),
                        maximum_reconstructed_minus_published_enu_m=np.abs(reconstruction_error).max(axis=0).tolist()),
        caveats=[
            'Whole-bag position spread includes possible motion after 100 seconds; do not label it static drift or true absolute error.',
            'Receiver accuracy estimates are not ground-truth error bounds.',
            'Receipt-time primary static window includes several pre-start measurement stamps; acquisition-time sensitivity is separate. Their buffering location is not established.',
            'MSL z is altitude minus fixed origin altitude; no barometer-derived height is used here.',
        ],
        first_100s=summary(n[(n.bag_s >= 0) & (n.bag_s < 100)]),
        common_6_100s_local_navigation=summary(common),
        common_6_100s_endpoint_windows=endpoint_windows(common, 6),
        first_100s_by_acquisition_time_sensitivity=summary(n[(n.stamp_s >= 0) & (n.stamp_s < 100)]),
        whole_recording=summary(n),
        local_navigation=dict(rows=len(local), observation_valid=counts(local, 'observation_valid'),
                              accuracy_known=counts(local, 'accuracy_known'), accuracy_ok=counts(local, 'accuracy_ok'),
                              reasons=counts(local, 'reason')),
    )
    keys = ['east_m', 'north_m', 'up_m']
    result['first_100s_endpoint_windows'] = endpoint_windows(n, 0)
    valid = n[n.valid_navigation].copy()
    gaps = valid[['bag_s', 'stamp_s', 'source_session']].copy()
    gaps['header_gap_s'] = valid['header.stamp_ns'].diff() * 1e-9
    gaps['previous_bag_s'] = valid.bag_s.shift()
    result['large_navigation_gaps'] = gaps[gaps.header_gap_s > .3].to_dict('records')
    result['invalid_navigation_events'] = n[~n.valid_navigation][['bag_s', 'stamp_s', 'fix_type', 'clock_aligned', 'heading_valid', 'source_session']].to_dict('records')
    heading_steps = valid[['bag_s', 'stamp_s', 'heading']].copy()
    heading_steps['heading_step_deg'] = np.r_[np.nan, np.diff(np.unwrap(valid.heading)) * 180 / np.pi]
    result['largest_heading_steps'] = heading_steps.loc[heading_steps.heading_step_deg.abs().nlargest(10).index].to_dict('records')
    diagnostics = {}
    for k in range(26):
        key = diagnostic[f'status.0.values.{k}.key'].iloc[0]
        if key in ['bad_frames', 'wrong_source', 'duplicates', 'stale', 'rejected_sync', 'tx_errors', 'sync_rtt_ms', 'imu_hz', 'gps_fix_hz', 'gps_velocity_hz']:
            diagnostics[key] = stats(diagnostic[f'status.0.values.{k}.value'])
    result['mavlink_diagnostic_numeric_fields'] = diagnostics
    bins = []
    for lo in range(0, 250, 10):
        d = valid[valid.bag_s.between(lo, lo + 10, inclusive='left')]
        if len(d):
            row = dict(start_s=lo, end_s=lo + 10, rows=len(d))
            for key in keys + ['heading', 'horizontal_accuracy', 'vertical_accuracy', 'velocity_accuracy', 'velocity.x', 'velocity.y', 'velocity.z']:
                row[key + '_mean'] = float(d[key].mean())
            row['speed_3d_p95'] = float(np.quantile(np.linalg.norm(d[['velocity.x', 'velocity.y', 'velocity.z']], axis=1), .95))
            bins.append(row)
    n.to_csv(HERE / 'gnss_samples.csv.gz', index=False)
    pd.DataFrame(bins).to_csv(HERE / 'gnss_10s_windows.csv', index=False)
    (HERE / 'gnss_quality.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
