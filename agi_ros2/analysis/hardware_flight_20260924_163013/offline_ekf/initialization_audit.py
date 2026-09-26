#!/usr/bin/env python3
"""Audit short IMU initialization windows without changing flight parameters.

Requires numpy, mcap and mcap-ros2-support; no ROS installation is needed.
ARM boundaries come from the independently decoded MSP timeline. Time windows
select records by bag receipt time and report their actual IMU header spans.
"""

import argparse
import json
from pathlib import Path

from mcap.reader import NonSeekingReader
from mcap_ros2.decoder import DecoderFactory
import numpy as np


def read_observations(bag):
    imu, navigation, sessions = [], [], {}
    start = None
    records = 0
    for path in sorted(bag.glob("*.mcap")):
        with path.open("rb") as source:
            reader = NonSeekingReader(source, validate_crcs=True)
            decoders = DecoderFactory()
            for schema, channel, record in reader.iter_messages(log_time_order=False):
                records += 1
                start = min(start, record.log_time) if start is not None else record.log_time
                if channel.topic not in ("/sensors/imu", "/sensors/navigation"):
                    continue
                decoder = decoders.decoder_for(channel.message_encoding, schema)
                if decoder is None:
                    raise ValueError(f"Unsupported embedded schema: {channel.topic}")
                message = decoder(record.data)
                stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                if channel.topic == "/sensors/imu":
                    if message.header.frame_id != "base_link":
                        raise ValueError("Expected base_link IMU frame")
                    acceleration, gyro = message.linear_acceleration, message.angular_velocity
                    imu.append((record.log_time, stamp, acceleration.x, acceleration.y,
                                acceleration.z, gyro.x, gyro.y, gyro.z))
                elif message.device_time_usec:
                    velocity = message.velocity
                    navigation.append((record.log_time, stamp, velocity.x, velocity.y,
                                       velocity.z, message.heading))
                    session = sessions.setdefault(message.source_session, {
                        "first_log_time_ns": record.log_time,
                        "first_header_time_ns": stamp,
                        "first_device_time_usec": message.device_time_usec,
                    })
                    session["last_log_time_ns"] = record.log_time
                    session["last_header_time_ns"] = stamp
    if not imu or not navigation:
        raise ValueError("The bag must contain IMU and valid navigation observations")

    def relative_array(rows):
        # Subtract integer nanoseconds before conversion to retain submicrosecond precision.
        return np.asarray([((row[0] - start) * 1e-9, (row[1] - start) * 1e-9,
                            *row[2:]) for row in rows], dtype=float)

    for session in sessions.values():
        for prefix in ("first", "last"):
            session[f"{prefix}_bag_s"] = (session.pop(f"{prefix}_log_time_ns") - start) * 1e-9
            session[f"{prefix}_header_s"] = (session.pop(f"{prefix}_header_time_ns") - start) * 1e-9
    return relative_array(imu), relative_array(navigation), sessions, start, records


def summarize_window(imu, navigation, lower, upper):
    samples = imu[(imu[:, 0] >= lower) & (imu[:, 0] <= upper)]
    fixes = navigation[(navigation[:, 0] >= lower) & (navigation[:, 0] <= upper)]
    if len(samples) < 2 or not len(fixes):
        raise ValueError(f"Insufficient observations in window {lower}, {upper}")
    acceleration, gyro = samples[:, 2:5], samples[:, 5:8]
    acceleration_mean, gyro_mean = acceleration.mean(axis=0), gyro.mean(axis=0)
    acceleration_std, gyro_std = acceleration.std(axis=0), gyro.std(axis=0)
    speed = np.linalg.norm(fixes[:, 2:5], axis=1)
    span = float(samples[-1, 1] - samples[0, 1])
    gravity_error = float(abs(np.linalg.norm(acceleration_mean) - 9.8066))
    return {
        "requested_bag_s": [float(lower), float(upper)],
        "imu_bag_s": samples[[0, -1], 0].tolist(),
        "imu_header_s": samples[[0, -1], 1].tolist(),
        "imu_samples": len(samples), "imu_header_span_s": span,
        "imu_header_max_gap_s": float(np.diff(samples[:, 1]).max()),
        "accel_mean_m_s2": acceleration_mean.tolist(),
        "accel_stddev_m_s2": acceleration_std.tolist(),
        "accel_variance_m2_s4": (acceleration_std ** 2).tolist(),
        "accel_mean_norm_m_s2": float(np.linalg.norm(acceleration_mean)),
        "gravity_norm_error_m_s2": gravity_error,
        "gyro_mean_rad_s": gyro_mean.tolist(),
        "gyro_mean_norm_rad_s": float(np.linalg.norm(gyro_mean)),
        "gyro_stddev_rad_s": gyro_std.tolist(),
        "gyro_variance_rad2_s2": (gyro_std ** 2).tolist(),
        "gravity_tilt_roll_pitch_deg": np.rad2deg([
            np.arctan2(acceleration_mean[1], acceleration_mean[2]),
            np.arctan2(-acceleration_mean[0], np.linalg.norm(acceleration_mean[1:])),
        ]).tolist(),
        "nav_samples": len(fixes), "nav_bag_s": fixes[[0, -1], 0].tolist(),
        "speed_m_s": {"min": float(speed.min()), "mean": float(speed.mean()),
                      "max": float(speed.max()), "p95": float(np.percentile(speed, 95))},
        "nav_velocity_mean_m_s": fixes[:, 2:5].mean(axis=0).tolist(),
        "heading_deg_min_mean_max": np.rad2deg([
            fixes[:, 5].min(), fixes[:, 5].mean(), fixes[:, 5].max(),
        ]).tolist(),
        "std_bias_gravity_thresholds_pass": bool(
            np.linalg.norm(gyro_mean) <= .15 and gyro_std.max() <= .02
            and acceleration_std.max() <= .2 and gravity_error <= .5),
        "all_speed_below_0p3_m_s": bool(speed.max() <= .3),
        "sample_duration_original_requirements_pass": bool(len(samples) >= 1000 and span >= 3),
    }


def analyze(bag, timeline_path):
    timeline = json.loads(timeline_path.read_text())
    if timeline["bag"] != bag.name:
        raise ValueError("MSP timeline does not identify the supplied bag")
    imu, navigation, sessions, start, records = read_observations(bag)
    runs = timeline["raw_status"]["arm_runs"]
    if len(runs) != 3 or [run["value"] for run in runs] != [False, True, False]:
        raise ValueError("This audit requires a disarmed / armed / disarmed recording")
    if len(sessions) != 2:
        raise ValueError("This audit expects the two navigation sessions in this recording")
    first, last = imu[0, 0], imu[-1, 0]
    post = imu[np.argmax(np.diff(imu[:, 0])) + 1, 0]
    limits = {
        "initial_first_1s": (first, first + 1),
        "initial_first_2s": (first, first + 2),
        "initial_entire_known_disarmed": (first, runs[0]["last_bag_s"]),
        "initial_before_first_armed": (first, runs[1]["first_bag_s"]),
        "last_disarmed": (runs[2]["first_bag_s"], last),
        "last_2s": (last - 2, last),
        "post_clock_first_1s": (post, post + 1),
        "post_clock_first_2s": (post, post + 2),
        "post_clock_first_3s": (post, post + 3),
    }
    early = navigation[navigation[:, 0] < runs[1]["first_bag_s"]]
    speed = np.linalg.norm(early[:, 2:5], axis=1)
    late_bins = []
    for lower in np.arange(126.5, 129.51, .5):
        samples = imu[(imu[:, 0] >= lower) & (imu[:, 0] < lower + .5)]
        if len(samples):
            late_bins.append({
                "bag_s": [float(lower), float(lower + .5)], "samples": len(samples),
                "accel_mean_norm_m_s2": float(np.linalg.norm(samples[:, 2:5].mean(axis=0))),
                "accel_max_axis_stddev_m_s2": float(samples[:, 2:5].std(axis=0).max()),
                "gyro_max_axis_stddev_rad_s": float(samples[:, 5:8].std(axis=0).max()),
            })
    return {
        "bag": bag.name, "timeline": timeline_path.name, "bag_start_ns": start,
        "validation": {"crc_checked": True, "embedded_schema_decoding": True,
                       "total_records": records, "imu_count": len(imu), "nav_count": len(navigation)},
        "method": {
            "window_selection": "bag receipt time, inclusive endpoints; actual duration from IMU header",
            "standard_deviation": "population, ddof=0; descriptive statistics, not a noise calibration",
            "imu_axes": "base_link FLU", "heading": "ENU radians converted to degrees; FC declination applied",
            "online_thresholds": {"duration_s": 3.0, "minimum_samples": 1000,
                                  "gyro_bias_norm_rad_s": .15, "gyro_axis_stddev_rad_s": .02,
                                  "accel_axis_stddev_m_s2": .2, "gravity_tolerance_m_s2": .5,
                                  "gravity_m_s2": 9.8066, "navigation_speed_m_s": .3},
        },
        "arm_runs": runs, "nav_sessions": sessions,
        "windows": {name: summarize_window(imu, navigation, *bounds) for name, bounds in limits.items()},
        "initial_gps_speed_exceedances": [
            {"bag_s": float(row[0]), "header_s": float(row[1]),
             "speed_m_s": float(value), "velocity_enu_m_s": row[2:5].tolist()}
            for row, value in zip(early, speed) if value > .3
        ],
        "late_half_second_bins": late_bins,
        "interpretation": [
            "Initial 1 s / 2 s windows are exploratory seeds; neither meets the online 3 s requirement.",
            "Initial disarmed IMU satisfies bias/stddev/gravity thresholds, but GPS speeds above 0.3 m/s reset online initialization.",
            "The second session begins while armed and moving; its means cannot identify stationary tilt or gyro bias.",
            "The final 3 s includes landing disturbance; the final quiet 2 s has acceleration norm about 9.264 m/s^2, outside the gravity tolerance.",
            "The initial-to-final acceleration baseline discrepancy is a calibration/data-consistency concern whose cause is not identified by this audit.",
            "No flight-window variance is used as measurement noise, and there is no independent position truth in this audit.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--timeline", type=Path,
                        default=Path(__file__).resolve().parent.parent / "msp_timeline.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.bag, args.timeline)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(f"Audited {result['validation']['imu_count']} IMU samples; saved {args.output}")


if __name__ == "__main__":
    main()
