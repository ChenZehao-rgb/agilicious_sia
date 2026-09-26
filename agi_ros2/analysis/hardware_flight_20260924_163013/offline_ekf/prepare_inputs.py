#!/usr/bin/env python3
"""Read bag inputs for conditional offline EKF reconstruction; never publish ROS.

Height is NOT the missing original GPS altitude: it is reconstructed from 1 Hz
MSL diagnostics. Two variants quantify linear interpolation versus prior hold.
"""
import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

from mcap.reader import NonSeekingReader
from mcap_ros2.decoder import DecoderFactory
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'agi_ros2/scripts'))
from shadow_support import local_position


def write_csv(path, header, rows):
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def prepare(bag, destination, profile, timing_file):
    timeline = json.loads(timing_file.read_text())
    assert len(timeline['clock_jumps']) == 1, 'Resolve each clock epoch explicitly'
    jump = timeline['clock_jumps'][0]
    step_ns = round(jump['inferred_wall_step_s']*1e9)
    boundary_ns = round((jump['before']['wall_s']+jump['after']['wall_s'])*.5e9)
    def corrected(ns):
        return ns - (step_ns if ns > boundary_ns else 0)
    def stamp(message):
        return message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
    imu, nav, fixes, diagnostics = [], [], [], []
    sessions = {}
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    selected = {'/sensors/imu', '/sensors/navigation', '/sensors/gps/fix', '/sensors/mavlink/status'}
    count = 0
    for name in metadata['relative_file_paths']:
        with (bag/name).open('rb') as stream:
            factory = DecoderFactory()
            for schema, channel, record in NonSeekingReader(stream, validate_crcs=True).iter_messages(log_time_order=False):
                count += 1
                if channel.topic not in selected:
                    continue
                message = factory.decoder_for(channel.message_encoding, schema)(record.data)
                if channel.topic == '/sensors/imu':
                    a, g = message.linear_acceleration, message.angular_velocity
                    imu.append([corrected(stamp(message)), a.x, a.y, a.z, g.x, g.y, g.z])
                elif channel.topic == '/sensors/navigation':
                    if not (message.device_time_usec and message.heading_valid and message.clock_aligned
                            and 3 <= message.fix_type <= 6 and math.isfinite(message.latitude)):
                        continue
                    epoch = sessions.setdefault(message.source_session, len(sessions))
                    v = message.velocity
                    nav.append([corrected(stamp(message)), epoch, message.latitude, message.longitude,
                                v.x, v.y, v.z, message.heading, message.horizontal_accuracy,
                                message.vertical_accuracy, message.velocity_accuracy, record.log_time])
                elif channel.topic == '/sensors/gps/fix':
                    fixes.append([record.log_time, corrected(stamp(message)), int(record.log_time > boundary_ns)])
                else:
                    for status in message.status:
                        if status.name != 'mavlink_sensor':
                            continue
                        values = {item.key:item.value for item in status.values}
                        if values.get('gps_ready') == 'true':
                            h = float(values['altitude_msl_m'])
                            if math.isfinite(h):
                                diagnostics.append([record.log_time, h])
    assert count == metadata['message_count']
    imu = np.array(sorted(imu), dtype=float)
    t0_ns = int(imu[0, 0])
    imu[:, 0] = (imu[:, 0]-t0_ns)*1e-9
    nav = np.array(sorted(nav), dtype=float)
    nav[:, 0] = (nav[:, 0]-t0_ns)*1e-9
    fixes.sort()
    fix_times = [x[0] for x in fixes]
    height = []
    for received, h in diagnostics:
        i = bisect.bisect_right(fix_times, received)-1
        if i >= 0 and received-fixes[i][0] < 300_000_000:
            height.append([(fixes[i][1]-t0_ns)*1e-9, fixes[i][2], h,
                           (received-fixes[i][0])*1e-9])
    height = np.array(height)
    config = yaml.safe_load(profile.read_text())
    n = config['navigation']
    origin = (nav[0, 2], nav[0, 3], height[0, 2])
    destination.mkdir(parents=True, exist_ok=True)
    (destination/'hardware_snapshot.yaml').write_bytes(profile.read_bytes())
    write_csv(destination/'imu.csv', ['t','ax','ay','az','gx','gy','gz'], imu)
    write_csv(destination/'height_snapshots.csv', ['t','epoch','msl_m','diagnostic_after_fix_receive_s'], height)
    nav_header = ['t','epoch','px','py','pz','vx','vy','vz','yaw','pvarx','pvary','pvarz','vvarx','vvary','vvarz','yawvar']
    variants = {}
    for method in ['linear', 'hold']:
        rows = []
        for row in nav:
            t, epoch, lat, lon, vx, vy, vz, yaw, ha, va, sa, _ = row
            h = height[height[:, 1] == epoch]
            if not len(h) or not h[0, 0] <= t <= h[-1, 0]:
                continue  # No extrapolation or interpolation across the resynchronization.
            altitude = (float(np.interp(t, h[:, 0], h[:, 2])) if method == 'linear'
                        else h[np.searchsorted(h[:, 0], t, side='right')-1, 2])
            p = local_position(origin, (lat, lon, altitude), 'msl')
            ph, pv, vv = max(ha, n['horizontal_stddev'])**2, max(va, n['vertical_stddev'])**2, max(sa, n['velocity_stddev'])**2
            rows.append([t, int(epoch), *p, vx, vy, vz, yaw+n['heading_correction_rad'], ph, ph, pv, vv, vv, vv, n['heading_stddev']**2])
        write_csv(destination/f'nav_{method}.csv', nav_header, rows)
        variants[method] = {'observations':len(rows), 'per_epoch':{str(e):sum(r[1] == e for r in rows) for e in sessions.values()}}
    seeds = {}
    for duration in [1., 2.]:
        samples = imu[imu[:, 0] < duration]
        a, g = samples[:, 1:4].mean(axis=0), samples[:, 4:7].mean(axis=0)
        seeds[str(duration)] = {'samples':len(samples), 'span_s':samples[-1, 0]-samples[0, 0],
                               'acceleration_mean':a.tolist(), 'gyro_mean':g.tolist(),
                               'acceleration_stddev':samples[:, 1:4].std(axis=0).tolist(),
                               'gyro_stddev':samples[:, 4:7].std(axis=0).tolist(),
                               'roll':math.atan2(a[1], a[2]),
                               'pitch':math.atan2(-a[0], math.hypot(a[1], a[2]))}
    audit = {'source_bag':bag.name, 'messages_checked':count, 'clock_step_seconds':step_ns*1e-9,
             'time_origin_header_ns':t0_ns, 'clock_boundary_header_ns':boundary_ns,
             'local_origin_lat_lon_msl':list(origin),
             'profile_snapshot':'hardware_snapshot.yaml',
             'profile_sha256':hashlib.sha256(profile.read_bytes()).hexdigest(),
             'source_sessions':sessions, 'imu_samples':len(imu), 'gps_observations':len(nav),
             'height_snapshot_count':len(height), 'variants':variants, 'seeds':seeds,
             'imu_max_gap_seconds':float(np.diff(imu[:, 0]).max()),
             'assumptions':[
                 'Offline only; neither online startup gates nor physical authorization are bypassed in production.',
                 'Time subtracts the measured wall step; missing IMU remains missing.',
                 'Height diagnostics are approximately associated with the latest earlier GPS fix in the same epoch.',
                 'Reconstructed height is only 1 Hz evidence; linear interpolation uses future samples and is not online replay.',
                 'Hold variant tests this reconstruction dependence; neither recovers original full-rate altitude.',
                 'No height extrapolation, no interpolation across the clock/session boundary.',
                 'ENU origin is the first GPS horizontal fix and first MSL snapshot, not a verified stationary origin.',
                 'Only 1/2 second startup seeds are explored; neither passes the actual 3 second readiness rule.',
                 'GPS position/velocity are filter inputs and cannot serve as independent localization truth.']}
    (destination/'input_audit.json').write_text(json.dumps(audit, indent=2)+'\n')
    return audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', type=Path, default=ROOT/'agi_ros2/config/hardware.yaml')
    parser.add_argument('--timing', type=Path, default=Path(__file__).resolve().parent.parent/'msp_timeline.json')
    args = parser.parse_args()
    print(json.dumps(prepare(args.bag, args.output, args.profile, args.timing), indent=2))


if __name__ == '__main__':
    main()
