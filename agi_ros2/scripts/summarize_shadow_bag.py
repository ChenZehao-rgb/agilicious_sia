#!/usr/bin/env python3
"""Inspect a shadow bag; report measurements without asserting physical readiness."""
import argparse
import collections
import json
import math
from pathlib import Path
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml


def stats(values):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return dict(count=0)
    return dict(count=len(values), minimum=values[0], mean=statistics.fmean(values),
                p95=values[min(len(values)-1, math.ceil(.95*len(values))-1)], maximum=values[-1])


def summarize(path):
    metadata = yaml.safe_load((Path(path)/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=metadata['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    state = collections.defaultdict(lambda: dict(count=0, acquisition=[], received=[], ages=[]))
    solve, warm, rtt, imu_age, nav_age, rc_age = [], [], [], [], [], []
    positions, yaw, navigation_sessions = [], [], set()
    code200 = permits = active = success = computations = 0
    errors = collections.Counter()
    configurations, origins = [], []
    controllers = collections.Counter()
    latest_mavlink_diagnostics = {}
    while reader.has_next():
        topic, raw, timestamp = reader.read_next()
        typ = types[topic]
        if typ not in ('sensor_msgs/msg/Imu', 'agi_ros2/msg/Navigation', 'agi_ros2/msg/Authority',
                       'agi_ros2/msg/FusedState', 'agi_ros2/msg/ComputationStatus', 'agi_ros2/msg/MspEvent',
                       'agi_ros2/msg/ControlCommand', 'agi_ros2/msg/OutputStatus', 'agi_ros2/msg/NavigationOrigin',
                       'diagnostic_msgs/msg/DiagnosticArray', 'std_msgs/msg/String'):
            continue
        message = deserialize_message(raw, get_message(typ))
        received = timestamp * 1e-9
        if hasattr(message, 'header'):
            acquisition = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
            s = state[topic]
            s['count'] += 1
            s['received'].append(received)
            s['acquisition'].append(acquisition)
            s['ages'].append(received-acquisition)
        if typ == 'agi_ros2/msg/ComputationStatus':
            computations += 1
            success += getattr(message, 'controller_success', message.mpc_success)
            controllers[getattr(message, 'controller_type', '') or 'legacy/unknown'] += 1
            solve.append(message.solve_seconds)
            warm.append(message.warm_cycles)
        elif typ == 'agi_ros2/msg/MspEvent':
            if message.event == 'tx' and message.code == 200:
                code200 += 1
            if message.event in ('timeout', 'error', 'transport_error', 'late'):
                errors[message.event] += 1
        elif typ == 'agi_ros2/msg/ControlCommand':
            permits += message.permit_override
            e = message.evidence
            imu_age.append(e.now-e.imu_time)
            nav_age.append(e.now-e.rtk_time)
            rc_age.append(e.now-e.rc_time)
        elif typ == 'agi_ros2/msg/OutputStatus':
            active += message.override_active
        elif typ == 'agi_ros2/msg/FusedState':
            p, q = message.position, message.orientation
            positions.append((p.x, p.y, p.z))
            yaw.append(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            navigation_sessions.add(message.navigation_session)
        elif typ == 'agi_ros2/msg/NavigationOrigin':
            record = dict(session=message.session_id, latitude=message.latitude, longitude=message.longitude,
                          altitude=message.altitude, altitude_reference=message.altitude_reference,
                          heading_correction_rad=message.heading_correction_rad,
                          fc_declination_applied=message.fc_declination_applied)
            if record not in origins:
                origins.append(record)
        elif typ == 'diagnostic_msgs/msg/DiagnosticArray':
            for status in message.status:
                if status.name == 'mavlink_sensor':
                    latest_mavlink_diagnostics = {value.key: value.value for value in status.values}
                for value in status.values:
                    if value.key == 'sync_rtt_ms':
                        try:
                            rtt.append(float(value.value))
                        except ValueError:
                            pass
        elif typ == 'std_msgs/msg/String' and topic.endswith('/msp/config'):
            if message.data not in configurations:
                configurations.append(message.data)
    streams = {}
    for topic, s in state.items():
        unique = sorted(set(t for t in s['acquisition'] if t > 0))
        span = unique[-1]-unique[0] if len(unique) > 1 else 0
        streams[topic] = dict(messages=s['count'], unique_acquisitions=len(unique),
                             unique_rate_hz=(len(unique)-1)/span if span else None,
                             acquisition_gaps_seconds=stats([b-a for a, b in zip(unique, unique[1:])]),
                             bag_receive_age_seconds=stats(s['ages']))
    extents = [max(p[i] for p in positions)-min(p[i] for p in positions) for i in range(3)] if positions else None
    yaw_offsets = [math.remainder(y-yaw[0], 2*math.pi) for y in yaw] if yaw else []
    computation = dict(cycles=computations, successes=success, solve_seconds=stats(solve),
                       max_warm_cycles=max(warm, default=0), controller_types=dict(controllers))
    return dict(streams=streams, controller=computation, mpc=dict(computation, deprecated_alias=True),
                evidence_ages_seconds=dict(imu=stats(imu_age), navigation=stats(nav_age), rc=stats(rc_age)),
                timesync_rtt_ms=stats(rtt), latest_mavlink_diagnostics=latest_mavlink_diagnostics, msp_errors=dict(errors),
                output_isolation=dict(code200_tx_events=code200, permitted_commands=permits, active_statuses=active,
                    note='Zero recorded events is not proof of complete UART capture; check bag discovery/coverage.'),
                observed_position_extent_m=extents, observed_yaw_offset_rad=stats(yaw_offsets),
                navigation_sessions=sorted(navigation_sessions), origins=origins, msp_configurations=configurations,
                interpretation='Extents are drift only for a stationary experiment; no physical readiness verdict. '
                               'Acquisition gaps indicate missing updates, not a provable wire packet-loss rate. '
                               'Bag receive age includes DDS/recording delay. Inspect separate navigation sessions.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    args = parser.parse_args()
    print(json.dumps(summarize(args.bag), ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
