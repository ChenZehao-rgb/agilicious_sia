#!/usr/bin/env python3
"""Summarize a hardware diagnostic MCAP bag using its embedded ROS 2 schemas.

Dependencies: mcap, mcap-ros2-support, numpy, pyyaml. No ROS installation required.
This recording was collected while carrying the vehicle around a sports field;
it cannot provide stationary IMU noise/bias or GPS drift calibration.
"""

import argparse
import collections
import json
import math
from pathlib import Path
import struct

from mcap.reader import NonSeekingReader
from mcap_ros2.decoder import DecoderFactory
import numpy as np
import yaml


CONFIG_CODES = {1, 2, 3, 34, 44, 64, 111, 119, 125, 238, 0x3010}
SELECTED = {
    '/sensors/imu', '/sensors/navigation', '/sensors/gps/fix',
    '/sensors/gps/velocity', '/sensors/fc_heading', '/sensors/mavlink/status',
    '/msp/events', '/msp/config', '/msp/decoded_state', '/fused_state',
    '/navigation/status', '/health/status', '/health', '/authority',
}


def statistics(values):
    values = list(values)
    finite = np.asarray([x for x in values if math.isfinite(x)], dtype=float)
    result = {'samples': len(values), 'finite': len(finite),
              'unavailable': len(values) - len(finite)}
    if len(finite):
        result.update(minimum=float(finite.min()), median=float(np.median(finite)),
                      p95=float(np.percentile(finite, 95)),
                      p99=float(np.percentile(finite, 99)), maximum=float(finite.max()),
                      mean=float(finite.mean()))
    return result


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def stream_statistics(samples):
    received = sorted(t for t, _ in samples)
    stamps = sorted(set(t for _, t in samples))
    def rate(times):
        span = times[-1] - times[0] if times else 0
        return (len(times) - 1) * 1e9 / span if span else None
    return {
        'messages': len(samples), 'unique_stamps': len(stamps),
        'received_rate_hz': rate(received), 'unique_stamp_rate_hz': rate(stamps),
        'acquisition_gap_seconds': statistics((b-a)*1e-9 for a, b in zip(stamps, stamps[1:])),
        'receive_gap_seconds': statistics((b-a)*1e-9 for a, b in zip(received, received[1:])),
        'bag_receive_age_seconds': statistics((t-s)*1e-9 for t, s in samples),
    }


def record_fields(counters, prefix, message, names):
    for name in names:
        value = getattr(message, name)
        key = str(value).lower() if isinstance(value, bool) else str(value)
        counters[prefix + '/' + name][key] += 1


def decode_configuration(code, name, payload):
    """Decode this project's BTFL API 1.48 fields; preserve bytes for inspection."""
    def u16(offset):
        return struct.unpack_from('<H', payload, offset)[0]
    if code == 1 and len(payload) == 3:
        return dict(protocol=payload[0], api=f'{payload[1]}.{payload[2]}')
    if code == 2:
        return dict(variant=payload.decode('ascii', errors='replace'))
    if code == 3 and len(payload) >= 3:
        result = dict(version_bytes=list(payload[:3]), payload_length=len(payload))
        if len(payload) >= 4 and len(payload) == 4 + payload[3]:
            result['version_string'] = payload[4:].decode('ascii', errors='replace')
        return result
    if code == 34 and len(payload) % 4 == 0:
        return dict(active_ranges=[dict(permanent_id=payload[i], aux_index=payload[i+1],
                                        low=900+25*payload[i+2], high=900+25*payload[i+3])
                                  for i in range(0, len(payload), 4) if payload[i+2] < payload[i+3]])
    if code == 44 and len(payload) >= 7:
        return dict(midrc=u16(3), min_check=u16(5))
    if code == 64:
        return dict(rx_map=list(payload))
    if code == 111 and len(payload) >= 23:
        if payload[22] != 3:
            return dict(rates_type=payload[22], interpretation='Only ACTUAL rates are decoded here')
        return dict(rates_type=payload[22], throttle_limit_type=payload[14],
                    center_rate_deg_s=[payload[i]*10 for i in (0, 12, 11)],
                    max_rate_deg_s=[payload[i]*10 for i in (2, 3, 4)],
                    expo_percent=[payload[i] for i in (1, 13, 10)],
                    rate_limits_deg_s=[u16(16+2*i) for i in range(3)])
    if code == 119:
        return dict(box_permanent_ids=list(payload))
    if code == 125 and len(payload) >= 2:
        return dict(deadband=payload[0], yaw_deadband=payload[1])
    if code == 0x3010:
        return dict(request_name=name, reply=payload.decode('ascii', errors='replace'))
    return {}


def summarize(bag):
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    expected = {t['topic_metadata']['name']: t['message_count']
                for t in metadata['topics_with_message_count']}
    counts, counters = collections.Counter(), collections.defaultdict(collections.Counter)
    streams, values = collections.defaultdict(list), collections.defaultdict(list)
    config_payloads, configurations = collections.Counter(), collections.Counter()
    first_time, last_time = None, None
    for filename in metadata['relative_file_paths']:
        with (bag/filename).open('rb') as source:
            reader = NonSeekingReader(source, validate_crcs=True)
            decoders = DecoderFactory()
            for schema, channel, record in reader.iter_messages(log_time_order=False):
                topic, time = channel.topic, record.log_time
                counts[topic] += 1
                first_time = min(first_time, time) if first_time is not None else time
                last_time = max(last_time, time) if last_time is not None else time
                if topic not in SELECTED:
                    continue
                decoder = decoders.decoder_for(channel.message_encoding, schema)
                if decoder is None:
                    raise ValueError(f'Missing supported embedded ROS 2 schema: {topic}')
                message = decoder(record.data)
                if hasattr(message, 'header'):
                    streams[topic].append((time, stamp_ns(message)))
                if topic == '/sensors/navigation':
                    observation = (message.device_time_usec > 0 and math.isfinite(message.latitude)
                                   and math.isfinite(message.longitude))
                    group = 'gps_observations' if observation else 'navigation_revocations'
                    streams[group].append((time, stamp_ns(message)))
                    record_fields(counters, group, message,
                                  ['fix_type', 'heading_valid', 'clock_aligned', 'altitude_reference'])
                    for field in ['horizontal_accuracy', 'vertical_accuracy', 'velocity_accuracy',
                                  'altitude', 'heading']:
                        values[group + '/' + field].append(getattr(message, field))
                    if observation:
                        v = message.velocity
                        values[group + '/speed_m_s'].append(math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z))
                elif topic == '/sensors/gps/fix':
                    record_fields(counters, topic, message.status, ['status', 'service'])
                    record_fields(counters, topic, message, ['position_covariance_type'])
                elif topic == '/sensors/fc_heading':
                    record_fields(counters, topic, message, ['valid'])
                elif topic == '/sensors/mavlink/status':
                    for status in message.status:
                        if status.name != 'mavlink_sensor':
                            continue
                        counters['mavlink/level'][str(status.level)] += 1
                        for item in status.values:
                            if item.key in {'altitude_msl_m', 'sync_rtt_ms', 'imu_hz', 'gps_fix_hz',
                                            'gps_velocity_hz', 'heading_hz'}:
                                values['mavlink/' + item.key].append(float(item.value))
                            else:
                                counters['mavlink/' + item.key][item.value] += 1
                elif topic == '/msp/events':
                    key = f'{message.event}:{message.code}:{message.request_name}'
                    counters['msp_events'][key] += 1
                    if message.event != 'rx':
                        continue
                    payload = bytes(message.payload)
                    values[f'msp_latency_seconds/{message.code}'].append(message.latency_seconds)
                    if message.code in CONFIG_CODES:
                        config_payloads[(message.code, message.request_name, payload)] += 1
                    elif message.code == 150 and len(payload) >= 16:
                        counters['msp_status/pid_profile'][str(payload[10])] += 1
                        counters['msp_status/rate_profile'][str(payload[14])] += 1
                        counters['msp_status/sensor_mask'][hex(struct.unpack_from('<H', payload, 4)[0])] += 1
                        extra = payload[15]
                        if len(payload) >= 23 + extra:
                            counters['msp_status/mode_bits'][(payload[6:10]+payload[16:16+extra]).hex()] += 1
                            flags = struct.unpack_from('<I', payload, 17+extra)[0]
                            counters['msp_status/arming_disable_flags'][hex(flags)] += 1
                            counters['msp_status/arming_disable_flag_count'][str(payload[16+extra])] += 1
                    elif message.code == 130 and len(payload) >= 11 and payload[0] > 0:
                        voltage = struct.unpack_from('<H', payload, 9)[0] * .01
                        values['msp_battery/voltage_v'].append(voltage if voltage > 0 else math.nan)
                        counters['msp_battery/cells'][str(payload[0])] += 1
                elif topic == '/msp/config':
                    configurations[message.data] += 1
                elif topic == '/navigation/status':
                    status = json.loads(message.data)
                    counters[topic + '/reason'][status.get('reason', '')] += 1
                elif topic == '/health/status':
                    status = json.loads(message.data)
                    for reason in status.get('reasons', []):
                        counters[topic + '/reasons'][reason] += 1
                elif topic == '/msp/decoded_state':
                    record_fields(counters, topic, message,
                                  ['reason', 'config_verified', 'receiver_valid', 'armed', 'auto_switch',
                                   'kill', 'rc_link', 'transport_healthy', 'imu_ready', 'control_mode_ok',
                                   'pid_profile', 'rate_profile', 'override_timeout_ms'])
                elif topic == '/fused_state':
                    record_fields(counters, topic, message,
                                  ['readiness_reason', 'initialized', 'imu_ready', 'estimator_ready',
                                   'navigation_ready', 'navigation_valid', 'heading_valid', 'accuracy_known'])
                elif topic == '/authority':
                    record_fields(counters, topic, message, ['armed', 'auto_switch', 'kill', 'rc_link'])
                elif topic == '/health':
                    record_fields(counters, topic, message,
                                  ['config_verified', 'transport_healthy', 'imu_ready', 'estimator_ready',
                                   'navigation_ready', 'thrust_calibrated', 'geofence_ok'])
    mismatches = {topic: {'metadata': expected.get(topic, 0), 'observed': counts[topic]}
                  for topic in expected.keys() | counts.keys() if expected.get(topic, 0) != counts[topic]}
    if mismatches or sum(counts.values()) != metadata['message_count']:
        raise ValueError(f'Message counts differ from metadata: {mismatches}')
    return {
        'bag': bag.name,
        'interpretation': [
            'Hand-carried moving recording, not stationary calibration; no IMU noise/bias fit.',
            'Accuracy fields are receiver-reported estimates, not measured ground-truth errors.',
            'Navigation observations and revocation events are counted separately.',
            'Receive age includes DDS and recorder delay; timing gaps are not a wire-loss measurement.',
            'Satellite count, HDOP, and raw ellipsoid height were not recorded by these topics.',
            'Raw STATUS profiles are distinct from fail-closed decoded-state defaults (255).',
        ],
        'validation': {'reader': 'mcap.NonSeekingReader with embedded ROS 2 schemas',
                       'validate_crcs': True, 'crc_scope': 'stored chunk/data CRCs, where present',
                       'metadata_counts_match': True, 'messages': sum(counts.values()),
                       'metadata_messages': metadata['message_count'], 'topic_count': len(expected),
                       'duration_seconds': (last_time-first_time)*1e-9},
        'topic_messages': dict(sorted({**expected, **counts}.items())),
        'streams': {key: stream_statistics(data) for key, data in sorted(streams.items())},
        'measurements': {key: statistics(data) for key, data in sorted(values.items())},
        'counts': dict(sorted(counters.items())),
        'msp_readbacks': [dict(code=code, request_name=name, count=count, payload_hex=payload.hex(),
                               decoded=decode_configuration(code, name, payload))
                          for (code, name, payload), count in sorted(config_payloads.items())],
        'msp_configurations': [dict(count=count, configuration=yaml.safe_load(raw))
                               for raw, count in configurations.items()],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path, help='Directory containing metadata.yaml and MCAP files')
    parser.add_argument('--output', type=Path, help='Write JSON here; otherwise use stdout')
    args = parser.parse_args()
    result = json.dumps(summarize(args.bag), ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result)
    else:
        print(result, end='')


if __name__ == '__main__':
    main()
