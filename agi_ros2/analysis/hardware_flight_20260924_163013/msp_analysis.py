#!/usr/bin/env python3
"""Reproduce raw MSP authority/configuration and wall/steady clock evidence.

Dependencies: mcap, mcap-ros2-support, pyyaml. Embedded schemas avoid ROS imports.
Times ending in ``bag_s`` use the bag's uncorrected ROS/wall clock. ARM means
FC-reported armed state, not a measured takeoff/landing time.
"""
import argparse
import collections
import json
from pathlib import Path
import struct

from mcap.reader import NonSeekingReader
from mcap_ros2.decoder import DecoderFactory
import yaml


CONFIG_CODES = {1, 2, 3, 34, 238, 64, 44, 119, 111, 125, 0x3010}
TOPICS = {'/msp/events', '/msp/decoded_state', '/health/status', '/rosout'}


def ros_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def decode_config(code, payload):
    u16 = lambda offset: struct.unpack_from('<H', payload, offset)[0]
    if code == 1:
        return {'protocol': payload[0], 'api': f'{payload[1]}.{payload[2]}'}
    if code == 2:
        return {'variant': payload.decode('ascii')}
    if code == 3:
        return {'version_bytes': list(payload[:3]), 'version_string': payload[4:].decode('ascii')}
    if code == 34:
        return {'active_ranges': [dict(permanent_id=payload[i], aux_index=payload[i+1],
                                      low=900+25*payload[i+2], high=900+25*payload[i+3])
                                  for i in range(0, len(payload), 4) if payload[i+2] < payload[i+3]]}
    if code == 238:
        return {'entries': [dict(permanent_id=payload[i], logic=payload[i+1], linked_to=payload[i+2])
                            for i in range(1, len(payload), 3)]}
    if code == 44:
        return {'midrc': u16(3), 'min_check': u16(5)}
    if code == 64:
        return {'rx_map': list(payload)}
    if code == 119:
        return {'box_permanent_ids': list(payload)}
    if code == 111:
        return {'rates_type': payload[22], 'throttle_limit_type': payload[14],
                'center_rate_deg_s': [payload[i]*10 for i in (0, 12, 11)],
                'max_rate_deg_s': [payload[i]*10 for i in (2, 3, 4)],
                'expo_percent': [payload[i] for i in (1, 13, 10)],
                'rate_limits_deg_s': [u16(i) for i in (16, 18, 20)]}
    if code == 125:
        return {'deadband': payload[0], 'yaw_deadband': payload[1]}
    if code == 0x3010:
        return {'reply': payload.decode('ascii')}
    return {}


def runs(rows, key):
    result = []
    for row in rows:
        if not result or result[-1]['value'] != row[key]:
            result.append({'first_bag_s': row['bag_s'], 'first_steady_s': row['steady_s'],
                           'value': row[key], 'samples': 0})
        result[-1].update(last_bag_s=row['bag_s'], last_steady_s=row['steady_s'])
        result[-1]['samples'] += 1
    return result


def summarize(bag, previous_summary):
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    start = metadata['starting_time']['nanoseconds_since_epoch']
    counts, event_counts, configs = collections.Counter(), collections.Counter(), collections.Counter()
    status, rc, clock_jumps, logs = [], [], [], []
    state_reasons, health_first = [], {}
    first_event = previous_event = None
    errors_max = 0
    sessions = set()
    for filename in metadata['relative_file_paths']:
        with (bag/filename).open('rb') as source:
            decoder_factory = DecoderFactory()
            for schema, channel, record in NonSeekingReader(source, validate_crcs=True).iter_messages(log_time_order=False):
                counts[channel.topic] += 1
                if channel.topic not in TOPICS:
                    continue
                message = decoder_factory.decoder_for(channel.message_encoding, schema)(record.data)
                t = (record.log_time-start)*1e-9
                if channel.topic == '/rosout':
                    if 'Runtime configuration:' in message.msg or message.level >= 30:
                        logs.append(dict(bag_s=t, name=message.name, level=message.level, message=message.msg))
                    continue
                if channel.topic == '/health/status':
                    for reason in json.loads(message.data).get('reasons', []):
                        health_first.setdefault(reason, t)
                    continue
                if channel.topic == '/msp/decoded_state':
                    if not state_reasons or state_reasons[-1]['reason'] != message.reason:
                        state_reasons.append(dict(bag_s=t, reason=message.reason))
                    continue
                event = dict(bag_s=t, wall_s=ros_seconds(message.header.stamp), steady_s=message.steady_time,
                             event=message.event, code=message.code, request_name=message.request_name,
                             request_wall_s=ros_seconds(message.request_stamp),
                             request_steady_s=message.request_steady_time,
                             latency_s=message.latency_seconds if message.event == 'rx' else None)
                sessions.add(message.session_id)
                errors_max = max(errors_max, message.errors)
                event_counts[(message.event, message.code, message.request_name)] += 1
                if previous_event is not None:
                    dw = event['wall_s'] - previous_event['wall_s']
                    ds = event['steady_s'] - previous_event['steady_s']
                    if abs(dw-ds) > .1:
                        clock_jumps.append(dict(before=previous_event, after=event, wall_delta_s=dw,
                                                steady_delta_s=ds, inferred_wall_step_s=dw-ds))
                if first_event is None:
                    first_event = event
                previous_event = event
                if message.event != 'rx':
                    continue
                payload = bytes(message.payload)
                if message.code in CONFIG_CODES:
                    configs[(message.code, message.request_name, payload)] += 1
                if message.code == 150:
                    extra = payload[15]
                    if len(payload) < 23+extra or payload[16+extra] != 30:
                        raise ValueError('Unexpected STATUS_EX layout')
                    flags = struct.unpack_from('<I', payload, 17+extra)[0]
                    status.append(dict(bag_s=t, steady_s=message.steady_time,
                                       mode_bits=int.from_bytes(payload[6:10]+payload[16:16+extra], 'little'),
                                       flags=flags, flag_count=payload[16+extra], rx_failsafe=bool(flags & 4),
                                       sensor_mask=struct.unpack_from('<H', payload, 4)[0],
                                       pid_profile=payload[10], rate_profile=payload[14]))
                if message.code == 105:
                    rc.append(list(struct.unpack('<'+'H'*(len(payload)//2), payload)))
    expected = {x['topic_metadata']['name']: x['message_count'] for x in metadata['topics_with_message_count']}
    if any(counts[t] != n for t, n in expected.items()) or sum(counts.values()) != metadata['message_count']:
        raise ValueError('Message counts differ from metadata')
    id_payloads = {p for (code, _, p) in configs if code == 119}
    if len(id_payloads) != 1:
        raise ValueError('BOXIDS changed or absent; resolve configuration by time before interpreting modes')
    ids = list(next(iter(id_payloads)))
    for row in status:
        row['active_boxes'] = [box for i, box in enumerate(ids) if row['mode_bits'] & (1 << i)]
        row['armed'] = 0 in row['active_boxes']
    readbacks = [dict(code=code, request_name=name, count=count, payload_hex=payload.hex(),
                      decoded=decode_config(code, payload)) for (code, name, payload), count in sorted(configs.items())]
    previous = json.loads(previous_summary.read_text())
    old = {(x['code'], x['request_name']): x for x in previous['msp_readbacks']}
    changes = []
    for item in readbacks:
        prior = old.get((item['code'], item['request_name']))
        if prior is None or prior['payload_hex'] != item['payload_hex']:
            changes.append(dict(code=item['code'], request_name=item['request_name'],
                                previous=decode_config(item['code'], bytes.fromhex(prior['payload_hex'])) if prior else None,
                                current=item['decoded']))
    arm_runs = runs(status, 'armed')
    arm_intervals = []
    for index, run in enumerate(arm_runs):
        if run['value']:
            end = arm_runs[index+1] if index+1 < len(arm_runs) else None
            arm_intervals.append(dict(first_armed_bag_s=run['first_bag_s'], last_armed_bag_s=run['last_bag_s'],
                                      first_disarmed_bag_s=end['first_bag_s'] if end else None,
                                      sampled_transition_duration_steady_s=end['first_steady_s']-run['first_steady_s'] if end else None))
    return dict(bag=bag.name, previous_bag=previous['bag'],
                validation=dict(metadata_counts_match=True, validate_crcs=True, messages=sum(counts.values())),
                time_interpretation='bag_s is uncorrected ROS/wall time; steady_s measures elapsed time independently of wall steps',
                recording_wall_duration_s=metadata['duration']['nanoseconds']*1e-9,
                msp_steady_span_s=previous_event['steady_s']-first_event['steady_s'],
                clock_jumps=clock_jumps, errors_counter_max=errors_max, sessions=sorted(sessions),
                event_counts=[dict(event=e, code=c, request_name=n, count=k) for (e, c, n), k in sorted(event_counts.items())],
                state_reason_transitions=state_reasons, first_health_reason_bag_s=health_first, runtime_logs=logs,
                raw_status=dict(samples=len(status), boxids=ids, arm_intervals=arm_intervals,
                                arm_runs=arm_runs, active_mode_runs=runs(status, 'active_boxes'),
                                distributions={key: dict(collections.Counter(str(row[key]) for row in status))
                                               for key in ('flags', 'flag_count', 'rx_failsafe', 'sensor_mask', 'pid_profile', 'rate_profile')}),
                raw_rc=dict(samples=len(rc), channel_ranges=[dict(zero_based_index=i, minimum=min(r[i] for r in rc),
                                                                 maximum=max(r[i] for r in rc)) for i in range(len(rc[0]))]),
                configuration_changes=changes, configuration_readbacks=readbacks)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--previous-summary', type=Path,
                        default=Path(__file__).resolve().parent.parent/'hardware_diagnostic_20260924'/'summary.json')
    args = parser.parse_args()
    result = summarize(args.bag, args.previous_summary)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(f"Wrote {args.output}; {result['validation']['messages']} messages, "
          f"{len(result['clock_jumps'])} wall-clock discontinuity, {result['raw_status']['samples']} raw STATUS frames")


if __name__ == '__main__':
    main()
