#!/usr/bin/env python3
"""Decode embedded ROS2 MCAP schemas without a sourced ROS installation.

Requires mcap and mcap-ros2-support. Missing instrumentation in older bags is
reported as unavailable, never estimated from recorder arrival timestamps.
"""
import argparse
import collections
import csv
import json
import math
from pathlib import Path


TOPICS = ('/sensors/imu/timing', '/fused_state', '/control_command',
          '/output_timing', '/output_status', '/msp/write_timing')


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def field(message, name, default=None):
    return getattr(message, name, default)


def milliseconds(later, earlier):
    if later is None or earlier is None:
        return None
    if not math.isfinite(later) or not math.isfinite(earlier) or later <= 0 or earlier <= 0:
        return None
    return (later - earlier) * 1000


def stats(values):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    if not values:
        return {'n': 0}
    return {'n': len(values), 'min': values[0], 'p50': values[len(values) // 2],
            'p95': values[int((len(values) - 1) * .95)],
            'p99': values[int((len(values) - 1) * .99)], 'max': values[-1]}


def write_csv(path, rows):
    columns = list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: None if isinstance(v, float) and not math.isfinite(v) else v
                             for k, v in row.items()})


def analyze(paths, namespace=''):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    topics = {namespace.rstrip('/') + topic: topic for topic in TOPICS}
    counts = collections.Counter()
    imu = collections.defaultdict(list)
    fusion = {}
    commands = []
    outputs = {}
    writes = []
    faults = []
    last_fault = {}
    for path in paths:
        with path.open('rb') as stream:
            reader = make_reader(stream, decoder_factories=[DecoderFactory()])
            for _, channel, _, message in reader.iter_decoded_messages(topics=list(topics)):
                topic = topics[channel.topic]
                counts[topic] += 1
                clock = field(message, 'clock_id', '').removesuffix(':ros')
                if topic == '/sensors/imu/timing':
                    imu[clock, stamp_ns(message.header.stamp)].append(message)
                elif topic == '/fused_state':
                    fusion[clock, message.imu_receive_time] = (
                        stamp_ns(message.header.stamp), message.published_steady_time)
                elif topic == '/control_command':
                    commands.append(message)
                elif topic == '/output_timing':
                    outputs[clock, message.control_session_start, message.command_sequence] = message
                elif topic == '/msp/write_timing':
                    row = {name: field(message, name) for name in (
                        'clock_id', 'session_id', 'control_session_start', 'command_sequence',
                        'attempt_id', 'code', 'outcome', 'frame_bytes', 'bytes_written', 'system_error',
                        'started_steady_time', 'finished_steady_time', 'deadline_steady_time',
                        'write_calls', 'eagain_count', 'failure_attempt_id', 'failure_code')}
                    for name in ('elapsed_seconds', 'deadline_overrun_seconds', 'write_syscall_seconds',
                                 'poll_seconds', 'thread_cpu_seconds'):
                        value = field(message, name)
                        row[name.removesuffix('_seconds') + '_ms'] = value * 1000 if math.isfinite(value) else None
                    writes.append(row)
                elif topic == '/output_status':
                    key = clock, message.session_start
                    if message.fault_count > last_fault.get(key, 0):
                        faults.append(dict(ros_time_ns=stamp_ns(message.header.stamp), clock_id=clock,
                                           output_session_start=message.session_start,
                                           steady_time=message.steady_time, fault_count=message.fault_count,
                                           reason=message.reason, last_fault=message.last_fault))
                    last_fault[key] = message.fault_count

    pipeline = []
    for command in commands:
        clock = command.clock_id.removesuffix(':ros')
        session = field(command, 'control_session_start', 0)
        output = outputs.get((clock, session, command.sequence))
        received = command.evidence.imu_time if not command.clock_id.endswith(':ros') else None
        state = fusion.get((clock, received))
        state_stamp = stamp_ns(command.state_stamp) if field(command, 'state_stamp') else (state[0] if state else None)
        state_pub = field(command, 'state_published_steady_time', state[1] if state else None)
        sensor = None
        if state_stamp is not None and received is not None:
            candidates = [sample for sample in imu.get((clock, state_stamp), [])
                          if 0 <= received - sample.published_steady_time < 1]
            sensor = max(candidates, key=lambda s: s.published_steady_time, default=None)
        row = dict(ros_time_ns=stamp_ns(command.header.stamp), clock_id=clock,
                   control_session_start=session, sequence=command.sequence,
                   permit_override=command.permit_override, state_stamp_ns=state_stamp,
                   output_active=field(output, 'override_active'),
                   output_fault_count=field(output, 'fault_count'), output_reason=field(output, 'reason'),
                   imu_age_control_ms=milliseconds(command.evidence.now, received),
                   mapped_sample_age_ms=sensor.mapped_sample_age * 1000 if sensor else None)
        pairs = {
            'sensor_decode_ms': (field(sensor, 'published_steady_time'), field(sensor, 'receive_steady_time')),
            'sensor_publish_ms': (field(sensor, 'publish_return_steady_time'), field(sensor, 'published_steady_time')),
            'imu_to_fusion_ms': (received, field(sensor, 'published_steady_time')),
            'fusion_ms': (state_pub, received),
            'fusion_to_control_ms': (field(command, 'state_received_steady_time'), state_pub),
            'control_wait_ms': (field(command, 'control_start_steady_time'), field(command, 'state_received_steady_time')),
            'control_ms': (field(command, 'published_steady_time'), field(command, 'control_start_steady_time')),
            'command_to_output_ms': (field(output, 'command_received_steady_time'), field(command, 'published_steady_time')),
            'output_precheck_ms': (field(output, 'output_check_steady_time'), field(output, 'command_received_steady_time')),
            'output_ms': (field(output, 'output_finished_steady_time'), field(output, 'command_received_steady_time')),
            'imu_age_output_ms': (field(output, 'output_check_steady_time'), received),
            'imu_to_command_ms': (field(command, 'published_steady_time'), received),
        }
        row.update({name: milliseconds(*times) for name, times in pairs.items()})
        pipeline.append(row)
    return counts, pipeline, writes, faults


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--namespace', default='', help='ROS namespace prefix, e.g. /bench')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    paths = sorted(args.bag.glob('*.mcap')) if args.bag.is_dir() else [args.bag]
    if not paths or any(not path.is_file() for path in paths):
        parser.error('No MCAP files found')
    try:
        counts, pipeline, writes, faults = analyze(paths, args.namespace)
    except ModuleNotFoundError as error:
        parser.error(f'{error}; install mcap and mcap-ros2-support')
    args.output.mkdir(parents=True, exist_ok=True)
    write_csv(args.output / 'pipeline.csv', pipeline)
    write_csv(args.output / 'writes.csv', writes)
    write_csv(args.output / 'faults.csv', faults)
    summary = {'topic_counts': dict(counts), 'faults': faults, 'missing_topics': [
        topic for topic in TOPICS if counts[topic] == 0], 'pipeline_ms': {}, 'writes_ms': {}}
    metrics = [k for row in pipeline[:1] for k in row if k.endswith('_ms')]
    for label, selection in [('all', pipeline), ('permitted', [r for r in pipeline if r['permit_override']])]:
        summary['pipeline_ms'][label] = {k: stats(row[k] for row in selection) for k in metrics}
    for code in sorted(set(row['code'] for row in writes)):
        selection = [r for r in writes if r['code'] == code and r['write_calls'] > 0]
        summary['writes_ms'][str(code)] = {k: stats(row[k] for row in selection) for k in (
            'elapsed_ms', 'deadline_overrun_ms', 'write_syscall_ms', 'poll_ms', 'thread_cpu_ms')}
    summary['write_outcomes'] = dict(collections.Counter(row['outcome'] for row in writes))
    summary['failed_writes'] = [row for row in writes if row['outcome'] in ('failed', 'transport_latched')]
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    print(f'Commands={len(pipeline)}, write attempts={len(writes)}, faults={len(faults)}')
    print(f'Results: {args.output.resolve()}')
    if summary['missing_topics']:
        print('Unavailable topics: ' + ', '.join(summary['missing_topics']))
    for name, result in summary['pipeline_ms']['permitted'].items():
        if result['n']:
            print(f"{name}: n={result['n']}, p95={result['p95']:.3f}, max={result['max']:.3f}")


if __name__ == '__main__':
    main()
