#!/usr/bin/env python3
"""Compare post-calibration flight telemetry, preserving observation validity.

Dependencies: mcap, mcap-ros2-support, numpy, pyyaml. Reads files only.
The operator reports flight after IST8310 calibration; heading truth is unmeasured.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone, timedelta
import importlib.util
import json
import math
from pathlib import Path

from mcap.reader import NonSeekingReader
from mcap_ros2.decoder import DecoderFactory

BASE = Path(__file__).resolve().parents[1] / 'hardware_diagnostic_20260924/analyze.py'
spec = importlib.util.spec_from_file_location('diagnostic_base', BASE)
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)


def summarize(bag):
    result = base.summarize(bag)
    result['interpretation'][0] = (
        'Operator-reported flight after magnetometer calibration (IST8310); '
        'not a stationary sensor-noise calibration or true-north heading validation.')
    result['operator_context'] = {'magnetometer': 'IST8310', 'gps': 'M1025',
                                  'takeoff_mass_kg': 0.734, 'experiment': 'post-calibration flight',
                                  'true_north_alignment_confirmed': True, 'fc_declination_applied': True,
                                  'ros_heading_correction_rad': 0.0, 'fc_mag_declination_value': None}
    selected = ['/sensors/imu', '/sensors/navigation', '/fused_state', '/msp/events',
                '/sensors/mavlink/status', '/rosout', '/navigation/status']
    previous = {}
    gaps = defaultdict(list)
    coverage = {}
    logs = []
    sessions = defaultdict(set)
    heading = []
    heading_steps = []
    imu_resets = Counter()
    start = None
    for filename in sorted(bag.glob('*.mcap')):
        with filename.open('rb') as source:
            reader = NonSeekingReader(source, validate_crcs=True)
            factory = DecoderFactory()
            for schema, channel, record in reader.iter_messages(log_time_order=False):
                t = record.log_time
                start = min(start, t) if start is not None else t
                topic = channel.topic
                if topic not in selected:
                    continue
                decoder = factory.decoder_for(channel.message_encoding, schema)
                if decoder is None:
                    raise ValueError(f'Missing decoder for {topic}')
                message = decoder(record.data)
                stamp = base.stamp_ns(message) if hasattr(message, 'header') else None
                steady = (message.published_steady_time if topic == '/fused_state' else
                          message.steady_time if topic == '/msp/events' else None)
                item = {'log_time_ns': t, 'stamp_ns': stamp, 'steady_time': steady}
                if topic in previous:
                    last = previous[topic]
                    gap = (t-last['log_time_ns'])*1e-9
                    if gap > 0.5 or gap < 0:
                        gaps[topic].append({
                            'from_log_time_ns': last['log_time_ns'], 'to_log_time_ns': t,
                            'record_gap_seconds': gap,
                            'header_gap_seconds': (stamp-last['stamp_ns'])*1e-9 if stamp is not None else None,
                            'steady_gap_seconds': steady-last['steady_time'] if steady is not None else None})
                previous[topic] = item
                if topic not in coverage:
                    coverage[topic] = {'first_log_time_ns': t, 'last_log_time_ns': t, 'messages': 0}
                coverage[topic]['last_log_time_ns'] = t
                coverage[topic]['messages'] += 1
                if topic == '/rosout' and message.name != 'rosbag2_recorder':
                    logs.append({'log_time_ns': t, 'node': message.name, 'level': message.level,
                                 'message': message.msg})
                if topic == '/sensors/navigation':
                    sessions[topic].add(message.source_session)
                    if message.heading_valid and math.isfinite(message.heading):
                        heading.append(message.heading)
                        if len(heading) > 1:
                            heading_steps.append(math.degrees(math.remainder(heading[-1]-heading[-2], 2*math.pi)))
                if topic == '/fused_state':
                    sessions[topic].add(message.clock_id)
                    imu_resets[str(message.reset_counter)] += 1
                if topic == '/msp/events':
                    sessions[topic].add(message.session_id)
    for gap_list in gaps.values():
        for gap in gap_list:
            gap['from_bag_seconds'] = (gap['from_log_time_ns']-start)*1e-9
            gap['to_bag_seconds'] = (gap['to_log_time_ns']-start)*1e-9
    for item in coverage.values():
        item['first_bag_seconds'] = (item['first_log_time_ns']-start)*1e-9
        item['last_bag_seconds'] = (item['last_log_time_ns']-start)*1e-9
    result['temporal_evidence'] = {
        'bag_start_local': datetime.fromtimestamp(start/1e9, timezone(timedelta(hours=8))).isoformat(),
        'coverage': coverage, 'record_gaps_over_half_second_or_backwards': dict(gaps),
        'sessions': {k: sorted(v) for k, v in sessions.items()},
        'fused_reset_counter_counts': dict(imu_resets), 'business_logs': logs,
        'heading_enu_degrees': base.statistics(math.degrees(h) for h in heading),
        'successive_heading_step_degrees': base.statistics(heading_steps),
        'heading_caveat': 'Range/continuity without a true heading reference cannot validate mounting or declination.',
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = summarize(args.bag)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(f"Read {result['validation']['messages']} messages; saved {args.output}")


if __name__ == '__main__':
    main()
