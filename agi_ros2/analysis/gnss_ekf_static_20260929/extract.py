#!/usr/bin/env python3
"""Read the original MCAP with embedded schemas and validate stored CRCs.

Dependencies: mcap, mcap-ros2-support, pandas, numpy, pyyaml.
Outputs preserve per-topic message grain and integer acquisition/receipt times.
"""
from collections import Counter, defaultdict
import json
from pathlib import Path

from mcap.reader import NonSeekingReader
from mcap_ros2.decoder import DecoderFactory
import pandas as pd
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BAGS = ['hardware_20260929_112215_827325']
SELECTED = {'/sensors/imu', '/sensors/navigation', '/sensors/local_navigation',
            '/fused_state', '/authority', '/navigation/status', '/navigation/origin',
            '/sensors/mavlink/status', '/rosout', '/parameter_events', '/health', '/msp/decoded_state'}


def flatten(message, prefix=''):
    if isinstance(message, bytes):
        return {prefix: message.hex()}
    if hasattr(message, 'sec') and hasattr(message, 'nanosec'):
        return {prefix + '_ns': message.sec * 1_000_000_000 + message.nanosec}
    if isinstance(message, (bool, int, float, str)):
        return {prefix: message}
    if isinstance(message, (list, tuple)):
        result = {}
        for i, value in enumerate(message):
            result.update(flatten(value, f'{prefix}.{i}'))
        return result
    result = {}
    for field in message.__slots__:
        result.update(flatten(getattr(message, field), f'{prefix}.{field}' if prefix else field))
    return result


def extract(name):
    bag = ROOT / 'bags' / name
    target = HERE / name
    target.mkdir(parents=True, exist_ok=True)
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    start = metadata['starting_time']['nanoseconds_since_epoch']
    expected = {x['topic_metadata']['name']: x['message_count'] for x in metadata['topics_with_message_count']}
    rows, counts, schemas = defaultdict(list), Counter(), {}
    for filename in metadata['relative_file_paths']:
        with (bag / filename).open('rb') as source:
            factory = DecoderFactory()
            for schema, channel, record in NonSeekingReader(source, validate_crcs=True).iter_messages(log_time_order=False):
                topic = channel.topic
                counts[topic] += 1
                if topic not in SELECTED:
                    continue
                decoder = factory.decoder_for(channel.message_encoding, schema)
                if decoder is None:
                    raise ValueError(f'No decoder for embedded schema: {topic}')
                message = decoder(record.data)
                row = {'log_ns': record.log_time, 'bag_s': (record.log_time - start) * 1e-9}
                row.update(flatten(message))
                if 'header.stamp_ns' in row:
                    row['stamp_s'] = (row['header.stamp_ns'] - start) * 1e-9
                rows[topic].append(row)
                schemas[topic] = schema.name
    assert all(counts[k] == v for k, v in expected.items()), (counts, expected)
    assert sum(counts.values()) == metadata['message_count']
    manifest = {'bag': str(bag), 'start_ns': start,
                'duration_s': metadata['duration']['nanoseconds'] * 1e-9,
                'topic_counts': expected, 'schemas': schemas,
                'validation': {'embedded_schemas': True, 'stored_crcs_checked': True,
                               'all_topic_counts_match_metadata': True}, 'tables': {}}
    for topic, records in rows.items():
        frame = pd.DataFrame(records).sort_values('log_ns', kind='stable')
        filename = topic.strip('/').replace('/', '_') + '.csv.gz'
        frame.to_csv(target / filename, index=False, compression='gzip')
        manifest['tables'][topic] = filename
    (target / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    print(name, len(counts), sum(counts.values()), 'messages verified', flush=True)


if __name__ == '__main__':
    for name in BAGS:
        extract(name)
