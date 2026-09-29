#!/usr/bin/env python3
"""Verify existing analysis tables against the user-selected original MCAP.

Reads embedded schemas, validates available MCAP CRCs, checks every topic's
count, and compares every cached row/field used by this investigation.
"""
from collections import Counter, defaultdict
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
import pandas as pd
import yaml
from mcap.reader import NonSeekingReader
from mcap_ros2.decoder import DecoderFactory

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BAG = ROOT / 'bags/hardware_20260926_161859_702449'
CACHE = HERE.parent / 'hardware_diagnostic_20260926' / BAG.name
spec = importlib.util.spec_from_file_location('prior_extract', CACHE.parent / 'extract.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def main():
    metadata = yaml.safe_load((BAG / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    start = metadata['starting_time']['nanoseconds_since_epoch']
    expected = {x['topic_metadata']['name']: x['message_count']
                for x in metadata['topics_with_message_count']}
    manifest = json.loads((CACHE / 'manifest.json').read_text())
    rows, counts, hashes = defaultdict(list), Counter(), {}
    for filename in metadata['relative_file_paths']:
        path = BAG / filename
        with path.open('rb') as stream:
            digest = hashlib.sha256()
            while chunk := stream.read(1024 * 1024):
                digest.update(chunk)
        hashes[filename] = digest.hexdigest()
        with path.open('rb') as stream:
            factory = DecoderFactory()
            for schema, channel, record in NonSeekingReader(stream, validate_crcs=True).iter_messages(log_time_order=False):
                counts[channel.topic] += 1
                if channel.topic not in manifest['tables']:
                    continue
                message = factory.decoder_for(channel.message_encoding, schema)(record.data)
                row = {'log_ns': record.log_time, 'bag_s': (record.log_time - start) * 1e-9}
                row.update(module.flatten(message))
                if 'header.stamp_ns' in row:
                    row['stamp_s'] = (row['header.stamp_ns'] - start) * 1e-9
                rows[channel.topic].append(row)
    assert all(counts[k] == v for k, v in expected.items())
    assert sum(counts.values()) == metadata['message_count']
    checks = {}
    for topic, filename in manifest['tables'].items():
        fresh = pd.DataFrame(rows[topic]).sort_values('log_ns', kind='stable').reset_index(drop=True)
        string_columns = {column: str for column in fresh.columns
                          if fresh[column].dropna().map(lambda value: isinstance(value, str)).all()
                          and fresh[column].notna().any()}
        cached = pd.read_csv(CACHE / filename, dtype=string_columns).reset_index(drop=True)
        assert set(fresh.columns) == set(cached.columns), topic
        fresh = fresh[cached.columns].replace('', np.nan)
        for column in fresh:
            if column.endswith('_ns') and fresh[column].notna().all():
                np.testing.assert_array_equal(fresh[column].to_numpy(), cached[column].to_numpy())
        pd.testing.assert_frame_equal(fresh, cached, check_dtype=False, check_exact=False,
                                      rtol=1e-12, atol=1e-12)
        checks[topic] = {'rows': len(fresh), 'columns': len(fresh.columns), 'all_values_match': True}
        print(topic, len(fresh), 'rows match', flush=True)
    result = {'bag': str(BAG), 'start_ns': start,
              'duration_s': metadata['duration']['nanoseconds'] * 1e-9,
              'total_messages': sum(counts.values()), 'sha256': hashes,
              'stored_crcs_checked': True, 'all_counts_match_metadata': True,
              'cached_tables_verified': checks,
              'stationary_interval': 'User-confirmed log receipt time 0 <= bag_s < 100; no external position truth',
              'runtime_identity_limit': 'No full fusion parameter snapshot or running binary identity in the bag'}
    (HERE / 'source_validation.json').write_text(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
