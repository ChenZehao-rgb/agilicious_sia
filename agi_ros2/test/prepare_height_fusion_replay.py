#!/usr/bin/env python3
"""Prepare the six native height-replay CSVs directly from an original MCAP bag.

No ROS installation or generated message classes are needed: embedded schemas
are decoded with mcap-ros2-support. Install dependencies in a virtual environment:
  python3 -m pip install mcap mcap-ros2-support numpy pandas PyYAML
Then run:
  python3 agi_ros2/test/prepare_height_fusion_replay.py --bag bags/BAG --output /tmp/height-inputs
  python3 agi_ros2/test/replay_height_fusion.py --input /tmp/height-inputs

This reproduces the recorded-initialization/reference-inference method used for
hardware_20260930_171830_188540. It does not replay ROS scheduling or independently
establish stationary reference eligibility. CRCs, all topic counts, source hashes,
package versions, and the resulting input hashes are recorded in manifest.json.
"""
import argparse
from collections import Counter
from contextlib import ExitStack
import csv
import hashlib
from importlib.metadata import version
import io
import json
from pathlib import Path
import tempfile

import numpy as np
import pandas as pd
import yaml
from mcap.reader import NonSeekingReader
from mcap_ros2.decoder import DecoderFactory


TOPICS = ('/fused_state', '/sensors/imu', '/sensors/local_navigation',
          '/sensors/baro/sample', '/fusion/baro/status')
OUTPUTS = ('initial', 'imu', 'nav', 'baro', 'references', 'recorded')
ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def flatten(message, prefix=''):
    """Use the original extractor's scalar representations, including integer ns."""
    if hasattr(message, 'sec') and hasattr(message, 'nanosec'):
        return {prefix + '_ns': message.sec * 10**9 + message.nanosec}
    if isinstance(message, (bool, int, float, str)):
        return {prefix: message}
    fields = enumerate(message) if isinstance(message, (list, tuple)) else (
        (field, getattr(message, field)) for field in message.__slots__)
    result = {}
    for field, value in fields:
        result.update(flatten(value, f'{prefix}.{field}' if prefix else str(field)))
    return result


def extract(bag, temporary):
    metadata_path = bag / 'metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text())['rosbag2_bagfile_information']
    if metadata['storage_identifier'] != 'mcap':
        raise ValueError('Expected an MCAP bag with embedded ROS 2 schemas')
    start = metadata['starting_time']['nanoseconds_since_epoch']
    expected = {item['topic_metadata']['name']: item['message_count']
                for item in metadata['topics_with_message_count']}
    counts, writers, columns = Counter(), {}, {}
    with ExitStack() as stack:
        for filename in metadata['relative_file_paths']:
            with (bag / filename).open('rb') as source:
                factory = DecoderFactory()
                records = NonSeekingReader(source, validate_crcs=True).iter_messages(log_time_order=False)
                for schema, channel, record in records:
                    counts[channel.topic] += 1
                    if channel.topic not in TOPICS:
                        continue
                    decoder = factory.decoder_for(channel.message_encoding, schema)
                    if decoder is None:
                        raise ValueError('No embedded-schema decoder for ' + channel.topic)
                    row = {'log_ns': record.log_time, 'bag_s': (record.log_time - start) * 1e-9}
                    row.update(flatten(decoder(record.data)))
                    row['stamp_s'] = (row['header.stamp_ns'] - start) * 1e-9
                    if channel.topic not in writers:
                        name = channel.topic.strip('/').replace('/', '_')
                        stream = stack.enter_context((temporary / (name + '.csv')).open('w', newline=''))
                        writers[channel.topic] = csv.DictWriter(stream, fieldnames=list(row))
                        writers[channel.topic].writeheader()
                        columns[channel.topic] = set(row)
                    if set(row) != columns[channel.topic]:
                        raise ValueError('Replay requires a stable schema for ' + channel.topic)
                    writers[channel.topic].writerow(row)
    mismatches = {key: [counts[key], expected.get(key, 0)] for key in set(counts) | set(expected)
                  if counts[key] != expected.get(key, 0)}
    if mismatches or sum(counts.values()) != metadata['message_count']:
        raise ValueError(f'MCAP topic counts differ from metadata: {mismatches}')
    if set(writers) != set(TOPICS):
        raise ValueError(f'Missing required topics: {set(TOPICS) - set(writers)}')
    return dict(start_ns=start, message_count=sum(counts.values()), topic_counts=dict(counts),
                validation=dict(embedded_schemas=True, stored_crcs_checked=True, all_topic_counts_match_metadata=True),
                source_sha256={name: digest(bag / name) for name in [*metadata['relative_file_paths'], 'metadata.yaml']})


def seconds(values):
    # Match ROS stamp.sec + stamp.nanosec * 1e-9, not float(total_ns) * 1e-9.
    ns = np.asarray(values, dtype=np.int64)
    return (ns // 10**9).astype(float) + (ns % 10**9).astype(float) * 1e-9


def xyz(table, source, prefix):
    return {prefix + axis: table[source + '.' + axis].to_numpy() for axis in 'xyz'}


def variances(table, source, prefix):
    return {prefix + axis: table[source + '.' + str(index)].to_numpy() for index, axis in enumerate('xyz')}


def prepare(temporary):
    frames = {name: pd.read_csv(temporary / (name + '.csv'), float_precision='round_trip')
              for name in ('fused_state', 'sensors_imu', 'sensors_local_navigation', 'sensors_baro_sample')}
    fused = frames['fused_state'].loc[lambda frame: frame.initialized].reset_index(drop=True)
    if fused.empty or fused.reset_counter.nunique() != 1:
        raise ValueError('Replay requires one nonempty initialized EKF session')
    imu = frames['sensors_imu'].set_index('header.stamp_ns', drop=False)
    if not imu.index.is_unique:
        raise ValueError('IMU timestamps must be unique for recorded-state matching')
    imu = imu.loc[fused['header.stamp_ns']].reset_index(drop=True)
    time = seconds(imu['header.stamp_ns'])
    nav = frames['sensors_local_navigation']
    nav = nav[nav.observation_valid & nav.heading_valid & nav.clock_aligned].copy()
    nav = nav.drop_duplicates('header.stamp_ns').sort_values('header.stamp_ns')
    nav = nav[seconds(nav['header.stamp_ns']) > time[0]]
    pressure = frames['sensors_baro_sample']
    pressure = pressure[pressure.valid & pressure.clock_aligned].drop_duplicates('header.stamp_ns')
    pressure = pressure.sort_values('header.stamp_ns').reset_index(drop=True)

    # Preserve analyze.py's default pandas parsing before prepare_inputs.py's
    # round-trip read. In particular, diagnostic values are rounded strings.
    raw = pd.read_csv(temporary / 'fusion_baro_status.csv').sort_values('log_ns').reset_index(drop=True)
    status = raw[['bag_s', 'stamp_s', 'status.0.message']].copy()
    for column in raw:
        if column.endswith('.key'):
            if raw[column].nunique() != 1:
                raise ValueError('Diagnostic keys changed within the recorded session')
            status[raw[column].iloc[0]] = pd.to_numeric(raw[column.replace('.key', '.value')], errors='raise')
    status = pd.read_csv(io.StringIO(status.to_csv(index=False)), float_precision='round_trip')
    valid = status[status.reference_valid.eq(1)]
    refs = valid[valid.reference_pressure_pa.diff().ne(0)]
    if refs.empty:
        raise ValueError('No recorded pressure reference is available to infer')
    references = []
    for status_index, ref in refs.iterrows():
        count = int(ref.reference_samples)
        mean = pressure.pressure_pa.rolling(count).mean()
        mask = pressure.stamp_s.between(ref.stamp_s - .5, ref.stamp_s - .19)
        candidates = (mean - ref.reference_pressure_pa).abs()[mask].dropna()
        if candidates.empty:
            raise ValueError('No pressure window matches the recorded reference time')
        best = candidates.min()
        # Equal quantized means are resolved to the latest matching window,
        # matching the previous 5 Hz status that still had an incomplete reference.
        endpoint = candidates[candidates.le(best + 1e-9)].index[-1]
        window = pressure.loc[:endpoint].tail(count)
        future_resets = status.loc[status_index:][lambda frame: frame.reference_resets.gt(ref.reference_resets)]
        stop = float(future_resets.iloc[0].last_accepted_stamp) if len(future_resets) else time[-1]
        references.append(dict(t=float(seconds(window['header.stamp_ns'])[-1]), pressure=float(window.pressure_pa.mean()),
                               independent_variance=float(ref.reference_independent_variance_m2), stop=stop,
                               recorded_height=float(ref.reference_height_m), window_match_error_pa=float(best)))
    initial = {key + axis: float(fused.iloc[0][source + '.' + axis])
               for source, key, axes in [('position', 'p', 'xyz'), ('velocity', 'v', 'xyz'), ('orientation', 'q', 'wxyz')]
               for axis in axes}
    initial['t'] = time[0]
    for index, axis in enumerate('xyz'):
        initial['bw' + axis] = float(imu.iloc[0]['angular_velocity.' + axis] - fused.iloc[0]['body_rates.' + axis])
        initial['ba' + axis] = 0.
        initial['pvar' + axis] = float(fused.iloc[0]['position_variance.' + str(index)])
        initial['vvar' + axis] = float(fused.iloc[0]['velocity_variance.' + str(index)])
    return {
        'initial': pd.DataFrame([initial]),
        'imu': pd.DataFrame({'t': time, **xyz(imu, 'linear_acceleration', 'a'), **xyz(imu, 'angular_velocity', 'g')}),
        'nav': pd.DataFrame({'t': seconds(nav['header.stamp_ns']), **xyz(nav, 'position', 'p'), **xyz(nav, 'velocity', 'v'),
                             **variances(nav, 'position_variance', 'pvar'), **variances(nav, 'velocity_variance', 'vvar'),
                             'heading': nav.heading.to_numpy(), 'hvar': nav.heading_variance.to_numpy()}),
        'baro': pd.DataFrame({'t': seconds(pressure['header.stamp_ns']), 'pressure': pressure.pressure_pa,
                              'variance': pressure.pressure_variance.replace(0, 4.)}),
        'references': pd.DataFrame(references),
        'recorded': pd.DataFrame({'t': time, 'z': fused['position.z'], 'vz': fused['velocity.z'],
                                  'pvarz': fused['position_variance.2']}),
    }, references


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--bag', type=Path, required=True, help='Original ROS 2 MCAP bag directory')
    parser.add_argument('--output', type=Path, default=ROOT / 'agi_ros2/analysis/baro_weighted_20260930/inputs',
                        help='Destination for six CSVs and manifest.json (default: analysis/baro_weighted_20260930/inputs)')
    args = parser.parse_args()
    bag = args.bag.resolve()
    with tempfile.TemporaryDirectory(prefix='height-replay-extract-') as name:
        temporary = Path(name)
        extraction = extract(bag, temporary)
        outputs, references = prepare(temporary)
    args.output.mkdir(parents=True, exist_ok=True)
    for name, frame in outputs.items():
        if frame.empty or not np.isfinite(frame.to_numpy()).all():
            raise ValueError('Replay input must be nonempty and finite: ' + name)
        frame.to_csv(args.output / (name + '.csv'), index=False, float_format='%.17g')
    manifest = dict(source_bag=bag.name, source_path=str(bag), **extraction,
                    preparation_script_sha256=digest(Path(__file__)),
                    package_versions={name: version(name) for name in ('mcap', 'mcap-ros2-support', 'numpy', 'pandas', 'PyYAML')},
                    rows={name: len(frame) for name, frame in outputs.items()}, references=references,
                    limitations=[
                        'Native EKF numerical replay, not full ROS/DDS callback and timer playback.',
                        'Initialization is seeded from first initialized fused output, with zero initial acceleration bias.',
                        'Reference windows and source-reset cutoff times are inferred from recorded 5 Hz status.',
                        'Recorded-reference replay uses all inferred windows and recorded reset cutoffs.',
                        'Continuous-reference candidates keep the first inferred reference and omit recorded later resets.',
                        'Neither reference mode re-runs physical stationary-reference acquisition gates.',
                        'Sorted observations execute at measurement time plus 0.2 s; DDS late drops are not reproduced.',
                        'Current hardware parameters are not a proven deployment snapshot.',
                        'Static accuracy uses user-reported stationarity; no independently surveyed position truth.'],
                    input_sha256={name + '.csv': digest(args.output / (name + '.csv')) for name in OUTPUTS})
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
    print(json.dumps(manifest, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
