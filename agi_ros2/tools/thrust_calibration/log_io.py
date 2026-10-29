"""Read-only adapters. Physical units always come from explicit evidence/configuration."""
import csv
import hashlib
import importlib.metadata
import json
import re
from pathlib import Path
import subprocess

import numpy as np

VERSION = 'manual-blackbox-px4/1'


def digest(path):
    if not Path(path).is_file():
        raise ValueError('Only regular input files are supported (no devices/pipes)')
    h = hashlib.sha256()
    with open(path, 'rb') as stream:
        for part in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(part)
    return h.hexdigest()


def kind(path):
    if not Path(path).is_file():
        raise ValueError('Only regular input files are supported (no devices/pipes)')
    with open(path, 'rb') as stream:
        head = stream.read(4096)
    if head.startswith(b'ULog\x01\x12\x35'):
        return 'ulog'
    if b'H Product:Blackbox flight data recorder' in head:
        return 'blackbox'
    try:
        if ',' in head.decode('utf-8-sig'):
            return 'csv'
    except UnicodeDecodeError:
        pass
    raise ValueError(f'Unrecognized content: {path}')


def csv_columns(path):
    with open(path, newline='', encoding='utf-8-sig') as stream:
        reader = csv.reader(stream, skipinitialspace=True)
        names = [s.strip() for s in next(reader)]
        if len(names) != len(set(names)):
            raise ValueError('Duplicate CSV columns')
        rows = list(reader)
    if not rows or any(len(row) != len(names) for row in rows):
        raise ValueError('Empty CSV or malformed row')
    columns = {}
    for i, name in enumerate(names):
        try:
            values = [row[i] for row in rows]
            columns[name] = np.array([int(x) for x in values]) if all(re.fullmatch(r'[+-]?\d+', x.strip()) for x in values) else np.array([float(x) for x in values])
        except ValueError:
            columns[name] = np.array([row[i] for row in rows])
    return columns


def read_columns(path, config):
    fmt = kind(path)
    if fmt == 'csv':
        return csv_columns(path), []
    if fmt != 'ulog':
        raise ValueError('Run decode first; select one Blackbox log explicitly')
    from pyulog import ULog
    log = ULog(str(path))
    if log.file_corruption:
        raise ValueError('Corrupt ULog; inspect/recover explicitly before calibration')
    matches = [d for d in log.data_list if d.name == config['topic'] and d.multi_id == config['instance']]
    if len(matches) != 1:
        raise ValueError('Select exactly one PX4 topic/instance, no implicit concatenation')
    dropouts = [{'timestamp_us': int(d.timestamp), 'duration_ms': int(d.duration)} for d in log.dropouts]
    return matches[0].data, dropouts


def inspect_file(path):
    path = Path(path)
    result = dict(path=str(path.resolve()), sha256=digest(path), format=kind(path), tool=VERSION,
                  physical_units='unknown until configured with evidence')
    if result['format'] == 'blackbox':
        # Headers only; never interpret binary frame contents as CSV.
        data = path.read_bytes()
        starts = [m.start() for m in re.finditer(b'H Product:Blackbox flight data recorder', data)]
        logs = []
        for index, start in enumerate(starts, 1):
            headers = []
            for line in data[start:].split(b'\n'):
                if not line.startswith(b'H '):
                    break
                headers.append(line.decode('ascii', errors='replace'))
            logs.append(dict(log_index=index, byte_offset=start, headers=headers))
        result['logs'] = logs
        result['next_step'] = 'decode selected log; binary fields/units require decoder and firmware evidence'
        return result
    if result['format'] == 'ulog':
        from pyulog import ULog
        log = ULog(str(path))
        datasets = [(d.name, d.multi_id, d.data) for d in log.data_list]
        result['file_corruption'] = log.file_corruption
        result['pyulog_version'] = importlib.metadata.version('pyulog')
        result['dropouts'] = [dict(timestamp_us=int(d.timestamp), duration_ms=int(d.duration)) for d in log.dropouts]
    else:
        datasets = [('csv', None, csv_columns(path))]
    result['datasets'] = []
    for topic, instance, cols in datasets:
        fields = {}
        for name, values in cols.items():
            a = np.asarray(values)
            entry = dict(dtype=str(a.dtype), count=len(a))
            if np.issubdtype(a.dtype, np.number):
                good = a[np.isfinite(a)]
                entry.update(finite_count=len(good), min=float(np.min(good)) if len(good) else None,
                             max=float(np.max(good)) if len(good) else None)
                if 'time' in name.lower() and len(a) > 1:
                    dt = np.diff(a.astype(np.float64))
                    entry.update(first=a[0].item(), last=a[-1].item(), duplicates=int(np.sum(dt == 0)),
                                 reversals=int(np.sum(dt < 0)), median_positive_dt=float(np.median(dt[dt > 0])) if np.any(dt > 0) else None)
            fields[name] = entry
        result['datasets'].append(dict(topic=topic, instance=instance, fields=fields))
    result['mode_observability'] = dict(msp_override='unknown: candidate firmware stores only low 32 internal BOX bits',
                                       armed='mode condition alone is not actual arming')
    result['voltage_semantics'] = 'BF vbatLatest=unfiltered; MSP130=displayFiltered; steady-state compatibility must be checked'
    result['axis_semantics'] = {'rcCommand[3]': 'processed throttle, NOT raw RC',
                                'setpoint[3]': 'mixer throttle x1000, NOT raw RC',
                                'motor[i]': 'internal motor command, NOT RC/RPM/force',
                                'debug[3]': 'rcData only with verified RX_SIGNAL_LOSS firmware semantics'}
    return result


def decode_blackbox(path, output, decoder, log_index):
    """Probe executable first; no shell, --raw or simulated channels."""
    if kind(path) != 'blackbox' or log_index < 1:
        raise ValueError('Expected Blackbox container and 1-based log index')
    out = Path(output).resolve()
    if out.exists():
        raise ValueError('Decode output directory must be new (preserve prior artifacts)')
    probe = subprocess.run([decoder, '--help'], capture_output=True, text=True, check=False)
    help_text = probe.stdout + probe.stderr
    required = ['--index', '--stdout', '--unit-frame-time', '--unit-vbat', '--output-dir']
    if any(option not in help_text for option in required):
        raise ValueError('Decoder help lacks required explicit index/stdout/unit options')
    if log_index > len(inspect_file(path)['logs']):
        raise ValueError('Selected Blackbox log index is absent')
    out.mkdir(parents=True)
    command = [decoder, '--index', str(log_index), '--stdout', '--output-dir', str(out), '--unit-frame-time', 'us', '--unit-vbat', 'V', str(Path(path).resolve())]
    with (out / 'blackbox.csv').open('w') as stream:
        run = subprocess.run(command, stdout=stream, stderr=subprocess.PIPE, text=True, cwd=out, check=False)
    metadata = dict(tool=VERSION, command=command, decoder_sha256=digest(decoder), help=help_text,
                    decoder_version='unconfirmed; help and executable hash preserved',
                    source=inspect_file(path), log_index=log_index, returncode=run.returncode,
                    frame_statistics_and_warnings=run.stderr, time_unit='us', voltage_unit='V',
                    decoder_quality_reviewed=False)
    (out / 'decode.json').write_text(json.dumps(metadata, indent=2))
    if run.returncode:
        raise ValueError('Decoder failed; see decode.json. Partial output cannot be calibrated')
    return metadata


def column(columns, name):
    if not name or name not in columns:
        raise ValueError(f'Missing required field: {name!r}')
    return np.asarray(columns[name], dtype=float)


def time_seconds(columns, cfg):
    if not cfg.get('timestamp_meaning'):
        raise ValueError('timestamp_meaning must describe actual sample/publish/receive semantics')
    factor = {'s': 1., 'ms': .001, 'us': 1e-6}.get(cfg.get('time_unit'))
    if factor is None:
        raise ValueError('Unknown time unit')
    raw = column(columns, cfg['time_field'])
    t = raw * factor
    if not np.all(np.isfinite(t)) or len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError('Time duplicate/reset/nonfinite: explicitly segment source; do not sort or merge')
    return t


def recover_rc(columns, cfg):
    field = cfg.get('field')
    if cfg.get('kind') != 'rx_signal_loss' or field != 'debug[3]':
        raise ValueError('axis_unresolved: only evidenced RX_SIGNAL_LOSS RX-range inversion is implemented')
    required = ('source_commit', 'firmware_match_evidence', 'range_evidence', 'manual_msp_same_path_evidence', 'validity_evidence')
    if cfg.get('debug_mode') != 'RX_SIGNAL_LOSS' or not all(cfg.get(k) for k in required):
        raise ValueError('axis_unresolved: debug mode/source/range/validity evidence missing')
    if cfg.get('decoded_scale') != 1 or cfg.get('receiver_smoothing') != 'none':
        raise ValueError('debug scaling or receiver moving-average semantics unresolved')
    lo, hi = cfg['rx_min'], cfg['rx_max']
    if not (np.isfinite(lo) and np.isfinite(hi) and lo < hi):
        raise ValueError('Invalid RX range')
    r = column(columns, field)
    c = lo + (r - 1000.) * (hi - lo) / 1000.
    valid = np.isfinite(r) & (r > cfg['clip_min']) & (r < cfg['clip_max']) & (c >= 1000) & (c <= 2000)
    for check in cfg['validity_checks']:
        valid &= column(columns, check['field']) == check['equals']
    if not cfg['validity_checks']:
        raise ValueError('Explicit validity/failsafe checks required; debug[3] alone can hold stale RC')
    # DEBUG_SET truncates float to integer: report a full quantum, not false exactness.
    return c, valid, abs(hi - lo) / 1000.


def force_newtons(columns, cfg, times, tare):
    if not all(cfg.get(k) for k in ('unit_evidence', 'calibration_evidence', 'topic', 'timestamp_meaning')):
        raise ValueError('Force unit/calibration/topic/time evidence unresolved')
    if cfg.get('fields'):
        direction = np.asarray(cfg['projection'], float)
        if len(direction) != len(cfg['fields']) or not np.isclose(np.linalg.norm(direction), 1.):
            raise ValueError('Force projection must be an explicit unit direction')
        raw = sum(column(columns, f) * d for f, d in zip(cfg['fields'], direction))
    else:
        raw = column(columns, cfg['force_field'])
    scales = {'N': 1., 'kgf': 9.80665, 'gf': .00980665}
    scale = cfg.get('scale_to_n') if cfg['unit'] == 'counts' else scales.get(cfg['unit'])
    if scale is None or not np.isfinite(scale) or scale <= 0 or cfg['sign'] not in (-1, 1):
        raise ValueError('Unknown force unit, scale or sign')
    if tare.get('state') != 'unpowered_no_thrust' or not tare.get('evidence'):
        raise ValueError('Tare must be measured without thrust, never armed idle')
    pre, post = [((times >= a) & (times <= b)) for a, b in (tare['before_s'], tare['after_s'])]
    if min(np.sum(pre), np.sum(post)) < 3:
        raise ValueError('Insufficient before/after tare samples')
    zero = float(np.mean(raw[pre]))
    drift = float((np.mean(raw[post]) - zero) * scale * cfg['sign'])
    force = (raw - zero) * scale * cfg['sign']
    valid = np.isfinite(force)
    for check in cfg.get('validity_checks', []):
        valid &= column(columns, check['field']) == check['equals']
    if cfg.get('sequence_field'):
        seq = column(columns, cfg['sequence_field'])
        valid[1:] &= np.diff(seq) > 0
    return force, valid, dict(raw_zero=zero, drift_n=drift, scale_to_n=scale, sign=cfg['sign'], raw_unit=cfg['unit'])
