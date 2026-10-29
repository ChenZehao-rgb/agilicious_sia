"""Browse first, then report every unresolved contract without inventing defaults."""
from pathlib import Path
import numpy as np
from log_io import inspect_file, read_columns, digest
from semantics import times, raw_rc, force_newtons, numeric, VOLTAGE, require_evidence


def inspect_pair(session, config):
    report = {'run_id': session['run_id'], 'pairing': session, 'files': {}, 'gaps': [],
              'deployable_axis': False, 'force_verified': False}
    for source in ('bf', 'px4'):
        path = session[source]['file']
        report['files'][source] = inspect_file(path)
        cfg = config[source]
        try:
            cols, dropouts = read_columns(path, {**cfg, **session[source]})
            t = times(cols, cfg)
            report[source] = {'time_first_s': float(t[0]), 'time_last_s': float(t[-1]),
                              'duplicate_or_reverse_count': int(np.sum(np.diff(t) <= 0)),
                              'dropouts': dropouts, 'raw_timestamp_field': cfg['timestamp_field'],
                              'timestamp_meaning': cfg['timestamp_meaning']}
            if source == 'bf':
                c, valid, error = raw_rc(cols, cfg['axis'])
                require_evidence(cfg, ['voltage_unit', 'voltage_evidence', 'decoder_quality_reviewed'])
                if cfg['decoder_quality_reviewed'] is not True:
                    raise ValueError('Decoder damaged-frame statistics not reviewed')
                numeric(cols, cfg['voltage_field']) * VOLTAGE[cfg['voltage_unit']]
                for field in cfg['motor_fields']:
                    numeric(cols, field)
                report.update(deployable_axis=True, axis_uncertainty_rc=error)
                report[source]['axis_valid_fraction'] = float(np.mean(valid))
            else:
                raw, force, valid = force_newtons(cols, cfg)
                require_evidence(session[source], ['topic', 'instance'])
                report.update(force_verified=True)
                report[source]['force_valid_fraction'] = float(np.mean(valid))
                report[source]['independent_sample_rate'] = cfg.get('independent_sample_rate', 'unknown')
        except (ValueError, KeyError, TypeError, ImportError) as error:
            report['gaps'].append(f'{source}: {error}')
    report['quality'] = 'ready_for_alignment' if not report['gaps'] else 'unresolved'
    return report


def load_sessions(path):
    import yaml
    path = Path(path).resolve()
    data = yaml.safe_load(path.read_text())
    runs = data['sessions']
    seen = set()
    for s in runs:
        require_evidence(s, ['run_id', 'battery_id', 'block_id', 'arm_cycle', 'split', 'tested_motor_index',
                             'pairing_evidence', 'configuration_file', 'provenance'])
        if s['run_id'] in seen or s['split'] not in ('train', 'validation', 'test'):
            raise ValueError('Unique run_id per paired segment and explicit train/validation/test required')
        if not isinstance(s['run_id'], str) or not s['run_id'] or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in s['run_id']):
            raise ValueError('run_id must be a safe filename component')
        if type(s['tested_motor_index']) is not int or not 0 <= s['tested_motor_index'] < 4:
            raise ValueError('tested_motor_index must be 0..3')
        seen.add(s['run_id'])
        if s['provenance'] not in ('synthetic', 'measured'):
            raise ValueError('Unknown provenance')
        if s['provenance'] == 'measured':
            require_evidence(s, ['manual_control_source_evidence', 'original_source_evidence'])
        s['configuration_file'] = str((path.parent / s['configuration_file']).resolve())
        s['configuration_sha256'] = digest(s['configuration_file'])
        for name in ('bf', 'px4'):
            require_evidence(s[name], ['file', 'interval_s'])
            interval=s[name]['interval_s']
            if not isinstance(interval,list) or len(interval)!=2 or not all(type(x) in (int,float) and np.isfinite(x) for x in interval) or interval[0]>=interval[1]:
                raise ValueError('Explicit increasing finite interval_s required')
            s[name]['file'] = str((path.parent / s[name]['file']).resolve())
            s[name]['sha256'] = digest(s[name]['file'])
        if not isinstance(s['bf'].get('log_index'), int) or s['bf']['log_index'] < 1:
            raise ValueError('Explicit 1-based BF log_index required even for decoded CSV')
    # An arm/block cannot leak across splits, even if represented by several segments.
    splits = {}
    for s in runs:
        key = (s['battery_id'], s['block_id'])
        if key in splits and splits[key] != s['split']:
            raise ValueError('Block leakage across training/validation/test')
        splits[key] = s['split']
    for i, a in enumerate(runs):
        for b in runs[i+1:]:
            if a['split']==b['split']: continue
            for side in ('bf','px4'):
                aa,bb=a[side],b[side]
                identity_keys=('log_index',) if side=='bf' else ('topic','instance')
                same=aa['sha256']==bb['sha256'] and all(aa.get(k)==bb.get(k) for k in identity_keys)
                if same and max(aa['interval_s'][0],bb['interval_s'][0])<min(aa['interval_s'][1],bb['interval_s'][1]):
                    raise ValueError('Source interval leakage across data splits')
    return runs
