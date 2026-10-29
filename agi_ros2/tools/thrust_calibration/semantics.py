"""Explicit field contracts; never infer physical semantics from a numeric range."""
import numpy as np

TIME = {'s': 1., 'ms': .001, 'us': 1e-6}
VOLTAGE = {'V': 1., 'mV': .001, 'centivolt': .01}
FORCE = {'N': 1., 'kgf': 9.80665, 'gf': .00980665}


def require_evidence(config, keys):
    for key in keys:
        if config.get(key) is None or config.get(key) == '' or config.get(key) == 'unknown':
            raise ValueError(f'Missing verified configuration: {key}')


def numeric(cols, field):
    if field not in cols:
        raise ValueError(f'Missing field: {field}')
    return np.asarray(cols[field], dtype=float)


def times(cols, cfg):
    require_evidence(cfg, ['timestamp_field', 'timestamp_unit', 'timestamp_meaning'])
    return numeric(cols, cfg['timestamp_field']) * TIME[cfg['timestamp_unit']]


def raw_rc(cols, cfg):
    require_evidence(cfg, ['field', 'semantics', 'evidence', 'firmware_match', 'decode_scale'])
    if cfg['firmware_match'] != 'verified' or cfg.get('axis_verified') is not True:
        raise ValueError('axis_unresolved: firmware/log/config match not verified')
    field = cfg['field']
    r = numeric(cols, field) * float(cfg['decode_scale'])
    if cfg['semantics'] == 'rx_range_rcdata':
        if field != 'debug[3]' or cfg.get('debug_mode') != 'RX_SIGNAL_LOSS' or cfg['decode_scale'] != 1:
            raise ValueError('debug[3] requires RX_SIGNAL_LOSS and unscaled debug evidence')
        if cfg.get('same_msp_rx_path_verified') is not True or cfg.get('rx_smoothing') != 'none':
            raise ValueError('Unknown MSP/manual RX path or stateful PWM/PPM averaging')
        require_evidence(cfg, ['rx_min', 'rx_max', 'rcdata_clip_min', 'rcdata_clip_max', 'quantization_rcdata'])
        lo, hi = float(cfg['rx_min']), float(cfg['rx_max'])
        if not (hi > lo and cfg['quantization_rcdata'] >= 0):
            raise ValueError('Invalid RX range/quantization')
        c = lo + (r - 1000.) * (hi - lo) / 1000.
        uncertainty = cfg['quantization_rcdata'] * (hi - lo) / 1000.
        # Clipping boundaries have a set of preimages, not a unique raw command.
        valid = (r > cfg['rcdata_clip_min'] + cfg['quantization_rcdata']) & (r < cfg['rcdata_clip_max'] - cfg['quantization_rcdata'])
    elif cfg['semantics'] == 'msp_raw_rc_throttle':
        if field in ('rcCommand[3]', 'setpoint[3]', 'debug[3]') or field.startswith('motor'):
            raise ValueError('Processed/debug/motor fields cannot be relabelled raw RC')
        c, uncertainty, valid = r, float(cfg.get('quantization_rcdata', 1.)), np.ones(len(r), dtype=bool)
    else:
        raise ValueError('axis_unresolved: processed throttle inverse is not established; recorded-axis analysis only')
    checks = cfg.get('valid_when', {})
    if not checks:
        raise ValueError('Receiver validity/failsafe/held-value evidence required via valid_when')
    for name, expected in checks.items():
        valid &= numeric(cols, name) == expected
    valid &= np.isfinite(c) & (c >= 1000) & (c <= 2000)
    return c, valid, uncertainty


def force_newtons(cols, cfg):
    require_evidence(cfg, ['unit', 'sign', 'offset', 'calibration_evidence', 'zero_is_no_thrust'])
    if cfg['zero_is_no_thrust'] is not True or cfg['sign'] not in (-1, 1):
        raise ValueError('Force zero must be an independently confirmed no-thrust baseline; sign must be +/-1')
    if 'projection' in cfg:
        fields = cfg['projection']
        if not fields or not np.isclose(sum(float(x)**2 for x in fields.values()), 1.):
            raise ValueError('Force projection must be a unit direction, never a magnitude')
        raw = sum(numeric(cols, key)*float(value) for key, value in fields.items())
    else:
        raw = numeric(cols, cfg['force_field'])
    if cfg['unit'] == 'counts':
        require_evidence(cfg, ['scale_to_n'])
        scale = float(cfg['scale_to_n'])
    else:
        scale = FORCE[cfg['unit']]
        if 'scale_to_n' in cfg and cfg['scale_to_n'] != scale:
            raise ValueError('Conflicting force unit and conversion coefficient')
    if not np.isfinite(scale) or scale <= 0 or not np.isfinite(cfg['offset']):
        raise ValueError('Invalid force calibration')
    f = (raw-float(cfg['offset']))*scale*cfg['sign']
    valid = np.isfinite(f)
    for key, value in cfg.get('valid_when', {}).items():
        valid &= numeric(cols, key) == value
    return raw, f, valid
