"""Version 1 reference-table contract, shared by launch and the offline exporter."""
import csv
import math
import re
from pathlib import Path
import yaml

EXACT = dict(schema_version=1, model='reference_table', command_axis='msp_raw_rc_throttle', axis_verified=True,
             thrust_unit='N', thrust_scope='total_vehicle', calibration_voltage_source='blackbox_vbatLatest',
             runtime_voltage_source='msp130_display_filtered', voltage_compatibility='steady_state', no_ir_correction=True,
             tested_motor_count=1, motor_count=4, aggregation='single_motor_times_four')
NUMBERS = ('reference_voltage_v', 'voltage_exponent_p', 'valid_voltage_min_v', 'valid_voltage_max_v',
           'valid_rc_min', 'valid_rc_max', 'voltage_filter_time_constant_s', 'max_voltage_sample_age_s')
TEXT = ('calibration_id', 'axis_source', 'axis_conversion', 'axis_evidence')


def load_calibration(filename, shadow=False):
    filename = Path(filename)
    document = filename.read_text()
    tree = yaml.compose(document)
    if not isinstance(tree, yaml.MappingNode):
        raise ValueError('Calibration must be a mapping')
    entries = {k.value: v for k, v in tree.value}
    if len(entries) != len(tree.value):
        raise ValueError('Duplicate calibration key')
    m = yaml.safe_load(document)
    if not isinstance(m, dict):
        raise ValueError('Calibration must be a mapping')
    for key, expected in EXACT.items():
        if key == "axis_verified" and m.get("provenance") == "measured_motor_curve_with_nominal_rc_conversion":
            continue
        if type(m.get(key)) is not type(expected) or m[key] != expected:
            raise ValueError('Invalid calibration '+key)
        if type(expected) in (bool, int):
            canonical = str(expected).lower()
            if entries[key].value != canonical:
                raise ValueError('Noncanonical calibration '+key)
    for key in TEXT:
        if not isinstance(m.get(key), str) or not m[key].strip() or m[key] == 'unknown':
            raise ValueError('Missing calibration '+key)
    nominal = m.get('provenance') == 'measured_motor_curve_with_nominal_rc_conversion'
    if nominal:
        if (m.get('calibration_status') != 'provisional' or
                m.get('axis_mapping_basis') != 'source_and_config_assumptions' or
                m.get('selected_target_profile') not in ('static', 'dynamic_nominal') or
                type(m.get('measured_motor_index')) is not int or m.get('measured_motor_index') != 0 or
                entries['measured_motor_index'].value != '0' or 'axis_verified' in m):
            raise ValueError('Invalid provisional nominal mapping metadata')
    else:
        if m.get('axis_verified') is not True or entries['axis_verified'].value != 'true':
            raise ValueError('Invalid calibration axis_verified')
        if m.get('provenance') not in ('measured', 'synthetic') or m.get('calibration_status') not in ('validated','unvalidated'):
            raise ValueError('Invalid provenance/status')
        if not shadow and (m['provenance'] != 'measured' or m['calibration_status'] != 'validated'):
            raise ValueError('Production requires measured, independently validated calibration')
        if not shadow:
            validation=m.get('heldout_metrics',{})
            if validation.get('passed') is not True or validation.get('independent_final_test') is not True:
                raise ValueError('Production requires passed independent final-test evidence')
    if m.get('lower_policy', 'reject') not in ('reject', 'minimum_supported_rc'):
        raise ValueError('Invalid lower_policy')
    for key in NUMBERS:
        value=m.get(key)
        if type(value) not in (int,float) or not math.isfinite(value):
            raise ValueError('Nonfinite/missing calibration '+key)
    if not (0 < m['valid_voltage_min_v'] <= m['reference_voltage_v'] <= m['valid_voltage_max_v'] and
            m['valid_voltage_min_v'] < m['valid_voltage_max_v'] and
            1000 <= m['valid_rc_min'] < m['valid_rc_max'] <= 2000 and
            m['valid_rc_min'] == int(m['valid_rc_min']) and m['valid_rc_max'] == int(m['valid_rc_max']) and
            m['voltage_filter_time_constant_s'] >= 0 and m['max_voltage_sample_age_s'] > 0):
        raise ValueError('Invalid calibration domain/filter')
    table=m.get('table_file')
    if not isinstance(table,str) or not table:
        raise ValueError('Missing table_file')
    with (filename.parent / table).open(newline='') as f:
        rows=list(csv.reader(f))
    if not rows or rows.pop(0)!=['rc_throttle','total_thrust_n_at_vref'] or len(rows)<2 or any(len(r)!=2 for r in rows):
        raise ValueError('Wrong reference CSV schema')
    decimal = r'[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?'
    if any(re.fullmatch(decimal,v.strip()) is None for r in rows for v in r):
        raise ValueError('Expected finite decimal CSV values')
    curve=[tuple(float(v) for v in r) for r in rows]
    for i,(c,t) in enumerate(curve):
        if not math.isfinite(c+t) or c!=int(c) or not m['valid_rc_min']<=c<=m['valid_rc_max'] or t<0:
            raise ValueError('Invalid reference CSV value')
        if i and (c<=curve[i-1][0] or t<curve[i-1][1]):
            raise ValueError('Nonmonotonic/duplicate reference CSV')
    if curve[0][0]!=m['valid_rc_min'] or curve[-1][0]!=m['valid_rc_max'] or curve[-1][1]<=curve[0][1]:
        raise ValueError('CSV must cover exactly the declared RC domain with distinct forces')
    return m,curve


def inverse(curve, thrust):
    if not math.isfinite(thrust) or thrust<curve[0][1] or thrust>curve[-1][1]:
        raise ValueError('Thrust outside reference domain')
    for i,(c,f) in enumerate(curve):
        if f==thrust: return c
        if f>thrust:
            c0,f0=curve[i-1]
            return c0+(thrust-f0)*(c-c0)/(f-f0)
    raise ValueError('Invalid inverse')


def map_thrust(metadata, curve, total_n, voltage):
    """Same total-N compensation and boundaries as the C++ model.

    A zero request is an output-layer idle sentinel, not a measured zero-force point.
    """
    if (not math.isfinite(voltage) or not metadata['valid_voltage_min_v'] <= voltage <= metadata['valid_voltage_max_v'] or
            not math.isfinite(total_n) or total_n < 0):
        raise ValueError('Invalid thrust or voltage domain')
    scale = (metadata['reference_voltage_v'] / voltage) ** metadata['voltage_exponent_p']
    lookup = total_n * scale
    if not math.isfinite(scale) or scale <= 0 or not math.isfinite(lookup):
        raise ValueError('Compensation overflow')
    if total_n == 0:
        return dict(rc_continuous=1000., rc=1000, scale=scale, lookup_n=lookup, lower_saturated=False, idle_requested=True)
    lower = lookup < curve[0][1]
    if lower and metadata.get('lower_policy', 'reject') == 'minimum_supported_rc':
        rc = curve[0][0]
    else:
        rc = inverse(curve, lookup)
    return dict(rc_continuous=rc, rc=math.floor(rc+.5), scale=scale, lookup_n=lookup,
                lower_saturated=lower, idle_requested=False)
