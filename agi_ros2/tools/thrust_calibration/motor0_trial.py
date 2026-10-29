#!/usr/bin/env python3
"""Offline motor[0] trial: measured motor curve -> nominal target raw-RC table.

No ROS, serial, flight-controller writes or automatic hardware operations.
"""
import argparse
import csv
import json
import math
import re
import subprocess
from pathlib import Path
import shutil
import sys

import numpy as np
import yaml

from fit_reference import pava
from log_io import csv_columns, decode_blackbox, digest, inspect_file, read_columns

GF_TO_N = .00980665
OFFSETS = {1: 94.143109, 2: 116.526271}
BASELINES = {1: 147., 2: 196.}
SETTINGS = dict(bin_s=.1, motor_bin_span=4., segment_min_s=1.2,
                trim_front_s=.3, trim_back_s=.2, core_min_s=.7, duration_tolerance_s=.002,
                segmentation='maximal stable bin intervals; longest disjoint cores', bf_max_gap_s=.01, force_max_gap_s=.05, relative_floor_gf=50.)


def write_csv(path, rows, fields=None):
    with Path(path).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def load_run(input_dir, cache, run):
    bfpath, pxpath = input_dir / f'{run}.BFL', input_dir / f'{run}.ulg'
    meta = json.loads((cache / f'run{run}/decode.json').read_text())
    csvpath = cache / f'run{run}/blackbox.csv'
    if meta['source']['sha256'] != digest(bfpath) or meta['returncode'] != 0:
        raise ValueError('Decode cache source mismatch/failure')
    if meta.get('decoded_csv_sha256') != digest(csvpath):
        raise ValueError('Decode CSV hash missing/mismatch; run inspect again')
    if '--raw' in meta['command'] or meta['time_unit'] != 'us' or meta['voltage_unit'] != 'V':
        raise ValueError('Invalid decoder units/predictor mode')
    bf = csv_columns(csvpath)
    px, dropouts = read_columns(pxpath, dict(topic='pressure_force_sensor', instance=0))
    t, ft = bf['time (us)'].astype(float)*1e-6, px['timestamp'].astype(float)*1e-6
    m, v, force = bf['motor[0]'], bf['vbatLatest (V)'], px['sensor2'].astype(float)
    for clock in (t, ft):
        if not np.isfinite(clock).all() or np.any(np.diff(clock) <= 0):
            raise ValueError('Nonfinite/reversed/duplicate timestamps')
    if not all(np.isfinite(x).all() for x in (m, v, force)) or np.any(v <= 0):
        raise ValueError('Invalid motor/voltage/force')
    return dict(t=t, m=m, v=v, ft=ft, force=force, dropouts=dropouts,
                peak_time=float(t[np.argmax(m)]), bf=bf)


def windows(data, run, offset=None):
    cfg = SETTINGS
    offset = OFFSETS[run] if offset is None else offset
    t, m, v = (data[k] for k in ('t', 'm', 'v'))
    # Bin in original BF clock; never interpolate across a gap or pair frames by index.
    ids = np.floor(t/cfg['bin_s']).astype(int)
    bins = []
    for key in np.unique(ids):
        ix = np.flatnonzero(ids == key)
        if np.max(np.diff(t[ix]), initial=0) > cfg['bf_max_gap_s']:
            continue
        bins.append((key, float(np.median(m[ix])), ix[0], ix[-1]))
    # Enumerate maximal stable intervals from every bin boundary. A greedy reset
    # loses a valid plateau suffix when its first settling bin exceeds the span.
    candidates = []
    for first in range(len(bins)):
        group = [bins[first]]
        for b in bins[first+1:]:
            if (b[0] != group[-1][0]+1 or np.ptp([q[1] for q in group]+[b[1]]) > cfg['motor_bin_span'] or
                    t[b[2]]-t[group[-1][3]] > cfg['bf_max_gap_s']):
                break
            group.append(b)
        if t[group[-1][3]]-t[group[0][2]] >= cfg['segment_min_s']-cfg['duration_tolerance_s']:
            candidates.append(group)
    groups, used = [], set()
    for group in sorted(candidates, key=lambda g: t[g[-1][3]]-t[g[0][2]], reverse=True):
        keys = {b[0] for b in group}
        if not used.intersection(keys):
            groups.append(group)
            used.update(keys)
    groups.sort(key=lambda g: g[0][0])
    result = []
    for group in groups:
        start, end = t[group[0][2]], t[group[-1][3]]
        if end-start < cfg['segment_min_s']-cfg['duration_tolerance_s']:
            continue
        a, b = start+cfg['trim_front_s'], end-cfg['trim_back_s']
        bi = (t >= a) & (t <= b)
        pi = (data['ft'] >= a+offset) & (data['ft'] <= b+offset)
        if b-a < cfg['core_min_s']-cfg['duration_tolerance_s'] or pi.sum() < 20:
            continue
        times, pt = t[bi], data['ft'][pi]
        if np.max(np.diff(times)) > cfg['bf_max_gap_s'] or np.max(np.diff(pt)) > cfg['force_max_gap_s']:
            continue
        if any(a+offset <= d['timestamp_us']*1e-6+d['duration_ms']*.001 and
               b+offset >= d['timestamp_us']*1e-6 for d in data['dropouts']):
            continue
        f = data['force'][pi]-BASELINES[run]
        midpoint = (a+b)/2+offset
        half_delta = float(np.mean(f[pt >= midpoint])-np.mean(f[pt < midpoint]))
        result.append(dict(run=run, window=len(result), direction='up' if b < data['peak_time'] else 'down',
                           bf_start_s=float(start), bf_end_s=float(end), core_start_s=float(a), core_end_s=float(b),
                           px4_core_start_s=float(a+offset), px4_core_end_s=float(b+offset), baseline_gf=BASELINES[run],
                           motor_mean=float(np.mean(m[bi])), motor_min=int(min(m[bi])), motor_max=int(max(m[bi])),
                           voltage_mean=float(np.mean(v[bi])), voltage_min=float(min(v[bi])), voltage_max=float(max(v[bi])),
                           force_mean_gf=float(np.mean(f)), force_std_gf=float(np.std(f)), half_delta_gf=half_delta,
                           single_force_n=float(np.mean(f)*GF_TO_N), bf_count=int(bi.sum()), force_count=int(pi.sum())))
    return result


def curve_for(rows, p, vref):
    # Consolidate exact duplicate motor means (e.g. idle split around the BF gap).
    unique = {}
    for row in rows:
        key = row['motor_mean']
        if key not in unique or row['core_end_s']-row['core_start_s'] > unique[key]['core_end_s']-unique[key]['core_start_s']:
            unique[key] = row
    ordered = sorted(unique.values(), key=lambda r: r['motor_mean'])
    x = np.array([r['motor_mean'] for r in ordered])
    before = np.array([r['single_force_n']*(vref/r['voltage_mean'])**p for r in ordered])
    if np.any(np.diff(x) <= 0) or np.any(before < 0):
        raise ValueError('Invalid primary motor curve')
    after = pava(before, np.ones(len(before)))
    return x, before, after


def compare(rows, train, p, vref, baseline_delta=0):
    x, _, y = curve_for(train, p, vref)
    details = []
    low_errors = []
    for row in rows:
        force = row['single_force_n']+baseline_delta*GF_TO_N
        if not x[0] <= row['motor_mean'] <= x[-1]:
            continue
        predicted = float(np.interp(row['motor_mean'], x, y)*(row['voltage_mean']/vref)**p)
        if row['single_force_n'] < SETTINGS['relative_floor_gf']*GF_TO_N:
            low_errors.append(abs(predicted-force))
            continue
        details.append(dict(run=row['run'], window=row['window'], motor=row['motor_mean'], voltage=row['voltage_mean'],
                            measured_single_n=force, predicted_single_n=predicted, relative_error=(predicted-force)/force))
    errors = [abs(d['relative_error']) for d in details]
    return dict(count=len(details), low_force_count=len(low_errors),
                low_force_max_absolute_error_n=max(low_errors) if low_errors else None, mean_absolute_relative_error=float(np.mean(errors)) if errors else None,
                max_absolute_relative_error=max(errors) if errors else None, points=details)


def choose_p(rows, train, vref):
    # Deterministic bounded golden-section minimization, no SciPy dependency.
    def loss(p):
        points = compare(rows, train, p, vref)['points']
        if not points:
            raise ValueError('No repeated points for p selection')
        return float(np.mean([d['relative_error']**2 for d in points]))
    lo, hi = -2., 5.
    ratio = (math.sqrt(5)-1)/2
    for _ in range(90):
        a, b = hi-ratio*(hi-lo), lo+ratio*(hi-lo)
        if loss(a) < loss(b): hi = b
        else: lo = a
    return (lo+hi)/2


def target_motor(rc, target, variant=None):
    """Continuous nominal steady map; firmware integer quantization is an approximation."""
    parameters = target['parameters']
    variant = variant or target['selected_target_profile']
    if variant not in ('static', 'dynamic_nominal'):
        raise ValueError('Unknown selected target')
    required = dict(thr_mid=50, thr_expo=0, thr_hover=50, motor_output_limit=100,
                    throttle_limit_type=0, vbat_sag_compensation=0, thrust_linear=0, rpm_limit=False, feature_3d=False)
    if any(parameters.get(k) != value for k, value in required.items()):
        raise ValueError('This trial mapper requires the declared linear/unlimited target profile')
    low, high, check = [parameters[k] for k in ('rx_range_min', 'rx_range_max', 'mincheck')]
    if not all(math.isfinite(x) for x in (low, high, check)) or not low < high or not 1000 <= check < 2000:
        raise ValueError('Invalid RX range/mincheck')
    r = 1000+(np.asarray(rc)-low)*1000/(high-low)
    u = np.clip((r-check)/(2000-check), 0, 1)
    if parameters.get('nominal_dynamic_idle_increment') != 0:
        raise ValueError('Only zero nominal dynamic idle increment is implemented')
    if variant == 'static':
        idle = parameters['motor_idle']*.0001
        if not math.isfinite(idle) or not 0 <= idle < 1:
            raise ValueError('Invalid motor_idle')
        endpoint = 48+1999*idle
        return endpoint+(2047-endpoint)*u
    return 48+1999*np.maximum(u,.01)


def alignment_checks(data, offset):
    """Compare local positive/negative response derivatives at separated edges.

    This checks response alignment, not physical timestamp synchronization.
    """
    t, m = data['t'], data['m']
    grid = np.arange(t[0]+.15, t[-1]-.15, .02)
    for index in np.flatnonzero(np.diff(t) > SETTINGS['bf_max_gap_s']):
        grid = grid[(grid < t[index]-.4) | (grid > t[index+1]+.4)]
    dm = np.interp(grid+.06, t, m)-np.interp(grid-.06, t, m)
    chosen = []
    for ix in np.argsort(abs(dm))[::-1]:
        if abs(dm[ix]) < 35: break
        edge = float(grid[ix])
        if all(abs(edge-other) > 2 for other in chosen): chosen.append(edge)
    results = []
    for edge in sorted(chosen):
        q = np.arange(edge-.3, edge+.3,.01)
        derivative = np.interp(q+.05,t,m)-np.interp(q-.05,t,m)
        candidates = np.arange(-.25,.251,.005)
        scores = []
        for shift in candidates:
            f = np.interp(q+offset+shift+.05,data['ft'],data['force'])-np.interp(q+offset+shift-.05,data['ft'],data['force'])
            scores.append(float(np.corrcoef(derivative,f)[0,1]) if np.std(f) and np.std(derivative) else -1.)
        best = int(np.argmax(scores))
        results.append(dict(bf_edge_s=edge, residual_shift_s=float(candidates[best]), correlation=scores[best]))
    return results


def inspect(args):
    args.cache_dir.mkdir(parents=True, exist_ok=True)
    for run in (1, 2):
        for ext in ('BFL', 'ulg'):
            info = inspect_file(args.input_dir/f'{run}.{ext}')
            (args.cache_dir/f'inspect_{run}_{ext}.json').write_text(json.dumps(info, indent=2))
        destination = args.cache_dir/f'run{run}'
        if not destination.exists():
            decode_blackbox(args.input_dir/f'{run}.BFL', destination, str(args.decoder.resolve()), 1)
        meta_path = destination/'decode.json'
        meta = json.loads(meta_path.read_text())
        if meta['source']['sha256'] != digest(args.input_dir/f'{run}.BFL'):
            raise ValueError('Cache belongs to another source')
        # Bind the recovered CSV to its raw-file/decoder provenance.
        existing = meta.get('decoded_csv_sha256')
        actual = digest(destination/'blackbox.csv')
        if existing and existing != actual:
            raise ValueError('Cached CSV modified')
        failures = re.search(r'(\d+) frames failed to decode', meta['frame_statistics_and_warnings'])
        if meta['returncode'] or (failures and int(failures[1]) > 0):
            raise ValueError('Decoder reports damaged frames')
        meta['decoder_quality_reviewed'] = True
        meta['quality_note'] = 'run1 29 unreadable loop iterations: observed 30ms gap excluded; 75% missing reflects 4kHz/1kHz decimation'
        commit = subprocess.run(['git','-C',str(args.decoder.resolve().parent),'rev-parse','HEAD'],capture_output=True,text=True)
        meta['decoder_source_commit'] = commit.stdout.strip() if commit.returncode == 0 else 'unavailable; executable hash recorded'
        meta['decoded_csv_sha256'] = actual
        meta_path.write_text(json.dumps(meta, indent=2))
        print(f'run{run}: '+meta['frame_statistics_and_warnings'])


def fit(args):
    args.output_dir.mkdir(parents=True, exist_ok=True)
    target = yaml.safe_load(args.target.read_text())
    data = {run: load_run(args.input_dir, args.cache_dir, run) for run in (1, 2)}
    rows = [r for run in (1, 2) for r in windows(data[run], run)]
    train = [r for r in rows if r['run'] == 1 and r['direction'] == 'up']
    selection = [r for r in rows if r['run'] == 1 and r['direction'] == 'down']
    auxiliary = [r for r in rows if r['run'] == 2 and r['direction'] == 'up']
    for r in rows:
        r['role'] = 'primary_curve' if r in train else 'p_selection' if r in selection else 'auxiliary' if r in auxiliary else 'excluded_tail_baseline_uncertain'
    pfit = choose_p(selection, train, args.vref)
    p = pfit if args.exponent is None else args.exponent
    if not math.isfinite(p) or not 0 < args.vref:
        raise ValueError('Invalid p/Vref')
    x, before, y = curve_for(train, p, args.vref)
    write_csv(args.output_dir/'steady_windows.csv', rows)
    write_csv(args.output_dir/'motor_reference_thrust.csv', [dict(motor_command=float(a), single_thrust_n_at_vref=float(b)) for a,b in zip(x,y)])
    write_csv(args.output_dir/'motor_fit_points.csv', [dict(motor_command=float(a), before_n=float(b), after_n=float(c), adjustment_n=float(c-b), used_window_count=1, candidate_window_count=sum(r['motor_mean']==a for r in train)) for a,b,c in zip(x,before,y)])
    candidates = {str(q): compare(selection, train, q, args.vref) for q in (0., 1., 2., pfit)}
    report = dict(settings=SETTINGS, vref=args.vref, p_selected=p, p_fitted=pfit, p_objective='run1 down mean squared relative error; selection not independent test',
                  candidates=candidates, primary_windows=len(train), primary_unique_motor_points=len(x), duplicate_motor_policy='longest core at identical motor mean; all windows retained', auxiliary=compare(auxiliary,train,p,args.vref),
                  auxiliary_baseline142=compare(auxiliary,train,p,args.vref,54.),
                  run2_selected_p=None, run2_p_note='No eligible down repeat above 50gf after core extraction; tail baseline uncertain, not fitted',
                  alignment={str(n):dict(a=1., b=OFFSETS[n], checks=alignment_checks(data[n],OFFSETS[n]),
                      meaning='fixed response alignment includes force sensor delay; not hardware clock sync') for n in (1,2)},
                  analysis_code_sha256={name:digest(Path(__file__).with_name(name)) for name in ('motor0_trial.py','log_io.py','fit_reference.py')},
                  sources={f'{n}.{ext}':digest(args.input_dir/f'{n}.{ext}') for n in (1,2) for ext in ('BFL','ulg')},
                  observed_voltage_range={str(n):[float(min(data[n]['v'])),float(max(data[n]['v']))] for n in (1,2)},
                  fitted_steady_voltage_range=[min(r['voltage_min'] for r in train+selection), max(r['voltage_max'] for r in train+selection)],
                  baselines={str(n):dict(used_gf=BASELINES[n], pre_quiet_median_gf=float(np.median(data[n]['force'][(data[n]['ft']>data[n]['t'][0]+OFFSETS[n]-4)&(data[n]['ft']<data[n]['t'][0]+OFFSETS[n]-1)])),
                      post_quiet_median_gf=float(np.median(data[n]['force'][(data[n]['ft']>data[n]['t'][-1]+OFFSETS[n]+2)&(data[n]['ft']<data[n]['t'][-1]+OFFSETS[n]+5)]))) for n in (1,2)},
                  flight_validation_status='not_performed', maximum_monotonic_adjustment_n=float(max(abs(y-before))))
    report['alignment_sensitivity'] = []
    for shift in (-.08, .08):
        shifted = []
        for row in rows:
            d = data[row['run']]
            ix = (d['ft'] >= row['px4_core_start_s']+shift) & (d['ft'] <= row['px4_core_end_s']+shift)
            shifted.append({**row, 'single_force_n':float(np.mean(d['force'][ix]-row['baseline_gf'])*GF_TO_N)})
        tr = [r for r in shifted if r['role']=='primary_curve']
        sel = [r for r in shifted if r['role']=='p_selection']
        report['alignment_sensitivity'].append(dict(offset_perturbation_s=shift, p_fitted=choose_p(sel,tr,args.vref)))
    # Finite engineering allowance, not a claim of fully validated steady coverage.
    report['allowed_model_voltage_range'] = [21.,24.66]
    report['allowed_voltage_basis'] = 'finite trial approximation within overall observations; 20.82V transient excluded; sparse steady coverage'
    (args.output_dir/'fit_report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    shutil.copyfile(args.target,args.output_dir/'target_fc_mapping.yaml') if args.target.resolve()!= (args.output_dir/'target_fc_mapping.yaml').resolve() else None
    for n in (1,2):
        shutil.copyfile(args.cache_dir/f'run{n}/decode.json',args.output_dir/f'decode_run{n}.json')
    export(args.output_dir, args.target, args.hardware_profile)
    print(json.dumps({k:report[k] for k in ('p_selected','primary_windows','auxiliary','auxiliary_baseline142')},indent=2))


def export(output, target_path, hardware_profile=None):
    target = yaml.safe_load(target_path.read_text())
    report = json.loads((output/'fit_report.json').read_text())
    physical = np.genfromtxt(output/'motor_reference_thrust.csv',delimiter=',',names=True)
    x, y = physical['motor_command'], physical['single_thrust_n_at_vref']
    curves = {}
    for variant in ('static','dynamic_nominal'):
        rc = np.arange(1000,2001)
        m = target_motor(rc,target,variant)
        supported = (m >= x[0]) & (m <= x[-1])
        curve = [dict(rc_throttle=int(c),total_thrust_n_at_vref=float(4*np.interp(mm,x,y))) for c,mm in zip(rc[supported],m[supported])]
        if len(curve)<2: raise ValueError('Target has no supported integer RC domain')
        write_csv(output/f'reference_thrust_{variant}.csv',curve)
        curves[variant] = curve
    selected = target['selected_target_profile']
    if (selected == 'static') != (target['parameters']['dyn_idle_min_rpm'] == 0):
        raise ValueError('Selected profile disagrees with target dyn_idle_min_rpm')
    if target_path.resolve() != (output/'target_fc_mapping.yaml').resolve():
        shutil.copyfile(target_path, output/'target_fc_mapping.yaml')
    curve = curves[selected]
    write_csv(output/'reference_thrust.csv',curve)
    meta = dict(schema_version=1,model='reference_table',command_axis='msp_raw_rc_throttle',
                axis_mapping_basis='source_and_config_assumptions', thrust_unit='N',thrust_scope='total_vehicle',
                calibration_voltage_source='blackbox_vbatLatest',runtime_voltage_source='msp130_display_filtered',
                voltage_compatibility='steady_state',no_ir_correction=True,tested_motor_count=1,measured_motor_count=1,
                motor_count=4,measured_motor_index=0,aggregation='single_motor_times_four',
                calibration_id='motor0_20261029_run1_trial',axis_source='motor[0] measured; target nominal MSP200 map',
                axis_conversion='Tref(C)=4*fref(G_target(C)); nominal zero PID torque',
                axis_evidence='target_fc_mapping.yaml; local Betaflight source; documented defaults and historical profile assumptions',
                provenance='measured_motor_curve_with_nominal_rc_conversion',calibration_status='provisional',
                flight_validation_status='not_performed',selected_target_profile=selected,
                reference_voltage_v=report['vref'],voltage_exponent_p=report['p_selected'],table_file='reference_thrust.csv',
                valid_voltage_min_v=report['allowed_model_voltage_range'][0],valid_voltage_max_v=report['allowed_model_voltage_range'][1],
                valid_rc_min=curve[0]['rc_throttle'],valid_rc_max=curve[-1]['rc_throttle'],
                valid_motor_range=[float(x[0]),float(x[-1])],voltage_filter_time_constant_s=0.,
                max_voltage_sample_age_s=(yaml.safe_load(hardware_profile.read_text())['evidence']['battery_timeout'] if hardware_profile else 1.5),
                max_voltage_sample_age_source='hardware profile evidence.battery_timeout; request time remains age origin',
                lower_policy='minimum_supported_rc',zero_policy='output_layer_idle',upper_policy='reject',
                force_topic='pressure_force_sensor',force_instance=0,force_field='sensor2',force_unit='gf',
                gf_to_n=GF_TO_N,baseline_gf={'run1':147,'run2':196,'run2_tail_sensitivity':142},
                force_unit_evidence='PX4 09ca172b71887b1db6435b90275f39fda5fbb258 thrust_feedback_control_bemt.cpp kGramForceToNewton',
                firmware_refs=target['firmware_refs'],target_mapping=target,
                data_selection='run1 up curve; run1 down p selection; run2 up auxiliary; run2 down excluded from main fit',
                sources=report['sources'],alignment=report['alignment'],analysis_code_sha256=report['analysis_code_sha256'],
                primary_windows=report['primary_windows'],primary_unique_motor_points=report['primary_unique_motor_points'],
                alignment_sensitivity=report['alignment_sensitivity'],
                observed_voltage_range=report['observed_voltage_range'],fitted_steady_voltage_range=report['fitted_steady_voltage_range'],
                allowed_model_voltage_range=report['allowed_model_voltage_range'],allowed_voltage_basis=report['allowed_voltage_basis'],
                candidate_metrics=report['candidates'],auxiliary=report['auxiliary'],auxiliary_baseline142=report['auxiliary_baseline142'],
                unsupported='motor above stable measured endpoint; nonzero PID allocation; active dynamic idle increment; rapid voltage transient; four-motor variability')
    (output/'calibration.yaml').write_text(yaml.safe_dump(meta,sort_keys=False,allow_unicode=True))
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'launch'))
    from reference_calibration import load_calibration
    load_calibration(output/'calibration.yaml',shadow=False)
    if hardware_profile:
        source = hardware_profile.read_text()
        destination = hardware_profile.with_name('hardware_reference_trial.yaml')
        relative = __import__('os').path.relpath(output/'calibration.yaml',destination.parent)
        source = source.replace('  thrust_model: quadratic','  thrust_model: reference_table\n  thrust_calibration: '+relative,1)
        # Preserve all pre-existing authority, shadow and controller settings.
        destination.write_text('# Provisional motor[0] measured curve with nominal target RC conversion.\n'+source)
        report['mpc_limit_per_motor_n'] = yaml.safe_load(source)['pilot']['quadrotor']['thrust_max']
    report['selected_target_profile']=selected
    report['variant_rc_ranges']={name:[c[0]['rc_throttle'],c[-1]['rc_throttle']] for name,c in curves.items()}
    report['rc_range']=[curve[0]['rc_throttle'],curve[-1]['rc_throttle']]
    report['total_thrust_range_vref_n']=[curve[0]['total_thrust_n_at_vref'],curve[-1]['total_thrust_n_at_vref']]
    report['maximum_total_at_min_voltage_n']=curve[-1]['total_thrust_n_at_vref']*(meta['valid_voltage_min_v']/meta['reference_voltage_v'])**meta['voltage_exponent_p']
    (output/'fit_report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    # Refresh documentation and plots after a target-only export as well.
    with (output/'steady_windows.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        for key in ('run', 'motor_mean', 'single_force_n', 'voltage_mean'):
            row[key] = float(row[key])
    plot(output, rows, x, y, y, report)
    write_report(output, meta, report)


def write_report(output, meta, report):
    selection = report['candidates'][str(report['p_fitted'])]
    auxiliary = report['auxiliary']
    baseline = report['auxiliary_baseline142']
    rows = [
        '# motor[0] 实测工程初版（2026-10-29）', '',
        '状态：provisional；飞行验证：not_performed。原日志只读；单电机乘4是假设，目标RC域是名义转换。', '',
        f"Vref={meta['reference_voltage_v']} V；p={meta['voltage_exponent_p']:.9f}（可配置，不表示同等物理精度）。",
        f"第一组上扫保留 {report['primary_windows']} 个稳态窗口，形成 {report['primary_unique_motor_points']} 个不同motor点。",
        '相同motor均值的重复窗口选最长核心；缺口两侧窗口均保留在steady_windows.csv。',
        f"第一组下扫用于选p：{selection['count']} 点，平均/最大相对差 {100*selection['mean_absolute_relative_error']:.3f}% / {100*selection['max_absolute_relative_error']:.3f}%。这不是独立验收。",
        f"第二组上扫辅助：{auxiliary['count']} 点，平均/最大相对差 {100*auxiliary['mean_absolute_relative_error']:.3f}% / {100*auxiliary['max_absolute_relative_error']:.3f}%。",
        f"同样辅助点改去皮142gf：平均/最大相对差 {100*baseline['mean_absolute_relative_error']:.3f}% / {100*baseline['max_absolute_relative_error']:.3f}%。", '',
        '第一组去皮147gf；第二组前/中段196gf；第二组下降尾段排除主拟合，不拟合人为零漂。',
        '安静段复算中位数：run1开始147/结束146gf，run2开始196/结束142gf。',
        '每0.1s分箱，motor中位数跨度≤4，原段≥1.2s（容许两个1kHz采样间隔的边界误差）；去前0.3/后0.2s。',
        '从每个bin边界寻找最长稳定区间、优先选互不重叠长窗口，避免起始settling bin导致整段后缀丢失。',
        '统计核心内原生force样本均值/标准差/半段差；不把100Hz消息当成100Hz独立ADC样本。',
        '第一组约30ms缺口所在bin排除，窗口不跨缺口；ULog早期dropout不落在所选窗口。',
        '用motor稳态选窗，未按推力误差挑点。与提示词探索数值的差别来自分箱/窗口边界及最小二乘选p目标。', '',
        'p候选（第一组下扫，净力≥50gf）：', '',
        '| p | 平均相对差 | 最大相对差 |', '|---|---:|---:|',
    ]
    for p, result in report['candidates'].items():
        rows.append(f"| {float(p):.6f} | {100*result['mean_absolute_relative_error']:.3f}% | {100*result['max_absolute_relative_error']:.3f}% |")
    rows += ['',
        f"motor有效域 {meta['valid_motor_range']}；所选 {meta['selected_target_profile']}；RC域 {report['rc_range']}。",
        f"static对照RC域 {report['variant_rc_ranges']['static']}，dynamic_nominal域 {report['variant_rc_ranges']['dynamic_nominal']}。",
        f"{meta['reference_voltage_v']}V整机推力域 {report['total_thrust_range_vref_n']} N；仅在motor实测支持域内按整数RC采样。",
        'm=530时，static约RC1237.1，dynamic_nominal约RC1279.1；未将旧RC→motor回归写回模型。',
        f"目标dyn_idle_min_rpm={meta['target_mapping']['parameters']['dyn_idle_min_rpm']}；参数值及来源见target_fc_mapping.yaml；历史profile和默认参数均不是当前FC回读。",
        '动态怠速delta≈0、零PID力矩、线性油门及无FC电压补偿等条件见target_fc_mapping.yaml；没有回读或改写飞控。', '',
        f"原日志电压域：{report['observed_voltage_range']}；主拟合稳态电压域：{report['fitted_steady_voltage_range']} V。",
        f"允许模型电压域：{report['allowed_model_voltage_range']} V，是有限工程近似域；低端没有充分稳态验证，20.82V瞬态不纳入。",
        f"Tmax(V)=Tmax_ref*(V/Vref)^p；{meta['valid_voltage_min_v']}V时 {report['maximum_total_at_min_voltage_n']:.6f} N。",
        f"当前MPC每电机等效上限 {report.get('mpc_limit_per_motor_n', 'unknown')} N（整机上限为该值乘4），未改控制器限幅。",
        '该限幅检查仅是名义零力矩总推力裕量，不保证实际PID分配/动态怠速下的电机裕量。', '',
        '对齐使用a=1和给定候选b；早中晚阶跃的局部残差：run1约-10..60ms，run2约5..45ms。',
        '这是响应对齐，不是硬件时钟同步。保留固定偏移，300ms前裁剪覆盖这些残差。',
        f"偏移±80ms敏感性：{report['alignment_sensitivity']}。", '',
        '运行链：Tlookup=Tdesired*(Vref/V)^p → 单调表反解；仅导出RC表时乘4，运行时不乘质量/4，不缩放body_rates。',
        f"额外电压滤波τ=0，max_age={meta['max_voltage_sample_age_s']}s沿用hardware evidence.battery_timeout，按真实MSP报告身份/原请求时间计龄。",
        '0N返回既有输出层idle语义（不承诺实测0N）；正低端返回最低支持RC并置lower_saturated；超上限/电压越域/过期拒绝。',
        'Python和C++均允许本provisional实测motor名义转换文件，thrust_calibrated仍为false，thrust_mapping_ready仅表示成功加载。',
        '原授权/解锁/健康/接管/shadow门禁保留。未执行实机输出、写参、部署或飞行试验。', '',
        '![参考曲线及RC转换](curves.png)', '',
        '复现与选用命令见 ../../tools/thrust_calibration/MOTOR0_TRIAL_README.zh-CN.md。',
    ]
    (output/'REPORT.zh-CN.md').write_text('\n'.join(rows)+'\n')


def plot(output, rows, x, before, y, report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1,2,figsize=(12,4))
    for run in (1,2):
        rr=[r for r in rows if r['run']==run and r['direction']=='up']
        ax[0].scatter([r['motor_mean'] for r in rr],[r['single_force_n']*(report['vref']/r['voltage_mean'])**report['p_selected'] for r in rr],label=f'run{run} up')
    ax[0].plot(x,y,label=f"run1 reference ({report['vref']:g}V)")
    ax[0].set(xlabel='motor[0] internal command',ylabel='Single-motor thrust N'); ax[0].legend()
    for variant in ('static','dynamic_nominal'):
        c=np.genfromtxt(output/f'reference_thrust_{variant}.csv',delimiter=',',names=True)
        ax[1].plot(c['rc_throttle'],c['total_thrust_n_at_vref'],label=variant)
    ax[1].set(xlabel='Nominal raw MSP RC',ylabel=f"Total thrust N at {report['vref']:g}V"); ax[1].legend()
    fig.suptitle('Provisional engineering model; no flight validation')
    fig.tight_layout(); fig.savefig(output/'curves.png',dpi=150); plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    for name in ('inspect','fit'):
        p=sub.add_parser(name)
        p.add_argument('--input-dir',type=Path,required=True)
        p.add_argument('--cache-dir',type=Path,required=True)
        if name=='inspect': p.add_argument('--decoder',type=Path,required=True)
        else:
            p.add_argument('--output-dir',type=Path,required=True); p.add_argument('--target',type=Path,required=True)
            p.add_argument('--hardware-profile',type=Path); p.add_argument('--vref',type=float,default=24.)
            p.add_argument('--exponent',type=float)
    p=sub.add_parser('export'); p.add_argument('--output-dir',type=Path,required=True); p.add_argument('--target',type=Path,required=True)
    p.add_argument('--hardware-profile',type=Path)
    args=parser.parse_args()
    if args.command=='inspect': inspect(args)
    elif args.command=='fit': fit(args)
    else: export(args.output_dir,args.target,args.hardware_profile)


if __name__=='__main__': main()
