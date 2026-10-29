#!/usr/bin/env python3
"""File-only pipeline: inspect/decode/align/extract/fit/validate/report."""
import argparse
import csv
import hashlib
import html
import json
from pathlib import Path
import sys
import yaml
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'launch'))
from reference_calibration import EXACT, load_calibration
from log_io import VERSION, inspect_file, decode_blackbox, digest
from inspection import load_sessions, inspect_pair
from alignment import fit_alignment, mapped
from steady_windows import extract, load_pair
from fit_reference import fit, validate


def save_json(path,data):
    path.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')


def save_csv(path,rows):
    if not rows: raise ValueError('No rows to write')
    keys=list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w',newline='') as f:
        writer=csv.DictWriter(f,keys); writer.writeheader(); writer.writerows(rows)


def provenance(sessions,config):
    code=list(Path(__file__).parent.glob('*.py'))+[Path(__file__).resolve().parents[2]/'launch/reference_calibration.py']
    data=dict(tool=VERSION,implementation={p.name:digest(p) for p in sorted(code)},sessions=sessions,config=config)
    return hashlib.sha256(json.dumps(data,sort_keys=True).encode()).hexdigest()


def check_artifact(out,name,identity):
    data=json.loads((out/name).read_text())
    if data['input_identity']!=identity: raise ValueError('Inputs/configuration changed; rerun preceding stages')
    return data


def alignment_plot(out,s,cfg,a,windows=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    d=load_pair(s,cfg)
    fig,axes=plt.subplots(3,1,figsize=(12,8))
    axes[0].plot(d['tb'],d['c'],label='BF recovered C'); axes[0].set_ylabel('raw RC')
    twin=axes[0].twinx(); twin.plot(d['tp'],d['f'],color='tab:red',alpha=.5,label='PX4 original clock'); twin.set_ylabel('single motor N')
    axes[0].set_title('Before alignment: independent original clocks')
    axes[1].plot(mapped(d['tb'],a),d['c'],label='C on PX4 clock'); axes[1].set_ylabel('raw RC')
    twin=axes[1].twinx(); twin.plot(d['tp'],d['f'],color='tab:red',alpha=.5); twin.set_ylabel('single motor N')
    for i in range(4): axes[2].plot(mapped(d['tb'],a),d['motors'][:,i],label=f'motor[{i}]')
    for w in windows or []:
        if w.get('accepted'):
            for ax in axes[1:]: ax.axvspan(w['start_px4_s'],w['end_px4_s'],color='green',alpha=.12)
    axes[2].legend(); axes[2].set_xlabel('PX4 seconds (effective alignment includes response delay)')
    fig.tight_layout(); fig.savefig(out/(s['run_id']+'_alignment.png')); plt.close(fig)


def calibration(out,model,config,sessions,validation=None):
    curve=model['curve']
    save_csv(out/'reference_thrust.csv',[dict(rc_throttle=c,total_thrust_n_at_vref=f) for c,f in curve])
    save_csv(out/'reference_thrust_single_motor.csv',[dict(rc_throttle=c,single_motor_n_at_vref=f/4.) for c,f in curve])
    m={**EXACT,**{k:model[k] for k in ('reference_voltage_v','voltage_exponent_p','valid_voltage_min_v','valid_voltage_max_v','provenance')},
       'calibration_id':model['input_identity'], 'table_file':'reference_thrust.csv','valid_rc_min':int(curve[0][0]),'valid_rc_max':int(curve[-1][0]),
       'axis_source':config['bf']['axis']['field'],'axis_conversion':config['bf']['axis']['semantics'],
       'axis_evidence':config['bf']['axis']['evidence'], 'axis_configuration':config['bf']['axis'],
       'voltage_filter_time_constant_s':config['runtime']['voltage_filter_time_constant_s'],
       'max_voltage_sample_age_s':config['runtime']['max_voltage_sample_age_s'],
       'calibration_status':'validated' if validation and validation['passed'] and model['provenance']=='measured' else 'unvalidated',
       'flight_validation_status':'not_performed','force_configuration':config['px4'],
       'source_sessions':sessions,'train_blocks':model['train_blocks'],'validation_blocks':model['validation_blocks'],
       'heldout_metrics':validation,'uncertainty':{'p_block_ci95':model['p_block_ci95']},
       'assumptions':['single motor times four; motor equality untested','steady-state BF unfiltered / MSP displayFiltered compatibility',
                      'effective alignment includes unknown response delay'],
       'hardware_firmware_metadata':config.get('hardware_firmware_metadata','unknown')}
    (out/'calibration.yaml').write_text(yaml.safe_dump(m,sort_keys=False))
    load_calibration(out/'calibration.yaml',shadow=True)


def model_plot(out,rows,model,validation):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,9))
    for split in ('train','validation','test'):
        selected=[r for r in rows if r['accepted'] and r['split']==split]
        c=np.array([r['rc_mean'] for r in selected]); f=np.array([r['force_mean'] for r in selected]); v=np.array([r['voltage_mean'] for r in selected])
        axes[0,0].scatter(c,f,s=8,label=split)
        axes[0,1].scatter(c,4*f*(model['reference_voltage_v']/v)**model['voltage_exponent_p'],s=8,label=split)
    axes[0,0].set(xlabel='recovered MSP raw RC',ylabel='single motor measured N')
    axes[0,1].plot(*np.array(model['raw_curve']).T,'--',label='before PAVA')
    axes[0,1].plot(*np.array(model['curve']).T,label='total reference table')
    axes[0,1].set(xlabel='MSP raw RC',ylabel='total N at reference voltage')
    labels=list(model['candidates']); errors=[model['candidates'][k]['validation']['rmse_single_n'] for k in labels]
    axes[1,0].bar(labels,[e if e is not None else np.nan for e in errors]); axes[1,0].set_ylabel('selection validation RMSE, single N')
    points=validation['heldout']['points']
    if points:
        sc=axes[1,1].scatter([p['voltage'] for p in points],[p['error_n'] for p in points],c=[p['rc'] for p in points])
        fig.colorbar(sc,ax=axes[1,1],label='raw RC')
    axes[1,1].set(xlabel='voltage V',ylabel='final test prediction minus measurement N')
    for ax in axes[0]: ax.legend()
    fig.suptitle(model['provenance']+' — offline model only; physical compensation/flight not performed')
    fig.tight_layout(); fig.savefig(out/'model_validation.png'); plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=['inspect','decode','align','extract','fit','validate','report'])
    p.add_argument('--sessions'); p.add_argument('--config'); p.add_argument('--out',required=True)
    p.add_argument('--file'); p.add_argument('--decoder'); p.add_argument('--log-index',type=int)
    args=p.parse_args(); out=Path(args.out).resolve()
    if args.command=='decode':
        if not args.file or not args.decoder or not args.log_index: p.error('decode needs --file --decoder --log-index')
        decode_blackbox(args.file,out,str(Path(args.decoder).resolve()),args.log_index); return
    out.mkdir(parents=True,exist_ok=True)
    if args.command=='inspect' and args.file:
        if Path(args.file).resolve().is_relative_to(out): raise ValueError('Output directory must not contain inputs')
        save_json(out/'inspection.json',inspect_file(args.file)); return
    if not args.sessions or not args.config: p.error('paired analysis requires --sessions and --config')
    sessions=load_sessions(args.sessions); cfg=yaml.safe_load(Path(args.config).read_text())
    inputs=[args.sessions,args.config]+[s[k]['file'] for s in sessions for k in ('bf','px4')]+[s['configuration_file'] for s in sessions]
    if any(Path(path).resolve().is_relative_to(out) for path in inputs):
        raise ValueError('Output directory must not contain input files')
    identity=provenance(sessions,cfg)
    if args.command=='inspect':
        reports=[inspect_pair(s,cfg) for s in sessions]
        save_json(out/'inspection.json',dict(input_identity=identity,reports=reports)); return
    inspection=check_artifact(out,'inspection.json',identity)
    if any(r['quality']!='ready_for_alignment' for r in inspection['reports']):
        raise ValueError('Inspect has unresolved fields/units/axis; no deployment-axis products permitted')
    if args.command=='align':
        alignments={}
        for s in sessions:
            a=fit_alignment(s.get('anchors',[]),cfg['alignment'],s['bf']['interval_s'])
            d=load_pair(s,cfg)
            # Check each claimed anchor against actual signal direction on both native clocks.
            radius=cfg['alignment']['edge_check_radius_s']
            for r in s['anchors']:
                if not s['px4']['interval_s'][0] <= r['px4_s'] <= s['px4']['interval_s'][1]:
                    raise ValueError('PX4 anchor outside paired segment')
                for t,y,k,threshold in [(d['tb'],d['c'],'bf_s',cfg['alignment']['edge_min_rc']),
                                        (d['tp'],d['f'],'px4_s',cfg['alignment']['edge_min_force_n'])]:
                    left=y[(t>=r[k]-radius)&(t<r[k])]; right=y[(t>r[k])&(t<=r[k]+radius)]
                    if not len(left) or not len(right) or (np.mean(right)-np.mean(left))*r['direction']<threshold:
                        raise ValueError('Anchor lacks observed common edge direction/amplitude')
            a.update(sources={'bf':s['bf'],'px4':s['px4']},time_fields={k:{'field':cfg[k]['timestamp_field'],'unit':cfg[k]['timestamp_unit'],'meaning':cfg[k]['timestamp_meaning']} for k in ('bf','px4')})
            a['alignment_id']=hashlib.sha256(json.dumps(a,sort_keys=True).encode()).hexdigest()
            alignments[s['run_id']]=a; alignment_plot(out,s,cfg,a)
        result=dict(input_identity=identity,alignments=alignments)
        save_json(out/'alignment.json',result); (out/'alignment.yaml').write_text(yaml.safe_dump(result,sort_keys=False)); return
    a=check_artifact(out,'alignment.json',identity)['alignments']
    if args.command=='extract':
        rows=[]; tares={}
        for s in sessions:
            r,t=extract(s,cfg,a[s['run_id']]); rows.extend(r); tares[s['run_id']]=t
            alignment_plot(out,s,cfg,a[s['run_id']],r)
        save_json(out/'windows.json',dict(input_identity=identity,rows=rows,tares=tares))
        save_csv(out/'steady_windows.csv',rows); return
    windows=check_artifact(out,'windows.json',identity)
    if args.command=='fit':
        m=fit(windows['rows'],cfg['fit']); m['input_identity']=identity
        save_json(out/'model.json',m); calibration(out,m,cfg,sessions); return
    m=check_artifact(out,'model.json',identity)
    if args.command=='validate':
        result=validate(windows['rows'],m,cfg['fit']); result['input_identity']=identity
        # Recorded zero drift is a quality gate; do not remove it by detrending.
        result['tare_drift_failures']=[run for run,ts in windows['tares'].items()
                                     if len(ts)<2 or abs(ts[-1]['mean']-ts[0]['mean'])>cfg['steady']['max_tare_drift_n']]
        hardware=cfg.get('hardware_firmware_metadata',{})
        result['deployment_evidence_gaps']=[]
        if m['provenance']=='measured':
            for key,expected in [('configuration_match','verified'),('vbat_sag_compensation_disabled',True),('voltage_compatibility_verified',True)]:
                if hardware.get(key)!=expected: result['deployment_evidence_gaps'].append(key)
        result['passed'] &= not result['tare_drift_failures'] and not result['deployment_evidence_gaps']
        save_json(out/'validation.json',result); calibration(out,m,cfg,sessions,result); return
    validation=check_artifact(out,'validation.json',identity)
    model_plot(out,windows['rows'],m,validation)
    pictures='<img width="100%" src="model_validation.png">'+''.join(f'<figure><img width="100%" src="{html.escape(s["run_id"])}_alignment.png"><figcaption>{html.escape(s["run_id"])}</figcaption></figure>' for s in sessions)
    report=dict(tool=VERSION,provenance=m['provenance'],inspection=inspection,model=m,validation=validation,
                limitations=['No physical compensation or flight validation','Single-motor x4 assumption','Effective alignment, not hardware synchronization'],
                accepted_windows=sum(r['accepted'] for r in windows['rows']),rejected_windows=sum(not r['accepted'] for r in windows['rows']))
    (out/'report.html').write_text('<!doctype html><meta charset="utf-8"><title>Thrust calibration audit</title><h1>Offline thrust calibration</h1>'+pictures+'<pre>'+html.escape(json.dumps(report,indent=2))+'</pre>')


if __name__=='__main__':
    try: main()
    except (ValueError,KeyError,TypeError,ImportError,OSError) as e:
        print(f'ERROR: {e}',file=sys.stderr); sys.exit(2)
