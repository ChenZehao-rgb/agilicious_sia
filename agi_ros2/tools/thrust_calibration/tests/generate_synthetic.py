"""Reproducible SYNTHETIC fixture; no hardware or real calibration claims."""
from pathlib import Path
import csv
import sys
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]


def write_csv(path,cols):
    with path.open('w',newline='') as f:
        w=csv.writer(f); w.writerow(cols); w.writerows(zip(*cols.values()))


def generate(out):
    out=Path(out); out.mkdir(parents=True,exist_ok=True)
    cfg=yaml.safe_load((ROOT/'analysis_config.example.yaml').read_text())
    cfg['bf'].update(timestamp_field='time',timestamp_unit='us',timestamp_meaning='synthetic FC clock',
                     voltage_unit='V',voltage_evidence='synthetic V',decoder_quality_reviewed=True)
    cfg['bf']['axis'].update(debug_mode='RX_SIGNAL_LOSS',decode_scale=1,axis_verified=True,evidence='synthetic verified RX range',
                             firmware_match='verified',same_msp_rx_path_verified=True,rx_smoothing='none',rx_min=980,rx_max=2020,
                             rcdata_clip_min=750,rcdata_clip_max=2250,quantization_rcdata=1,
                             valid_when={'rx_ok':1,'failsafe':0})
    cfg['px4'].update(timestamp_field='timestamp',timestamp_unit='us',timestamp_meaning='synthetic publication clock',
                       force_field='load',unit='N',sign=1,offset=.3,zero_is_no_thrust=True,
                       calibration_evidence='synthetic zero/load generator',sample_seq_field='seq')
    cfg['runtime'].update(voltage_filter_time_constant_s=.5,max_voltage_sample_age_s=1.)
    cfg['hardware_firmware_metadata'].update(actual_firmware='synthetic',configuration_match='synthetic',vbat_sag_compensation_disabled=True)
    (out/'config.yaml').write_text(yaml.safe_dump(cfg))
    (out/'fixture_config.txt').write_text('SYNTHETIC: no physical motor, p=1.6, a=1.00008, clock b=9.1, response delay=0.13 s\n')
    rng=np.random.default_rng(217)
    sessions=[]
    for index,(split,voltage) in enumerate([(s,v) for s,n in [('train',3),('validation',1),('test',1)] for _ in range(n) for v in (21.,22.5,24.)]):
        run=f'{split}_{index:02d}'
        marker_c=[1150,1450,1150,1450,1150,1450,1150,1450]
        marker_d=[3,2,5,3,4,2,3,4]
        targets=marker_c+[1150,1300,1450,1600,1450,1300,1600,1150]+marker_c
        durations=marker_d+[4]*8+marker_d
        edges=np.r_[0,np.cumsum(durations)]; length=edges[-1]; base=100.+index*100
        tb=base+np.arange(0,length,.01)
        jitter=rng.integers(-1,2,len(targets)); levels=np.array(targets)+jitter
        ii=np.minimum(np.searchsorted(edges,tb-base,side='right')-1,len(targets)-1)
        c=levels[ii].astype(float)
        r=np.floor(1000+(c-980)*1000/1040)
        recovered=980+(r-1000)*1040/1000
        vv=voltage+0.008*np.sin((tb-base)*.1)
        cols={'time':np.rint(tb*1e6).astype(np.int64),'debug[3]':r,'rx_ok':np.ones(len(tb),int),'failsafe':np.zeros(len(tb),int),'vbatLatest':vv}
        for i in range(4): cols[f'motor[{i}]']=500+(recovered-1000)*1.5+i
        write_csv(out/(run+'_bf.csv'),cols)
        a,b=1.00008,9.23
        tp=np.arange(a*(base-5)+b,a*(base+length+5)+b,1/60)
        equiv=(tp-b)/a
        jj=np.clip(np.searchsorted(edges,equiv-base,side='right')-1,0,len(targets)-1)
        cc=980+(np.floor(1000+(levels[jj]-980)*1000/1040)-1000)*1040/1000
        force=(.2+((cc-1000)/300)**2)*( (voltage+.008*np.sin((equiv-base)*.1))/22.5)**1.6
        force[(equiv<base)|(equiv>=base+length)]=0
        # Modest physical force response smoothing; effective b already contains 130 ms delay.
        for j in range(1,len(force)): force[j]=force[j-1]+(1-np.exp(-1/60/.025))*(force[j]-force[j-1])
        force+=rng.normal(0,.002,len(force))+.3
        write_csv(out/(run+'_px4.csv'),{'timestamp':np.rint(tp*1e6).astype(np.int64),'load':force,'seq':np.arange(len(tp))})
        anchors=[]
        for j in list(range(1,8))+list(range(len(targets)-7,len(targets))):
            x=base+edges[j]
            anchors.append(dict(bf_s=float(x),px4_s=float(a*x+b+.015),direction=int(np.sign(levels[j]-levels[j-1])),
                                role='check' if j in (2,len(targets)-2) else 'fit',evidence='synthetic matching nonperiodic edge'))
        sessions.append(dict(run_id=run,battery_id='synthetic_'+split,block_id=run,arm_cycle=1,split=split,tested_motor_index=0,
                             provenance='synthetic',pairing_evidence='shared synthetic generator',configuration_file='fixture_config.txt',
                             direction='mixed',throttle_raised='synthetic known',bf=dict(file=run+'_bf.csv',log_index=1,interval_s=[base,base+float(length)]),
                             px4=dict(file=run+'_px4.csv',topic='synthetic_load',instance=0,interval_s=[float(tp[0]),float(tp[-1])]),
                             no_thrust_intervals_px4_s=[[float(tp[0]),a*(base-1)+b],[float(a*(base+length+1)+b),float(tp[-1])]],anchors=anchors))
    (out/'sessions.yaml').write_text(yaml.safe_dump({'sessions':sessions},sort_keys=False))
    return cfg,sessions

if __name__=='__main__': generate(sys.argv[1])
