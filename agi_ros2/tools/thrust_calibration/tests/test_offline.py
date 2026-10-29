import copy
import csv
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT.parents[1]/'launch'),str(Path(__file__).parent)]
from semantics import raw_rc,force_newtons,times
from log_io import inspect_file,read_columns,decode_blackbox,csv_columns
from alignment import fit_alignment
from steady_windows import extract
from inspection import load_sessions,inspect_pair
from fit_reference import fit,regression,validate,pava
from reference_calibration import load_calibration,inverse,EXACT
from generate_synthetic import generate


class Offline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();cls.directory=Path(cls.tmp.name)
        cls.cfg,_=generate(cls.directory)
        cls.sessions=load_sessions(cls.directory/'sessions.yaml')
        cls.rows=[];cls.alignments=[]
        for s in cls.sessions:
            a=fit_alignment(s['anchors'],cls.cfg['alignment'],s['bf']['interval_s']);a['alignment_id']='synthetic'
            cls.alignments.append(a);cls.rows.extend(extract(s,cls.cfg,a)[0])
        cls.model=fit(cls.rows,cls.cfg['fit'])

    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()

    def test_inspection_units_and_mode_unknown(self):
        report=inspect_pair(self.sessions[0],self.cfg)
        self.assertEqual(report['quality'],'ready_for_alignment')
        self.assertTrue(report['files']['bf']['mode_observability']['msp_override'].startswith('unknown'))
        cfg=copy.deepcopy(self.cfg);cfg['bf']['voltage_unit']=None
        self.assertEqual(inspect_pair(self.sessions[0],cfg)['quality'],'unresolved')

    def test_missing_force_and_unknown_axis(self):
        for side,key,value in [('px4','force_field','nonexistent'),('bf','decoder_quality_reviewed',False)]:
            cfg=copy.deepcopy(self.cfg);cfg[side][key]=value
            self.assertTrue(inspect_pair(self.sessions[0],cfg)['gaps'])

    def test_rx_range_inverse_and_boundaries(self):
        cfg=copy.deepcopy(self.cfg['bf']['axis'])
        cols={'debug[3]':np.array([750,1000,1500,2000,2250]),'rx_ok':np.ones(5),'failsafe':np.zeros(5)}
        c,valid,error=raw_rc(cols,cfg)
        self.assertAlmostEqual(c[2],1500);self.assertEqual(error,1.04)
        np.testing.assert_allclose(c[1:4],[980,1500,2020]);self.assertEqual(valid.tolist(),[False,False,True,False,False])
        cfg.update(rx_min=1000,rx_max=2000)
        np.testing.assert_allclose(raw_rc(cols,cfg)[0][1:4],[1000,1500,2000])
        cols['failsafe'][2]=1;self.assertFalse(raw_rc(cols,cfg)[1][2])

    def test_no_processed_field_relabel(self):
        for field in ('rcCommand[3]','setpoint[3]','motor[0]','debug[3]'):
            cfg={**self.cfg['bf']['axis'],'field':field,'semantics':'msp_raw_rc_throttle'}
            with self.assertRaises(ValueError): raw_rc({field:np.array([1500])},cfg)

    def test_debug_mode_and_scale_required(self):
        cols={'debug[3]':np.array([1500]),'rx_ok':np.ones(1),'failsafe':np.zeros(1)}
        for change in [dict(debug_mode='MAVLINK_TELEMETRY'),dict(decode_scale=.1),dict(same_msp_rx_path_verified=False),dict(rx_smoothing='moving_average')]:
            with self.assertRaises(ValueError): raw_rc(cols,{**self.cfg['bf']['axis'],**change})

    def test_force_units_projection_and_idle(self):
        cfg={**self.cfg['px4'],'offset':0}
        for unit,value in [('gf',1000),('kgf',1),('N',9.80665)]:
            self.assertAlmostEqual(force_newtons({'load':[value]},{**cfg,'unit':unit})[1][0],9.80665)
        self.assertEqual(force_newtons({'load':[.8]},cfg)[1][0],.8)
        with self.assertRaises(ValueError): force_newtons({'load':[1]},{**cfg,'zero_is_no_thrust':False})
        self.assertEqual(force_newtons({'x':[3],'y':[4]},{**cfg,'projection':{'x':1.,'y':0.}})[1][0],3)
        with self.assertRaises(ValueError):force_newtons({'x':[3]},{**cfg,'projection':{'x':2.}})

    def test_clock_recovery_and_response_delay_label(self):
        a=self.alignments[0]
        self.assertAlmostEqual(a['a'],1.00008,places=10);self.assertAlmostEqual(a['b_s'],9.245,places=8)
        self.assertTrue(a['response_delay_included']);self.assertFalse(a['pure_clock_offset'])

    def test_bad_and_insufficient_alignment(self):
        s=self.sessions[0]
        with self.assertRaises(ValueError):fit_alignment(s['anchors'][:2],self.cfg['alignment'],s['bf']['interval_s'])
        bad=copy.deepcopy(s['anchors']);bad[1]['px4_s']+=.5
        with self.assertRaises(ValueError):fit_alignment(bad,self.cfg['alignment'],s['bf']['interval_s'])
        bad=copy.deepcopy(s['anchors']);bad[1]['px4_s']=bad[0]['px4_s']
        with self.assertRaises(ValueError):fit_alignment(bad,self.cfg['alignment'],s['bf']['interval_s'])

    def test_native_sample_counts_and_history(self):
        r=next(r for r in self.rows if r['accepted'])
        self.assertLess(r['force_count'],r['rc_count'])
        self.assertEqual(r['force_count'],r['force_independent_samples'])
        self.assertEqual(r['throttle_raised'],'synthetic known')
        self.assertIn('force_mean_early',r)

    def test_gap_and_reset_rejected(self):
        s=copy.deepcopy(self.sessions[0]);cols=csv_columns(s['px4']['file']);path=self.directory/'bad.csv'
        from generate_synthetic import write_csv
        cols['timestamp'][100]=cols['timestamp'][99]
        write_csv(path,cols);s['px4']['file']=str(path)
        with self.assertRaises(ValueError):extract(s,self.cfg,self.alignments[0])
        cols=csv_columns(self.sessions[0]['px4']['file']);mask=np.ones(len(cols['timestamp']),bool);mask[400:430]=False
        write_csv(path,{k:v[mask] for k,v in cols.items()})
        rows,_=extract(s,self.cfg,self.alignments[0]);self.assertTrue(any('gap' in r['reason'] for r in rows))

    def test_alignment_margin_can_reject_windows(self):
        a={**self.alignments[0],'uncertainty_s':2.}
        rows,_=extract(self.sessions[0],self.cfg,a)
        self.assertLess(sum(r['accepted'] for r in rows),20)

    def test_recovers_p16_and_heldout(self):
        self.assertAlmostEqual(self.model['p_fit'],1.6,delta=.02)
        self.assertEqual(self.model['selected'],'p_fit')
        self.assertTrue(validate(self.rows,self.model,self.cfg['fit'])['passed'])
        self.assertLess(self.model['p_block_ci95'][1]-self.model['p_block_ci95'][0],.02)

    def test_unidentifiable_single_sweep(self):
        train=[r for r in self.rows if r['accepted'] and r['split']=='train']
        broken=[dict(r,voltage_mean=22.5) for r in train]
        with self.assertRaises(ValueError):regression(broken,10,self.cfg['fit'])
        broken=[dict(r,voltage_mean=26-r['rc_mean']/400.) for r in train]
        with self.assertRaises(ValueError):regression(broken,10,self.cfg['fit'])

    def test_local_p_mismatch(self):
        train=copy.deepcopy([r for r in self.rows if r['accepted'] and r['split']=='train'])
        for r in train:
            if r['rc_mean']<1200:r['force_mean']*=(22.5/r['voltage_mean'])**1.6
        with self.assertRaisesRegex(ValueError,'Single p'):regression(train,10,self.cfg['fit'])

    def test_final_test_never_fits_or_selects_model(self):
        changed=copy.deepcopy(self.rows)
        for r in changed:
            if r['split']=='test':r['force_mean']*=3
        new=fit(changed,self.cfg['fit'])
        self.assertEqual(self.model['p_fit'],new['p_fit']);self.assertEqual(self.model['curve'],new['curve'])
        self.assertFalse(validate(changed,new,self.cfg['fit'])['passed'])

    def test_x4_once_and_plateau(self):
        c=1300;expected=4*(.2+((c-1000)/300)**2)*(self.model['reference_voltage_v']/22.5)**1.6
        got=np.interp(c,*np.array(self.model['curve']).T)
        self.assertAlmostEqual(got,expected,delta=.05)
        curve=[(1000,.8),(1100,.8),(1200,2.8),(1300,6.8)]
        self.assertEqual(inverse(curve,.8),1000);self.assertEqual(inverse(curve,1.8),1150)
        self.assertEqual(inverse(curve[1:],.8),1100)
        self.assertEqual(inverse(curve,6.8),1300)
        for f in (0,-1,7,float('nan')):
            with self.assertRaises(ValueError):inverse(curve,f)
        np.testing.assert_allclose(pava([1,3,2,4],[1,1,1,1]),[1,2.5,2.5,4])

    def test_missing_final_test_is_not_validation(self):
        result=validate([r for r in self.rows if r['split']!='test'],self.model,self.cfg['fit'])
        self.assertFalse(result['passed']);self.assertFalse(result['independent_final_test'])

    def test_coarse_rc_groups_do_not_resolve_voltage(self):
        with self.assertRaises(ValueError):fit(self.rows,{**self.cfg['fit'],'rc_bin_width':500})

    def test_integer_timestamps_preserved_and_devices_rejected(self):
        path=self.directory/'integers.csv';path.write_text('timestamp,force\n9007199254740993,1\n9007199254740994,2\n')
        self.assertEqual(int(csv_columns(path)['timestamp'][0]),9007199254740993)
        with self.assertRaises(ValueError):inspect_file('/dev/null')

    def test_split_leakage(self):
        data=yaml.safe_load((self.directory/'sessions.yaml').read_text())
        data['sessions'][1].update(block_id=data['sessions'][0]['block_id'],split='test')
        path=self.directory/'leak.yaml';path.write_text(yaml.safe_dump(data))
        with self.assertRaisesRegex(ValueError,'leakage'):load_sessions(path)

    def test_ulog_topics_instances_dropouts(self):
        def msg(kind,payload):return struct.pack('<HB',len(payload),ord(kind))+payload
        data=b'ULog\x01\x12\x35'+bytes([1])+struct.pack('<Q',1000000)
        data+=msg('F',b'load:uint64_t timestamp;float force;')
        for instance in (0,1):data+=msg('A',struct.pack('<BH',instance,instance)+b'load')
        for i in range(4):
            for instance in (0,1):data+=msg('D',struct.pack('<HQf',instance,1000000+i*10000,float(i+instance)))
        data+=msg('O',struct.pack('<H',30))
        path=self.directory/'force.renamed';path.write_bytes(data)
        info=inspect_file(path);self.assertEqual(len(info['datasets']),2);self.assertEqual(info['dropouts'][0]['duration_ms'],30)
        cols,_=read_columns(path,dict(topic='load',instance=1));self.assertEqual(cols['force'][0],1)
        with self.assertRaises(ValueError):read_columns(path,dict(topic='guess',instance=0))

    def test_decoder_probes_and_never_raw(self):
        path=self.directory/'raw.not_bfl';path.write_bytes(b'H Product:Blackbox flight data recorder\nH debug_mode:RX_SIGNAL_LOSS\nIabc')
        script=self.directory/'decoder'
        script.write_text('#!/usr/bin/env python3\nimport sys\nif "--help" in sys.argv: print("--index --stdout --unit-frame-time --unit-vbat --output-dir")\nelse:\n print("time, vbatLatest\\n100, 22")\n print("1 valid, 0 corrupt",file=sys.stderr)\n')
        script.chmod(0o755)
        out=self.directory/'decoded';decode_blackbox(path,out,str(script),1)
        meta=json.loads((out/'decode.json').read_text());self.assertNotIn('--raw',meta['command']);self.assertIn('--output-dir',meta['command'])
        self.assertFalse(meta['decoder_quality_reviewed']);self.assertEqual(csv_columns(out/'blackbox.csv')['vbatLatest'][0],22)


class Contract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();cls.path=Path(cls.tmp.name)
        cls.probe=cls.path/'probe'
        subprocess.run(['g++','-std=c++17','-O2','-fno-finite-math-only','-Wall','-Wextra','-Werror','-I',str(ROOT.parents[2]/'agilib/include'),
                        str(Path(__file__).with_name('reference_model_probe.cpp')),'-lyaml-cpp','-o',str(cls.probe)],check=True)
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def setUp(self):
        self.meta={**EXACT,'calibration_id':'synthetic_contract','axis_source':'debug[3]','axis_conversion':'synthetic rx range',
                   'axis_evidence':'synthetic','provenance':'synthetic','calibration_status':'unvalidated',
                   'reference_voltage_v':22.,'voltage_exponent_p':1.6,'valid_voltage_min_v':20.,'valid_voltage_max_v':25.,
                   'valid_rc_min':1000,'valid_rc_max':1300,'voltage_filter_time_constant_s':.5,'max_voltage_sample_age_s':1.,'table_file':'curve.csv'}
        self.curve='rc_throttle,total_thrust_n_at_vref\n1000,0.8\n1100,0.8\n1200,2.8\n1300,6.8\n'
    def write(self):
        (self.path/'curve.csv').write_text(self.curve);(self.path/'calibration.yaml').write_text(yaml.safe_dump(self.meta))
    def probe_result(self,*args,shadow=True):
        return subprocess.run([str(self.probe),str(self.path/'calibration.yaml'),'shadow' if shadow else 'production',*map(str,args)],capture_output=True,text=True)
    def test_cpp_python_inverse_parity(self):
        self.write();m,curve=load_calibration(self.path/'calibration.yaml',True)
        for v in (20.,22.,25.):
            for lookup in (.8,1.8,2.8,6.7):
                t=lookup/(22/v)**1.6;r=self.probe_result(t,v)
                self.assertEqual(r.returncode,0,r.stderr)
                c,rc,scale,tn=map(float,r.stdout.split());self.assertAlmostEqual(c,inverse(curve,lookup),places=9)
                self.assertAlmostEqual(tn,lookup);self.assertEqual(rc,int(c+.5))
    def test_cpp_plateau_and_crop(self):
        self.write();self.assertEqual(float(self.probe_result(.8,22).stdout.split()[0]),1000)
        self.meta['valid_rc_min']=1100;self.curve=self.curve.replace('1000,0.8\n','');self.write()
        self.assertEqual(float(self.probe_result(.8,22).stdout.split()[0]),1100)
        self.assertEqual(float(self.probe_result(1.8,22).stdout.split()[0]),1150)
    def test_filter_uses_report_identity_and_real_age(self):
        self.write();r=self.probe_result('filter');self.assertEqual(r.returncode,0,r.stderr)
    def test_production_rejects_fixture(self):
        self.write();self.assertNotEqual(self.probe_result(shadow=False).returncode,0)
        with self.assertRaises(ValueError):load_calibration(self.path/'calibration.yaml')
    def test_model_domain_and_p_zero(self):
        self.meta['voltage_exponent_p']=0;self.write()
        for v in (20,25):self.assertEqual(float(self.probe_result(1.8,v).stdout.split()[0]),1150)
        self.assertEqual(float(self.probe_result(0,22).stdout.split()[0]),1000)
        for f,v in [(-1,22),(10,22),(1.8,0),(1.8,19),(1.8,26),('nan',22),(1.8,'nan'),(1.8,'inf')]:
            self.assertNotEqual(self.probe_result(f,v).returncode,0)
    def test_invalid_contracts_match(self):
        original=copy.deepcopy(self.meta)
        changes=[('axis_verified',False),('schema_version',2),('thrust_unit','gf'),('thrust_scope','single_motor'),
                 ('command_axis','rcCommand[3]'),('reference_voltage_v',float('nan')),('valid_rc_min',999),
                 ('voltage_filter_time_constant_s',-1),('max_voltage_sample_age_s',0),('voltage_exponent_p','1.6'),
                 ('aggregation','four_measured'),('calibration_status','unknown'),('axis_verified','true'),('schema_version','1')]
        for key,value in changes:
            self.meta={**original,key:value};self.write()
            with self.subTest(key=key):
                with self.assertRaises((ValueError,TypeError)):load_calibration(self.path/'calibration.yaml',True)
                self.assertNotEqual(self.probe_result().returncode,0)
    def test_duplicate_yaml_keys_rejected_by_both(self):
        self.write();path=self.path/'calibration.yaml';path.write_text(path.read_text()+'voltage_exponent_p: 2\n')
        with self.assertRaises(ValueError):load_calibration(path,True)
        self.assertNotEqual(self.probe_result().returncode,0)

    def test_bad_csv_rejected_by_both(self):
        for rows in ['1000,1\n1000,2\n1300,3\n','1000,1\n1300,1\n','1000,2\n1300,1\n','1000,-1\n1300,1\n','1000,nan\n1300,3\n','1100,1\n1300,3\n']:
            self.curve='rc_throttle,total_thrust_n_at_vref\n'+rows;self.write()
            with self.assertRaises(ValueError):load_calibration(self.path/'calibration.yaml',True)
            self.assertNotEqual(self.probe_result().returncode,0)

if __name__=='__main__':unittest.main()
