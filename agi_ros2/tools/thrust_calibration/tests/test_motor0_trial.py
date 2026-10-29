"""Offline physical-axis/export/runtime regression; no physical device is opened."""
import copy
import json
import shutil
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import yaml

TOOLS = Path(__file__).resolve().parents[1]
REPO = TOOLS.parents[2]
sys.path[:0] = [str(TOOLS), str(REPO/'agi_ros2/launch')]
from motor0_trial import GF_TO_N, SETTINGS, curve_for, export, target_motor, windows
from reference_calibration import load_calibration, map_thrust

ARTIFACT = REPO/'agi_ros2/config/thrust_motor0_20261029'


class MotorTrial(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.path = Path(cls.temporary.name)
        cls.probe = cls.path/'probe'
        subprocess.run(['g++', '-std=c++17', '-O2', '-fno-finite-math-only', '-Wall', '-Wextra', '-Werror',
                        '-I', str(REPO/'agilib/include'), str(TOOLS/'tests/reference_model_probe.cpp'),
                        '-lyaml-cpp', '-o', str(cls.probe)], check=True)
        cls.meta, cls.curve = load_calibration(ARTIFACT/'calibration.yaml', shadow=False)
        cls.target = yaml.safe_load((ARTIFACT/'target_fc_mapping.yaml').read_text())

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def cpp(self, thrust, voltage, filename=None):
        return subprocess.run([str(self.probe), str(filename or ARTIFACT/'calibration.yaml'), 'production',
                               str(thrust), str(voltage)], capture_output=True, text=True)

    def test_real_artifact_provenance_and_single_scaling(self):
        self.assertEqual(self.meta['calibration_status'], 'provisional')
        self.assertNotIn('axis_verified', self.meta)
        self.assertEqual(self.meta['selected_target_profile'], 'dynamic_nominal')
        physical = np.genfromtxt(ARTIFACT/'motor_reference_thrust.csv', delimiter=',', names=True)
        for rc, total in self.curve[::20]:
            m = target_motor(rc, self.target)
            expected = 4*np.interp(m, physical['motor_command'], physical['single_thrust_n_at_vref'])
            self.assertAlmostEqual(total, expected)
        self.assertAlmostEqual(100*GF_TO_N, .980665)
        self.assertGreater(physical['single_thrust_n_at_vref'][0], 0)
        report = json.loads((ARTIFACT/'fit_report.json').read_text())
        self.assertEqual(report['auxiliary']['count'], report['auxiliary_baseline142']['count'])

    def test_endpoints_and_motor530(self):
        self.assertAlmostEqual(float(target_motor(1050, self.target, 'static')), 157.945)
        self.assertAlmostEqual(float(target_motor(1050, self.target, 'dynamic_nominal')), 67.99)
        static = 1050+(530-157.945)/1889.055*950
        dynamic = 1050+(530-48)/1999*950
        self.assertAlmostEqual(float(target_motor(static, self.target, 'static')), 530)
        self.assertAlmostEqual(float(target_motor(dynamic, self.target, 'dynamic_nominal')), 530)
        self.assertAlmostEqual(dynamic-static, 42., delta=.2)
        unsupported = copy.deepcopy(self.target)
        unsupported['parameters']['thr_expo'] = 20
        with self.assertRaises(ValueError): target_motor(1500, unsupported)

    def test_provisional_load_and_python_cpp_parity(self):
        for voltage in (21., 24., 24.66):
            for total in (0., .01, 1., 7.747, 20., 31.36):
                expected = map_thrust(self.meta, self.curve, total, voltage)
                result = self.cpp(total, voltage)
                self.assertEqual(result.returncode, 0, result.stderr)
                c, rc, scale, lookup = map(float, result.stdout.split())
                self.assertAlmostEqual(c, expected['rc_continuous'], places=8)
                self.assertEqual(rc, expected['rc'])
                self.assertAlmostEqual(scale, expected['scale'])
                self.assertAlmostEqual(lookup, expected['lookup_n'])
        self.assertGreater(map_thrust(self.meta, self.curve, 5, 21)['scale'], 1)
        self.assertLess(map_thrust(self.meta, self.curve, 5, 24.66)['scale'], 1)

    def test_idle_lower_saturation_and_rejections(self):
        self.assertTrue(map_thrust(self.meta, self.curve, 0, 24)['idle_requested'])
        self.assertTrue(map_thrust(self.meta, self.curve, .01, 24)['lower_saturated'])
        self.assertEqual(map_thrust(self.meta, self.curve, .01, 24)['rc'], self.curve[0][0])
        for force, voltage in ((-1,24), (100,24), (1,0), (1,20.99), (1,24.67), (float('nan'),24), (1,float('nan')), (0,0)):
            with self.subTest(force=force, voltage=voltage):
                with self.assertRaises(ValueError): map_thrust(self.meta,self.curve,force,voltage)
                self.assertNotEqual(self.cpp(force,voltage).returncode,0)

    def test_cpp_saturation_diagnostics(self):
        for thrust, expected in [(0., [0.,1.]), (.01,[1.,0.]), (7.747,[0.,0.])]:
            result = subprocess.run([str(self.probe),str(ARTIFACT/'calibration.yaml'),'production',str(thrust),'24','diagnostics'],
                                    capture_output=True,text=True,cwd=self.path)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(list(map(float,result.stdout.split()[-2:])),expected)

    def test_target_only_reexport_leaves_motor_curve_unchanged(self):
        output = self.path/'reexport'
        shutil.copytree(ARTIFACT,output,dirs_exist_ok=True)
        before = (output/'motor_reference_thrust.csv').read_bytes()
        target = copy.deepcopy(self.target)
        target['selected_target_profile'] = 'static'
        target['parameters']['dyn_idle_min_rpm'] = 0
        target_path = self.path/'static_target.yaml'
        target_path.write_text(yaml.safe_dump(target))
        export(output,target_path)
        meta,curve = load_calibration(output/'calibration.yaml',shadow=False)
        self.assertEqual(meta['selected_target_profile'],'static')
        self.assertEqual(curve[0][0],1051)
        self.assertEqual((output/'motor_reference_thrust.csv').read_bytes(),before)
        self.assertEqual(yaml.safe_load((output/'target_fc_mapping.yaml').read_text()),target)

    def test_filter_bypass_and_real_report_age(self):
        result = subprocess.run([str(self.probe),str(ARTIFACT/'calibration.yaml'),'production','filter'],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_malformed_provisional_and_p_zero(self):
        original = copy.deepcopy(self.meta)
        original['table_file'] = str(ARTIFACT/'reference_thrust.csv')
        for key, value in [('selected_target_profile','guess'), ('measured_motor_index',True),
                           ('axis_mapping_basis','verified'), ('lower_policy','extrapolate'), ('calibration_status','unknown')]:
            metadata = {**original,key:value}
            filename = self.path/'calibration.yaml'
            filename.write_text(yaml.safe_dump(metadata))
            with self.assertRaises(ValueError): load_calibration(filename)
            self.assertNotEqual(self.cpp(1,24,filename).returncode,0)
        original['voltage_exponent_p'] = 0
        filename.write_text(yaml.safe_dump(original))
        outputs=[self.cpp(7.747,v,filename) for v in (21,24,24.66)]
        self.assertTrue(all(r.returncode==0 for r in outputs))
        self.assertEqual(len(set(r.stdout for r in outputs)),1)

    def test_no_window_crosses_gap_and_tare_not_idle_zero(self):
        t=np.arange(10,16,.001)
        t=t[(t<12.01)|(t>12.04)]
        ft=np.arange(100,116,.01)
        d=dict(t=t,m=np.full(len(t),158),v=np.full(len(t),24),ft=ft,
               force=np.full(len(ft),167.),dropouts=[],peak_time=100)
        rows=windows(d,1,offset=94.143109)
        self.assertGreaterEqual(len(rows),2)
        for row in rows:
            self.assertFalse(row['core_start_s']<12.01<row['core_end_s'])
            self.assertAlmostEqual(row['single_force_n'],20*GF_TO_N)
        _,before,after=curve_for(rows,1.2,24)
        self.assertEqual(len(before),1)
        self.assertGreater(after[0],0)


if __name__=='__main__': unittest.main()
