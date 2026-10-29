"""Reference configuration and direct-node rejection; no physical UART or motors."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import yaml
from test_runtime_config import runtime, ROOT


class FlowLists(yaml.SafeDumper):
    pass
FlowLists.add_representer(list,lambda d,v:d.represent_sequence('tag:yaml.org,2002:seq',v,flow_style=True))


class ReferenceRuntime(unittest.TestCase):
    def test_provisional_profile_paths_and_existing_authority(self):
        path = ROOT/'agi_ros2/config/hardware_reference_trial.yaml'
        document = yaml.safe_load(path.read_text())
        original = yaml.safe_load((ROOT/'agi_ros2/config/hardware.yaml').read_text())
        for section in ('pilot', 'bridge', 'evidence', 'output', 'state', 'fusion'):
            self.assertEqual(document[section], original[section])
        self.assertEqual(document['flight']['shadow_only'], original['flight']['shadow_only'])
        for shadow in (False, True):
            params = runtime.checked_thrust_mapping(document['flight'], 'hardware', path, shadow)
            self.assertEqual(Path(params['thrust_calibration']), path.parent/'thrust_motor0_20261029/calibration.yaml')
        with self.assertRaisesRegex(ValueError, 'hardware'):
            runtime.checked_thrust_mapping(document['flight'], 'sitl', path, True)

    def test_launch_and_direct_node_require_measured_validation(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location('reference_calibration',ROOT/'agi_ros2/launch/reference_calibration.py')
        contract=importlib.util.module_from_spec(spec);spec.loader.exec_module(contract)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)
            (path/'curve.csv').write_text('rc_throttle,total_thrust_n_at_vref\n1100,1\n1600,20\n')
            meta={**contract.EXACT,'table_file':'curve.csv','axis_source':'debug[3]','axis_conversion':'synthetic',
                  'axis_evidence':'synthetic','calibration_id':'synthetic_test','provenance':'synthetic','calibration_status':'unvalidated',
                  'reference_voltage_v':22.,'voltage_exponent_p':1.6,'valid_voltage_min_v':20.,'valid_voltage_max_v':25.,
                  'valid_rc_min':1100,'valid_rc_max':1600,'voltage_filter_time_constant_s':0.,'max_voltage_sample_age_s':1.}
            (path/'calibration.yaml').write_text(yaml.safe_dump(meta))
            flight=dict(thrust_model='reference_table',thrust_calibration='calibration.yaml')
            params=runtime.checked_thrust_mapping(flight,'hardware',path/'hardware.yaml',True)
            self.assertEqual(params['thrust_calibration'],str(path/'calibration.yaml'))
            with self.assertRaisesRegex(ValueError,'measured'):runtime.checked_thrust_mapping(flight,'hardware',path/'hardware.yaml',False)
            with self.assertRaisesRegex(ValueError,'hardware'):runtime.checked_thrust_mapping(flight,'sitl',path/'hardware.yaml',True)
            binary=Path(os.environ.get('AGI_ROS2_TEST_INSTALL',ROOT/'install/agi_ros2'))/'lib/agi_ros2/command_output_node'
            self.assertTrue(binary.is_file(),'Build command_output_node first')
            document=yaml.safe_load((ROOT/'agi_ros2/config/hardware.yaml').read_text())
            document['flight'].update(flight,shadow_only=False)
            (path/'hardware.yaml').write_text(yaml.dump(document,Dumper=FlowLists))
            result=subprocess.run([str(binary),'--ros-args','-p','mode:=hardware','-p','runtime_config:='+str(path/'hardware.yaml'),
                                   '-p','device:='+str(path/'no_physical_uart')],capture_output=True,text=True,timeout=10)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('Production requires measured validated calibration',result.stdout+result.stderr)
            self.assertNotIn('open '+str(path/'no_physical_uart'),result.stdout+result.stderr)

if __name__=='__main__': unittest.main()
