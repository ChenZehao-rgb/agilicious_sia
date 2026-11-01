#!/usr/bin/env python3
"""SITL FC launch wiring; builds actions without starting devices."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'test'))
import test_runtime_config as existing
runtime = existing.runtime


class SitlFcLaunchTests(unittest.TestCase):
    def test_default_fc_chain_uses_host_time(self):
        helper = existing.RuntimeProfiles()
        # The older helper explicitly selects ROS EKF; an empty override tests
        # the actual profile default instead.
        nodes = helper.actions(mode='sitl', state_source='')
        self.assertEqual(runtime.load_profile('sitl')[1]['state']['source'], 'fc_fused')
        self.assertEqual(set(nodes), {'mavlink_sensor_node', 'fc_state_adapter_node',
                                     'control_node', 'command_output_node'})
        self.assertTrue(all(not params['use_sim_time'] for params in nodes.values()))
        self.assertEqual(nodes['mavlink_sensor_node']['device'], 'tcp://127.0.0.1:5762')
        self.assertEqual(nodes['control_node']['navigation_source'], 'fc_fused')
        self.assertEqual(nodes['command_output_node']['mode'], 'sitl')

    def test_legacy_sensor_chain_remains_available(self):
        nodes = existing.RuntimeProfiles().actions(mode='sitl', state_source='ros_ekf')
        self.assertIn('state_fusion_node', nodes)
        self.assertNotIn('fc_state_adapter_node', nodes)
        self.assertNotIn('mavlink_sensor_node', nodes)
        self.assertTrue(nodes['control_node']['use_sim_time'])

    def test_invalid_combinations_fail_before_launch(self):
        for args in (dict(sitl_delay_test='true'), dict(mavlink_device='/dev/ttyUSB0'),
                     dict(mavlink_enabled='false')):
            with self.subTest(args=args), self.assertRaises(ValueError):
                existing.RuntimeProfiles().actions(mode='sitl', state_source='fc_fused', **args)


if __name__ == '__main__':
    unittest.main()
