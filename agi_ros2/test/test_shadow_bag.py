#!/usr/bin/env python3
"""Synthetic rosbag round trip for the evaluation report; no DDS or hardware."""
from pathlib import Path
import sys
import tempfile
import unittest

import rosbag2_py
from rclpy.serialization import serialize_message
from agi_ros2.msg import ComputationStatus, MspEvent, NavigationOrigin
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from summarize_shadow_bag import summarize


class BagTests(unittest.TestCase):
    def test_summary_preserves_evidence_and_counts(self):
        with tempfile.TemporaryDirectory() as directory:
            bag = str(Path(directory) / 'bag')
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
            types = [('computation_status', 'ComputationStatus'), ('msp/events', 'MspEvent'),
                     ('navigation/origin', 'NavigationOrigin')]
            for topic, typ in types:
                writer.create_topic(rosbag2_py.TopicMetadata(name='/'+topic, type='agi_ros2/msg/'+typ,
                                                          serialization_format='cdr'))
            for i in range(3):
                status = ComputationStatus()
                status.header.stamp.sec = 10 + i
                status.mpc_success = True
                status.solve_seconds = .002
                status.warm_cycles = 50
                writer.write('/computation_status', serialize_message(status), (10+i)*1000000000+1000000)
            event = MspEvent()
            event.event = 'tx'
            event.code = 200
            writer.write('/msp/events', serialize_message(event), 13000000000)
            origin = NavigationOrigin()
            origin.session_id = 'origin-test'
            origin.altitude_reference = 'msl'
            writer.write('/navigation/origin', serialize_message(origin), 13000000001)
            del writer
            report = summarize(bag)
            self.assertEqual(report['mpc']['successes'], 3)
            self.assertEqual(report['output_isolation']['code200_tx_events'], 1)
            self.assertEqual(report['origins'][0]['altitude_reference'], 'msl')
            self.assertEqual(report['streams']['/computation_status']['unique_rate_hz'], 1.)
            self.assertAlmostEqual(report['streams']['/computation_status']['bag_receive_age_seconds']['mean'], .001)


if __name__ == '__main__':
    unittest.main()
