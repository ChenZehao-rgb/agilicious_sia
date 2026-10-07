#!/usr/bin/env python3
"""Deterministic health freshness checks with real messages and MSP decoding."""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from agi_ros2.msg import FusedState, OutputStatus
from builtin_interfaces.msg import Time
from msp_evidence import EvidenceNode
from test_shadow_support import ready_decoder


class CapturedPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class HealthFreshnessTest(unittest.TestCase):
    def health(self, sample_age, publication_age=0.0):
        fused = FusedState()
        fused.header.stamp = Time(sec=10, nanosec=0)
        fused.clock_id = 'test-boot'
        fused.initialized = fused.imu_ready = fused.estimator_ready = True
        fused.navigation_ready = fused.navigation_accuracy_ok = fused.accuracy_known = True
        fused.heading_valid = fused.clock_aligned = True
        fused.published_steady_time = 100.0 - publication_age
        output = OutputStatus()
        output.header.stamp = Time(sec=10, nanosec=0)
        output.clock_id = 'test-boot'
        output.steady_time = 100.0
        output.transport_healthy = output.thrust_mapping_ready = True
        node = SimpleNamespace(
            decoder=ready_decoder(10.0), battery_timeout=1.5, clock_id='test-boot',
            minimum=[-1.0]*3, maximum=[1.0]*3, fused=fused, output=output,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round((10.0+sample_age)*1e9))))
        for name in ['decoded_pub', 'authority_pub', 'health_pub', 'status_pub']:
            setattr(node, name, CapturedPublisher())
        with patch('msp_evidence.time.monotonic', return_value=100.0):
            EvidenceNode.publish(node)
        return node.health_pub.messages[-1]

    def test_24ms_state_retains_diagnostic_readiness(self):
        health = self.health(.024, .024)
        self.assertTrue(health.fc_imu_ready)
        self.assertTrue(health.imu_ready and health.estimator_ready and health.navigation_ready)
        self.assertTrue(health.geofence_ok)

    def test_stale_cached_state_does_not_clear_fc_readiness(self):
        for sample_age, publication_age in [(.026, 0.0), (.001, .026)]:
            with self.subTest(sample_age=sample_age, publication_age=publication_age):
                health = self.health(sample_age, publication_age)
                self.assertTrue(health.fc_imu_ready)
                self.assertFalse(health.imu_ready or health.estimator_ready or health.navigation_ready or health.geofence_ok)
                self.assertIn('Fused state unavailable/stale', health.reason)

    def test_future_state_remains_rejected(self):
        self.assertFalse(self.health(-.001).navigation_ready)


if __name__ == '__main__':
    unittest.main()
