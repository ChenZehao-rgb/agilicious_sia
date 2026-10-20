#!/usr/bin/env python3
"""Clock startup gate tests without changing the host clock or launching hardware."""
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from wait_for_clock import positive_timeout, wait_for_sync


class ClockStartupTest(unittest.TestCase):
    def setUp(self):
        self.elapsed = 0.
        self.clock = patch('wait_for_clock.time.monotonic', side_effect=lambda: self.elapsed)
        self.sleep = patch('wait_for_clock.time.sleep', side_effect=self.advance)
        self.clock.start()
        self.sleep.start()
        self.addCleanup(self.clock.stop)
        self.addCleanup(self.sleep.stop)

    def advance(self, seconds):
        self.elapsed += seconds

    def test_initial_sync_before_deadline(self):
        replies = [SimpleNamespace(returncode=0, stdout=value, stderr='') for value in ('no\n', 'yes\n')]
        with patch('wait_for_clock.subprocess.run', side_effect=replies) as run:
            self.assertTrue(wait_for_sync(2.))
        self.assertEqual(run.call_count, 2)
        self.assertEqual(self.elapsed, .5)

    def test_unsynchronized_missing_service_and_query_timeout_all_fail_closed(self):
        cases = [SimpleNamespace(returncode=0, stdout='no\n', stderr=''),
                 SimpleNamespace(returncode=1, stdout='', stderr='No time service'),
                 FileNotFoundError('timedatectl'), subprocess.TimeoutExpired('timedatectl', 2.)]
        for case in cases:
            with self.subTest(case=case):
                self.elapsed = 0.
                options = dict(side_effect=case) if isinstance(case, Exception) else dict(return_value=case)
                with patch('wait_for_clock.subprocess.run', **options):
                    self.assertFalse(wait_for_sync(1.))
                self.assertEqual(self.elapsed, 1.)

    def test_invalid_deadlines_are_rejected(self):
        for timeout in ('0', '-1', 'nan', 'inf', 'bad'):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                positive_timeout(timeout)


if __name__ == '__main__':
    unittest.main()
