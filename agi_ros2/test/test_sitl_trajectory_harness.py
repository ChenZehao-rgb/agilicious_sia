#!/usr/bin/env python3
"""Pure checks of trajectory scoring; no ROS processes or simulators are started."""
import json
import csv
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from test_sitl_trajectories import (CSV_HEADER, describe_trajectory, json_ready, manual_throttle, receiver_inputs,
                                  rescore_directory, save_results, score_tracking, write_csv)


class TrajectoryHarness(unittest.TestCase):
    def test_original_units_and_mass_estimate_are_preserved(self):
        data = np.zeros((3, 30))
        data[:, 0] = [7., 7.25, 7.5]
        data[:, 4] = 1.
        data[:, 1] = [10., 11., 12.]
        data[:, 3] = [3., 1., 2.]
        data[:, 8] = [0., 4., 0.]
        data[:, 20:24] = 9.8066 / 4
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'trajectory.csv'
            np.savetxt(path, data, delimiter=',', header=CSV_HEADER, comments='')
            result = describe_trajectory(path, .54)
        self.assertEqual(result['duration_seconds'], .5)
        self.assertEqual(result['relative_position_min_m'], [0., 0., -2.])
        self.assertEqual(result['speed_max_mps'], 4.)
        self.assertAlmostEqual(result['source_mass_estimate_kg'], 1.)
        self.assertAlmostEqual(result['target_total_thrust_max_n'], .54 * 9.8066)

    def test_score_matches_timestamps_and_does_not_fit_away_constant_error(self):
        truth = [dict(stamp_ns=t, position=[t * 1e-9 + 2., 0., 0.], velocity=[1., 0., 0.])
                 for t in (0, 10_000_000, 20_000_000)]
        reference = [dict(stamp_ns=t, position=[t * 1e-9, 0., 0.], velocity=[1., 0., 0.])
                     for t in (5_000_000, 15_000_000)]
        computation = [dict(stamp_ns=t, trajectory_active=True, reference_elapsed=t * 1e-9)
                       for t in (5_000_000, 15_000_000)]
        summary, rows = score_tracking(reference, truth, computation, .02)
        self.assertEqual(len(rows), 2)
        self.assertAlmostEqual(summary['position_error_m']['rms'], 2.)
        self.assertAlmostEqual(summary['coverage_fraction'], .75)
        self.assertEqual(summary['scoring_fraction'], 1.)
        self.assertEqual(summary['actual_speed_max_mps'], 1.)
        self.assertEqual(summary['reference_speed_max_mps'], 1.)

    def test_score_rejects_truth_gaps_and_inactive_hover(self):
        truth = [dict(stamp_ns=t, position=[0., 0., 0.], velocity=[0., 0., 0.]) for t in (0, 100_000_000)]
        reference = [dict(stamp_ns=50_000_000, position=[0., 0., 0.], velocity=[0., 0., 0.])]
        computation = [dict(stamp_ns=50_000_000, trajectory_active=True, reference_elapsed=.05)]
        summary, rows = score_tracking(reference, truth, computation, 1.)
        self.assertFalse(rows)
        self.assertEqual(summary['scoring_fraction'], 0.)
        computation[0]['trajectory_active'] = False
        summary, rows = score_tracking(reference, truth, computation, 1.)
        self.assertEqual(summary['coverage_fraction'], 0.)
        self.assertEqual(summary['active_reference_samples'], 0)

    def test_manual_conditioning_uses_existing_sitl_mapping(self):
        bridge = dict(motor_idle=.055, hover_throttle=.38, min_check=1050)
        hover = manual_throttle(3., 0., 3., bridge)
        self.assertEqual(hover, round(1050 + 950 * .38))
        self.assertGreater(manual_throttle(0., 0., 3., bridge), hover)
        self.assertLess(manual_throttle(4., 0., 3., bridge), hover)
        self.assertEqual(json.loads(json.dumps(json_ready({'solver': float('nan')}), allow_nan=False)), {'solver': None})

    def test_nonfinite_truth_is_not_counted_as_successful_tracking_coverage(self):
        truth = [dict(stamp_ns=t, position=[float('nan'), 0., 0.], velocity=[0., 0., 0.]) for t in (0, 10_000_000)]
        reference = [dict(stamp_ns=5_000_000, position=[0., 0., 0.], velocity=[0., 0., 0.])]
        computation = [dict(stamp_ns=5_000_000, trajectory_active=True, reference_elapsed=.005)]
        summary, rows = score_tracking(reference, truth, computation, 1.)
        self.assertFalse(rows)
        self.assertEqual(summary['scoring_fraction'], 0.)
        self.assertEqual(summary['position_error_m']['count'], 0)

    def test_prearm_holds_low_throttle_before_manual_climb(self):
        bridge = dict(motor_idle=.055, hover_throttle=.38, min_check=1050)
        low = receiver_inputs('disarmed', .25, 0., 3., bridge)
        self.assertFalse(low['armed'])
        prearm = receiver_inputs('prearm', .25, 0., 3., bridge)
        self.assertTrue(prearm['armed'])
        self.assertFalse(prearm['auto_switch'])
        self.assertEqual(prearm['manual_aetr'][2], 1000)
        manual = receiver_inputs('manual', .25, 0., 3., bridge)
        self.assertTrue(manual['armed'])
        self.assertFalse(manual['auto_switch'])
        self.assertGreater(manual['manual_aetr'][2], 1050)

    def test_ros_numpy_rc_values_are_serializable_in_raw_csv(self):
        row = dict(sequence=np.uint64(5), manual_aetr=[np.uint16(1500), np.uint16(1500), np.uint16(1000), np.uint16(1500)])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'authority.csv'
            write_csv(path, [row])
            with path.open() as stream:
                stored = next(csv.DictReader(stream))
        self.assertEqual(stored['sequence'], '5')
        self.assertEqual(json.loads(stored['manual_aetr']), [1500, 1500, 1000, 1500])

    def test_raw_csv_failure_keeps_the_original_flight_result(self):
        summary = dict(passed=False, first_failure='AUTO reference revoked', acceptance_failures=['Did not complete'])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            with patch('test_sitl_trajectories.write_csv', side_effect=OSError('write failed')):
                result = save_results(path, summary, {'authority': [{'sequence': 1}]})
            stored = json.loads((path / 'result.json').read_text())
        self.assertEqual(stored, result)
        self.assertEqual(stored['first_failure'], 'AUTO reference revoked')
        self.assertFalse(stored['passed'])
        self.assertIn('authority: OSError: write failed', stored['logging_errors'])

    def test_reference_association_uses_measured_state_age_not_a_fitted_offset(self):
        truth = [dict(stamp_ns=t, position=[t * 1e-9, 0., 0.], velocity=[1., 0., 0.]) for t in (0, 10_000_000, 20_000_000)]
        reference = [dict(stamp_ns=9_000_000, position=[.009, 0., 0.], velocity=[1., 0., 0.])]
        computation = [dict(stamp_ns=10_000_000, trajectory_active=True, reference_elapsed=.01, state_age=0.)]
        summary, rows = score_tracking(reference, truth, computation, .02)
        self.assertFalse(rows)  # An unexplained 1 ms mismatch must not be associated.
        computation[0]['state_age'] = .001
        summary, rows = score_tracking(reference, truth, computation, .02)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['stamp_ns'], 9_000_000)
        self.assertAlmostEqual(rows[0]['reference_elapsed'], .009)
        self.assertEqual(summary['association_error_max_ns'], 0)
        self.assertAlmostEqual(summary['position_error_m']['rms'], 0.)
        reference[0]['stamp_ns'] += 1
        summary, rows = score_tracking(reference, truth, computation, .02)
        self.assertEqual(len(rows), 1)
        self.assertEqual(summary['association_error_max_ns'], 1)
        reference[0]['stamp_ns'] += 1_000_000
        summary, rows = score_tracking(reference, truth, computation, .02)
        self.assertFalse(rows)  # Evidence must match; it is not a broad nearest-cycle join.

    def test_offline_rescore_preserves_original_result_raw_data_and_runtime_failure(self):
        truth = [dict(stamp_ns=t, position=[t * 1e-9, 0., 0.], velocity=[1., 0., 0.]) for t in (0, 10_000_000, 20_000_000)]
        reference = [dict(stamp_ns=9_000_000, position=[.009, 0., 0.], velocity=[1., 0., 0.])]
        computations = [dict(stamp_ns=10_000_000, trajectory_active=True, reference_elapsed=.01, state_age=.001)]
        original = dict(dataset=dict(path='fake.csv', duration_seconds=.02), first_failure='original solver timeout',
                        trajectory_finished=False, final_output_inactive=True, controller_matches=True,
                        rmse_limit_m=1., passed=False, counts={'reference': 1})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            original_bytes = json.dumps(original).encode()
            (path / 'result.json').write_bytes(original_bytes)
            for name, rows in (('reference', reference), ('ground_truth', truth),
                               ('fused_state', truth), ('computation_status', computations)):
                write_csv(path / (name + '.csv'), rows)
            raw_before = (path / 'reference.csv').read_bytes()
            rescored = rescore_directory(path)
            rescore_directory(path)  # A later rescore must not replace the original backup.
            self.assertEqual((path / 'result.before_rescore.json').read_bytes(), original_bytes)
            self.assertEqual((path / 'reference.csv').read_bytes(), raw_before)
        self.assertEqual(rescored['first_failure'], original['first_failure'])
        self.assertFalse(rescored['passed'])
        self.assertEqual(rescored['counts'], original['counts'])
        self.assertEqual(rescored['tracking']['scored_samples'], 1)


if __name__ == '__main__':
    unittest.main()
