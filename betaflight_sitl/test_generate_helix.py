"""Numerical consistency checks for the shipped helix references."""
import json
import unittest

import numpy as np

from betaflight_sitl.generate_helix import G, HEADER, ROOT, trajectory, violation


class HelixReferenceTest(unittest.TestCase):
    def test_references(self):
        directory = ROOT / 'miscellaneous/datasets/ref_trajs/open_source'
        for speed in (20, 50):
            with self.subTest(speed=speed):
                path = directory / f'HELIX_FWD20_{speed}mps.csv'
                meta = json.loads(path.with_suffix('.json').read_text())
                quad = meta['quad_parameters']
                with path.open() as stream:
                    self.assertEqual(stream.readline().strip(), HEADER)
                rows = np.loadtxt(path, delimiter=',', skiprows=1)
                self.assertEqual(rows.shape[1], 30)
                self.assertTrue(np.isfinite(rows).all())
                t, p, q, v, w, a = (rows[:, :1].ravel(), rows[:, 1:4], rows[:, 4:8],
                                    rows[:, 8:11], rows[:, 11:14], rows[:, 14:17])
                self.assertTrue((np.diff(t) > 0).all())
                self.assertLessEqual(np.diff(t).max(), 0.005 + 1e-9)
                np.testing.assert_allclose(p[-1] - p[0], [20, 0, 0], atol=1e-8)
                np.testing.assert_allclose(rows[[0, -1]][:, [8, 9, 10, 14, 15, 16, 24, 25, 26, 27, 28, 29]], 0, atol=1e-8)
                self.assertAlmostEqual(np.linalg.norm(v, axis=1).max(), speed, places=7)
                radius = meta['diameter_m'] / 2
                np.testing.assert_allclose(p[:, 1]**2 + (p[:, 2] - radius)**2,
                                           radius**2, rtol=1e-9)
                phase = np.unwrap(np.arctan2(p[:, 1], radius - p[:, 2]))
                self.assertAlmostEqual(phase[-1] - phase[0], 6 * np.pi, places=7)
                np.testing.assert_allclose(np.linalg.norm(q, axis=1), 1, atol=1e-10)
                self.assertTrue((np.sum(q[1:] * q[:-1], axis=1) > 0).all())
                qw, qx, qy, qz = q.T
                body_z = np.stack((2*(qx*qz + qw*qy), 2*(qy*qz - qw*qx),
                                   1 - 2*(qx*qx + qy*qy)), axis=1)
                thrust = rows[:, 20:24]
                np.testing.assert_allclose(body_z * thrust.sum(axis=1)[:, None] / quad['mass'],
                                           a + [0, 0, G], atol=2e-8)
                self.assertGreaterEqual(thrust.min(), quad['thrust_min'])
                self.assertLessEqual(thrust.max(), quad['thrust_max'])
                self.assertTrue((np.abs(w) <= quad['omega_max']).all())
                # Independent finite differences check CSV derivatives, including q/body rates.
                np.testing.assert_allclose(np.gradient(p, t, axis=0)[2:-2], v[2:-2], atol=0.015)
                np.testing.assert_allclose(np.gradient(v, t, axis=0)[2:-2], a[2:-2], atol=0.03)
                qdot = np.gradient(q, t, axis=0)
                derived_w = 2 * (q[:, :1] * qdot[:, 1:] - qdot[:, :1] * q[:, 1:]
                                 - np.cross(q[:, 1:], qdot[:, 1:]))
                np.testing.assert_allclose(derived_w[2:-2], w[2:-2], atol=0.005)
                # A millimetre smaller radius exceeds the static envelope;
                # doubling the verification grid keeps the shipped radius feasible.
                fine = trajectory(radius, speed, 20, 3, quad, 48001)
                self.assertLessEqual(violation(fine, quad), 1e-5)
                smaller = trajectory(radius - 0.001, speed, 20, 3, quad, 24001)
                self.assertGreater(violation(smaller, quad), 0)


if __name__ == '__main__':
    unittest.main()
