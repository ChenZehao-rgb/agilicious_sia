#!/usr/bin/env python3
"""Generate rest-to-rest forward helices under an ideal static rigid-body model.

Requires numpy and PyYAML. The radius is a numerical minimum within the
documented path/time family, NOT a Gazebo aerodynamic feasibility certificate.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from numpy.polynomial import Polynomial
import yaml

ROOT = Path(__file__).resolve().parents[1]
HEADER = ("t,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,w_x,w_y,w_z,"
          "a_lin_x,a_lin_y,a_lin_z,a_rot_x,a_rot_y,a_rot_z,u_1,u_2,u_3,u_4,"
          "jerk_x,jerk_y,jerk_z,snap_x,snap_y,snap_z")
G = 9.80665
# Integral of 630*u^4*(1-u)^4: derivatives 1..4 vanish at both ends.
PROGRESS = Polynomial([0, 0, 0, 0, 0, 126, -420, 540, -315, 70])


def quaternion(rotation):
    """Continuous wxyz unit quaternions, including inverted attitudes."""
    # Davenport's symmetric matrix, xyzw convention.
    r = rotation
    k = np.empty((len(r), 4, 4))
    k[:, :3, :3] = r + r.transpose(0, 2, 1)
    trace = np.trace(r, axis1=1, axis2=2)
    k[:, :3, :3] -= trace[:, None, None] * np.eye(3)
    k[:, :3, 3] = np.stack((r[:, 2, 1] - r[:, 1, 2],
                             r[:, 0, 2] - r[:, 2, 0],
                             r[:, 1, 0] - r[:, 0, 1]), axis=1)
    k[:, 3, :3] = k[:, :3, 3]
    k[:, 3, 3] = trace
    q = np.linalg.eigh(k)[1][:, :, -1][:, [3, 0, 1, 2]]
    flips = np.where(np.sum(q[1:] * q[:-1], axis=1) < 0, -1, 1)
    q[1:] *= np.cumprod(flips)[:, None]
    if q[0, 0] < 0:
        q *= -1
    return q


def trajectory(radius, speed, distance, turns, quad, samples):
    angle = 2 * math.pi * turns
    length = math.hypot(distance, angle * radius)
    duration = length * PROGRESS.deriv()(0.5) / speed
    t = np.linspace(0, duration, samples)
    u = t / duration
    s = PROGRESS(u)
    d = [None] + [PROGRESS.deriv(i)(u) / duration**i for i in range(1, 5)]
    for v in d[1:]:
        v[[0, -1]] = 0
    phase = angle * s
    # p(s) = [distance*s, R*sin(angle*s), R*(1-cos(angle*s))].
    p = np.stack((distance * s, radius * np.sin(phase),
                  radius * (1 - np.cos(phase))), axis=1)
    ps = []
    for i in range(1, 5):
        ps.append(np.stack((np.full_like(s, distance if i == 1 else 0),
                            radius * angle**i * np.sin(phase + i * math.pi / 2),
                            -radius * angle**i * np.cos(phase + i * math.pi / 2)), axis=1))
    d1, d2, d3, d4 = [v[:, None] for v in d[1:]]
    v = ps[0] * d1
    a = ps[1] * d1**2 + ps[0] * d2
    j = ps[2] * d1**3 + 3 * ps[1] * d1 * d2 + ps[0] * d3
    snap = (ps[3] * d1**4 + 6 * ps[2] * d1**2 * d2
            + ps[1] * (3 * d2**2 + 4 * d1 * d3) + ps[0] * d4)
    force = a + [0, 0, G]
    norm = np.linalg.norm(force, axis=1)
    z = force / norm[:, None]
    y = np.cross(z, [1, 0, 0])
    ynorm = np.linalg.norm(y, axis=1)
    if np.min(norm) < 1e-6 or np.min(ynorm) < 1e-6:
        raise ValueError("singular thrust/heading frame")
    y /= ynorm[:, None]
    rotation = np.stack((np.cross(y, z), y, z), axis=2)
    rdot = np.gradient(rotation, t, axis=0, edge_order=2)
    skew = rotation.transpose(0, 2, 1) @ rdot
    omega = np.stack((skew[:, 2, 1] - skew[:, 1, 2],
                      skew[:, 0, 2] - skew[:, 2, 0],
                      skew[:, 1, 0] - skew[:, 0, 1]), axis=1) / 2
    omega[[0, -1]] = 0
    alpha = np.gradient(omega, t, axis=0, edge_order=2)
    alpha[[0, -1]] = 0
    inertia = np.asarray(quad['inertia'])
    torque = alpha * inertia + np.cross(omega, omega * inertia)
    arms = np.array([quad['tbm_' + name] for name in ('fr', 'bl', 'br', 'fl')])
    allocation = np.stack((np.ones(4), arms[:, 1], -arms[:, 0],
                           quad['kappa'] * np.array([-1, -1, 1, 1])))
    wrench = np.column_stack((quad['mass'] * norm, torque))
    thrust = wrench @ np.linalg.inv(allocation).T
    return t, p, rotation, v, omega, a, alpha, thrust, j, snap


def violation(data, quad):
    thrust, omega = data[7], data[4]
    return max(float(np.max(thrust) / quad['thrust_max'] - 1),
               float((quad['thrust_min'] - np.min(thrust)) / quad['thrust_max']),
               float(np.max(np.abs(omega) / quad['omega_max']) - 1))


def solve(speed, args, quad):
    # The angle floor excludes arbitrarily thin helices that approach a line.
    lower = args.distance * math.tan(math.radians(args.min_helix_angle)) / (2 * math.pi * args.turns)
    def evaluate(radius):
        return violation(trajectory(radius, speed, args.distance, args.turns,
                                    quad, 12001), quad)
    upper = lower
    for _ in range(250):
        if evaluate(upper) <= 0:
            break
        lower, upper = upper, upper * 1.08
    else:
        raise ValueError('No feasible radius found in the search interval')
    if upper != lower:
        for _ in range(35):
            middle = (lower + upper) / 2
            if evaluate(middle) <= 0:
                upper = middle
            else:
                lower = middle
    # Sub-millimetre numerical cushion; resample more finely for export/QA.
    radius = upper + 0.0001
    duration = math.hypot(args.distance, 2 * math.pi * args.turns * radius) * PROGRESS.deriv()(0.5) / speed
    samples = max(24001, 2 * math.ceil(duration / args.dt / 2) + 1)
    data = trajectory(radius, speed, args.distance, args.turns, quad, samples)
    if violation(data, quad) > 1e-5:
        raise ValueError('Fine-grid verification failed; refine the radius search')
    t, p, r, v, w, a, alpha, thrust, j, snap = data
    q = quaternion(r)
    matrix = np.column_stack((t, p, q, v, w, a, alpha, thrust, j, snap))
    assert np.isfinite(matrix).all()
    assert np.allclose(p[-1] - p[0], [args.distance, 0, 0], atol=1e-8)
    assert np.isclose(np.linalg.norm(v, axis=1).max(), speed, atol=1e-8)
    stride = max(1, int(args.dt / (t[1] - t[0])))
    indices = np.unique(np.r_[np.arange(0, samples, stride), samples // 2, samples - 1])
    matrix = matrix[indices]
    path = args.output / f'HELIX_FWD{args.distance:g}_{speed:g}mps.csv'
    np.savetxt(path, matrix, delimiter=',', header=HEADER, comments='', fmt='%.12g')
    return dict(file=path.name, max_speed_mps=speed, forward_distance_m=args.distance,
                turns=args.turns, diameter_m=2 * radius, duration_s=float(t[-1]),
                samples=len(matrix), max_dt_s=float(np.diff(matrix[:, 0]).max()),
                verification_samples=samples,
                min_rotor_thrust_N=float(thrust.min()), max_rotor_thrust_N=float(thrust.max()),
                max_abs_body_rate_rad_s=np.abs(w).max(axis=0).tolist(),
                model='ideal rigid body, static rotor limits; no aerodynamic drag or motor lag',
                diameter_definition='first feasible radius bracket, 8% scan + bisection, 12001 samples; verified at >=24001 samples; 0.1 mm radius cushion',
                min_helix_angle_deg=args.min_helix_angle,
                gazebo_tracking_validated=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speeds', type=float, nargs='+', default=[20, 50])
    parser.add_argument('--distance', type=float, default=20)
    parser.add_argument('--turns', type=int, default=3)
    parser.add_argument('--min-helix-angle', type=float, default=45)
    parser.add_argument('--dt', type=float, default=0.005, help='maximum CSV sample interval (s)')
    parser.add_argument('--quad', type=Path, default=ROOT / 'agilib/params/quads/betaloop_iris.yaml')
    parser.add_argument('--output', type=Path, default=ROOT / 'miscellaneous/datasets/ref_trajs/open_source')
    args = parser.parse_args()
    if (any(not math.isfinite(x) or x <= 0 for x in [*args.speeds, args.distance, args.dt])
            or args.turns <= 0 or not 0 < args.min_helix_angle < 90):
        parser.error('speeds/distance/dt/turns must be positive; angle must be in (0, 90) degrees')
    quad = yaml.safe_load(args.quad.read_text())
    args.output.mkdir(parents=True, exist_ok=True)
    for speed in args.speeds:
        result = solve(speed, args, quad)
        result['quad_parameters'] = quad
        metadata = args.output / (Path(result['file']).stem + '.json')
        metadata.write_text(json.dumps(result, indent=2) + '\n')
        print(f"{result['file']}: D={result['diameter_m']:.6f} m, "
              f"T={result['duration_s']:.6f} s, peak={speed:g} m/s")


if __name__ == '__main__':
    main()