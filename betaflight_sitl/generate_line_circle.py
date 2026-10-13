#!/usr/bin/env python3
"""Generate rest-to-rest lines and horizontal circles with a maximum speed.

Requires numpy and PyYAML, and uses the same ideal rigid-body constraints as
generate_helix.py. Fixed geometry is preserved; duration is increased until
rotor thrust and body-rate limits pass numerical verification. This does not
certify Gazebo tracking, aerodynamic feasibility, or motor-lag feasibility.

Examples:
  python3 betaflight_sitl/generate_line_circle.py line --length 20 --max-speed 10
  python3 betaflight_sitl/generate_line_circle.py line --start 0 0 2 --end 10 5 2 --round-trip
  python3 betaflight_sitl/generate_line_circle.py circle --radius 5 --center 0 0 2 --max-speed 8

For compatibility with the existing trajectory reader, CSVs have the legacy
14-field header and 30 numeric columns, exactly as generate_helix.py does.
JSON sidecars include the complete ordered column names and constraint checks.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

from generate_helix import G, HEADER, PROGRESS, ROOT, THRUST_MAX_FRACTION, quaternion, violation

COLUMNS = (HEADER + ",a_lin_x,a_lin_y,a_lin_z,a_rot_x,a_rot_y,a_rot_z,"
           "u_1,u_2,u_3,u_4,jerk_x,jerk_y,jerk_z,snap_x,snap_y,snap_z").split(",")


def path_length(args):
    if args.kind == "line":
        return float(np.linalg.norm(np.asarray(args.end) - args.start))
    return 2 * math.pi * args.radius * args.turns


def trajectory(args, duration, quad, samples):
    """Evaluate one leg/lap sequence; derivatives use the chain rule through snap."""
    t = np.linspace(0, duration, samples)
    u = t / duration
    s = PROGRESS(u)
    s[[0, -1]] = [0, 1]
    derivatives = [PROGRESS.deriv(i)(u) / duration**i for i in range(1, 5)]
    for derivative in derivatives:
        derivative[[0, -1]] = 0
    d1, d2, d3, d4 = [value[:, None] for value in derivatives]

    if args.kind == "line":
        delta = np.asarray(args.end) - args.start
        p = np.asarray(args.start) + s[:, None] * delta
        ps = [np.broadcast_to(delta, p.shape)] + [np.zeros_like(p) for _ in range(3)]
    else:
        angle = (-1 if args.clockwise else 1) * 2 * math.pi * args.turns
        phase = angle * s
        p = np.asarray(args.center) + args.radius * np.column_stack(
            (np.cos(phase), np.sin(phase), np.zeros_like(s)))
        p[-1] = p[0]
        ps = [args.radius * angle**i * np.column_stack(
            (np.cos(phase + i * math.pi / 2), np.sin(phase + i * math.pi / 2), np.zeros_like(s)))
            for i in range(1, 5)]

    v = ps[0] * d1
    a = ps[1] * d1**2 + ps[0] * d2
    jerk = ps[2] * d1**3 + 3 * ps[1] * d1 * d2 + ps[0] * d3
    snap = (ps[3] * d1**4 + 6 * ps[2] * d1**2 * d2
            + ps[1] * (3 * d2**2 + 4 * d1 * d3) + ps[0] * d4)

    # Same fixed world-X heading and static rotor allocation as the helix.
    force = a + [0, 0, G]
    norm = np.linalg.norm(force, axis=1)
    if np.min(norm) < 1e-6:
        raise ValueError("singular thrust frame")
    z = force / norm[:, None]
    y = np.cross(z, [1, 0, 0])
    ynorm = np.linalg.norm(y, axis=1)
    if np.min(ynorm) < 1e-6:
        raise ValueError("singular heading frame")
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
    inertia = np.asarray(quad["inertia"])
    torque = alpha * inertia + np.cross(omega, omega * inertia)
    arms = np.array([quad["tbm_" + name] for name in ("fr", "bl", "br", "fl")])
    allocation = np.stack((np.ones(4), arms[:, 1], -arms[:, 0],
                           quad["kappa"] * np.array([-1, -1, 1, 1])))
    thrust = np.column_stack((quad["mass"] * norm, torque)) @ np.linalg.inv(allocation).T
    return t, p, rotation, v, omega, a, alpha, thrust, jerk, snap


def evaluate(args, duration, quad, samples):
    """Check both directions because asymmetric rotor arms can affect feasibility."""
    data = trajectory(args, duration, quad, samples)
    legs = [data]
    if args.kind == "line" and args.round_trip:
        reverse = argparse.Namespace(**vars(args))
        reverse.start, reverse.end = args.end, args.start
        legs.append(trajectory(reverse, duration, quad, samples))
    if not all(np.isfinite(array).all() for leg in legs for array in leg):
        raise ValueError("non-finite trajectory")
    return max(violation(leg, quad) for leg in legs), legs


def solve(args, quad, speed):
    # The progress derivative peaks at u=0.5. Speed is an upper bound, not a
    # requirement to violate fixed geometry or the vehicle's static limits.
    duration = path_length(args) * PROGRESS.deriv()(0.5) / speed
    for _ in range(250):
        try:
            error, _ = evaluate(args, duration, quad, 12001)
            if error <= 0:
                samples = max(24001, 2 * math.ceil(duration / args.dt / 2) + 1)
                error, legs = evaluate(args, duration, quad, samples)
                if error <= 0:
                    break
        except ValueError:
            # Free-fall / heading singularities can disappear at longer times.
            pass
        duration *= 1.08
    else:
        raise ValueError("No feasible duration found after 250 time-scaling steps")

    matrices = []
    for index, leg in enumerate(legs):
        t, p, rotation, v, omega, a, alpha, thrust, jerk, snap = leg
        matrix = np.column_stack((t + index * duration, p, quaternion(rotation),
                                  v, omega, a, alpha, thrust, jerk, snap))
        stride = max(1, int(args.dt / (t[1] - t[0])))
        indices = np.unique(np.r_[np.arange(0, samples, stride), samples // 2, samples - 1])
        # The two legs share the stationary turnaround sample.
        matrices.append(matrix[indices] if index == 0 else matrix[indices[1:]])
    matrix = np.concatenate(matrices)
    thrust = np.concatenate([leg[7] for leg in legs])
    omega = np.concatenate([leg[4] for leg in legs])
    actual_speed = float(max(np.linalg.norm(leg[3], axis=1).max() for leg in legs))
    if actual_speed > speed * (1 + 1e-10):
        raise ValueError("Maximum speed verification failed")

    if args.kind == "line":
        mode = "ROUND_TRIP" if args.round_trip else "ONE_WAY"
        name = f"LINE_{mode}_L{path_length(args):g}_{speed:g}mps"
        geometry = dict(start_m=args.start, end_m=args.end, length_m=path_length(args),
                        round_trip=args.round_trip)
    else:
        direction = "CW" if args.clockwise else "CCW"
        name = f"CIRCLE_R{args.radius:g}_N{args.turns}_{direction}_{speed:g}mps"
        geometry = dict(center_m=args.center, radius_m=args.radius, turns=args.turns,
                        clockwise=args.clockwise, start_m=matrix[0, 1:4].tolist())
    path = args.output / (name + ".csv")
    metadata = dict(file=path.name, kind=args.kind, **geometry,
                    requested_max_speed_mps=speed, max_speed_mps=actual_speed,
                    speed_limited=bool(duration > path_length(args) * PROGRESS.deriv()(0.5) / speed),
                    duration_s=float(matrix[-1, 0]), samples=len(matrix),
                    max_dt_s=float(np.diff(matrix[:, 0]).max()),
                    verification_samples_per_leg=samples,
                    thrust_max_fraction=THRUST_MAX_FRACTION,
                    rotor_thrust_limit_N=THRUST_MAX_FRACTION * quad["thrust_max"],
                    min_rotor_thrust_N=float(thrust.min()), max_rotor_thrust_N=float(thrust.max()),
                    max_abs_body_rate_rad_s=np.abs(omega).max(axis=0).tolist(),
                    csv_columns=COLUMNS, quad_parameters=quad,
                    model="ideal rigid body, static rotor limits; no aerodynamic drag or motor lag",
                    duration_search="8% time increments; 12001 samples, verified at >=24001 samples per leg",
                    gazebo_tracking_validated=False)
    np.savetxt(path, matrix, delimiter=",", header=HEADER, comments="", fmt="%.12g")
    path.with_suffix(".json").write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n")
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--max-speed", "--speeds", dest="speeds", type=float, nargs="+", default=[10],
                        help="maximum speed(s) in m/s; duration may increase to meet vehicle limits")
    common.add_argument("--dt", type=float, default=0.005, help="maximum CSV sample interval (s)")
    common.add_argument("--quad", type=Path, default=ROOT / "agilib/params/quads/betaloop_iris.yaml")
    common.add_argument("--output", type=Path, default=ROOT / "miscellaneous/datasets/ref_trajs/open_source")
    subparsers = parser.add_subparsers(dest="kind", required=True)
    line = subparsers.add_parser("line", parents=[common], help="straight line, stopping at each endpoint")
    line.add_argument("--start", type=float, nargs=3, default=[0, 0, 0], metavar=("X", "Y", "Z"))
    endpoint = line.add_mutually_exclusive_group()
    endpoint.add_argument("--end", type=float, nargs=3, metavar=("X", "Y", "Z"), help="absolute target position (m)")
    endpoint.add_argument("--length", type=float, help="distance along world +X from start (default: 20 m)")
    line.add_argument("--round-trip", action="store_true", help="stop at target, then return to start")
    circle = subparsers.add_parser("circle", parents=[common], help="horizontal circle, starting at center + [radius,0,0]")
    circle.add_argument("--radius", type=float, default=5)
    circle.add_argument("--center", type=float, nargs=3, default=[0, 0, 0], metavar=("X", "Y", "Z"))
    circle.add_argument("--turns", type=int, default=1, help="number of complete revolutions")
    circle.add_argument("--clockwise", action="store_true", help="clockwise when viewed from +Z")
    args = parser.parse_args()

    try:
        values = [*args.speeds, args.dt]
        if any(not math.isfinite(value) or value <= 0 for value in values):
            raise ValueError("maximum speeds and dt must be finite and positive")
        if args.kind == "line":
            if args.end is None:
                length = args.length if args.length is not None else 20
                if not math.isfinite(length) or length <= 0:
                    raise ValueError("length must be finite and positive")
                args.end = (np.asarray(args.start) + [length, 0, 0]).tolist()
            coordinates = [*args.start, *args.end]
        else:
            if not math.isfinite(args.radius) or args.radius <= 0 or args.turns <= 0:
                raise ValueError("radius must be finite and positive; turns must be positive")
            coordinates = args.center
        if not all(math.isfinite(value) for value in coordinates):
            raise ValueError("coordinates must be finite")
        if not math.isfinite(path_length(args)) or path_length(args) <= 0:
            raise ValueError("path length must be finite and positive; line endpoints must differ")

        quad = yaml.safe_load(args.quad.read_text())
        for field in ("mass", "thrust_max", "kappa"):
            if not math.isfinite(quad[field]) or quad[field] <= 0:
                raise ValueError(f"quad {field} must be finite and positive")
        for field in ("inertia", "omega_max"):
            value = np.asarray(quad[field])
            if value.shape != (3,) or not np.isfinite(value).all() or np.any(value <= 0):
                raise ValueError(f"quad {field} must contain three finite positive values")
        for field in ("tbm_fr", "tbm_bl", "tbm_br", "tbm_fl"):
            value = np.asarray(quad[field])
            if value.shape != (3,) or not np.isfinite(value).all():
                raise ValueError(f"quad {field} must contain three finite coordinates")
        thrust_limit = THRUST_MAX_FRACTION * quad["thrust_max"]
        if not math.isfinite(quad["thrust_min"]) or quad["thrust_min"] < 0:
            raise ValueError("quad thrust_min must be finite and nonnegative")
        hover = quad["mass"] * G / 4
        if not quad["thrust_min"] < hover < thrust_limit:
            raise ValueError("hover thrust must lie strictly between thrust_min and the 70% rotor thrust limit")
        args.output.mkdir(parents=True, exist_ok=True)
        for speed in args.speeds:
            result = solve(args, quad, speed)
            print(f"{result['file']}: T={result['duration_s']:.6f} s, "
                  f"peak={result['max_speed_mps']:.6f} m/s (limit={speed:g} m/s)")
    except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError, np.linalg.LinAlgError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
