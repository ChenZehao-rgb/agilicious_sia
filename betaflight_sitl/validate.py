#!/usr/bin/env python3
"""Fly every reference trajectory through the SITL loop and score the tracking.

Each run executes the normal takeoff, hands over to the CSV trajectory and
records a reference-versus-state log.  The score covers only the samples where
the sampled trajectory is the active reference, so takeoff and the trailing
hover do not flatter the numbers.
"""

from __future__ import annotations

import argparse
import csv
import math
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_DIR = REPO_ROOT / "miscellaneous" / "datasets" / "ref_trajs"


def log(message: str) -> None:
    print(f"[VALIDATE] {message}", flush=True)


def trajectory_duration(path: Path) -> float:
    with path.open() as handle:
        rows = list(csv.DictReader(handle))
    return float(rows[-1]["t"]) - float(rows[0]["t"])


def norm(row: Dict[str, str], prefix: str, suffix: str = "") -> float:
    return math.sqrt(
        sum(float(row[f"{prefix}{axis}{suffix}"]) ** 2 for axis in ("x", "y", "z"))
    )


def score(log_path: Path) -> Optional[dict]:
    """Summarise tracking over the trajectory-following part of a flight."""
    with log_path.open() as handle:
        tracked = [
            row
            for row in csv.DictReader(handle)
            if row["has_reference"] == "1" and row["in_trajectory"] == "1"
        ]
    if not tracked:
        return None

    errors = [
        math.sqrt(
            sum(
                (float(row[f"ref_p_{axis}"]) - float(row[f"p_{axis}"])) ** 2
                for axis in ("x", "y", "z")
            )
        )
        for row in tracked
    ]
    speeds = [norm(row, "v_") for row in tracked]
    reference_speeds = [norm(row, "ref_v_") for row in tracked]
    thrusts = [float(row["cmd_thrust"]) for row in tracked]
    saturated = sum(1 for row in tracked if int(row["rc_t"]) >= 1999)
    return {
        "samples": len(tracked),
        "seconds": float(tracked[-1]["t"]) - float(tracked[0]["t"]),
        "rmse": math.sqrt(sum(error**2 for error in errors) / len(errors)),
        "max_error": max(errors),
        "max_speed": max(speeds),
        "max_reference_speed": max(reference_speeds),
        "max_thrust": max(thrusts),
        "throttle_saturated_pct": 100.0 * saturated / len(tracked),
        "min_altitude": min(float(row["p_z"]) for row in tracked),
    }


def wait_for_free_ports(timeout: float = 60.0) -> None:
    """Block until the previous run's simulator has released its sockets.

    run.py refuses to start while Betaflight's CLI port or the SITL UDP ports
    are still bound, and the kernel keeps them for a moment after the process
    tree exits.
    """
    deadline = time.monotonic() + timeout
    while True:
        probes = []
        try:
            for kind, port in (
                (socket.SOCK_STREAM, 5761),
                (socket.SOCK_DGRAM, 9002),
                (socket.SOCK_DGRAM, 9003),
                (socket.SOCK_DGRAM, 9004),
            ):
                probe = socket.socket(socket.AF_INET, kind)
                probes.append(probe)
                probe.bind(("127.0.0.1", port))
            return
        except OSError:
            if time.monotonic() > deadline:
                raise RuntimeError("simulator ports never became free")
            time.sleep(1.0)
        finally:
            for probe in probes:
                probe.close()


def run_one(
    trajectory: Path, log_path: Path, extra: List[str], margin: float
) -> Optional[dict]:
    # Disarmed hold, pre-arm, takeoff, the trajectory itself and a little slack.
    duration = 6.0 + 2.0 + 2.0 + trajectory_duration(trajectory) + margin
    command = [
        sys.executable,
        str(REPO_ROOT / "betaflight_sitl" / "run.py"),
        "--arm",
        "--no-build",
        "--trajectory",
        str(trajectory),
        "--duration",
        f"{duration:.1f}",
        "--log",
        str(log_path),
        *extra,
    ]
    wait_for_free_ports()
    log(f"{trajectory.name}: flying {duration:.0f} s")
    completed = subprocess.run(
        command,
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=duration + 180.0,
    )
    console = log_path.with_suffix(".console.log")
    console.write_text(completed.stdout, encoding="utf-8")
    if completed.returncode != 0:
        log(f"{trajectory.name}: run.py exited {completed.returncode}, see {console}")
        return None
    return score(log_path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "trajectories",
        nargs="*",
        type=Path,
        help="trajectory CSVs (default: every file under miscellaneous/datasets)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "build" / "betaflight_sitl" / "validation",
    )
    parser.add_argument("--margin", type=float, default=4.0)
    parser.add_argument(
        "--rmse-limit",
        type=float,
        default=1.0,
        help="fail a trajectory whose position RMSE exceeds this (m)",
    )
    parser.add_argument("--controller", choices=("mpc", "geo"), default="mpc")
    parser.add_argument(
        "--rtk-msp",
        action="store_true",
        help=(
            "score the realistic state pipeline (RTK + companion IMU) instead "
            "of the zero-latency ground-truth baseline"
        ),
    )
    args = parser.parse_args()

    trajectories = args.trajectories or sorted(TRAJECTORY_DIR.rglob("*.csv"))
    if not trajectories:
        parser.error(f"no trajectories found under {TRAJECTORY_DIR}")
    args.output.mkdir(parents=True, exist_ok=True)

    results: Dict[str, Optional[dict]] = {}
    for trajectory in trajectories:
        trajectory = trajectory.resolve()
        results[trajectory.name] = run_one(
            trajectory,
            args.output / f"{trajectory.stem}.csv",
            ["--controller", args.controller]
            + (["--rtk-msp"] if args.rtk_msp else []),
            args.margin,
        )

    print()
    header = (
        f"{'trajectory':<16}{'secs':>7}{'RMSE':>8}{'max err':>9}"
        f"{'v max':>8}{'v ref':>8}{'a max':>8}{'sat %':>7}{'z min':>7}"
    )
    print(header)
    print("-" * len(header))
    failures = []
    for name, result in results.items():
        if result is None:
            print(f"{name:<16}{'FAILED — see console log':>54}")
            failures.append(name)
            continue
        print(
            f"{name:<16}{result['seconds']:>7.1f}{result['rmse']:>8.3f}"
            f"{result['max_error']:>9.3f}{result['max_speed']:>8.2f}"
            f"{result['max_reference_speed']:>8.2f}{result['max_thrust']:>8.1f}"
            f"{result['throttle_saturated_pct']:>7.1f}{result['min_altitude']:>7.2f}"
        )
        if result["rmse"] > args.rmse_limit:
            failures.append(name)
    print()
    print("RMSE and max err in m, v in m/s, a in m/s^2, z min in m.")
    if failures:
        log(f"{len(failures)} trajectory/ies outside the limit: {', '.join(failures)}")
        return 1
    log("all trajectories tracked within the limit")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
