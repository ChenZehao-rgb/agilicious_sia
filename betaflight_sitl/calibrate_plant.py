#!/usr/bin/env python3
"""Identify the Gazebo rotor plant seen by Betaflight's motor outputs.

Betaflight SITL sends four normalised motor commands to the Gazebo
``BetaflightPlugin`` and receives a 144-byte FDM packet back.  This tool speaks
that protocol directly, with no Betaflight in the loop, so the measured curve
describes the *plant* alone:

    motor command u  ->  rotor speed  ->  collective thrust / mass [m/s^2]

The model is free-flying, so between measurements a small altitude/climb-rate
controller returns it to the test altitude with near-zero vertical speed.  Each
sample is then taken from a short constant-command burst, where the body-frame
IMU specific force along +z is exactly the mass-normalised collective thrust.

The result is what ``betaflight_udp.yaml`` needs for its throttle mapping and
what the MPC needs for ``thrust_max``.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

from prepare_assets import prepare_assets

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BETALOOP_HOME = Path.home() / "betaloop"

MOTOR_PORT = 9002
FDM_PORT = 9003
FDM_FORMAT = "<18d"
FDM_SIZE = struct.calcsize(FDM_FORMAT)
SERVO_FORMAT = "<4f"

TEST_ALTITUDE = 25.0
RECOVERY_TIMEOUT = 25.0


def log(message: str) -> None:
    print(f"[CALIB] {message}", flush=True)


class Fdm:
    """Decoded FDM packet (patched 18-double Betaflight 2026.6 layout)."""

    __slots__ = ("t", "omega", "accel", "quat", "vel_enu", "lon_lat_alt")

    def __init__(self, raw: bytes) -> None:
        values = struct.unpack(FDM_FORMAT, raw)
        self.t = values[0]
        self.omega = values[1:4]
        self.accel = values[4:7]
        self.quat = values[7:11]
        self.vel_enu = values[11:14]
        self.lon_lat_alt = values[14:17]

    @property
    def altitude(self) -> float:
        return self.lon_lat_alt[2]

    @property
    def climb_rate(self) -> float:
        return self.vel_enu[2]


class PlantLink:
    """Lock-step UDP link to the Gazebo BetaflightPlugin."""

    def __init__(self) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", FDM_PORT))
        self.socket.settimeout(0.05)
        self.command = (0.0, 0.0, 0.0, 0.0)

    def close(self) -> None:
        self.socket.close()

    def set_command(self, motors: Tuple[float, float, float, float]) -> None:
        self.command = motors

    def _send(self) -> None:
        self.socket.sendto(
            struct.pack(SERVO_FORMAT, *self.command), ("127.0.0.1", MOTOR_PORT)
        )

    def poll(self, timeout: float) -> Optional[Fdm]:
        """Answer one FDM packet with the current command, in lock step."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._send()
            try:
                raw, _ = self.socket.recvfrom(4096)
            except socket.timeout:
                continue
            if len(raw) != FDM_SIZE:
                continue
            self._send()
            return Fdm(raw)
        return None

    def handshake(self, timeout: float = 30.0) -> Fdm:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            packet = self.poll(0.5)
            if packet is not None:
                return packet
        raise RuntimeError("Gazebo BetaflightPlugin never answered on UDP 9003")


def uniform(value: float) -> Tuple[float, float, float, float]:
    return (value, value, value, value)


def recover(link: PlantLink, hover_command: float) -> Fdm:
    """Return to the test altitude with a bounded climb rate."""
    deadline = time.monotonic() + RECOVERY_TIMEOUT
    last: Optional[Fdm] = None
    while time.monotonic() < deadline:
        packet = link.poll(1.0)
        if packet is None:
            raise RuntimeError("lost the Gazebo plant link during recovery")
        last = packet
        altitude_error = TEST_ALTITUDE - packet.altitude
        target_climb = max(-6.0, min(6.0, 0.8 * altitude_error))
        correction = 0.06 * (target_climb - packet.climb_rate)
        link.set_command(uniform(max(0.02, min(1.0, hover_command + correction))))
        if abs(altitude_error) < 1.0 and abs(packet.climb_rate) < 0.35:
            return packet
    if last is None:
        raise RuntimeError("lost the Gazebo plant link during recovery")
    log(
        f"WARNING: recovery timed out at alt={last.altitude:.1f} m "
        f"vz={last.climb_rate:.2f} m/s"
    )
    return last


def hold_climb(link: PlantLink, target_climb: float, trim: float) -> None:
    """Settle on a bounded climb rate before a burst, without overshooting.

    Each rotor is two gz-sim LiftDrag blade elements whose lift coefficient is
    ``cla * alpha`` with ``alpha = a0 - atan(climb / tangential_speed)``.  Axial
    inflow therefore biases thrust badly, and the only uncontaminated sample is
    the one taken as the vehicle passes through zero climb rate.  Entering a
    burst from a climb of the opposite sign guarantees such a crossing.
    """
    deadline = time.monotonic() + 12.0
    settled_since: Optional[float] = None
    while time.monotonic() < deadline:
        packet = link.poll(1.0)
        if packet is None:
            raise RuntimeError("lost the Gazebo plant link while preconditioning")
        error = target_climb - packet.climb_rate
        link.set_command(uniform(max(0.02, min(1.0, trim + 0.08 * error))))
        if abs(error) < 0.4:
            settled_since = settled_since or time.monotonic()
            if time.monotonic() - settled_since > 0.3:
                return
        else:
            settled_since = None
    log(f"WARNING: could not settle at climb {target_climb:+.1f} m/s")


def measure(link: PlantLink, command: float, window: float, inflow_band: float) -> dict:
    """Average the mass-normalised thrust over the zero-inflow crossing."""
    link.set_command(uniform(command))
    accels: List[float] = []
    climbs: List[float] = []
    all_accels: List[float] = []
    all_climbs: List[float] = []
    deadline = time.monotonic() + window
    crossed = False
    while time.monotonic() < deadline:
        packet = link.poll(1.0)
        if packet is None:
            raise RuntimeError("lost the Gazebo plant link while measuring")
        # The FDM frame is z-down and Gazebo reports kinematic acceleration, so
        # mass-normalised collective thrust is g minus the reported value.
        accel = 9.8066 - packet.accel[2]
        all_accels.append(accel)
        all_climbs.append(packet.climb_rate)
        if abs(packet.climb_rate) <= inflow_band:
            accels.append(accel)
            climbs.append(packet.climb_rate)
            crossed = True
        elif crossed:
            break

    if len(accels) >= 3:
        return {
            "command": command,
            "static_accel": sum(accels) / len(accels),
            "mean_climb": sum(climbs) / len(climbs),
            "samples": len(accels),
            "extrapolated": False,
        }

    # A command that sits right at the hover trim never crosses back through
    # zero climb rate.  Thrust is affine in the climb rate over a short burst,
    # so fit that line over whatever the burst covered and read it at zero.
    if len(all_accels) < 8:
        raise RuntimeError(f"motor command {command} produced too few samples")
    count = float(len(all_climbs))
    mean_climb = sum(all_climbs) / count
    mean_accel = sum(all_accels) / count
    variance = sum((value - mean_climb) ** 2 for value in all_climbs)
    covariance = sum(
        (climb - mean_climb) * (accel - mean_accel)
        for climb, accel in zip(all_climbs, all_accels)
    )
    slope = covariance / variance if variance > 1e-6 else 0.0
    return {
        "command": command,
        "static_accel": mean_accel - slope * mean_climb,
        "mean_climb": mean_climb,
        "samples": len(all_accels),
        "extrapolated": True,
    }


def find_hover_trim(link: PlantLink) -> float:
    """Bisect the motor command that holds altitude, i.e. thrust equal to weight.

    A trim search needs no acceleration reading at all: it only asks whether the
    vehicle rose or sank over a fixed interval, which makes it the one plant
    measurement that axial inflow cannot bias.
    """
    low, high = 0.05, 1.0
    for _ in range(12):
        middle = 0.5 * (low + high)
        recover(link, middle)
        link.set_command(uniform(middle))
        start = link.poll(1.0)
        last = start
        deadline = time.monotonic() + 1.2
        while time.monotonic() < deadline:
            last = link.poll(1.0)
            if last is None:
                raise RuntimeError("lost the Gazebo plant link during trim search")
        if start is None:
            raise RuntimeError("lost the Gazebo plant link during trim search")
        if last.altitude > start.altitude:
            high = middle
        else:
            low = middle
    return 0.5 * (low + high)


def start_gazebo(world: Path, env: dict) -> subprocess.Popen:
    log("starting headless Gazebo")
    return subprocess.Popen(
        ["gz", "sim", "-s", "-r", str(world)],
        env=env,
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def stop(process: Optional[subprocess.Popen]) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def fit_hover_command(samples: List[dict], gravity: float) -> Optional[float]:
    """Interpolate the motor command whose thrust exactly cancels gravity."""
    for previous, current in zip(samples, samples[1:]):
        if previous["static_accel"] <= gravity <= current["static_accel"]:
            span = current["static_accel"] - previous["static_accel"]
            if span <= 0.0:
                continue
            ratio = (gravity - previous["static_accel"]) / span
            return previous["command"] + ratio * (
                current["command"] - previous["command"]
            )
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--betaloop-home", type=Path, default=DEFAULT_BETALOOP_HOME)
    parser.add_argument(
        "--commands",
        type=str,
        default="0.15,0.25,0.35,0.45,0.55,0.65,0.75,0.85,0.95,1.0",
        help="comma separated normalised motor commands to sample",
    )
    parser.add_argument("--window", type=float, default=1.80)
    parser.add_argument(
        "--inflow-band",
        type=float,
        default=0.30,
        help="|climb rate| accepted as zero inflow while sampling a burst",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "build" / "betaflight_sitl" / "plant_calibration.json",
    )
    args = parser.parse_args()

    commands = sorted({float(value) for value in args.commands.split(",")})
    if not commands or commands[0] <= 0.0 or commands[-1] > 1.0:
        parser.error("--commands must be in (0, 1]")

    sys.path.insert(0, str(REPO_ROOT / "betaflight_sitl"))
    import configparser

    config = configparser.ConfigParser()
    config.read(args.betaloop_home.expanduser().resolve() / "config.txt")
    aeroloop = Path(config["Betaloop"]["AeroloopGazeboHome"]).expanduser().resolve()
    world = Path(config["Betaloop"]["World"]).expanduser()
    if not world.is_absolute():
        world = aeroloop / "worlds" / world

    runtime = REPO_ROOT / "build" / "betaflight_sitl" / "runtime" / "assets"
    overlay_world, _ = prepare_assets(aeroloop, world.resolve(), runtime)

    env = os.environ.copy()
    for key, path in (
        # The overlay model keeps the source mesh URIs, so Aeroloop's own model
        # directory has to stay resolvable next to the generated overlay.
        ("SDF_PATH", aeroloop / "models"),
        ("GZ_SIM_RESOURCE_PATH", aeroloop / "models"),
        ("GZ_SIM_RESOURCE_PATH", aeroloop / "worlds"),
        ("SDF_PATH", runtime / "models"),
        ("GZ_SIM_RESOURCE_PATH", runtime / "models"),
        ("GZ_SIM_RESOURCE_PATH", runtime / "worlds"),
        (
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            REPO_ROOT / "build" / "betaflight_sitl" / "plugin",
        ),
    ):
        env[key] = str(path) + (os.pathsep + env[key] if env.get(key) else "")

    gazebo = None
    link = None
    trim = float("nan")
    samples: List[dict] = []
    try:
        link = PlantLink()
        gazebo = start_gazebo(overlay_world, env)
        link.handshake()
        log("plant link established")

        # Climb to the test altitude with an open-loop guess, then refine.
        link.set_command(uniform(0.5))
        recover(link, 0.5)
        trim = find_hover_trim(link)
        log(f"hover motor command (trim search) = {trim:.4f}")

        for command in commands:
            recover(link, trim)
            # Enter each burst climbing the other way, so it sweeps through the
            # zero-inflow point where thrust is unbiased.
            hold_climb(link, -2.0 if command > trim else 2.0, trim)
            sample = measure(link, command, args.window, args.inflow_band)
            samples.append(sample)
            log(
                f"u={command:.2f}  thrust={sample['static_accel']:7.3f} m/s^2  "
                f"climb={sample['mean_climb']:+.3f} m/s  n={sample['samples']}"
                + ("  (fitted to zero inflow)" if sample["extrapolated"] else "")
            )
    finally:
        if link is not None:
            link.set_command(uniform(0.0))
            for _ in range(20):
                link.poll(0.02)
            link.close()
        stop(gazebo)

    samples.sort(key=lambda sample: sample["command"])
    hover_command = fit_hover_command(samples, 9.8066)
    result = {
        "samples": samples,
        "hover_motor_command": trim,
        "hover_motor_command_interpolated": hover_command,
        "max_accel": max((sample["static_accel"] for sample in samples), default=0.0),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    log(f"wrote {args.output}")
    log(f"hover motor command ~ {trim:.4f}")
    log(f"max mass-normalised thrust ~ {result['max_accel']:.2f} m/s^2")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
