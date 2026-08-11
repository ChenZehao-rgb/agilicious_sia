#!/usr/bin/env python3
"""Build and run the isolated Agilicious -> Betaflight SITL closed loop."""

from __future__ import annotations

import argparse
import ast
import configparser
import math
import os
import re
import shutil
import signal
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, Optional

from prepare_assets import JOINT_TOPIC, ODOM_TOPIC, prepare_assets


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BETALOOP_HOME = Path.home() / "betaloop"


def log(message: str) -> None:
    print(f"[AGI-SITL] {message}", flush=True)


def interrupt_for_shutdown(_signum: int, _frame: object) -> None:
    """Route TERM through the same ordered cleanup path as Ctrl-C."""
    raise KeyboardInterrupt


def nonnegative_seconds(value: str) -> float:
    try:
        seconds = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"expected a number, got: {value}") from exc
    if not math.isfinite(seconds) or seconds < 0.0:
        raise argparse.ArgumentTypeError("must be a finite value >= 0")
    return seconds


def prepend_env(env: Dict[str, str], key: str, path: Path) -> None:
    old = env.get(key, "")
    env[key] = str(path) + (os.pathsep + old if old else "")


def load_betaloop_config(home: Path) -> tuple[Path, Path, Path]:
    config_path = home / "config.txt"
    parser = configparser.ConfigParser()
    if not parser.read(config_path) or "Betaloop" not in parser:
        raise RuntimeError(f"cannot read [Betaloop] from {config_path}")
    section = parser["Betaloop"]
    aeroloop = Path(section["AeroloopGazeboHome"]).expanduser().resolve()
    world = Path(section["World"]).expanduser()
    if not world.is_absolute():
        world = aeroloop / "worlds" / world
    elf = Path(section["BetaflightElf"]).expanduser().resolve()
    return aeroloop, world.resolve(), elf


def build_adapter(build_dir: Path) -> Path:
    binary = build_dir / "bin" / "agilicious_betaflight_sitl"
    configure = [
        "cmake",
        "-S",
        str(REPO_ROOT / "agilib"),
        "-B",
        str(build_dir),
        "-DCMAKE_BUILD_TYPE=Release",
        "-DBUILD_TESTS=OFF",
        "-DBUILD_BENCH=OFF",
        "-DBUILD_BETAFLIGHT_SITL=ON",
        "-DFETCH_ACADOS=OFF",
        "-DWARNINGS_AS_ERRORS=OFF",
    ]
    env = os.environ.copy()
    env["CCACHE_DISABLE"] = "1"
    log("configuring standalone adapter")
    subprocess.run(configure, check=True, cwd=REPO_ROOT, env=env)
    log("building standalone adapter")
    subprocess.run(
        ["cmake", "--build", str(build_dir), "--parallel"],
        check=True,
        cwd=REPO_ROOT,
        env=env,
    )
    if not binary.is_file():
        raise RuntimeError(f"build completed but adapter is missing: {binary}")
    return binary


def build_gazebo_plugin(aeroloop: Path, build_dir: Path) -> Path:
    source = aeroloop / "plugins" / "BetaflightPlugin.cc"
    if not source.is_file():
        raise FileNotFoundError(f"Aeroloop Betaflight plugin source not found: {source}")
    library = build_dir / "libAgiliciousBetaflightPlugin.so"
    log("configuring non-blocking Gazebo Betaflight plugin")
    subprocess.run(
        [
            "cmake",
            "-S",
            str(REPO_ROOT / "betaflight_sitl" / "plugin"),
            "-B",
            str(build_dir),
            "-DCMAKE_BUILD_TYPE=Release",
            f"-DAEROLOOP_PLUGIN_SOURCE={source}",
        ],
        check=True,
        cwd=REPO_ROOT,
    )
    log("building non-blocking Gazebo Betaflight plugin")
    subprocess.run(
        ["cmake", "--build", str(build_dir), "--parallel"],
        check=True,
        cwd=REPO_ROOT,
    )
    if not library.is_file():
        raise RuntimeError(f"Gazebo plugin build completed but is missing: {library}")
    return library


def wait_for_tcp(port: int, process: subprocess.Popen, timeout: float = 8.0) -> socket.socket:
    deadline = time.monotonic() + timeout
    last_error: Optional[OSError] = None
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(
                f"Betaflight exited before TCP {port} became ready (code {process.returncode})"
            )
        try:
            connection = socket.create_connection(("127.0.0.1", port), timeout=0.25)
            connection.settimeout(0.25)
            return connection
        except OSError as exc:
            last_error = exc
            time.sleep(0.1)
    raise RuntimeError(f"TCP {port} did not become ready: {last_error}")


def drain_socket(connection: socket.socket, seconds: float) -> bytes:
    deadline = time.monotonic() + seconds
    chunks = []
    while time.monotonic() < deadline:
        try:
            chunk = connection.recv(4096)
            if not chunk:
                break
            chunks.append(chunk)
        except socket.timeout:
            pass
    return b"".join(chunks)


def read_cli_response(
    connection: socket.socket, operation: str, timeout: float = 6.0
) -> str:
    """Read one interactive CLI response, including its trailing prompt."""
    deadline = time.monotonic() + timeout
    response = bytearray()
    while time.monotonic() < deadline:
        try:
            chunk = connection.recv(4096)
            if not chunk:
                break
            response.extend(chunk)
            if response.endswith(b"\r\n# "):
                break
        except socket.timeout:
            pass
    if not response.endswith(b"\r\n# "):
        rendered = response.decode(errors="replace")
        raise RuntimeError(
            f"Betaflight CLI did not finish {operation!r}: {rendered}"
        )
    rendered = response.decode(errors="replace").replace("\r", "")
    lowered = rendered.lower()
    if (
        "###error" in lowered
        or "parse error" in lowered
        or "unknown command" in lowered
    ):
        raise RuntimeError(f"Betaflight CLI rejected {operation!r}: {rendered}")
    return rendered


def enter_betaflight_cli(connection: socket.socket) -> None:
    connection.sendall(b"#\n")
    response = read_cli_response(connection, "enter CLI")
    if "Entering CLI Mode" not in response:
        raise RuntimeError(f"Betaflight did not enter CLI mode: {response}")


def run_cli_command(connection: socket.socket, command: str) -> str:
    connection.sendall(command.encode("ascii") + b"\n")
    return read_cli_response(connection, command)


def yaml_value(config_text: str, key: str) -> str:
    match = re.search(
        rf"^\s*{re.escape(key)}\s*:\s*([^#\n]+?)\s*$",
        config_text,
        flags=re.MULTILINE,
    )
    if not match:
        raise RuntimeError(f"missing {key!r} in Betaflight bridge configuration")
    return match.group(1).strip()


def integral_setting(value: float, description: str) -> int:
    if not math.isfinite(value) or not math.isclose(value, round(value), abs_tol=1e-6):
        raise RuntimeError(
            f"{description}={value} cannot be represented by Betaflight's integer CLI setting"
        )
    return int(round(value))


def expected_betaflight_settings(bridge_config: Path) -> Dict[str, str]:
    try:
        config_text = bridge_config.read_text(encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"cannot read bridge configuration {bridge_config}: {exc}") from exc

    def vector(key: str) -> tuple[float, float, float]:
        try:
            parsed = ast.literal_eval(yaml_value(config_text, key))
            values = tuple(float(value) for value in parsed)
        except (SyntaxError, ValueError, TypeError) as exc:
            raise RuntimeError(f"invalid {key!r} in {bridge_config}") from exc
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            raise RuntimeError(f"{key!r} must contain three finite values")
        return values  # type: ignore[return-value]

    def scalar_int(key: str) -> int:
        try:
            value = float(yaml_value(config_text, key))
        except ValueError as exc:
            raise RuntimeError(f"invalid {key!r} in {bridge_config}") from exc
        return integral_setting(value, key)

    centers = vector("center_rate_deg_s")
    maximums = vector("max_rate_deg_s")
    expos = vector("expo_percent")
    settings: Dict[str, str] = {
        "rates_type": "ACTUAL",
        # The bridge uses 1500 as the neutral point and applies the normal
        # Betaflight yaw sign explicitly in its AETR packet conversion.
        "mid_rc": "1500",
        "min_check": str(scalar_int("min_check")),
        "deadband": str(scalar_int("deadband")),
        "yaw_deadband": str(scalar_int("yaw_deadband")),
        "yaw_control_reversed": "OFF",
        # Aeroloop's Iris is a props-in airframe (M0/M3 CW, M1/M2 CCW).
        # This is the motor-yaw mixer polarity, not the RC yaw direction above.
        "yaw_motors_reversed": "ON",
        # The bridge's throttle equation assumes Betaflight's curve is linear.
        "thr_mid": "50",
        "thr_expo": "0",
    }
    # Inner-loop gains. Yaw in particular has to match this airframe's low yaw
    # authority, so keep them in the bridge configuration rather than in a
    # hand-edited EEPROM.
    for axis in ("roll", "pitch", "yaw"):
        for term in ("p", "i", "d", "f"):
            name = f"{term}_{axis}"
            settings[name] = str(scalar_int(name))
    axes = ("roll", "pitch", "yaw")
    for index, axis in enumerate(axes):
        settings[f"{axis}_rc_rate"] = str(
            integral_setting(centers[index] / 10.0, f"{axis} center rate / 10")
        )
        settings[f"{axis}_srate"] = str(
            integral_setting(maximums[index] / 10.0, f"{axis} max rate / 10")
        )
        settings[f"{axis}_expo"] = str(
            integral_setting(expos[index], f"{axis} expo")
        )
    return settings


def cli_setting(response: str, name: str) -> Optional[str]:
    match = re.search(
        rf"^[ \t]*{re.escape(name)}[ \t]*=[ \t]*([^\n]+?)[ \t]*$",
        response,
        flags=re.MULTILINE | re.IGNORECASE,
    )
    return match.group(1).strip() if match else None


def apply_bridge_settings(connection: socket.socket, bridge_config: Path) -> None:
    """Write the settings the UDP bridge inverts into the isolated EEPROM.

    The bridge configuration is the single source of truth for the rate curve,
    deadbands and throttle range.  Writing them here means changing
    betaflight_udp.yaml is enough, instead of also hand-editing an EEPROM that
    validate_prearm_configuration() would otherwise simply reject.
    """
    for name, value in expected_betaflight_settings(bridge_config).items():
        run_cli_command(connection, f"set {name} = {value}")
    run_cli_command(connection, "map AETR1234")
    log("wrote Betaflight rate profile / RX mapping from the bridge config")


def validate_prearm_configuration(
    connection: socket.socket, bridge_config: Path
) -> None:
    """Fail closed unless SITL settings match the UDP adapter assumptions."""
    expected = expected_betaflight_settings(bridge_config)
    failures = []
    for name, wanted in expected.items():
        actual = cli_setting(run_cli_command(connection, f"get {name}"), name)
        if actual is None:
            # `get` is idempotent, and the CLI occasionally answers a long
            # settings sweep too slowly to parse on the first attempt.
            actual = cli_setting(run_cli_command(connection, f"get {name}"), name)
        if actual != wanted:
            failures.append(f"{name}: expected {wanted}, got {actual or 'unreadable'}")

    map_response = run_cli_command(connection, "map")
    map_match = re.search(r"^[ \t]*map[ \t]+([A-Z0-9]+)[ \t]*$", map_response, re.MULTILINE)
    actual_map = map_match.group(1) if map_match else None
    if actual_map != "AETR1234":
        failures.append(f"map: expected AETR1234, got {actual_map or 'unreadable'}")

    aux_response = run_cli_command(connection, "aux")
    if not re.search(
        r"^[ \t]*aux 0 0 0 1700 2100 0 0[ \t]*$",
        aux_response,
        flags=re.MULTILINE,
    ):
        failures.append("ARM mode: expected AUX1 1700-2100 on aux slot 0")

    feature_response = run_cli_command(connection, "feature")
    enabled_match = re.search(
        r"^[ \t]*Enabled:[ \t]*(.*?)[ \t]*$",
        feature_response,
        flags=re.MULTILINE | re.IGNORECASE,
    )
    if not enabled_match:
        failures.append("features: enabled-feature list was unreadable")
    else:
        enabled = {feature.upper() for feature in enabled_match.group(1).split()}
        if "RX_UDP" not in enabled:
            failures.append("feature RX_UDP must be enabled")
        for forbidden in ("3D", "MOTOR_STOP"):
            if forbidden in enabled:
                failures.append(f"feature {forbidden} must be disabled")

    if failures:
        formatted = "\n  - ".join(failures)
        raise RuntimeError(
            "refusing --arm because Betaflight does not match the UDP bridge:\n"
            f"  - {formatted}"
        )
    log("pre-arm Betaflight configuration verified (AETR / ACTUAL rates / RX / features)")


def ensure_simulator_ports_free() -> None:
    checks = []
    try:
        for socket_type, port in (
            (socket.SOCK_STREAM, 5761),
            (socket.SOCK_DGRAM, 9002),
            (socket.SOCK_DGRAM, 9003),
            (socket.SOCK_DGRAM, 9004),
        ):
            probe = socket.socket(socket.AF_INET, socket_type)
            checks.append(probe)
            try:
                probe.bind(("127.0.0.1", port))
            except OSError as exc:
                raise RuntimeError(
                    f"port {port} is already in use; stop the existing "
                    f"Betaloop / Betaflight process first ({exc})"
                ) from exc
    finally:
        for probe in checks:
            probe.close()


def stop_process(process: Optional[subprocess.Popen], grace: float = 3.0) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=grace)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()


def send_fallback_disarm() -> None:
    """Best-effort disarm if the C++ adapter could not clean up itself."""
    channels = [1000] * 16
    channels[0] = 1500
    channels[1] = 1500
    channels[3] = 1500
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as connection:
            for _ in range(3):
                packet = struct.pack("<d16H", time.time(), *channels)
                connection.sendto(packet, ("127.0.0.1", 9004))
                time.sleep(0.01)
    except OSError as exc:
        # Simulator cleanup must still continue if the UDP receiver is gone.
        log(f"WARNING: fallback disarm could not be sent: {exc}")


def prepare_runtime_betaflight(
    source_elf: Path,
    runtime_dir: Path,
    arm_bridge_config: Optional[Path] = None,
) -> Path:
    if not source_elf.is_file():
        raise FileNotFoundError(f"Betaflight ELF not found: {source_elf}")
    runtime_dir.mkdir(parents=True, exist_ok=True)
    runtime_elf = runtime_dir / source_elf.name
    shutil.copy2(source_elf, runtime_elf)
    source_eeprom = source_elf.parent / "eeprom.bin"
    runtime_eeprom = runtime_dir / "eeprom.bin"
    # Never inherit configuration from a previous isolated run.  If the source
    # has no EEPROM, Betaflight will create a fresh default one below.
    runtime_eeprom.unlink(missing_ok=True)
    if source_eeprom.is_file():
        shutil.copy2(source_eeprom, runtime_eeprom)

    # Configure only the isolated EEPROM copy.  The user's original EEPROM is
    # never opened for writing.  AUX1 high now selects ARM, while Acro remains
    # the default flight mode.
    config_process = subprocess.Popen(
        [str(runtime_elf)],
        cwd=runtime_dir,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        with wait_for_tcp(5761, config_process) as connection:
            enter_betaflight_cli(connection)
            run_cli_command(connection, "aux 0 0 0 1700 2100 0 0")
            # The generic SITL target defaults this to OFF, while Betaflight's
            # official SITL_GAZEBO target and the Aeroloop Iris require props-in.
            run_cli_command(connection, "set yaw_motors_reversed = ON")
            if arm_bridge_config is not None:
                apply_bridge_settings(connection, arm_bridge_config)
                validate_prearm_configuration(connection, arm_bridge_config)
            connection.sendall(b"save\n")
            save_response = drain_socket(connection, 1.0).decode(errors="replace")
            if "###ERROR" in save_response or "Parse error" in save_response:
                raise RuntimeError(
                    "Betaflight rejected the isolated EEPROM save: " + save_response
                )
        try:
            returncode = config_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired as exc:
            stop_process(config_process)
            raise RuntimeError("Betaflight did not exit after saving isolated EEPROM") from exc
        if returncode != 0:
            raise RuntimeError(
                f"Betaflight exited with code {returncode} while preparing isolated EEPROM"
            )
    finally:
        stop_process(config_process)
    if not runtime_eeprom.is_file():
        raise RuntimeError("Betaflight did not create the isolated eeprom.bin")
    log("prepared isolated Betaflight EEPROM with ARM on AUX1")
    return runtime_elf


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--betaloop-home", type=Path, default=DEFAULT_BETALOOP_HOME)
    parser.add_argument("--gazebo", action="store_true", help="show the Gazebo GUI")
    parser.add_argument(
        "--arm",
        action="store_true",
        help="arm after the safe pre-arm interval and execute the 1 m takeoff",
    )
    parser.add_argument("--disarmed-seconds", type=nonnegative_seconds, default=6.0)
    parser.add_argument("--prearm-seconds", type=nonnegative_seconds, default=2.0)
    parser.add_argument("--duration", type=nonnegative_seconds, default=0.0)
    parser.add_argument(
        "--trajectory",
        type=Path,
        help=(
            "CSV trajectory executed after takeoff; relative paths are resolved "
            "from the project root"
        ),
    )
    parser.add_argument(
        "--trajectory-source-mass",
        type=float,
        default=0.0,
        help=(
            "vehicle mass used to generate CSV u_1..u_4; the default estimates "
            "it from the trajectory file itself"
        ),
    )
    parser.add_argument(
        "--ground-clearance",
        type=float,
        default=0.8,
        help="minimum trajectory altitude above the takeoff point (m)",
    )
    parser.add_argument(
        "--log",
        type=Path,
        help="write a reference-vs-state CSV for betaflight_sitl/validate.py",
    )
    parser.add_argument(
        "--controller",
        choices=("mpc", "geo"),
        default="mpc",
        help="Agilib outer controller (default: mpc)",
    )
    parser.add_argument("--no-build", action="store_true")
    args = parser.parse_args()

    if args.trajectory_source_mass < 0 or not math.isfinite(
        args.trajectory_source_mass
    ):
        parser.error("--trajectory-source-mass must be finite and >= 0")
    if args.ground_clearance < 0 or not math.isfinite(args.ground_clearance):
        parser.error("--ground-clearance must be finite and >= 0")
    if args.trajectory is not None and not args.arm:
        parser.error("--trajectory requires --arm")

    betaloop_home = args.betaloop_home.expanduser().resolve()
    aeroloop_home, source_world, source_elf = load_betaloop_config(betaloop_home)
    ensure_simulator_ports_free()
    build_root = REPO_ROOT / "build" / "betaflight_sitl"
    cmake_build = build_root / "agilib"
    plugin_build = build_root / "plugin"
    runtime = build_root / "runtime"
    params = REPO_ROOT / "agilib" / "params"
    bridge_config = params / "betaflight_udp.yaml"
    overlay_world, _ = prepare_assets(aeroloop_home, source_world, runtime / "assets")
    runtime_elf = prepare_runtime_betaflight(
        source_elf,
        runtime / "betaflight",
        bridge_config if args.arm else None,
    )
    binary = cmake_build / "bin" / "agilicious_betaflight_sitl"
    plugin_library = plugin_build / "libAgiliciousBetaflightPlugin.so"
    if not args.no_build:
        plugin_library = build_gazebo_plugin(aeroloop_home, plugin_build)
        binary = build_adapter(cmake_build)
    else:
        for artifact in (binary, plugin_library):
            if not artifact.is_file():
                raise FileNotFoundError(
                    f"--no-build requested but artifact is absent: {artifact}"
                )

    env = os.environ.copy()
    prepend_env(env, "SDF_PATH", runtime / "assets" / "models")
    prepend_env(env, "GZ_SIM_RESOURCE_PATH", runtime / "assets" / "models")
    prepend_env(env, "GZ_SIM_RESOURCE_PATH", runtime / "assets" / "worlds")
    prepend_env(env, "GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_build)
    # libacados.so has sibling hpipm / blasfeo dependencies.  DT_RUNPATH on the
    # executable is not applied transitively on glibc, so expose the vendored
    # directory explicitly for this isolated process tree.
    prepend_env(
        env,
        "LD_LIBRARY_PATH",
        REPO_ROOT / "agilib" / "externals" / "acados-src" / "lib",
    )

    betaloop_cmd = [
        sys.executable,
        str(betaloop_home / "start.py"),
        "--world",
        str(overlay_world),
        "--elf",
        str(runtime_elf),
        # This loop talks directly to the SITL UDP receiver.  Avoid starting
        # either optional RC producer: the Configurator proxy or Betaloop's
        # virtual transmitter.  Agilicious must be the only writer on UDP
        # 9004, and this also avoids conflicts with an existing TCP 6761
        # proxy.
        "--disable-transmitter",
        "--disable-websockify",
    ]
    if args.gazebo:
        betaloop_cmd.append("--gazebo")

    pilot_config = (
        params / "pilot_betaflight_mpc_sitl.yaml"
        if args.controller == "mpc"
        else params / "pilot_betaflight_sitl.yaml"
    )
    log(f"selected Agilib outer controller: {args.controller.upper()}")
    controller_cmd = [
        str(binary),
        "--pilot-config",
        str(pilot_config),
        "--params-dir",
        str(params),
        "--quad",
        str(params / "quads" / "betaloop_iris.yaml"),
        "--bridge-config",
        str(bridge_config),
        "--odom-topic",
        ODOM_TOPIC,
        "--joint-topic",
        JOINT_TOPIC,
        "--disarmed-seconds",
        str(args.disarmed_seconds),
        "--prearm-seconds",
        str(args.prearm_seconds),
    ]
    if args.arm:
        controller_cmd.append("--arm")
    if args.duration > 0:
        controller_cmd.extend(("--duration", str(args.duration)))
    if args.trajectory is not None:
        trajectory = args.trajectory.expanduser()
        if not trajectory.is_absolute():
            trajectory = REPO_ROOT / trajectory
        trajectory = trajectory.resolve()
        if not trajectory.is_file():
            parser.error(f"trajectory is not a file: {trajectory}")
        controller_cmd.extend(("--trajectory", str(trajectory)))
        if args.trajectory_source_mass > 0:
            controller_cmd.extend(
                ("--trajectory-source-mass", str(args.trajectory_source_mass))
            )
        controller_cmd.extend(("--ground-clearance", str(args.ground_clearance)))
    if args.log is not None:
        log_path = args.log.expanduser()
        if not log_path.is_absolute():
            log_path = REPO_ROOT / log_path
        log_path.parent.mkdir(parents=True, exist_ok=True)
        controller_cmd.extend(("--log", str(log_path)))

    betaloop_process: Optional[subprocess.Popen] = None
    controller_process: Optional[subprocess.Popen] = None
    signal.signal(signal.SIGTERM, interrupt_for_shutdown)
    try:
        log("starting Betaloop with the generated model overlay")
        betaloop_process = subprocess.Popen(
            betaloop_cmd,
            cwd=betaloop_home,
            env=env,
            start_new_session=True,
        )
        # Betaloop deliberately waits for Gazebo before it launches
        # Betaflight.  Do not start the pre-arm timer merely because Gazebo
        # odometry is already present: UDP packets sent before SITL binds its
        # receiver are lost, and the SITL startup can briefly stall Gazebo.
        log("waiting for Betaflight SITL TCP 5761")
        with wait_for_tcp(5761, betaloop_process, timeout=15.0):
            pass
        log("Betaflight SITL is ready")
        log("starting Agilicious UDP adapter")
        controller_process = subprocess.Popen(
            controller_cmd,
            cwd=REPO_ROOT,
            env=env,
            start_new_session=True,
        )
        while True:
            controller_status = controller_process.poll()
            betaloop_status = betaloop_process.poll()
            if controller_status is not None:
                if controller_status != 0:
                    log(f"adapter exited with code {controller_status}")
                return controller_status
            if betaloop_status is not None:
                log(f"Betaloop exited with code {betaloop_status}")
                return betaloop_status or 1
            time.sleep(0.25)
    except KeyboardInterrupt:
        log("stopping; disarm is sent before simulator shutdown")
        return 0
    finally:
        stop_process(controller_process)
        # Keep this after the adapter has stopped and before Betaflight exits:
        # it covers crashes / forced termination where C++ destructors did not
        # get a chance to publish their normal disarm sequence.
        send_fallback_disarm()
        stop_process(betaloop_process, grace=8.0)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, RuntimeError, subprocess.CalledProcessError) as exc:
        log(f"ERROR: {exc}")
        raise SystemExit(1)
