#!/usr/bin/env python3
"""Build and run the isolated Agilicious -> Betaflight SITL closed loop."""

# 中文说明（注意：模块 docstring 会被 argparse 当作 --help 的描述，因此总览
# 放在这里而不是 docstring 里）。
#
# 本脚本是整条 “Agilicious 控制器 <-> Betaflight SITL 飞控 <-> Gazebo 仿真”
# 闭环的一键启动入口，主要做四件事：
#
# 1. 编译：构建 Agilicious 侧的 UDP 适配器（C++ 可执行文件）以及 Gazebo 侧的
#    非阻塞 Betaflight 插件。
# 2. 隔离：把用户原始的 Betaflight ELF 与 EEPROM 复制到独立的 runtime 目录，
#    所有配置写入都只作用于这份副本，绝不污染用户本地的原始配置。
# 3. 校验：在 --arm（真正解锁起飞）之前，把桥接配置 betaflight_udp.yaml 中的
#    速率曲线 / 死区 / PID 等写进隔离 EEPROM，并逐项回读确认，不一致就拒绝解锁。
# 4. 编排：拉起 Betaloop（Gazebo + Betaflight SITL）与适配器进程，监控两者状态，
#    退出时按 “先停适配器 -> 补发 disarm -> 再停仿真器” 的顺序清理。
#
# 端口约定（均为本机回环）：
#   TCP 5761  Betaflight CLI
#   UDP 9002  Betaflight -> 仿真器 的电机输出
#   UDP 9003  仿真器 -> Betaflight 的传感器状态
#   UDP 9004  遥控通道输入（本闭环中只允许 Agilicious 作为唯一写入方）

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

from prepare_assets import (
    COMPANION_IMU_TOPIC,
    JOINT_TOPIC,
    ODOM_TOPIC,
    prepare_assets,
)


# 仓库根目录：本文件位于 <repo>/betaflight_sitl/run.py，故上溯两级。
REPO_ROOT = Path(__file__).resolve().parents[1]
# Betaloop（Gazebo + Betaflight SITL 启动器）的默认安装位置。
DEFAULT_BETALOOP_HOME = Path.home() / "betaloop"


def log(message: str) -> None:
    """统一带前缀的日志输出；flush 保证与子进程输出交错时不丢序。"""
    print(f"[AGI-SITL] {message}", flush=True)


def interrupt_for_shutdown(_signum: int, _frame: object) -> None:
    """Route TERM through the same ordered cleanup path as Ctrl-C.

    把 SIGTERM 转换成 KeyboardInterrupt，使 kill 与 Ctrl-C 走同一条 finally
    清理路径（停适配器 -> 补发 disarm -> 停仿真器），避免直接被信号杀掉时
    飞控仍处于解锁状态。
    """
    raise KeyboardInterrupt


def nonnegative_seconds(value: str) -> float:
    """argparse 类型校验器：解析出一个有限且非负的秒数。"""
    try:
        seconds = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"expected a number, got: {value}") from exc
    # 拒绝 nan / inf / 负数，防止它们被传给下游 C++ 的时间参数。
    if not math.isfinite(seconds) or seconds < 0.0:
        raise argparse.ArgumentTypeError("must be a finite value >= 0")
    return seconds


def prepend_env(env: Dict[str, str], key: str, path: Path) -> None:
    """把 path 前置到环境变量 key 的搜索路径中（原值保留在后面）。"""
    old = env.get(key, "")
    env[key] = str(path) + (os.pathsep + old if old else "")


def load_betaloop_config(home: Path) -> tuple[Path, Path, Path]:
    """读取 Betaloop 的 config.txt，返回 (Aeroloop 目录, 世界文件, Betaflight ELF)。"""
    config_path = home / "config.txt"
    parser = configparser.ConfigParser()
    if not parser.read(config_path) or "Betaloop" not in parser:
        raise RuntimeError(f"cannot read [Betaloop] from {config_path}")
    section = parser["Betaloop"]
    aeroloop = Path(section["AeroloopGazeboHome"]).expanduser().resolve()
    world = Path(section["World"]).expanduser()
    # World 允许写成相对名（如 iris.sdf），此时到 Aeroloop 的 worlds/ 下查找。
    if not world.is_absolute():
        world = aeroloop / "worlds" / world
    elf = Path(section["BetaflightElf"]).expanduser().resolve()
    return aeroloop, world.resolve(), elf


def build_adapter(build_dir: Path) -> Path:
    """配置并编译 Agilicious 侧的独立 UDP 适配器，返回可执行文件路径。"""
    binary = build_dir / "bin" / "agilicious_betaflight_sitl"
    # 只开 SITL 适配器目标，关掉测试 / benchmark / acados 下载，缩短构建时间。
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
    # CMake 成功但产物缺失，说明目标名或安装路径变了，尽早报错。
    if not binary.is_file():
        raise RuntimeError(f"build completed but adapter is missing: {binary}")
    return binary


def build_gazebo_plugin(aeroloop: Path, build_dir: Path) -> Path:
    """编译非阻塞版 Gazebo Betaflight 插件，返回生成的动态库路径。

    插件源码取自 Aeroloop，但用本仓库 betaflight_sitl/plugin 下的 CMake 重新
    构建，以获得不阻塞仿真步进的版本。
    """
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
            # 通过 CMake 变量把 Aeroloop 的插件源码位置传进去。
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
    """轮询等待端口可连接，返回已建立的连接。

    轮询期间同时监视 process：若 Betaflight 在端口就绪前就退出，立即报错，
    而不是白白等到超时。
    """
    deadline = time.monotonic() + timeout
    last_error: Optional[OSError] = None
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(
                f"Betaflight exited before TCP {port} became ready (code {process.returncode})"
            )
        try:
            connection = socket.create_connection(("127.0.0.1", port), timeout=0.25)
            # 后续 CLI 交互依赖短超时来做非阻塞式收包。
            connection.settimeout(0.25)
            return connection
        except OSError as exc:
            last_error = exc
            time.sleep(0.1)
    raise RuntimeError(f"TCP {port} did not become ready: {last_error}")


def drain_socket(connection: socket.socket, seconds: float) -> bytes:
    """在给定时间窗内尽量读空 socket，返回读到的全部字节。"""
    deadline = time.monotonic() + seconds
    chunks = []
    while time.monotonic() < deadline:
        try:
            chunk = connection.recv(4096)
            if not chunk:
                break
            chunks.append(chunk)
        except socket.timeout:
            # 超时只表示这一轮没数据，继续等到 deadline 为止。
            pass
    return b"".join(chunks)


def read_cli_response(
    connection: socket.socket, operation: str, timeout: float = 6.0
) -> str:
    """Read one interactive CLI response, including its trailing prompt.

    以 "\\r\\n# " 提示符作为一条命令回显结束的标志；未等到提示符即视为超时，
    并把回显里出现的错误关键字转成异常。
    """
    deadline = time.monotonic() + timeout
    response = bytearray()
    while time.monotonic() < deadline:
        try:
            chunk = connection.recv(4096)
            if not chunk:
                break
            response.extend(chunk)
            # 收到命令提示符，说明本条命令已执行完毕。
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
    # CLI 对错误命令返回 0 退出码，只能靠回显文本判断是否被拒绝。
    if (
        "###error" in lowered
        or "parse error" in lowered
        or "unknown command" in lowered
    ):
        raise RuntimeError(f"Betaflight CLI rejected {operation!r}: {rendered}")
    return rendered


def enter_betaflight_cli(connection: socket.socket) -> None:
    """向 Betaflight 发送 '#' 进入 CLI 模式，并确认握手成功。"""
    connection.sendall(b"#\n")
    response = read_cli_response(connection, "enter CLI")
    if "Entering CLI Mode" not in response:
        raise RuntimeError(f"Betaflight did not enter CLI mode: {response}")


def run_cli_command(connection: socket.socket, command: str) -> str:
    """执行一条 CLI 命令并返回其完整回显。"""
    connection.sendall(command.encode("ascii") + b"\n")
    return read_cli_response(connection, command)


def yaml_value(config_text: str, key: str) -> str:
    """从桥接配置文本中抓取 `key: value` 的原始值（忽略行尾 # 注释）。

    这里刻意用正则做轻量解析，避免为读几个标量而引入 YAML 依赖。
    """
    match = re.search(
        rf"^\s*{re.escape(key)}\s*:\s*([^#\n]+?)\s*$",
        config_text,
        flags=re.MULTILINE,
    )
    if not match:
        raise RuntimeError(f"missing {key!r} in Betaflight bridge configuration")
    return match.group(1).strip()


def integral_setting(value: float, description: str) -> int:
    """把浮点配置值转成 Betaflight CLI 需要的整数，非整数值直接报错。"""
    if not math.isfinite(value) or not math.isclose(value, round(value), abs_tol=1e-6):
        raise RuntimeError(
            f"{description}={value} cannot be represented by Betaflight's integer CLI setting"
        )
    return int(round(value))


def expected_betaflight_settings(bridge_config: Path) -> Dict[str, str]:
    """由桥接配置推导出 Betaflight 应有的全部 CLI 设置项 {名称: 期望值}。

    该字典同时用于写入（apply_bridge_settings）和回读校验
    （validate_prearm_configuration），保证 “写什么就验什么”。
    """
    try:
        config_text = bridge_config.read_text(encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"cannot read bridge configuration {bridge_config}: {exc}") from exc

    def vector(key: str) -> tuple[float, float, float]:
        """读取形如 [r, p, y] 的三元组，并校验元素个数与有限性。"""
        try:
            parsed = ast.literal_eval(yaml_value(config_text, key))
            values = tuple(float(value) for value in parsed)
        except (SyntaxError, ValueError, TypeError) as exc:
            raise RuntimeError(f"invalid {key!r} in {bridge_config}") from exc
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            raise RuntimeError(f"{key!r} must contain three finite values")
        return values  # type: ignore[return-value]

    def scalar_int(key: str) -> int:
        """读取单个标量并转成 CLI 所需的整数。"""
        try:
            value = float(yaml_value(config_text, key))
        except ValueError as exc:
            raise RuntimeError(f"invalid {key!r} in {bridge_config}") from exc
        return integral_setting(value, key)

    # ACTUAL 速率曲线的三要素：中心速率、最大速率、expo。
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
        # 注意区分：这是混控器的电机偏航极性，不是上面的遥控偏航方向。
        "yaw_motors_reversed": "ON",
        # The bridge's throttle equation assumes Betaflight's curve is linear.
        # 桥接侧的油门换算假设曲线为线性，因此必须把 mid/expo 固定成线性。
        "thr_mid": "50",
        "thr_expo": "0",
    }
    # Inner-loop gains. Yaw in particular has to match this airframe's low yaw
    # authority, so keep them in the bridge configuration rather than in a
    # hand-edited EEPROM.
    # 内环 PID(F) 增益：逐轴逐项从桥接配置读取，共 3 轴 x 4 项。
    for axis in ("roll", "pitch", "yaw"):
        for term in ("p", "i", "d", "f"):
            name = f"{term}_{axis}"
            settings[name] = str(scalar_int(name))
    axes = ("roll", "pitch", "yaw")
    for index, axis in enumerate(axes):
        # Betaflight 的 rc_rate / srate 以 “度每秒 / 10” 为单位存储。
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
    """从 `get <name>` 的回显里解析出 `name = value` 的值；解析不到返回 None。"""
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

    中文：桥接配置是速率曲线 / 死区 / 油门范围的唯一权威来源。在这里统一写入，
    意味着以后只改 betaflight_udp.yaml 即可，无需再手工编辑 EEPROM。
    """
    for name, value in expected_betaflight_settings(bridge_config).items():
        run_cli_command(connection, f"set {name} = {value}")
    # 通道映射固定为 AETR，与适配器打包 UDP 包时的通道顺序一致。
    run_cli_command(connection, "map AETR1234")
    log("wrote Betaflight rate profile / RX mapping from the bridge config")


def validate_prearm_configuration(
    connection: socket.socket, bridge_config: Path
) -> None:
    """Fail closed unless SITL settings match the UDP adapter assumptions.

    中文：解锁前的“失败即拒绝”校验。逐项回读 Betaflight 的实际配置，只要有一项
    与桥接配置的假设不符，就收集进 failures 并最终抛异常，阻止 --arm 起飞。
    """
    expected = expected_betaflight_settings(bridge_config)
    failures = []
    for name, wanted in expected.items():
        actual = cli_setting(run_cli_command(connection, f"get {name}"), name)
        if actual is None:
            # `get` is idempotent, and the CLI occasionally answers a long
            # settings sweep too slowly to parse on the first attempt.
            # `get` 是幂等的，因此首次解析失败时可以安全地重试一次。
            actual = cli_setting(run_cli_command(connection, f"get {name}"), name)
        if actual != wanted:
            failures.append(f"{name}: expected {wanted}, got {actual or 'unreadable'}")

    # 1) 通道映射必须是 AETR1234。
    map_response = run_cli_command(connection, "map")
    map_match = re.search(r"^[ \t]*map[ \t]+([A-Z0-9]+)[ \t]*$", map_response, re.MULTILINE)
    actual_map = map_match.group(1) if map_match else None
    if actual_map != "AETR1234":
        failures.append(f"map: expected AETR1234, got {actual_map or 'unreadable'}")

    # 2) ARM 必须绑定在 AUX1 的 1700-2100 区间（对应 aux 槽位 0）。
    aux_response = run_cli_command(connection, "aux")
    if not re.search(
        r"^[ \t]*aux 0 0 0 1700 2100 0 0[ \t]*$",
        aux_response,
        flags=re.MULTILINE,
    ):
        failures.append("ARM mode: expected AUX1 1700-2100 on aux slot 0")

    # 3) 特性开关：必须开 RX_UDP；3D 与 MOTOR_STOP 会破坏油门映射，必须关闭。
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

    # 一次性汇报所有不匹配项，避免反复试错。
    if failures:
        formatted = "\n  - ".join(failures)
        raise RuntimeError(
            "refusing --arm because Betaflight does not match the UDP bridge:\n"
            f"  - {formatted}"
        )
    log("pre-arm Betaflight configuration verified (AETR / ACTUAL rates / RX / features)")


def ensure_simulator_ports_free() -> None:
    """启动前占位探测 CLI/仿真所需端口，确认没有遗留进程仍在占用。

    通过 bind 成功与否判断端口是否空闲，随后在 finally 中全部关闭；这样后续
    真正的进程才能绑定成功。
    """
    checks = []
    try:
        for socket_type, port in (
            (socket.SOCK_STREAM, 5761),  # Betaflight CLI
            (socket.SOCK_DGRAM, 9002),   # 电机输出
            (socket.SOCK_DGRAM, 9003),   # 传感器状态
            (socket.SOCK_DGRAM, 9004),   # 遥控通道输入
        ):
            probe = socket.socket(socket.AF_INET, socket_type)
            checks.append(probe)
            # 与真实监听方保持一致：Betaflight / 仿真器都用 SO_REUSEADDR 绑定，
            # 否则上一次运行留下的 TIME-WAIT 连接会让这里误报“端口被占用”。
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(("127.0.0.1", port))
            except OSError as exc:
                raise RuntimeError(
                    f"port {port} is already in use; stop the existing "
                    f"Betaloop / Betaflight process first ({exc})"
                ) from exc
    finally:
        # 探测完必须立刻释放，否则会挡住真正要用这些端口的进程。
        for probe in checks:
            probe.close()


def stop_process(process: Optional[subprocess.Popen], grace: float = 3.0) -> None:
    """按进程组先 TERM 后 KILL 地停止子进程。

    子进程都以 start_new_session=True 启动，因而自成进程组；用 killpg 才能
    连它们派生的孙进程（Gazebo、Betaflight 等）一起收掉。
    """
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=grace)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        # 宽限期内没退干净，就强制 KILL 并等待回收，避免留下僵尸进程。
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()


def send_fallback_disarm() -> None:
    """Best-effort disarm if the C++ adapter could not clean up itself.

    中文：兜底上锁。直接向 UDP 9004 连发三帧 “油门最低、横滚/俯仰/偏航居中、
    AUX 全低” 的通道包，把 ARM 开关拉低。
    """
    channels = [1000] * 16
    channels[0] = 1500  # A: 副翼居中
    channels[1] = 1500  # E: 升降居中
    # 索引 2 是油门，保持 1000（最低）；索引 3 为方向舵，居中。
    channels[3] = 1500
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as connection:
            for _ in range(3):
                # 包格式：小端 double 时间戳 + 16 个 uint16 通道值。
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
    """把 Betaflight ELF 复制到隔离目录并配置好其 EEPROM，返回运行用的 ELF 路径。

    arm_bridge_config 非空（即使用 --arm）时，额外写入桥接配置并做解锁前校验。
    """
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
    # 以 runtime_dir 为工作目录启动，Betaflight 就只会读写这份 EEPROM 副本。
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
                # 先写入，再立即回读校验：两者用的是同一份期望值。
                apply_bridge_settings(connection, arm_bridge_config)
                validate_prearm_configuration(connection, arm_bridge_config)
            # save 会写盘并让飞控自行退出，这里不能用 read_cli_response
            # （它等待的命令提示符不会再出现）。
            connection.sendall(b"save\n")
            save_response = drain_socket(connection, 1.0).decode(errors="replace")
            if "###ERROR" in save_response or "Parse error" in save_response:
                raise RuntimeError(
                    "Betaflight rejected the isolated EEPROM save: " + save_response
                )
        try:
            # 正常情况下 save 之后进程会自己退出。
            returncode = config_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired as exc:
            stop_process(config_process)
            raise RuntimeError("Betaflight did not exit after saving isolated EEPROM") from exc
        if returncode != 0:
            raise RuntimeError(
                f"Betaflight exited with code {returncode} while preparing isolated EEPROM"
            )
    finally:
        # 任何异常路径下都不能把这个临时配置进程留在后台占用 TCP 5761。
        stop_process(config_process)
    if not runtime_eeprom.is_file():
        raise RuntimeError("Betaflight did not create the isolated eeprom.bin")
    log("prepared isolated Betaflight EEPROM with ARM on AUX1")
    return runtime_elf


def main() -> int:
    """解析参数、准备资源、拉起两个进程并守护到退出；返回进程退出码。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--betaloop-home", type=Path, default=DEFAULT_BETALOOP_HOME)
    parser.add_argument("--gazebo", action="store_true", help="show the Gazebo GUI")
    parser.add_argument(
        "--arm",
        action="store_true",
        help="arm after the safe pre-arm interval and execute the 1 m takeoff",
    )
    # 解锁前的两段等待：先保持上锁 disarmed-seconds，再预解锁 prearm-seconds。
    parser.add_argument("--disarmed-seconds", type=nonnegative_seconds, default=6.0)
    parser.add_argument("--prearm-seconds", type=nonnegative_seconds, default=2.0)
    # 0 表示不限时长，一直运行到手动停止。
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
    # 真实传感链路模拟：位置/速度走 MockVIO 模拟的 RTK，姿态/角速度来自机载
    # IMU（Gazebo 里带噪声的独立传感器）。默认关闭，保持 Gazebo 真值直通行为。
    parser.add_argument(
        "--rtk-msp",
        action="store_true",
        help=(
            "emulate the intended hardware state pipeline: RTK-like position "
            "and velocity through MockVIO, attitude and body rates from the "
            "companion computer's own 1 kHz IMU, and Betaflight polled over "
            "MSP as a monitor only"
        ),
    )
    parser.add_argument(
        "--no-msp-monitor",
        action="store_true",
        help="skip the Betaflight MSP monitor that --rtk-msp otherwise starts",
    )
    parser.add_argument(
        "--msp-rate",
        type=float,
        default=20.0,
        help=(
            "MSP monitor poll rate in Hz (default: 20); Betaflight serves MSP "
            "from a ~100 Hz task and nothing here is in a control loop"
        ),
    )
    parser.add_argument("--no-build", action="store_true")
    parser.add_argument("--ros2", action="store_true",
                        help="use shared ROS 2 control; ARM/AUTO are explicit ROS receiver inputs")
    args = parser.parse_args()

    # argparse 的 type=float 不拦截 nan/inf 和负数，这里补齐校验。
    if args.trajectory_source_mass < 0 or not math.isfinite(
        args.trajectory_source_mass
    ):
        parser.error("--trajectory-source-mass must be finite and >= 0")
    if args.ground_clearance < 0 or not math.isfinite(args.ground_clearance):
        parser.error("--ground-clearance must be finite and >= 0")
    # 不解锁就飞不了轨迹，提前拒绝这种自相矛盾的组合。
    if args.trajectory is not None and not args.arm and not args.ros2:
        parser.error("--trajectory requires --arm")
    # 这些检查必须留在这里：下面会启动 Betaflight 写隔离 EEPROM，
    # 等到那之后再 parser.error 就已经产生了副作用。
    if args.rtk_msp and args.controller == "geo":
        parser.error("--rtk-msp currently only wires up the MPC pilot config")
    if not math.isfinite(args.msp_rate) or args.msp_rate <= 0:
        parser.error("--msp-rate must be finite and > 0")

    if args.ros2 and (args.controller != "mpc" or args.log is not None or args.trajectory_source_mass != 0):
        parser.error("--ros2 supports MPC, ROS topics/bag logging and automatic CSV source-mass detection")

    # ---- 路径与产物布局 ----
    betaloop_home = args.betaloop_home.expanduser().resolve()
    aeroloop_home, source_world, source_elf = load_betaloop_config(betaloop_home)
    # 确保模拟器端口可用
    ensure_simulator_ports_free()
    build_root = REPO_ROOT / "build" / "betaflight_sitl"
    cmake_build = build_root / "agilib"
    plugin_build = build_root / "plugin"
    runtime = build_root / "runtime"
    params = REPO_ROOT / "agilib" / "params"
    bridge_config = params / "betaflight_udp.yaml"
    # 生成模型 / 世界文件的叠加副本，同样不改动 Aeroloop 原始资源。
    # 把aeroloop中的世界和模型复制到runtime/assets中，并返回叠加后的世界文件路径
    overlay_world, _ = prepare_assets(aeroloop_home, source_world, runtime / "assets")
    # 把 Betaflight ELF 复制到隔离目录并配置好其 EEPROM，返回运行用的 ELF 路径。
    runtime_elf = prepare_runtime_betaflight(
        source_elf,
        runtime / "betaflight",
        # 只有真要解锁时才写入并强制校验桥接配置。
        bridge_config if args.arm or args.ros2 else None,
    )
    binary = (REPO_ROOT / "install/agi_ros2/lib/agi_ros2/control_node") if args.ros2 else cmake_build / "bin" / "agilicious_betaflight_sitl"
    plugin_library = plugin_build / "libAgiliciousBetaflightPlugin.so"
    #---- 编译产物 ----
    if not args.no_build:
        plugin_library = build_gazebo_plugin(aeroloop_home, plugin_build)
        if args.ros2:
            subprocess.run([str(REPO_ROOT / "agi_ros2/scripts/build.sh")], cwd=REPO_ROOT, check=True)
        else:
            binary = build_adapter(cmake_build)
    else:
        # 跳过编译时，至少确认上次构建的产物还在。
        artifacts = [binary, plugin_library]
        if args.ros2:
            artifacts.extend(binary.parent / name for name in
                             ("state_fusion_node", "command_output_node",
                              "gazebo_sensors"))
        for artifact in artifacts:
            if not artifact.is_file():
                raise FileNotFoundError(
                    f"--no-build requested but artifact is absent: {artifact}"
                )

    # ---- 子进程环境变量：让 Gazebo 找到叠加后的模型、世界与插件 ----
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

    # ---- 进程一：Betaloop（负责拉起 Gazebo 与 Betaflight SITL）----
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
    ]
    msp_monitor = args.rtk_msp and not args.no_msp_monitor
    if msp_monitor:
        # websockify 会把 TCP 6761 代理到 5761。它只在有人连 6761 时才真正建立
        # 到 5761 的连接，但适配器现在要独占 5761 跑 MSP，所以直接关掉，避免
        # 有人打开 Configurator 时抢占串口。
        betaloop_cmd.append("--disable-websockify")
    if args.gazebo:
        betaloop_cmd.append("--gazebo")

    # ---- 进程二：Agilicious 适配器（外环控制器 + UDP 桥接）----
    if args.controller == "geo":
        pilot_config = params / "pilot_betaflight_sitl.yaml"
    elif args.rtk_msp:
        # 这份配置把估计器换成 MockVIO 并关掉 velocity_in_bodyframe，与适配器
        # 交出的世界系速度匹配。
        pilot_config = params / "pilot_betaflight_mpc_rtk_sitl.yaml"
    else:
        pilot_config = params / "pilot_betaflight_mpc_sitl.yaml"
    log(f"selected Agilib outer controller: {args.controller.upper()}")
    if args.rtk_msp:
        log(
            "state pipeline: RTK (MockVIO) position/velocity, companion AHRS "
            "attitude (complementary filter on the 1 kHz IMU + RTK heading), "
            "companion IMU body rates and dead reckoning"
        )
        log(
            "Betaflight over MSP: "
            + ("monitor only (armed state / arming-disable flags / battery / "
               "its own attitude)" if msp_monitor else "not polled")
        )
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
        # 话题名从 prepare_assets 导入，保证与生成的模型 SDF 完全一致。
        "--odom-topic",
        ODOM_TOPIC,
        "--joint-topic",
        JOINT_TOPIC,
        "--disarmed-seconds",
        str(args.disarmed_seconds),
        "--prearm-seconds",
        str(args.prearm_seconds),
    ]
    if args.rtk_msp:
        # 状态链路：世界系速度 + 机载 IMU 航位推算。Betaflight 完全不参与。
        controller_cmd.extend(
            ("--rtk-state", "--imu-topic", COMPANION_IMU_TOPIC)
        )
    if msp_monitor:
        controller_cmd.extend(("--msp-monitor", "--msp-rate", str(args.msp_rate)))
    if args.arm:
        controller_cmd.append("--arm")
    if args.duration > 0:
        controller_cmd.extend(("--duration", str(args.duration)))
    if args.trajectory is not None:
        # 相对路径一律相对仓库根目录解析，避免受当前工作目录影响。
        trajectory = args.trajectory.expanduser()
        if not trajectory.is_absolute():
            trajectory = REPO_ROOT / trajectory
        trajectory = trajectory.resolve()
        if not trajectory.is_file():
            parser.error(f"trajectory is not a file: {trajectory}")
        controller_cmd.extend(("--trajectory", str(trajectory)))
        # 质量为 0 表示让适配器自行从轨迹文件估计。
        if args.trajectory_source_mass > 0:
            controller_cmd.extend(
                ("--trajectory-source-mass", str(args.trajectory_source_mass))
            )
        controller_cmd.extend(("--ground-clearance", str(args.ground_clearance)))
    if args.log is not None:
        log_path = args.log.expanduser()
        if not log_path.is_absolute():
            log_path = REPO_ROOT / log_path
        # 提前建好目录，免得 C++ 侧因为父目录不存在而写日志失败。
        log_path.parent.mkdir(parents=True, exist_ok=True)
        controller_cmd.extend(("--log", str(log_path)))

    if args.ros2:
        # Use the installed environment even on the first build in this shell.
        controller_cmd = [str(REPO_ROOT / "agi_ros2/scripts/launch.sh"), "mode:=sitl",
                          "sitl_config_verified:=true"]
        if args.trajectory is not None:
            controller_cmd.append("trajectory:=" + str(trajectory))
        if args.arm:
            log("ROS 2 does not auto-arm: use sim_rc ARM/AUTO/KILL parameters")

    betaloop_process: Optional[subprocess.Popen] = None
    controller_process: Optional[subprocess.Popen] = None
    # 让 SIGTERM 也走下面的 finally 清理流程。
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
        # 因此这里以 “TCP 5761 可连接” 作为 SITL 就绪的判据，而不是看 Gazebo。
        log("waiting for Betaflight SITL TCP 5761")
        with wait_for_tcp(5761, betaloop_process, timeout=45.0):
            pass  # 只探测就绪状态，连接随即关闭。
        log("Betaflight SITL is ready")
        log("starting Agilicious UDP adapter")
        controller_process = subprocess.Popen(
            controller_cmd,
            cwd=REPO_ROOT,
            env=env,
            start_new_session=True,
        )
        ros2_started = time.monotonic()
        # 守护循环：任一进程退出即结束，并把其退出码作为本脚本的退出码。
        while True:
            if args.ros2 and args.duration > 0 and time.monotonic() - ros2_started >= args.duration:
                return 0
            controller_status = controller_process.poll()
            betaloop_status = betaloop_process.poll()
            if controller_status is not None:
                if controller_status != 0:
                    log(f"adapter exited with code {controller_status}")
                return controller_status
            if betaloop_status is not None:
                log(f"Betaloop exited with code {betaloop_status}")
                # 仿真器先退出属于异常，即使它返回 0 也报失败。
                return betaloop_status or 1
            time.sleep(0.25)
    except KeyboardInterrupt:
        # Ctrl-C / SIGTERM：视为用户主动停止，按正常退出处理。
        log("stopping; disarm is sent before simulator shutdown")
        return 0
    finally:
        # 清理顺序很关键，不要调整下面三步的先后。
        stop_process(controller_process)
        # Keep this after the adapter has stopped and before Betaflight exits:
        # it covers crashes / forced termination where C++ destructors did not
        # get a chance to publish their normal disarm sequence.
        send_fallback_disarm()
        # 仿真器关闭较慢，给它更长的宽限期。
        stop_process(betaloop_process, grace=8.0)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, RuntimeError, subprocess.CalledProcessError) as exc:
        # 把预期内的失败收敛成一行日志 + 退出码 1，不向用户抛完整堆栈。
        log(f"ERROR: {exc}")
        raise SystemExit(1)
