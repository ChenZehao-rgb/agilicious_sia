#!/usr/bin/env python3
"""Launch hardware nodes with per-thread FIFO scheduling, retaining safety gates.

Run `sudo -v` first if passwordless sudo is unavailable. Only chrt runs as root;
ROS nodes retain the invoking user's identity. No persistent host settings change.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time


PRIORITIES = {
    'mavlink_sensor_node': 30,
    'state_fusion_node': 25,
    'command_output_node': 20,
    'control_node': 15,
}
SUPPORT_PRIORITY = 5
ROOT = Path(__file__).resolve().parents[2]
BIN = (ROOT / 'install/agi_ros2/lib/agi_ros2').resolve()


def find_nodes(group=None):
    found = {}
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():
            continue
        try:
            pid = int(item.name)
            executable = Path(os.readlink(item / 'exe')).resolve()
            if executable.parent != BIN or executable.name not in PRIORITIES:
                continue
            if group is not None and os.getpgid(pid) != group:
                continue
        except (OSError, ValueError):
            continue
        if executable.name in found:
            raise RuntimeError('Duplicate node: ' + executable.name)
        found[executable.name] = pid
    return found


def configure_thread(tid, priority):
    try:
        if (os.sched_getscheduler(tid) == os.SCHED_FIFO and
                os.sched_getparam(tid).sched_priority == priority):
            return
        # The thread can exit between enumeration and the privileged operation.
        result = subprocess.run(['sudo', '-n', 'chrt', '-f', '-p', str(priority), str(tid)],
                                capture_output=True, text=True)
        if result.returncode and Path(f'/proc/{tid}').exists():
            raise RuntimeError(result.stderr.strip() or 'chrt failed')
        if Path(f'/proc/{tid}').exists() and (
                os.sched_getscheduler(tid) != os.SCHED_FIFO or
                os.sched_getparam(tid).sched_priority != priority):
            raise RuntimeError(f'Thread {tid}: FIFO priority {priority} not applied')
    except ProcessLookupError:
        pass


def stop_launch(process):
    # Stop the owned process group even if the launcher has already exited.
    for sig, timeout in ((signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
        # A failed ROS launcher may have left children in its group.
        if not find_nodes(process.pid):
            break
        time.sleep(.2)


def main():
    arguments = sys.argv[1:]
    if arguments == ['--help'] or not arguments:
        print(__doc__)
        print('Usage: launch_realtime.py mode:=hardware shadow_only:=false bench_fixed_gps:=true')
        return 0
    modes = [value.split(':=', 1)[1] for value in arguments if value.startswith('mode:=')]
    if modes != ['hardware']:
        raise RuntimeError('Specify mode:=hardware exactly once; this entry is for hardware only')
    if find_nodes():
        raise RuntimeError('Hardware nodes already running; stop them before starting this entry')
    subprocess.run(['sudo', '-n', 'true'], check=True)
    subprocess.run(['chrt', '--version'], check=True, stdout=subprocess.DEVNULL)

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    process = subprocess.Popen([str(ROOT / 'agi_ros2/scripts/launch.sh'), *arguments],
                               start_new_session=True)
    try:
        deadline = time.monotonic() + 15
        while process.poll() is None:
            nodes = find_nodes(process.pid)
            if len(nodes) == len(PRIORITIES):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('Timed out waiting for all four hardware C++ nodes')
            time.sleep(.05)
        else:
            raise RuntimeError(f'Launch exited before scheduling setup: {process.returncode}')
        # Allow rclcpp/DDS startup threads to appear before setting the main policy.
        time.sleep(1)
        announced = set()
        while process.poll() is None:
            current = find_nodes(process.pid)
            if current != nodes:
                raise RuntimeError('Hardware node exited or restarted; stopping this launch')
            for name, pid in nodes.items():
                for task in (Path('/proc') / str(pid) / 'task').iterdir():
                    tid = int(task.name)
                    configure_thread(tid, PRIORITIES[name] if tid == pid else SUPPORT_PRIORITY)
                if name not in announced:
                    print(f'[AGI-RT] {name}: PID={pid}, main FIFO={PRIORITIES[name]}, '
                          f'support FIFO={SUPPORT_PRIORITY}', flush=True)
                    announced.add(name)
            # Also configure auxiliary threads created after startup.
            time.sleep(1)
        return process.returncode
    except KeyboardInterrupt:
        return 130
    finally:
        # A second interrupt must not bypass cleanup of the owned nodes.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        stop_launch(process)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f'[AGI-RT] {error}', file=sys.stderr)
        sys.exit(1)
