#!/usr/bin/env bash
# Read-only runtime evidence. Does not start nodes, open UARTs, or change scheduling.
set -eo pipefail

mode=${1:-identity}
duration=${2:-20}
repo=${AGI_REPO:-/home/ubuntu/agilicious_sia}
case "$mode" in identity|sched|write) ;; *) echo 'Usage: bash collect_runtime.sh identity|sched|write [seconds]' >&2; exit 2;; esac
[[ "$duration" =~ ^[0-9]+$ ]] && ((duration >= 1 && duration <= 60)) || {
    echo 'Duration must be 1..60 seconds' >&2; exit 2;
}
[[ -d "$repo/.git" || -f "$repo/.git" ]] || { echo "Repository missing: $repo" >&2; exit 2; }
out=$(mktemp -d /tmp/agi_${mode}.XXXXXX)
printf 'Evidence directory: %s\n' "$out"
printf '%s\n' "$mode" > "$out/mode.txt"
git -C "$repo" rev-parse HEAD > "$out/git_head.txt"
git -C "$repo" status --short > "$out/git_status.txt"
git -C "$repo" diff --binary HEAD > "$out/tracked_changes.patch"
uname -a > "$out/kernel.txt"
cat /proc/sys/kernel/random/boot_id > "$out/clock_id.txt"
ps -eLo pid,tid,comm,args > "$out/threads.txt"

python3 - "$out" <<'PY'
import hashlib
import json
import os
from pathlib import Path
import sys
import time

out = Path(sys.argv[1])
names = ('mavlink_sensor_node', 'state_fusion_node', 'control_node', 'command_output_node')
processes = []
errors = []
for process in Path('/proc').iterdir():
    if not process.name.isdigit():
        continue
    try:
        exe = os.readlink(process / 'exe')
        name = Path(exe.removesuffix(' (deleted)')).name
        if name not in names:
            continue
        pid = int(process.name)
        maps = (process / 'maps').read_text()
        (out / f'{name}_{pid}.maps').write_text(maps)
        threads = []
        for task in (process / 'task').iterdir():
            threads.append({'tid': int(task.name), 'comm': (task / 'comm').read_text().strip()})
        libraries = []
        paths = sorted({line.split(maxsplit=5)[5] for line in maps.splitlines()
                        if len(line.split(maxsplit=5)) == 6})
        for path in paths:
            if '.so' not in path or not path.startswith('/'):
                continue
            try:
                digest = hashlib.sha256(Path(path).read_bytes()).hexdigest()
                libraries.append({'path': path, 'sha256': digest})
            except OSError as error:
                libraries.append({'path': path, 'error': str(error)})
        processes.append({'node_executable': name, 'pid': pid, 'exe': exe,
                          'exe_sha256': hashlib.sha256((process / 'exe').read_bytes()).hexdigest(),
                          'cmdline': (process / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace'),
                          'threads': threads, 'libraries': libraries})
    except (OSError, ValueError) as error:
        errors.append({'pid': process.name, 'error': str(error)})
counts = {name: sum(p['node_executable'] == name for p in processes) for name in names}
(out / 'identity.json').write_text(json.dumps(
    {'monotonic_seconds': time.monotonic(), 'processes': processes, 'counts': counts,
     'errors': errors}, ensure_ascii=False, indent=2) + '\n')
for process in processes:
    print(f"{process['node_executable']}: PID={process['pid']} exe={process['exe']}")
if all(value == 1 for value in counts.values()):
    (out / 'pids.txt').write_text(','.join(str(p['pid']) for p in processes) + '\n')
    output = next(p for p in processes if p['node_executable'] == 'command_output_node')
    (out / 'output_pid.txt').write_text(str(output['pid']) + '\n')
else:
    print('Missing or duplicate nodes; tracing disabled. Counts:', counts)
PY

# CLI discovery and library hashing finish before the timed trace window.
if command -v ros2 >/dev/null; then
    for topic in /sensors/imu /fused_state /control_command; do
        timeout 15 ros2 topic info "$topic" --verbose >> "$out/qos.txt" 2>&1 || true
    done
    for interface in ControlCommand MspWriteTiming; do
        timeout 15 ros2 interface show "agi_ros2/msg/$interface" > "$out/$interface.txt" 2>&1 || true
    done
fi
if [[ "$mode" == identity ]]; then
    printf 'Identity capture finished: %s\n' "$out"
    exit 0
fi
[[ -s "$out/pids.txt" ]] || { echo 'Need exactly one running process per hardware node' >&2; exit 1; }
command -v perf >/dev/null || { echo 'perf unavailable; preserve this directory and kernel.txt' >&2; exit 1; }
perf --version > "$out/perf_version.txt"
sudo -v
pids=$(cat "$out/pids.txt")
output_pid=$(cat "$out/output_pid.txt")
python3 -c 'import time; print(time.monotonic())' > "$out/trace_start_monotonic.txt"
printf 'Tracing %s for %s seconds; keep the existing ROS bag running.\n' "$mode" "$duration"
if [[ "$mode" == sched ]]; then
    sudo perf sched record -a --clockid mono -o "$out/perf.data" -- sleep "$duration" \
        > "$out/perf_record.log" 2>&1 || {
        cat "$out/perf_record.log" >&2; exit 1;
    }
else
    # Current single-thread spin performs UART I/O on the output process main TID.
    sudo perf record -a --clockid mono \
        -e sched:sched_switch -e sched:sched_wakeup -e sched:sched_wakeup_new \
        -e syscalls:sys_enter_write --filter "common_pid == $output_pid" \
        -e syscalls:sys_exit_write --filter "common_pid == $output_pid" \
        -o "$out/perf.data" -- sleep "$duration" > "$out/perf_record.log" 2>&1 || {
        cat "$out/perf_record.log" >&2; exit 1;
    }
fi
python3 -c 'import time; print(time.monotonic())' > "$out/trace_end_monotonic.txt"
sudo perf script -i "$out/perf.data" --ns > "$out/perf_script.txt" 2> "$out/perf_script.log" || {
    echo 'perf script failed; preserve perf.data and perf_script.log' >&2;
}
sudo perf sched -i "$out/perf.data" timehist -p "$pids" -w -M --state \
    > "$out/timehist.txt" 2> "$out/timehist.log" || {
    echo 'timehist failed; preserve perf.data and timehist.log' >&2;
}
ps -eLo pid,tid,comm,args > "$out/threads_after.txt"
printf 'Finished: %s\nKeep perf.data, logs, identity.json and the matching bag/BFL.\n' "$out"
