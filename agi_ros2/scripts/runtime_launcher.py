"""Fixed control-host entry points. No remote argv, RC actions, or restart policy."""
import fcntl
import hashlib
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import uuid

COMMANDS = {
    'hardware': {'launch_realtime': ['python3', 'agi_ros2/scripts/launch_realtime.py',
                                   'mode:=hardware', 'shadow_only:=false']},
    'simulation': {'sitl': ['python3', 'betaflight_sitl/run.py'],
                   'flight': ['./agi_ros2/scripts/launch.sh'],
                   'sim_rc': ['ros2', 'run', 'agi_ros2', 'sim_rc.py', '--ros-args', '-p', 'use_sim_time:=false']},
}


def entry_paths(mode):
    return [argv[1] if argv[0] == 'python3' else argv[0] if argv[0].startswith('./')
            else 'agi_ros2/scripts/sim_rc.py' for argv in COMMANDS[mode].values()]


def repository_root(mode):
    # Only inspect ancestors of the invocation directory and this actual
    # module (source or installed). Never assume a fixed install depth or
    # search other checkouts under /home; source and install remain colocated.
    cwd = Path.cwd().resolve()
    module_dir = Path(__file__).resolve().parent
    seen = set()
    for start in (cwd, module_dir):
        for candidate in (start, *start.parents):
            if candidate in seen:
                continue
            seen.add(candidate)
            if (candidate / 'agi_ros2/package.xml').is_file() and all(
                    (candidate / relative).is_file() for relative in entry_paths(mode)):
                return candidate
    # Preserve the listener and report the missing entry when explicitly
    # started, including the actual attempted path, as before.
    return cwd


class RuntimeLauncher:
    def __init__(self, mode, notify, root=None):
        if mode not in COMMANDS:
            raise ValueError('runtime_mode must be hardware or simulation')
        self.mode = mode
        self.root = repository_root(mode) if root is None else Path(root).resolve()
        self.notify = notify
        self.hostname = socket.gethostname()
        self.instance = f'{self.hostname}:{os.getpid()}:{uuid.uuid4().hex[:12]}'
        self.lock = threading.RLock()
        self.closing = threading.Event()
        self.stop_requested = threading.Event()
        self.worker = None
        self.processes = {}
        self.cleaned = set()
        self.state, self.reason = 'idle', '等待显式启动请求'
        self.entries = {name: dict(state='idle', pid='', exit_code='', log='', reason='尚未启动')
                        for name in COMMANDS[mode]}
        self.lock_file = None
        self.available = False
        deployment = hashlib.sha256(os.fsencode(self.root)).hexdigest()[:16]
        self.directory = Path.home() / '.local/state/agi_ros2/runtime' / deployment / mode
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            self.lock_file = (self.directory / 'listener.lock').open('a')
            fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.available = True
        except OSError as error:
            self.state, self.reason = 'failed', f'本机部署监听锁不可用（可能已有同模式监听）：{error}'

    def snapshot(self):
        with self.lock:
            return dict(mode=self.mode, hostname=self.hostname, instance=self.instance, repo_root=str(self.root),
                        state=self.state, reason=self.reason,
                        can_start=self.available and not self.closing.is_set() and self.state not in ('starting', 'stopping')
                        and not (self.worker and self.worker.is_alive())
                        and not any(e['state'] == 'running' for e in self.entries.values()),
                        can_stop=self.available and not self.closing.is_set() and self.state != 'stopping'
                        and (self.state == 'starting' or any(e['state'] == 'running' for e in self.entries.values())),
                        processes={name: dict(entry) for name, entry in self.entries.items()})

    def update(self, state=None, reason=None, name=None, **values):
        with self.lock:
            if state is not None and not self.stop_requested.is_set():
                self.state = state
            if reason is not None and not self.stop_requested.is_set():
                self.reason = reason
            if name:
                self.entries[name].update(values)
        self.notify()

    def start(self):
        # The ROS callback does no IO or waiting. Concurrent requests share this transition.
        with self.lock:
            if not self.snapshot()['can_start'] or (self.worker and self.worker.is_alive()):
                self.notify()
                return
            self.stop_requested.clear()
            self.state, self.reason = 'starting', '检查固定入口与环境'
            self.entries = {name: dict(state='idle', pid='', exit_code='', log='', reason='尚未启动')
                            for name in COMMANDS[self.mode]}
            self.processes = {}
            self.cleaned = set()
            self.worker = threading.Thread(target=self.run, name='runtime-launch', daemon=True)
            self.worker.start()
        self.notify()

    def stop(self):
        # Cancellation shares the existing worker, so the executor never waits
        # for children and start/stop cannot create overlapping cleanup owners.
        with self.lock:
            if not self.snapshot()['can_stop']:
                self.notify()
                return
            self.state, self.reason = 'stopping', '正在结束本次任务，等待所属进程退出'
            self.stop_requested.set()
            if not self.worker or not self.worker.is_alive():
                self.worker = threading.Thread(target=self.finish_stop, name='runtime-stop', daemon=True)
                self.worker.start()
        self.notify()

    def finish_stop(self):
        try:
            self.cleanup()
            remaining = [name for name, process in self.processes.items() if process.poll() is None]
            with self.lock:
                self.state = 'failed' if remaining else 'stopped'
                self.reason = ('进程尚未退出：' + ', '.join(remaining)) if remaining else '本次任务已结束；监听服务保留，可再次启动'
        except Exception as error:
            with self.lock:
                self.state, self.reason = 'failed', f'结束任务失败：{error}'
        self.notify()

    def environment(self):
        env = os.environ.copy()
        env['PYTHONUNBUFFERED'] = '1'
        # launch.sh sources this overlay itself. sim_rc must receive the same
        # environment; source only deployment-owned, known paths, never HTTP text.
        setup = self.root / 'install/agi_ros2/local_setup.bash'
        if not setup.is_file():
            raise RuntimeError(f'项目安装环境不存在：{setup}')
        distro = env.get('ROS_DISTRO')
        if distro and (not distro.isalnum()):
            raise RuntimeError('ROS_DISTRO must be an alphanumeric distribution name')
        ros_setup = Path('/opt/ros') / (distro or ('jazzy' if Path('/opt/ros/jazzy/setup.bash').is_file()
                                                else 'humble')) / 'setup.bash'
        if not ros_setup.is_file():
            raise RuntimeError(f'ROS 环境不存在：{ros_setup}')
        result = subprocess.run(['/bin/bash', '-c', 'set -e; source "$1" >&2; source "$2" >&2; env -0',
                                 'runtime-environment', str(ros_setup), str(setup)],
                                cwd=self.root, env=env, capture_output=True, timeout=15)
        if result.returncode:
            raise RuntimeError('source 环境失败：' + result.stderr.decode(errors='replace')[-1200:])
        return dict(item.decode().split('=', 1) for item in result.stdout.split(b'\0') if b'=' in item)

    def spawn(self, name, env, readers):
        if self.stop_requested.is_set():
            raise RuntimeError('本次任务正在结束')
        log = self.directory / (uuid.uuid4().hex + '-' + name + '.log')
        self.update(name=name, reason='正在启动 ' + name, log=str(log))
        with log.open('wb') as output:
            process = subprocess.Popen(COMMANDS[self.mode][name], cwd=self.root, env=env,
                                       stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        self.processes[name] = process
        readers[name] = log.open('rb')
        with self.lock:
            self.entries[name].update(state='running', pid=str(process.pid), reason='入口进程存活')
        self.notify()

    def read_log(self, name, readers, pending):
        # Files, not PIPEs: builds cannot block on a full stdout pipe. Bounded
        # reads keep monitoring responsive even during very verbose builds.
        chunk = readers[name].read(65536)
        lines = (pending.get(name, b'') + chunk).split(b'\n')
        pending[name] = lines.pop()[-4096:]
        return [line.decode(errors='replace').strip() for line in lines]

    def record_exit(self, name, code):
        entry = self.entries[name]
        with Path(entry['log']).open('rb') as stream:
            stream.seek(0, 2)
            stream.seek(max(0, stream.tell() - 8192))
            tail = stream.read().decode(errors='replace').strip()[-1200:]
        with self.lock:
            entry.update(state='exited', exit_code=str(code), reason=f'退出码 {code}；{tail}')
        self.notify()

    def run(self):
        readers, pending = {}, {}
        step = 'environment'
        starting = True
        try:
            for relative in entry_paths(self.mode):
                if not (self.root / relative).is_file():
                    raise RuntimeError(f'固定入口不存在：{self.root / relative}；未能从工作目录或监听脚本的父目录找到完整仓库，请 cd 到实际仓库根目录后启动监听服务')
            env = self.environment()
            step = 'launch_realtime' if self.mode == 'hardware' else 'sitl'
            self.spawn(step, env, readers)
            announced = set()
            ready = False
            while not self.stop_requested.wait(.1):
                for name, process in list(self.processes.items()):
                    lines = self.read_log(name, readers, pending)
                    if starting and name == step and lines:
                        self.update(reason=f'{step}：{lines[-1][-240:]}')
                    if name == 'sitl':
                        ready |= any(line == '[AGI-SITL] Betaflight SITL is ready' for line in lines)
                    if name == 'launch_realtime':
                        for line in lines:
                            if line.startswith('[AGI-RT] ') and 'main FIFO=' in line:
                                announced.add(line.split(':', 1)[0])
                        ready = len(announced) == 4
                    code = process.poll()
                    if code is not None and self.entries[name]['state'] != 'exited':
                        self.record_exit(name, code)
                        if starting:
                            step = name
                            raise RuntimeError(self.entries[name]['reason'])
                        self.update(state='failed', reason=f'{name} 已退出；{self.entries[name]["reason"]}')
                        self.cleanup([name])
                if starting and ready:
                    # The marker is only read from this invocation's new log,
                    # and all owned entries were polled alive immediately above.
                    if self.mode == 'simulation':
                        for step in ('flight', 'sim_rc'):
                            for name, process in self.processes.items():
                                if process.poll() is not None:
                                    self.record_exit(name, process.returncode)
                                    step = name
                                    raise RuntimeError(self.entries[name]['reason'])
                            self.spawn(step, env, readers)
                        ready = False
                        # One more monitoring pass catches immediate spawn failures.
                        continue
                if starting and ((self.mode == 'hardware' and ready) or len(self.processes) == 3):
                    starting = False
                    self.update(state='running', reason='所有入口进程存活；不代表飞行就绪或 ARM/AUTO/输出授权')
                if not starting and all(entry['state'] == 'exited' for entry in self.entries.values()):
                    break
        except Exception as error:
            self.update(state='failed', reason=f'{step}：{error}')
            if step in self.entries and step not in self.processes:
                with self.lock:
                    self.entries[step].update(state='failed', reason=str(error))
            self.cleanup()
        finally:
            if self.stop_requested.is_set():
                self.finish_stop()
            for reader in readers.values():
                reader.close()
            self.notify()

    def cleanup(self, names=None):
        # Give each entry's own finally (including independent SITL sessions)
        # time to clean up, then address only its owned original process group.
        for name, process in reversed(list(self.processes.items())):
            if name in self.cleaned or (names is not None and name not in names):
                continue
            for sig, timeout in ((signal.SIGINT, 25), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    continue
                # A launcher can exit before descendants in its group.
                if sig != signal.SIGKILL:
                    time.sleep(.1)
            if process.poll() is not None:
                self.cleaned.add(name)
                self.record_exit(name, process.returncode)

    def close(self):
        self.closing.set()
        self.stop_requested.set()
        if self.worker:
            self.worker.join()
        # Running monitor may have returned after every entry exited.
        if self.processes and not self.worker.is_alive():
            self.cleanup()
        if self.lock_file:
            self.lock_file.close()
