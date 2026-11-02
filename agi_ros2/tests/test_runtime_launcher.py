#!/usr/bin/env python3
"""Fixed-launch tests using owned short processes, never the flight/hardware chain."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import runtime_launcher as runtime


def wait_for(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError('condition timed out')


class RepositoryRootTests(unittest.TestCase):
    def test_source_installed_and_subdirectory_for_both_deployments(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp).resolve()
            unrelated = base / 'outside'
            unrelated.mkdir()
            for user, directory, mode in [('sia', 'agilicious_internal-main', 'simulation'),
                                          ('ubuntu', 'agilicious_sia', 'hardware')]:
                root = base / 'home' / user / directory
                for relative in ['agi_ros2/package.xml', *runtime.entry_paths(mode)]:
                    file = root / relative
                    file.parent.mkdir(parents=True, exist_ok=True)
                    file.touch()
                for relative in ['agi_ros2/scripts/runtime_launcher.py',
                                 'install/agi_ros2/lib/agi_ros2/runtime_launcher.py']:
                    module = root / relative
                    with patch.object(runtime, '__file__', str(module)), patch.object(runtime.Path, 'cwd', return_value=unrelated):
                        self.assertEqual(runtime.repository_root(mode), root)
                # An explicit invocation inside a checkout takes precedence.
                with patch.object(runtime, '__file__', str(unrelated / 'runtime_launcher.py')), \
                        patch.object(runtime.Path, 'cwd', return_value=root / 'agi_ros2/scripts'):
                    self.assertEqual(runtime.repository_root(mode), root)
            # No whole-disk or sibling search: the deployments above cannot be
            # discovered from an unrelated installation and unrelated cwd.
            with patch.object(runtime, '__file__', str(unrelated / 'runtime_launcher.py')), \
                    patch.object(runtime.Path, 'cwd', return_value=unrelated):
                self.assertEqual(runtime.repository_root('simulation'), unrelated)


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        for file in ('agi_ros2/scripts/launch_realtime.py', 'betaflight_sitl/run.py',
                     'agi_ros2/scripts/launch.sh', 'agi_ros2/scripts/sim_rc.py'):
            path = self.root / file
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
        self.home = patch.object(runtime.Path, 'home', return_value=self.root)
        self.home.start()
        self.addCleanup(self.home.stop)
        self.calls = []
        self.events = []
        self.scripts = {}
        self.real_popen = subprocess.Popen
        self.fake = patch.object(runtime.subprocess, 'Popen', side_effect=self.spawn)
        self.fake.start()
        self.addCleanup(self.fake.stop)
        self.env = patch.object(runtime.RuntimeLauncher, 'environment', return_value=dict(os.environ, PYTHONUNBUFFERED='1'))
        self.env.start()
        self.addCleanup(self.env.stop)

    def spawn(self, argv, **kwargs):
        self.calls.append((list(argv), kwargs))
        name = next(name for commands in runtime.COMMANDS.values() for name, value in commands.items() if argv == value)
        script = self.scripts.get(name, 'import time; time.sleep(60)')
        return self.real_popen([sys.executable, '-u', '-c', script], **kwargs)

    def launcher(self, mode='simulation'):
        item = runtime.RuntimeLauncher(mode, lambda: self.events.append(time.monotonic()), self.root)
        self.addCleanup(item.close)
        return item

    def ready_script(self):
        return ("import time; from pathlib import Path; "
                "print('early TCP open is not readiness', flush=True); "
                "\nwhile not Path('ready').exists(): time.sleep(.02)\n"
                "print('[AGI-SITL] Betaflight SITL is ready', flush=True); time.sleep(60)")

    def test_order_fixed_arguments_concurrency_and_ownership(self):
        self.scripts['sitl'] = self.ready_script()
        item = self.launcher()
        # An old log must not be mistaken for this invocation's readiness.
        (item.directory / 'old.log').write_text('[AGI-SITL] Betaflight SITL is ready\n')
        external = self.real_popen([sys.executable, '-c', 'import time; time.sleep(60)'])
        self.addCleanup(lambda: (external.terminate(), external.wait()))
        threads = [threading.Thread(target=item.start) for _ in range(20)]
        for thread in threads: thread.start()
        for thread in threads: thread.join()
        wait_for(lambda: len(self.calls) == 1)
        time.sleep(.3)
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(item.snapshot()['state'], 'starting')
        duplicate = self.launcher()
        self.assertFalse(duplicate.snapshot()['can_start'])
        duplicate.start()
        self.assertEqual(len(self.calls), 1)
        (self.root / 'ready').touch()
        wait_for(lambda: item.snapshot()['state'] == 'running')
        self.assertEqual([call[0] for call in self.calls], list(runtime.COMMANDS['simulation'].values()))
        for _, kwargs in self.calls:
            self.assertEqual(kwargs['cwd'], self.root)
            self.assertTrue(kwargs['start_new_session'])
            self.assertEqual(kwargs['env']['PYTHONUNBUFFERED'], '1')
            self.assertNotIn('shell', kwargs)
        item.start()
        self.assertEqual(len(self.calls), 3)
        item.processes['flight'].terminate()
        wait_for(lambda: item.snapshot()['state'] == 'failed')
        item.start()
        self.assertEqual(len(self.calls), 3)  # Partial running group is not duplicated.
        self.assertIsNone(external.poll())
        item.close()
        self.assertTrue(all(p.poll() is not None for p in item.processes.values()))
        self.assertIsNone(external.poll())

    def test_failure_before_marker_does_not_continue(self):
        self.scripts['sitl'] = "print('EEPROM startup failure', flush=True); raise SystemExit(7)"
        item = self.launcher()
        item.start()
        wait_for(lambda: item.snapshot()['state'] == 'failed')
        item.worker.join()
        self.assertEqual(len(self.calls), 1)
        self.assertIn('EEPROM startup failure', item.snapshot()['reason'])
        self.assertEqual(item.snapshot()['processes']['sitl']['exit_code'], '7')

    def test_partial_spawn_failure_cleans_only_owned(self):
        self.scripts['sitl'] = self.ready_script()
        (self.root / 'ready').touch()
        spawn = self.spawn
        def fail(argv, **kwargs):
            if argv == runtime.COMMANDS['simulation']['sim_rc']:
                raise OSError('synthetic sim_rc spawn failure')
            return spawn(argv, **kwargs)
        self.fake.stop()
        with patch.object(runtime.subprocess, 'Popen', side_effect=fail):
            item = self.launcher()
            item.start()
            wait_for(lambda: item.snapshot()['state'] == 'failed')
            item.worker.join()
            self.assertIn('sim_rc', item.snapshot()['reason'])
            self.assertTrue(all(p.poll() is not None for p in item.processes.values()))
            self.assertEqual(item.snapshot()['processes']['sim_rc']['state'], 'failed')

    def test_hardware_permission_error_and_fixed_command(self):
        self.scripts['launch_realtime'] = "print('sudo: a password is required'); raise SystemExit(1)"
        item = self.launcher('hardware')
        item.start()
        wait_for(lambda: item.snapshot()['state'] == 'failed')
        item.worker.join()
        self.assertEqual(self.calls[0][0], ['python3', 'agi_ros2/scripts/launch_realtime.py',
                                          'mode:=hardware', 'shadow_only:=false'])
        self.assertIn('sudo: a password is required', item.snapshot()['reason'])
        self.assertNotIn('bench_fixed_gps', str(self.calls[0][0]))
        self.scripts['launch_realtime'] = "print('bench_fixed_gps requires hardware MAVLink sensors'); raise SystemExit(1)"
        item.start()  # Only an explicit new request retries an entirely exited group.
        wait_for(lambda: item.snapshot()['state'] == 'failed')
        item.worker.join()
        self.assertEqual(len(self.calls), 2)
        self.assertIn('bench_fixed_gps requires hardware MAVLink sensors', item.snapshot()['reason'])

    def test_hardware_waits_for_existing_scheduling_messages(self):
        self.scripts['launch_realtime'] = ("import time\nfor n in ['mavlink_sensor_node', 'fc_state_adapter_node', "
            "'command_output_node', 'control_node']: print('[AGI-RT] '+n+': PID=123, main FIFO=5, support FIFO=5', flush=True)\n"
            "time.sleep(60)")
        item = self.launcher('hardware')
        item.start()
        wait_for(lambda: item.snapshot()['state'] == 'running')
        self.assertEqual(len(self.calls), 1)
        self.assertGreater(len(self.events), 2)
        item.stop()
        item.worker.join(timeout=5)
        self.assertEqual(item.snapshot()['state'], 'stopped')
        self.assertIsNotNone(item.processes['launch_realtime'].poll())

    def test_term_during_eeprom_uses_run_py_early_cleanup(self):
        # Exercise run.py's real preparation finally with an independent-session
        # fake ELF; never bind SITL ports or load the user's simulator config.
        source = self.root / 'fake-elf'
        source.write_text('#!' + sys.executable + '\nimport os,time\nfrom pathlib import Path\n'
                          'Path("config-pid").write_text(str(os.getpid()))\ntime.sleep(60)\n')
        source.chmod(0o755)
        wrapper = """
import sys,time
from pathlib import Path
sys.path.insert(0, sys.argv[1])
import run
root = Path(sys.argv[2])
sys.argv = [run.__file__]
def wait(*args, **kwargs):
    while True: time.sleep(.02)
run.wait_for_tcp = wait
def prepare(_):
    run.prepare_runtime_betaflight(root/'fake-elf', root/'configured')
run.load_betaloop_config = prepare
run.main()
"""
        process = self.real_popen([sys.executable, '-c', wrapper,
                                  str(Path(__file__).resolve().parents[2] / 'betaflight_sitl'), str(self.root)],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        try:
            pidfile = self.root / 'configured/config-pid'
            wait_for(pidfile.exists)
            child = int(pidfile.read_text())
            self.assertNotEqual(os.getpgid(child), process.pid)
            process.terminate()
            process.wait(timeout=6)
            wait_for(lambda: not Path(f'/proc/{child}').exists())
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=6)

    def test_close_during_build_cleans_owned_group(self):
        self.scripts['sitl'] = ("import subprocess,sys,time; from pathlib import Path; "
            "p=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); "
            "Path('build-pid').write_text(str(p.pid)); "
            "\ntry: p.wait()\nfinally: p.wait()\n")
        item = self.launcher()
        item.start()
        wait_for(lambda: (self.root / 'build-pid').exists())
        child = int((self.root / 'build-pid').read_text())
        item.close()
        wait_for(lambda: not Path(f'/proc/{child}').exists())
        self.assertEqual(len(self.calls), 1)

    def test_stop_during_start_and_running_then_start_again(self):
        self.scripts['sitl'] = self.ready_script()
        item = self.launcher()
        external = self.real_popen([sys.executable, '-c', 'import time; time.sleep(60)'])
        self.addCleanup(lambda: (external.terminate(), external.wait()))
        item.start()
        wait_for(lambda: len(item.processes) == 1)
        started = time.monotonic()
        item.stop()
        self.assertLess(time.monotonic() - started, .1)
        self.assertEqual(item.snapshot()['state'], 'stopping')
        threads = [threading.Thread(target=action) for action in [item.stop, item.start] * 10]
        for thread in threads: thread.start()
        for thread in threads: thread.join()
        item.worker.join(timeout=5)
        self.assertEqual(item.snapshot()['state'], 'stopped')
        self.assertEqual(len(self.calls), 1)  # Cancel before readiness: no flight/sim_rc.
        self.assertTrue(item.snapshot()['can_start'])
        self.assertFalse(item.snapshot()['can_stop'])
        self.assertIsNone(external.poll())
        (self.root / 'ready').touch()
        item.start()
        wait_for(lambda: item.snapshot()['state'] == 'running')
        self.assertEqual(len(self.calls), 4)
        self.assertTrue(item.snapshot()['can_stop'])
        item.stop()
        item.worker.join(timeout=5)
        self.assertEqual(item.snapshot()['state'], 'stopped')
        self.assertTrue(all(p.poll() is not None for p in item.processes.values()))
        self.assertTrue(all(e['state'] == 'exited' for e in item.snapshot()['processes'].values()))
        self.assertIsNone(external.poll())
        item.stop()
        self.assertEqual(item.snapshot()['state'], 'stopped')

    def test_stop_while_sourcing_prevents_any_entry(self):
        item = self.launcher('hardware')
        gate = threading.Event()
        with patch.object(item, 'environment', side_effect=lambda: (gate.wait(5), dict(os.environ))[1]):
            item.start()
            item.stop()
            gate.set()
            item.worker.join(timeout=5)
        self.assertEqual(item.snapshot()['state'], 'stopped')
        self.assertEqual(self.calls, [])
        self.assertTrue(item.snapshot()['can_start'])

    def test_shutdown_fixed_command_async_idempotent_and_hardware_only(self):
        hardware = self.launcher('hardware')
        simulation = self.launcher('simulation')
        duplicate = self.launcher('hardware')
        gate = threading.Event()
        entered = threading.Event()
        def execute(argv, **kwargs):
            self.assertEqual(argv, ['sudo', '-n', 'shutdown', 'now'])
            self.assertEqual(kwargs['cwd'], self.root)
            self.assertEqual(kwargs['env'], dict(os.environ))
            self.assertEqual(kwargs['stdin'], subprocess.DEVNULL)
            self.assertNotIn('shell', kwargs)
            entered.set()
            gate.wait(5)
            return subprocess.CompletedProcess(argv, 0)
        with patch.object(runtime.subprocess, 'run', side_effect=execute) as run:
            try:
                simulation.shutdown()
                duplicate.shutdown()
                run.assert_not_called()
                threads = [threading.Thread(target=hardware.shutdown) for _ in range(20)]
                for thread in threads: thread.start()
                for thread in threads: thread.join(timeout=1)
                self.assertTrue(all(not thread.is_alive() for thread in threads))
                self.assertTrue(entered.wait(1))
                self.assertEqual(hardware.snapshot()['shutdown_state'], 'requesting')
                self.assertFalse(hardware.snapshot()['can_start'])
                hardware.start()
                self.assertEqual(self.calls, [])
                self.assertEqual(run.call_count, 1)
            finally:
                gate.set()
                hardware.shutdown_worker.join(timeout=5)
            self.assertEqual(hardware.snapshot()['shutdown_state'], 'accepted')
            self.assertEqual(hardware.snapshot()['shutdown_exit_code'], '0')
            self.assertFalse(hardware.snapshot()['can_shutdown'])
            self.assertFalse(hardware.snapshot()['can_start'])
            hardware.shutdown()
            self.assertEqual(run.call_count, 1)

    def test_shutdown_permission_failure_preserves_runtime_and_explicit_retry(self):
        item = self.launcher('hardware')
        item.update(state='running', reason='existing runtime')
        def denied(argv, **kwargs):
            kwargs['stdout'].write(b'sudo: a password is required\n')
            return subprocess.CompletedProcess(argv, 1)
        with patch.object(runtime.subprocess, 'run', side_effect=denied), patch.object(item, 'cleanup') as cleanup:
            item.shutdown()
            item.shutdown_worker.join(timeout=5)
            cleanup.assert_not_called()
        snapshot = item.snapshot()
        self.assertEqual(snapshot['state'], 'running')
        self.assertEqual(snapshot['reason'], 'existing runtime')
        self.assertEqual(snapshot['shutdown_state'], 'failed')
        self.assertIn('sudo: a password is required', snapshot['shutdown_reason'])
        self.assertTrue(Path(snapshot['shutdown_log']).is_file())
        self.assertTrue(snapshot['can_shutdown'])
        with patch.object(runtime.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0)) as run:
            item.shutdown()
            item.shutdown_worker.join(timeout=5)
            run.assert_called_once()
        self.assertEqual(item.snapshot()['shutdown_state'], 'accepted')

    def test_shutdown_timeout_is_unknown_and_not_retried(self):
        item = self.launcher('hardware')
        with patch.object(runtime.subprocess, 'run', side_effect=subprocess.TimeoutExpired(['fake-shutdown'], 15)) as run:
            item.shutdown()
            item.shutdown_worker.join(timeout=5)
            self.assertEqual(item.snapshot()['shutdown_state'], 'unknown')
            self.assertIn('未确认', item.snapshot()['shutdown_reason'])
            item.shutdown()
            self.assertEqual(run.call_count, 1)
            self.assertFalse(item.snapshot()['can_shutdown'])

    def test_missing_entry_and_invalid_mode(self):
        with self.assertRaises(ValueError): self.launcher('other')
        (self.root / 'betaflight_sitl/run.py').unlink()
        item = self.launcher()
        item.start()
        wait_for(lambda: item.snapshot()['state'] == 'failed')
        self.assertIn('固定入口不存在', item.snapshot()['reason'])
        self.assertEqual(self.calls, [])


def smoke(output):
    """Real UI launch in an explicitly isolated local domain; no RC manipulation."""
    import json
    import socket
    import urllib.request
    from playwright.sync_api import sync_playwright, expect
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('Smoke requires an explicitly isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    root = Path(__file__).resolve().parents[2]
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    url = f'http://127.0.0.1:{port}'
    processes, streams, owned_pids = [], [], []
    result = dict(domain=os.environ['ROS_DOMAIN_ID'], rc_actions=0, transitions=[])
    def start(name, argv):
        stream = (output / (name + '.log')).open('w')
        streams.append(stream)
        process = subprocess.Popen(argv, cwd=root, env=os.environ.copy(), stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process
    def snapshot():
        with urllib.request.urlopen(url + '/api/state', timeout=2) as response:
            return json.load(response)
    def values(item):
        return {v['key']: v['value'] for v in item['values']}
    def status(snap):
        return {item['name']: values(item) for item in snap['topics'].get('runtime/simulation/status',
                                                    {}).get('data', {}).get('status', [])}
    def alive(pid):
        try:
            return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0] != 'Z'
        except FileNotFoundError:
            return False
    def descendants(roots):
        parents = {}
        for path in Path('/proc').iterdir():
            if path.name.isdigit():
                try:
                    parents[int(path.name)] = int((path / 'stat').read_text().rsplit(')', 1)[1].split()[1])
                except (OSError, ValueError):
                    pass
        found = set(roots)
        while True:
            added = {pid for pid, parent in parents.items() if parent in found} - found
            if not added: return sorted(found)
            found.update(added)
    try:
        start('listener', ['ros2', 'run', 'agi_ros2', 'ground_station_telemetry.py',
                           '--ros-args', '-p', 'runtime_mode:=simulation'])
        ui = start('ui', ['ros2', 'run', 'agi_ros2', 'map_ui.py', '--port', str(port),
                          '--mbtiles', str(output / 'no-map-needed.mbtiles')])
        def listening():
            try: return status(snapshot()).get('summary', {}).get('state') == 'idle'
            except (OSError, ValueError): return False
        wait_for(listening, 20)
        assert snapshot()['position'] is None
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(executable_path='/opt/google/chrome/chrome',
                                                 headless=True, args=['--no-sandbox'])
            page = browser.new_page(viewport={'width':1440, 'height':1100})
            page.route('**/*', lambda route: route.continue_() if route.request.url.startswith(url) else route.abort())
            requests=[]
            stops=[]
            page.on('request', lambda r: requests.append(r.url) if r.url.endswith('/api/runtime/start') else None)
            page.goto(url)
            page.locator('#runtime-mode').select_option('simulation')
            expect(page.locator('#runtime-service')).to_contain_text('启动服务在线')
            expect(page.locator('#runtime-start')).to_be_enabled()
            page.screenshot(path=str(output / 'idle.png'), full_page=True)
            page.locator('#runtime-start').click()
            expect(page.locator('#runtime-request')).to_have_text('启动请求已发送，等待执行端反馈')
            deadline = time.monotonic() + 600  # Test budget, not a production startup deadline.
            previous = None
            while time.monotonic() < deadline:
                snap = snapshot()
                current = status(snap)
                summary = current['summary']
                key = (summary['state'], summary['reason'])
                if key != previous:
                    result['transitions'].append(dict(at=time.time(), **summary))
                    print('SMOKE', key, flush=True)
                    previous = key
                if summary['state'] == 'failed': raise AssertionError(summary['reason'])
                if summary['state'] == 'running': break
                page.wait_for_timeout(500)
            else: raise AssertionError('smoke test startup budget exhausted')
            result['running'] = current
            owned_pids = [int(current[name]['pid']) for name in ('sitl', 'flight', 'sim_rc')]
            def receiver():
                authority = snapshot()['topics'].get('authority', {}).get('data')
                if not authority: return False
                assert not authority['armed'] and not authority['auto_switch'] and authority['kill']
                assert authority['manual_aetr'] == [1500,1500,1000,1500]
                result['authority'] = authority
                return True
            wait_for(receiver, 15)
            page.reload()
            expect(page.locator('#runtime-mode')).to_have_value('simulation')
            expect(page.locator('#runtime-state')).to_have_text('运行中')
            page.wait_for_timeout(1500)
            assert len(requests) == 1
            after = status(snapshot())
            assert [int(after[name]['pid']) for name in ('sitl','flight','sim_rc')] == owned_pids
            result['refresh_same_pids'] = True
            result['start_requests'] = len(requests)
            page.screenshot(path=str(output / 'running.png'), full_page=True)
            page.close()
            assert all(alive(pid) for pid in owned_pids)
            result['page_close_keeps_runtime'] = True
            page = browser.new_page()
            page.on('request', lambda r: stops.append(r.url) if r.url.endswith('/api/runtime/stop') else None)
            page.goto(url)
            page.locator('#runtime-mode').select_option('simulation')
            expect(page.locator('#runtime-stop')).to_be_enabled()
            descendants_before_stop = descendants(owned_pids)
            result['owned_descendants_before_stop'] = descendants_before_stop
            stop_time = time.monotonic()
            page.locator('#runtime-stop').click()
            expect(page.locator('#runtime-request')).to_have_text('结束任务请求已发送，等待执行端反馈')
            expect(page.locator('#runtime-state')).to_have_text('已结束', timeout=110000)
            expect(page.locator('#runtime-service')).to_contain_text('启动服务在线')
            expect(page.locator('#runtime-start')).to_be_enabled()
            expect(page.locator('#runtime-stop')).to_be_disabled()
            wait_for(lambda: all(not alive(pid) for pid in descendants_before_stop), 15)
            result['all_recorded_descendants_exited'] = True
            result['stop_seconds'] = time.monotonic() - stop_time
            assert all(not alive(pid) for pid in owned_pids)
            assert status(snapshot())['summary']['instance'] == current['summary']['instance']
            page.reload()
            expect(page.locator('#runtime-state')).to_have_text('已结束')
            assert len(stops) == 1
            result['stop_requests'] = len(stops)
            result['stop_keeps_listener'] = True
            result['stopped'] = status(snapshot())
            page.screenshot(path=str(output / 'stopped.png'), full_page=True)
            page.close()
            os.killpg(ui.pid, signal.SIGINT)
            ui.wait(timeout=10)
            browser.close()
        result['passed'] = True
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=115)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for stream in streams: stream.close()
        result['owned_entries_cleaned'] = all(not alive(pid) for pid in owned_pids)
        (output / 'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False))
    assert result['owned_entries_cleaned']
    print('SMOKE PASS:', output, flush=True)


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--smoke':
        smoke(sys.argv[2])
    else:
        unittest.main()
