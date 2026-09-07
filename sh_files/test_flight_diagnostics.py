#!/usr/bin/env python3

import importlib.util
import json
import os
from pathlib import Path
import signal
import shutil
import shlex
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import flight_diagnostics as diag


class DiagnosticsTests(unittest.TestCase):
    def test_privileged_probe_uses_stdin_eof_instead_of_signals(self):
        real_popen = subprocess.Popen
        def fake_helper(_argv, **kwargs):
            return real_popen([sys.executable, '-c',
                               'import sys; print("ready", flush=True); sys.stdin.buffer.read()'], **kwargs)
        with tempfile.TemporaryDirectory() as directory, patch.object(diag.subprocess, 'Popen', side_effect=fake_helper):
            probe = diag.StreamProbe(['sudo', '-n', '/usr/local/libexec/a8mini-kernel-log-reader'],
                                     Path(directory) / 'kernel.jsonl', 4096).start()
            with patch.object(probe.process, 'terminate', side_effect=AssertionError('must use EOF')):
                probe.close()
            self.assertEqual(probe.process.returncode, 0)
            self.assertFalse(probe.thread.is_alive())

    def test_kernel_access_direct_sudo_and_denied(self):
        for results, expected in (([{"returncode": 0}], "direct"),
                                  ([{"returncode": 1}, {"returncode": 0}], "sudo_read_only"),
                                  ([{"returncode": 1}, {"returncode": 1}], "unavailable")):
            with self.subTest(mode=expected), patch.object(diag, 'command_snapshot', side_effect=results) as call:
                command, status = diag.kernel_command()
                self.assertEqual(status['mode'], expected)
                if expected == 'sudo_read_only':
                    self.assertEqual(command, ['sudo', '-n', '/usr/local/libexec/a8mini-kernel-log-reader'])
                if expected != 'direct':
                    self.assertEqual(call.call_args.args[0][:2], ['sudo', '-n'])

    def test_rotation_retains_newest_records_with_bounded_size(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'metrics.jsonl'
            log = diag.BoundedLog(path, max_bytes=80, backups=2)
            for index in range(20):
                self.assertTrue(log.write({'index': index, 'text': 'x' * 30}))
            log.close()
            files = list(Path(directory).iterdir())
            self.assertEqual(len(files), 3)
            self.assertTrue(all(p.stat().st_size <= 80 for p in files))
            self.assertEqual(json.loads(path.read_text())['index'], 19)
            self.assertEqual(json.loads(Path(str(path) + '.2').read_text())['index'], 17)

    def test_log_write_failure_disables_it_instead_of_raising(self):
        with tempfile.TemporaryDirectory() as directory:
            log = diag.BoundedLog(Path(directory) / 'missing' / 'log.jsonl')
            self.assertFalse(log.write({'event': 'test'}))
            self.assertIsNotNone(log.error)
            self.assertFalse(log.write({'event': 'second'}))

    def test_counter_reset_or_first_observation_is_unknown(self):
        self.assertIsNone(diag.counter_rate(12, None, 1))
        self.assertIsNone(diag.counter_rate(12, 13, 1))
        self.assertIsNone(diag.counter_rate(12, 0, 0))
        self.assertEqual(diag.counter_rate(300, 100, 2), 100)

    def test_cpu_guest_not_double_counted_and_missing_sources_are_explicit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'stat').write_text('cpu 100 0 0 100 0 0 0 0 100 0\n')
            sampler = diag.HostSampler(root, proc=root, sysfs=root)
            self.assertIsNone(sampler.cpu()['cpu']['busy_percent'])
            (root / 'stat').write_text('cpu 200 0 0 200 0 0 0 0 200 0\n')
            self.assertEqual(sampler.cpu()['cpu']['busy_percent'], 50)
            row = sampler.sample()
            self.assertIn('unavailable', row['network'])
            self.assertIn('unavailable', row['pressure']['cpu'])
            self.assertIsInstance(row['disk_free_bytes'], int)

    def test_spawn_children_included_and_pid_reuse_does_not_fake_cpu_load(self):
        def process(root, pid, parent, start, ticks, command):
            path = root / str(pid)
            path.mkdir(exist_ok=True)
            fields = ['0'] * 22
            fields[0], fields[1], fields[11], fields[17], fields[19], fields[21] = (
                'S', str(parent), str(ticks), '2', str(start), '10')
            (path / 'stat').write_text(str(pid) + ' (python (worker)) ' + ' '.join(fields))
            (path / 'cmdline').write_bytes(command)
            (path / 'comm').write_text('python3')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            process(root, 100, 1, 5, 10, b'python\0a8mini_detection.py\0rtsp://secret-password@camera\0')
            process(root, 101, 100, 6, 20, b'python\0multiprocessing.spawn\0')
            sampler = diag.HostSampler(root, proc=root, sysfs=root)
            rows = sampler.processes(1)
            self.assertEqual([r['pid'] for r in rows], [100, 101])
            self.assertNotIn('secret-password', json.dumps(rows))
            self.assertTrue(all(r['cpu_percent_one_core'] is None for r in rows))
            process(root, 100, 1, 50, 9999, b'python\0a8mini_detection.py\0')
            rows = sampler.processes(1)
            self.assertIsNone(rows[0]['cpu_percent_one_core'])
            self.assertEqual(rows[1]['cpu_percent_one_core'], 0)

    def test_stream_probe_records_missing_tool_and_stops_owned_child(self):
        with tempfile.TemporaryDirectory() as directory:
            missing = diag.StreamProbe(['/not-a-real-diagnostics-tool'], Path(directory) / 'missing.jsonl', 2048).start()
            missing.close()
            self.assertIn('unavailable', (Path(directory) / 'missing.jsonl').read_text())
            probe = diag.StreamProbe([sys.executable, '-u', '-c',
                'import time; print("probe sample", flush=True); time.sleep(60)'],
                Path(directory) / 'probe.jsonl', 4096).start()
            try:
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    probe_path = Path(directory) / 'probe.jsonl'
                    if probe_path.exists() and 'probe sample' in probe_path.read_text():
                        break
                    time.sleep(0.01)
                else:
                    self.fail('probe did not report a sample')
            finally:
                probe.close()
            self.assertIsNotNone(probe.process.poll())
            self.assertFalse(probe.thread.is_alive())

    def test_collector_cli_writes_clock_pair_and_clean_exit(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run([sys.executable, str(Path(diag.__file__)), '--log-dir', directory,
                                     '--duration', '0.7', '--interval', '0.5', '--no-probes'],
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            rows = [json.loads(x) for x in (Path(directory) / 'host_metrics.jsonl').read_text().splitlines()]
            self.assertEqual(rows[-1]['event'], 'collector_exit')
            self.assertEqual(rows[-1]['reason'], 'duration')
            self.assertIn('monotonic_sec', rows[0])
            self.assertIn('unix_sec', rows[0])
            self.assertGreaterEqual(len(rows), 3)

    def test_debug_launcher_collects_and_cleans_probes_without_real_ros_or_camera(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scripts, binaries = root / 'sh_files', root / 'bin'
            scripts.mkdir()
            binaries.mkdir()
            (root / 'devel').mkdir()
            (root / 'devel/setup.zsh').write_text('')
            original = Path(diag.__file__).parent
            launcher = (original / 'run_single_lio_debug.sh').read_text()
            mission_csv = root / 'mission.csv'
            launcher = launcher.replace('MISSION_CSV_SOURCE="/tmp/a8mini_mission_timestamps.csv"',
                                        'MISSION_CSV_SOURCE=' + shlex.quote(str(mission_csv)))
            (scripts / 'run_single_lio_debug.sh').write_text(launcher)
            shutil.copyfile(original / 'flight_diagnostics.py', scripts / 'flight_diagnostics.py')
            (scripts / 'run_single_lio.sh').write_text('#!/bin/zsh\necho waypoint_id,arrived_time > ' +
                                                     shlex.quote(str(mission_csv)) + '\nsleep 0.5\ntouch ' +
                                                     shlex.quote(str(root / 'ready')) + '\nsleep 2\nexit 0\n')
            (scripts / 'run_single_lio.sh').chmod(0o755)
            fake = '''#!/usr/bin/python3
import os, signal, sys, time
name = os.path.basename(sys.argv[0])
if name == 'dmesg' and '--follow' not in sys.argv:
    sys.exit(0)
elif name == 'rosnode':
    if os.path.exists(os.environ['FAKE_READY']): print('/multipointplan')
elif name == 'rosparam':
    if sys.argv[1] == 'get':
        sys.exit(0 if os.path.exists(os.environ['FAKE_READY']) else 1)
    if sys.argv[1] == 'dump':
        with open(sys.argv[2], 'w') as output:
            output.write('multipointplan: loaded' if os.path.exists(os.environ['FAKE_READY']) else 'initial: partial')
elif name in ('tegrastats', 'dmesg', 'ping'):
    print('fake probe sample', flush=True)
    time.sleep(60)
elif name == 'rosbag' and len(sys.argv) > 1 and sys.argv[1] == 'record':
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    time.sleep(60)
else:
    sys.exit(0)
'''
            for name in ('rosbag', 'rosnode', 'rostopic', 'rosparam', 'ip', 'ss', 'tegrastats', 'dmesg', 'ping'):
                path = binaries / name
                path.write_text(fake)
                path.chmod(0o755)
            env = dict(os.environ, PATH=str(binaries) + os.pathsep + os.environ['PATH'],
                       FAKE_READY=str(root / 'ready'),
                       UAV_DEBUG_LOG_ROOT=str(root / 'logs'), UAV_DEBUG_DIAGNOSTICS='1',
                       UAV_DEBUG_DIAG_INTERVAL='0.5', UAV_DEBUG_DIAG_PROBES='1', UAV_DEBUG_CAMERA_PING='1')
            process = subprocess.Popen(['zsh', str(scripts / 'run_single_lio_debug.sh')], env=env,
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                       start_new_session=True)
            try:
                output, _ = process.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.communicate()
                raise
            self.assertEqual(process.returncode, 0, output)
            run_dir = next((root / 'logs').iterdir())
            metadata = (run_dir / 'metadata.txt').read_text()
            self.assertIn('diagnostics_exit_status=0', metadata)
            self.assertIn('run_status=0', metadata)
            self.assertIn('state=ready', (run_dir / 'snapshot_status.log').read_text())
            self.assertIn('initial: partial', (run_dir / 'rosparams.initial.yaml').read_text())
            self.assertIn('multipointplan: loaded', (run_dir / 'rosparams.yaml').read_text())
            self.assertTrue((run_dir / 'rosparams.final.yaml').is_file())
            # diagnostics_exit_status updates metadata at shutdown; it must not
            # invalidate a mission CSV written earlier in this same run.
            self.assertTrue((run_dir / 'mission_timestamps.csv').is_file())
            self.assertEqual((run_dir / 'mission_timestamps.csv').read_bytes(), mission_csv.read_bytes())
            rows = [json.loads(x) for x in (run_dir / 'host_metrics.jsonl').read_text().splitlines()]
            self.assertEqual(rows[-1]['event'], 'collector_exit')
            self.assertEqual(len(rows[-1]['probes']), 3)
            self.assertTrue(all(p['returncode'] is not None for p in rows[-1]['probes']))
            for name in ('tegrastats', 'kernel', 'camera_ping'):
                events = [json.loads(x) for x in (run_dir / (name + '.jsonl')).read_text().splitlines()]
                child_pid = next(x['pid'] for x in events if x['event'] == 'started')
                with self.assertRaises(ProcessLookupError):
                    os.kill(child_pid, 0)


if __name__ == '__main__':
    unittest.main()
