#!/usr/bin/env python3
"""Real process tests, without ROS, camera libraries or flight hardware."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

SUPERVISOR = Path(__file__).with_name('process_supervisor.py')
WORKER = '''
import os, signal, subprocess, sys, time
from pathlib import Path
path = Path(sys.argv[1])
ignore = sys.argv[2] == "ignore"
for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    signal.signal(sig, signal.SIG_IGN if ignore else lambda *args: sys.exit(0))
child = subprocess.Popen([sys.executable, "-c", "import signal,time; "
    "signal.signal(signal.SIGINT, signal.SIG_IGN); "
    "signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)"], start_new_session=True)
path.write_text(str(os.getpid()) + " " + str(child.pid))
if sys.argv[2] == "fail":
    time.sleep(0.4)
    sys.exit(7)
while True:
    time.sleep(0.1)
'''


def alive(pid):
    result = subprocess.run(['ps', '-o', 'stat=', '-p', str(pid)],
                            capture_output=True, text=True)
    state = result.stdout.strip()
    return bool(state and not state.startswith('Z'))


class SupervisorTest(unittest.TestCase):
    def exercise(self, sig, mode='normal', ignore_terminal=False):
        with tempfile.TemporaryDirectory() as directory:
            ready = Path(directory) / 'pids'
            command = [sys.executable, str(SUPERVISOR), '--grace', '0.2']
            if ignore_terminal:
                command += ['--ignore-terminal-int']
            command += ['--', sys.executable, '-c', WORKER, str(ready), mode]
            unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
            proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            owned = []
            try:
                deadline = time.monotonic() + 4
                while not ready.exists() and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(ready.exists())
                owned = list(map(int, ready.read_text().split()))
                time.sleep(0.2)
                if ignore_terminal:
                    proc.send_signal(signal.SIGINT)
                    time.sleep(0.15)
                    self.assertIsNone(proc.poll())
                    self.assertTrue(all(alive(pid) for pid in owned))
                if sig is not None:
                    proc.send_signal(sig)
                out, err = proc.communicate(timeout=8)
                self.assertEqual(proc.returncode, 128 + sig if sig is not None else 7, (out, err))
                self.assertFalse(any(alive(pid) for pid in owned), (owned, out, err))
                self.assertIsNone(unrelated.poll())
            finally:
                for pid in owned:
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                if proc.poll() is None:
                    proc.kill()
                proc.communicate()
                unrelated.terminate()
                unrelated.wait()

    def test_ctrl_c_stops_detached_grandchildren(self):
        self.exercise(signal.SIGINT)

    def test_terminal_close_stops_tree(self):
        self.exercise(signal.SIGHUP)

    def test_stuck_native_worker_is_killed(self):
        self.exercise(signal.SIGTERM, 'ignore')

    def test_failed_startup_cleans_children_and_preserves_exit_code(self):
        self.exercise(None, 'fail')

    def test_recorder_ignores_terminal_ctrl_c_until_ordered_stop(self):
        self.exercise(signal.SIGTERM, ignore_terminal=True)


if __name__ == '__main__':
    unittest.main()
