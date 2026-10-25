#!/usr/bin/env python3
"""No ROS/hardware: assert startup deadlines never terminate an owned live stack."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest


class StartupLifecycleTest(unittest.TestCase):
    def check_mode(self, mode):
        with tempfile.TemporaryDirectory(prefix='gcs-lifecycle-', dir='/tmp') as temp:
            root = Path(temp)
            scripts, bin_dir = root/'sh_files', root/'bin'
            scripts.mkdir(); bin_dir.mkdir(); (root/'devel').mkdir()
            (root/'devel/setup.zsh').write_text('')
            setup = root/'ros.zsh'; setup.write_text('')
            shutil.copy2(Path(__file__).with_name('one_click_takeoff.sh'), scripts/'one_click_takeoff.sh')
            marker = root/'stack-events'
            stack = scripts/'run_single_lio_debug.sh'
            stack.write_text('#!/bin/zsh\ntrap \'print stopped >> "$GCS_TEST_MARKER"; exit 0\' TERM INT HUP\nprint started >> "$GCS_TEST_MARKER"\nwhile true; do sleep 0.1; done\n')
            stack.chmod(0o755)
            for command in ('roslaunch','rosnode','rostopic','rosservice'):
                path = bin_dir/command
                path.write_text('#!/bin/sh\necho "'+command+' $*" >> "$GCS_TEST_CALLS"\nexit 1\n')
                path.chmod(0o755)
            path = bin_dir/'timeout'; path.write_text('#!/bin/sh\nshift\nexec "$@"\n'); path.chmod(0o755)
            path = bin_dir/'ps'; path.write_text('#!/bin/sh\necho S\n'); path.chmod(0o755)
            env = dict(os.environ, PATH=str(bin_dir)+':'+os.environ['PATH'], ROS_SETUP_FILE=str(setup),
                       ONE_CLICK_READY_TIMEOUT='1', ONE_CLICK_READY_STABLE_SEC='0',
                       GCS_TEST_MARKER=str(marker), GCS_TEST_CALLS=str(root/'calls'))
            process = subprocess.Popen(['/bin/zsh', str(scripts/'one_click_takeoff.sh'), mode],
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
            try:
                deadline = time.monotonic()+3
                while not marker.exists() and time.monotonic()<deadline: time.sleep(.03)
                self.assertTrue(marker.exists(), 'Mock stack did not start')
                time.sleep(2.2)  # Exceed the deliberately shortened preflight timeout.
                self.assertIsNone(process.poll(), 'Startup timeout killed the running stack')
                self.assertEqual(marker.read_text().splitlines(), ['started'])
                calls = (root/'calls').read_text()
                self.assertNotIn('rostopic pub', calls, 'Lifecycle handling emitted a flight command')
            finally:
                process.terminate()
                try: output = process.communicate(timeout=5)[0]
                except subprocess.TimeoutExpired:
                    process.kill(); output = process.communicate(timeout=2)[0]
            self.assertIn('stopped', marker.read_text())
            if mode == '--start-only': self.assertNotIn('等待系统就绪', output)
            else: self.assertIn('保留飞行栈', output)

    def test_rc_start_has_no_ground_preflight_deadline(self): self.check_mode('--start-only')
    def test_explicit_auto_preflight_failure_keeps_live_stack(self): self.check_mode('--takeoff')


if __name__ == '__main__': unittest.main()
