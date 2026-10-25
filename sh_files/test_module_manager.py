#!/usr/bin/env python3
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location('module_manager', str(Path(__file__).with_name('module_manager.py')))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class ManagerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir='/tmp')
        self.manager = mod.Manager(Path(self.temp.name), Path(self.temp.name))

    def tearDown(self):
        self.temp.cleanup()

    def child(self, name, options=None):
        process = Mock(pid=12345)
        process.poll.return_value = None
        self.manager.children[name] = dict(process=process, started=time.monotonic(), options=mod.normalize(options or {}))
        return process

    def test_dependencies_never_enable_flight(self):
        options = mod.normalize({'auto_record': True, 'save_video': True})
        self.assertTrue(options['gimbal'] and options['yolo'])
        self.assertFalse(options['flight'] or options['rviz'])
        self.assertTrue(mod.normalize({'yolo': True})['yolo'])
        self.assertFalse(mod.normalize({'yolo': True})['gimbal'])
        for invalid in ({'other': True}, {'flight': 'true'}, [], None):
            with self.assertRaises(ValueError):
                mod.normalize(invalid)

    def test_flight_launch_is_headless_rc_only(self):
        args, env = self.manager.command('flight', mod.normalize({}))
        self.assertIn('--start-only', args)
        self.assertNotIn('--takeoff', args)
        self.assertEqual(env['UAV_START_RVIZ'], 'false')
        self.assertEqual(env['A8MINI_START_GIMBAL_NODE'], 'false')
        self.assertEqual(env['A8MINI_START_DETECTION'], 'false')

    @patch.object(mod, 'registered_nodes', return_value=set())
    def test_connect_and_empty_selection_launch_nothing(self, nodes):
        with patch.object(self.manager, 'spawn') as spawn, patch.object(self.manager, 'ensure_ros') as core:
            response = self.manager.dispatch({'action': 'status'})
            self.assertTrue(response['ok'])
            self.assertEqual(self.manager.start({}), [])
            spawn.assert_not_called()
            core.assert_not_called()

    @patch.object(mod, 'registered_nodes', return_value=set())
    def test_camera_selection_does_not_launch_flight(self, nodes):
        with patch.object(self.manager, 'spawn') as spawn, patch.object(self.manager, 'ensure_ros'):
            result = self.manager.start({'save_video': True})
            self.assertTrue(result[0]['ok'])
            self.assertEqual([call.args[0] for call in spawn.call_args_list], ['yolo'])

    @patch.object(mod, 'registered_nodes', return_value={'/px4ctrl'})
    def test_external_flight_not_adopted_or_duplicated(self, nodes):
        with patch.object(self.manager, 'spawn') as spawn:
            result = self.manager.start({'flight': True})
            self.assertFalse(result[0]['ok'])
            spawn.assert_not_called()
        self.assertIn('external', self.manager.status()['modules'][0]['state'])

    @patch.object(mod, 'registered_nodes', return_value=set())
    def test_live_configuration_change_does_not_restart(self, nodes):
        self.child('yolo', {'yolo': True})
        with patch.object(self.manager, 'spawn') as spawn:
            self.assertFalse(self.manager.start({'save_video': True})[0]['ok'])
            spawn.assert_not_called()

    def test_stop_flight_guard_failure_never_signals(self):
        self.child('flight')
        with patch.object(self.manager, 'ground_guard', side_effect=ValueError('armed')), patch.object(mod.os, 'killpg') as kill:
            with self.assertRaises(ValueError):
                self.manager.stop('flight')
            kill.assert_not_called()

    def test_ground_guard_requires_fresh_messages(self):
        for state in (dict(connected=True, armed=False, ground=True, fresh=False),
                      dict(connected=True, armed=True, ground=True, fresh=True),
                      dict(connected=True, armed=False, ground=False, fresh=True)):
            with patch.object(mod.subprocess, 'run', return_value=Mock(stdout=json.dumps(state))):
                with self.assertRaises(ValueError): self.manager.ground_guard()
        state = dict(connected=True, armed=False, ground=True, fresh=True)
        with patch.object(mod.subprocess, 'run', return_value=Mock(stdout=json.dumps(state))):
            self.manager.ground_guard()

    def test_stop_only_owned_tree(self):
        self.child('flight')
        with patch.object(self.manager, 'ground_guard'), patch.object(mod.os, 'killpg') as kill:
            self.manager.stop('flight')
            kill.assert_called_once_with(12345, signal.SIGINT)
        with patch.object(mod.os, 'killpg') as kill:
            self.manager.stop('gimbal')
            kill.assert_not_called()

    @patch.object(mod, 'registered_nodes', return_value=set())
    def test_optional_failure_does_not_stop_flight(self, nodes):
        flight = self.child('flight')
        with patch.object(self.manager, 'command', side_effect=ValueError('no display')), patch.object(mod.os, 'killpg') as kill:
            self.assertFalse(self.manager.start({'rviz': True})[0]['ok'])
            kill.assert_not_called()
            self.assertTrue(self.manager.alive('flight'))

    def test_unknown_and_shell_like_requests_rejected(self):
        for request in ({'action': 'exec', 'command': 'x'}, {'action': 'stop', 'module': ';kill -9 1'}, {'action': 'logs', 'module': '../secret'}):
            with self.assertRaises(ValueError):
                self.manager.dispatch(request)

    def test_headless_payload_launch_overrides_yaml(self):
        path = Path(__file__).with_name('gcs_payload.launch')
        if not path.exists():
            path = Path(__file__).resolve().parents[1] / 'src/user_command/multipoint/launch/gcs_payload.launch'
        text = path.read_text()
        self.assertIn('name="display" value="false"', text)
        self.assertIn('name="enabled" value="true"', text)
        self.assertNotIn('one_click_takeoff', text)

    def test_ssh_client_exit_does_not_stop_manager(self):
        home = Path(self.temp.name)
        script = home / 'workspace/sh_files/module_manager.py'
        script.parent.mkdir(parents=True)
        script.write_bytes(Path(mod.__file__).read_bytes())
        env = dict(os.environ, HOME=str(home))
        daemon_pid = None
        try:
            def request():
                result = subprocess.run([sys.executable, str(script), '--request'], input='{"action":"status"}\n',
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env, timeout=10)
                self.assertEqual(result.returncode, 0, result.stderr)
                return json.loads(result.stdout)
            first = request()
            daemon_pid = first['manager_pid']
            second = request()
            self.assertEqual(second['manager_pid'], daemon_pid)
            self.assertTrue(all(row['state'] in ('stopped', 'external') for row in second['status']['modules']))
        finally:
            if daemon_pid:
                os.kill(daemon_pid, signal.SIGTERM)
                time.sleep(.1)


if __name__ == '__main__':
    unittest.main()
