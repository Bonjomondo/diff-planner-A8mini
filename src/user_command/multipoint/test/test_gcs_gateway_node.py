#!/usr/bin/env python3
"""Exercise the real ROS adapter's mapping/freshness with lightweight ROS stubs."""
import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))


class Message:
    LANDED_STATE_ON_GROUND = 1

    def __init__(self, **fields):
        self.connected = self.armed = False
        self.mode = ''
        self.ground_origin = types.SimpleNamespace(x=0., y=0., z=0.)
        self.__dict__.update(fields)


class RosError(Exception):
    pass


def load_adapter():
    ros = types.ModuleType('rospy')
    ros.Subscriber, ros.ServiceProxy, ros.wait_for_service = Mock(), Mock(), Mock()
    ros.Publisher, ros.Timer, ros.Duration = Mock(), Mock(), Mock()
    ros.Time = Mock()
    ros.Time.now.return_value.to_sec.return_value = 100.
    ros.ROSException = ros.ServiceException = RosError
    replacements = {'rospy': ros}
    for package, names in [('mavros_msgs', ['State', 'ExtendedState']), ('sensor_msgs', ['BatteryState']),
                           ('nav_msgs', ['Odometry']), ('std_msgs', ['Float64MultiArray', 'Bool', 'Float64', 'String']),
                           ('multipoint', ['MissionStatus', 'RemoteWaypoint'])]:
        replacements[package] = types.ModuleType(package)
        messages = types.ModuleType(package + '.msg')
        for name in names:
            setattr(messages, name, Message)
        replacements[package + '.msg'] = messages
    services = types.ModuleType('multipoint.srv')
    services.MissionCommand = services.MissionCommandRequest = Message
    replacements['multipoint.srv'] = services
    nav_services = types.ModuleType('nav_msgs.srv')
    nav_services.GetPlan = nav_services.GetPlanRequest = Message
    replacements['nav_msgs.srv'] = nav_services
    spec = importlib.util.spec_from_file_location('tested_gcs_adapter', str(SCRIPTS / 'gcs_gateway_node.py'))
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, replacements):
        spec.loader.exec_module(module)
    return module


MODULE = load_adapter()


class AdapterTest(unittest.TestCase):
    def setUp(self):
        self.clock = patch.object(MODULE.time, 'monotonic', return_value=100.)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.backend = MODULE.RosBackend()
        fields = {key: 0 for key in MODULE.STATUS_FIELDS}
        fields.update(frame_id='world', map_session_id='map', mission_id='route', revision=1,
                      state='READY', reason='', odom_fresh=True, flight_ready=True,
                      max_abs_xy=30., min_z=.3, max_z=2.5)
        self.message = Message(**fields)
        self.backend.on_status(self.message)
        self.backend.on_state(Message(connected=True, armed=True, mode='OFFBOARD'))

    def test_telemetry_mapping_battery_and_stale_executor(self):
        self.backend.on_battery(Message(percentage=.67))
        telemetry = self.backend.telemetry()
        self.assertEqual(telemetry['protocol'], 2)
        self.assertEqual(telemetry['server'], 'agri-onboard-ros')
        self.assertEqual(telemetry['battery_percent'], 67.)
        self.assertTrue(telemetry['mission_ready'])
        self.now.return_value = 102.
        stale = self.backend.telemetry()
        self.assertFalse(stale['mission_ready'])
        self.assertFalse(stale['odom_fresh'])
        self.assertFalse(stale['mission']['flight_ready'])
        self.assertIn('unknown', stale['mission']['reason'])
        self.now.return_value = 106.
        self.assertFalse(self.backend.telemetry()['fcu_connected'])
        self.assertEqual(self.backend.telemetry()['battery_percent'], -1.)

    def test_preflight_details_expire_instead_of_reporting_stale_causes(self):
        self.backend.on_preflight('controller', Message(data='rc_stale,battery_low'))
        self.backend.on_preflight('mission', Message(data='trajectory_not_ready'))
        telemetry = self.backend.telemetry()
        self.assertEqual(telemetry['preflight_controller_reasons'], ['rc_stale', 'battery_low'])
        self.assertEqual(telemetry['preflight_mission_reasons'], ['trajectory_not_ready'])
        self.now.return_value = 102.
        self.assertIsNone(self.backend.telemetry()['preflight_controller_reasons'])
        self.assertIsNone(self.backend.telemetry()['preflight_mission_reasons'])

    def test_invalid_battery_is_unknown(self):
        for percentage in (float('nan'), float('inf'), -1., 2.):
            self.backend.on_battery(Message(percentage=percentage))
            self.assertEqual(self.backend.telemetry()['battery_percent'], -1.)

    def command(self):
        return dict(request_id='request', operation='update', map_session_id='map',
                    mission_id='route', revision=2, frame_id='world', replace_from=1,
                    waypoints=[dict(id=2, x=1., y=0., z=1., hover_sec=2.)])

    def test_typed_service_mapping_and_authoritative_ack(self):
        response = Message(accepted=True, reason='Accepted', mission=self.message)
        self.backend.proxy = Mock(return_value=response)
        accepted, reason, status = self.backend.command(self.command())
        request = self.backend.proxy.call_args[0][0]
        self.assertEqual(request.operation, 'update')
        self.assertEqual(request.replace_from, 1)
        self.assertEqual(request.revision, 2)
        self.assertEqual(request.waypoints[0].id, 2)
        self.assertEqual(request.waypoints[0].hover_sec, 2.)
        self.assertTrue(accepted)
        self.assertEqual(status['mission_id'], 'route')
        self.assertEqual(reason, 'Accepted')

    def test_automatic_metadata_is_forwarded_without_starting(self):
        response = Message(accepted=True, reason='Accepted', mission=self.message)
        self.backend.proxy = Mock(return_value=response)
        command = self.command()
        command.update(operation='upload', automatic=True, ground_origin=dict(x=.4, y=-.2, z=.1))
        self.assertTrue(self.backend.command(command)[0])
        request = self.backend.proxy.call_args[0][0]
        self.assertTrue(request.automatic)
        self.assertEqual(request.ground_origin.z, .1)
        self.assertEqual(request.operation, 'upload')

    def test_initialize_passes_height_to_async_worker(self):
        response = Message(accepted=True, reason='Accepted', mission=self.message)
        self.backend.proxy = Mock(return_value=response)
        self.backend.initializer.start = Mock(return_value=(True, 'started'))
        command = self.command()
        command.update(operation='initialize', bounds=[-1., -1., -.1, 1., 1., 2.],
                       project_dir='/home/nv/project', takeoff_height=.8)
        self.assertTrue(self.backend.command(command)[0])
        self.backend.proxy.assert_not_called()
        self.backend.initializer.start.assert_called_once_with(command['bounds'], '/home/nv/project', .8)
        self.assertTrue(self.backend.telemetry()['init_height_supported'])
        self.backend._clear_executor_for_initialization()
        self.assertEqual(self.backend.proxy.call_args[0][0].operation, 'clear')
        response.accepted = False
        with self.assertRaises(RuntimeError):
            self.backend._clear_executor_for_initialization()

    def test_old_fence_and_start_rejected_until_recalibration(self):
        self.backend.initializer.phase = 'awaiting_calibration'
        self.backend.on_fence(Message(data=[-1, -1, 0, 1, 1, 2]))
        self.assertEqual(self.backend.fence, [])
        self.backend.proxy = Mock()
        command = self.command()
        command['operation'] = 'start'
        accepted, reason, _ = self.backend.command(command)
        self.assertFalse(accepted)
        self.assertIn('确认', reason)
        self.backend.proxy.assert_not_called()

    def test_stale_and_unknown_service_results(self):
        self.backend.proxy = Mock(side_effect=RosError('lost reply'))
        accepted, reason, _ = self.backend.command(self.command())
        self.assertFalse(accepted)
        self.assertIn('result unknown', reason)
        self.now.return_value = 102.
        self.backend.proxy.reset_mock()
        self.assertFalse(self.backend.command(self.command())[0])
        self.backend.proxy.assert_not_called()


class ControllerHeightTest(unittest.TestCase):
    def setUp(self):
        self.globals = MODULE.Initializer._ensure_controller.__globals__
        self.init = MODULE.Initializer()
        self.init._wait_message = Mock(return_value=True)
        self.init._set = Mock()
        self.active, self.height, self.armed = True, .5, False
        def running(pattern):
            return [12345] if self.active and ('px4ctrl_node' in pattern or 'run_ctrl_indoor_automatic.launch' in pattern) else []
        def wait(topic, cls, timeout):
            if topic == '/px4ctrl/takeoff_height': return Message(data=self.height)
            if topic == '/mavros/state': return Message(connected=True, armed=self.armed)
            if topic == '/mavros/extended_state': return Message(landed_state=1)
            raise AssertionError(topic)
        def stop(pid, sig):
            self.assertEqual(pid, 12345)
            self.active = False
        def spawn(args, log):
            self.height = float(args[-1].split(':=')[1])
            self.active = True
        self.stop = Mock(side_effect=stop)
        self.spawn = Mock(side_effect=spawn)
        replacements = patch.dict(self.globals, {'running': Mock(side_effect=running),
                                                'spawn': self.spawn, 'os': types.SimpleNamespace(kill=self.stop)})
        waiting = patch.object(self.globals['rospy'], 'wait_for_message', side_effect=wait, create=True)
        for item in (replacements, waiting):
            item.start(); self.addCleanup(item.stop)

    def test_existing_matching_height_does_not_restart(self):
        self.assertTrue(self.init._ensure_controller(.5))
        self.assertTrue(self.active)
        self.spawn.assert_not_called()
        self.stop.assert_not_called()

    def test_changed_height_restarts_and_verifies_on_ground(self):
        self.assertTrue(self.init._ensure_controller(.8))
        self.assertEqual(self.height, .8)
        self.stop.assert_called_once()
        self.spawn.assert_called_once()

    def test_armed_aircraft_never_restarts_controller(self):
        self.armed = True
        self.assertFalse(self.init._ensure_controller(.8))
        self.stop.assert_not_called()
        self.spawn.assert_not_called()


class FullReinitializationTest(unittest.TestCase):
    def setUp(self):
        self.init = MODULE.Initializer()
        self.init._set = Mock()
        self.init._stop_launchers = Mock()
        self.init._ensure_controller = Mock(return_value=True)
        self.init._wait_message = Mock(return_value=True)
        self.globals = MODULE.Initializer._restart_all.__globals__

    def test_armed_aircraft_is_rejected_before_any_stop(self):
        ros = self.globals['rospy']
        with patch.object(ros, 'wait_for_message', side_effect=[
                Message(connected=True, armed=True), Message(landed_state=1)], create=True):
            with self.assertRaisesRegex(RuntimeError, '落地并上锁'):
                self.init._restart_all('/workspace', .7)
        self.init._stop_launchers.assert_not_called()

    def test_restart_waits_for_new_map_calibration_before_planner(self):
        self.init._require_ground = Mock()
        self.init._ensure_mavros = Mock()
        calls = []
        node_lists = iter(['/multipointplan\n/gimbal_landing\n', ''])
        def run(args, **kwargs):
            calls.append(args)
            if args == ['rosnode', 'list']:
                return types.SimpleNamespace(returncode=0, stdout=next(node_lists))
            return types.SimpleNamespace(returncode=0, stdout='/multipointplan\n/gimbal_landing\n')
        ros = self.globals['rospy']
        with patch.object(ros, 'get_param', return_value=True, create=True), \
             patch.dict(self.globals, {'running': Mock(side_effect=lambda p: [9] if p.startswith('rosbag record') else []), 'spawn': Mock()}), \
             patch.object(self.globals['subprocess'], 'run', side_effect=run), \
             patch.object(self.globals['os'].path, 'isfile', return_value=True), \
             patch.object(self.globals['os'], 'makedirs'):
            self.init._restart_all('/workspace', .7)
            started = self.globals['spawn'].call_args_list
        self.assertTrue(any('mapping_mid360.launch' in str(c) for c in self.init._stop_launchers.call_args_list))
        self.assertTrue(any('mavros' in str(c) for c in self.init._stop_launchers.call_args_list))
        self.assertTrue(any(a[0][:3] == ['roslaunch', 'multipoint', 'reinitialized_components.launch'] for a, _ in started))
        self.assertFalse(any('planner.launch' in str(a) for a, _ in started))
        self.assertEqual(self.init._set.call_args_list[-1].args[0], 'awaiting_calibration')
        self.assertIn(['rosnode', 'kill', '/multipointplan'], calls)
        self.assertNotIn(['rosnode', 'kill', '/gcs_gateway'], calls)

    def test_rejected_clear_never_restarts_services(self):
        self.init._prepare_ground_services = Mock()
        self.init._require_ground = Mock()
        self.init.clear_executor = Mock(side_effect=RuntimeError('clear rejected'))
        self.init._restart_all = Mock()
        self.init._run([], '/workspace', .7)
        self.init._restart_all.assert_not_called()
        self.assertEqual(self.init._set.call_args.args[0], 'failed')

    def test_confirm_new_bounds_starts_planner_without_resetting_map_again(self):
        self.init._require_ground = Mock()
        self.init._restart_planner = Mock(return_value=True)
        self.init._wait_bool = Mock(return_value=True)
        self.init._finish_calibration([-1, -1, 0, 1, 1, 2], '/workspace')
        self.init._restart_planner.assert_called_once()
        self.init._stop_launchers.assert_not_called()
        self.assertEqual(self.init._set.call_args_list[-1].args[0], 'ready')



if __name__ == '__main__':
    unittest.main()
