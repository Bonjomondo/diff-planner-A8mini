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
    def __init__(self, **fields):
        self.connected = self.armed = False
        self.mode = ''
        self.__dict__.update(fields)


class RosError(Exception):
    pass


def load_adapter():
    ros = types.ModuleType('rospy')
    ros.Subscriber, ros.ServiceProxy, ros.wait_for_service = Mock(), Mock(), Mock()
    ros.ROSException = ros.ServiceException = RosError
    replacements = {'rospy': ros}
    for package, names in [('mavros_msgs', ['State']), ('sensor_msgs', ['BatteryState']),
                           ('multipoint', ['MissionStatus', 'RemoteWaypoint'])]:
        replacements[package] = types.ModuleType(package)
        messages = types.ModuleType(package + '.msg')
        for name in names:
            setattr(messages, name, Message)
        replacements[package + '.msg'] = messages
    services = types.ModuleType('multipoint.srv')
    services.MissionCommand = services.MissionCommandRequest = Message
    replacements['multipoint.srv'] = services
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

    def test_stale_and_unknown_service_results(self):
        self.backend.proxy = Mock(side_effect=RosError('lost reply'))
        accepted, reason, _ = self.backend.command(self.command())
        self.assertFalse(accepted)
        self.assertIn('result unknown', reason)
        self.now.return_value = 102.
        self.backend.proxy.reset_mock()
        self.assertFalse(self.backend.command(self.command())[0])
        self.backend.proxy.assert_not_called()


if __name__ == '__main__':
    unittest.main()
