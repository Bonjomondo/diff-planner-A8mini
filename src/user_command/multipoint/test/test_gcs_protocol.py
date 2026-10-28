#!/usr/bin/env python3
import copy
import json
import socket
import sys
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from gcs_protocol import (GatewayServer, LineBuffer, MAX_FRAME_BYTES, ProtocolError,
                          decode_line, validate_command)


def upload():
    return dict(type='command', protocol=2, operation='upload', request_id='request-1',
                map_session_id='map-1', mission_id='mission-1', revision=1,
                frame_id='world', waypoints=[dict(id=1, x=1.25, y=-2, z=1, hover_sec=2)])


class FakeBackend:
    def __init__(self):
        self.commands = []

    def telemetry(self):
        return dict(type='telemetry', protocol=2, server='agri-onboard-ros',
                    fcu_connected=False, armed=False, mission_ready=False,
                    odom_fresh=True, mode='', battery_percent=-1.0,
                    world_x=0., world_y=0., world_z=0.,
                    mission=dict(frame_id='world', map_session_id='map-1', mission_id='',
                                 revision=0, state='IDLE', current_waypoint_id=0, completed=0,
                                 total=0, reason='', odom_fresh=True, flight_ready=False,
                                 world_x=0., world_y=0., world_z=0., max_abs_xy=30., min_z=.3, max_z=2.5))

    def command(self, command):
        self.commands.append(command)
        return True, 'Accepted', self.telemetry()['mission']


class ProtocolTest(unittest.TestCase):
    def test_frontend_upload_start_query_and_update_shapes(self):
        command = validate_command(upload())
        self.assertEqual(command['waypoints'][0]['x'], 1.25)
        self.assertEqual(command['replace_from'], 0)
        for operation in ('start', 'get_status'):
            obj = upload()
            obj['operation'] = operation
            self.assertEqual(validate_command(obj)['waypoints'], [])
        obj = upload()
        obj.update(operation='update', revision=2, replace_from=1, waypoints=[])
        self.assertEqual(validate_command(obj)['replace_from'], 1)

    def test_explicit_automatic_upload_and_ground_origin(self):
        obj = upload()
        obj.update(automatic=True, ground_origin=dict(x=0., y=0., z=.2))
        self.assertEqual(validate_command(obj)['ground_origin']['z'], .2)
        for bad in (None, {}, dict(x=0., y=0.), dict(x=True, y=0., z=0.),
                    dict(x=10**400, y=0., z=0.), dict(x=0., y=0., z=float('nan'))):
            with self.subTest(bad=str(bad)), self.assertRaises(ProtocolError):
                validate_command(dict(obj, ground_origin=bad))
        with self.assertRaises(ProtocolError):
            validate_command(dict(obj, automatic='true'))

    def test_strict_numbers_frames_and_envelopes(self):
        for field, value in [('protocol', True), ('protocol', 1), ('request_id', ''),
                             ('map_session_id', ''), ('mission_id', 'm' * 81),
                             ('revision', -1), ('revision', 1.5), ('revision', True),
                             ('revision', 10**400), ('frame_id', 'gps'),
                             ('operation', 'takeoff'), ('waypoints', []),
                             ('waypoints', [{}] * 201)]:
            obj = upload()
            obj[field] = value
            with self.subTest(field=field, value=str(value)[:30]), self.assertRaises(ProtocolError):
                validate_command(obj)
        for field, value in [('x', '1'), ('y', True), ('z', float('inf')),
                             ('hover_sec', float('nan')), ('id', 0), ('id', 2**32),
                             ('x', 10**400)]:
            obj = upload()
            obj['waypoints'][0][field] = value
            with self.subTest(field=field), self.assertRaises(ProtocolError):
                validate_command(obj)
        for raw in (b'{"protocol":2,"protocol":1}', b'{"x":NaN}', b'[]', b'\xff'):
            with self.assertRaises(ProtocolError):
                decode_line(raw)

    def test_fragmentation_coalescing_and_bounded_frames(self):
        buffer = LineBuffer()
        self.assertEqual(list(buffer.feed(b'{"a":')), [])
        self.assertEqual(list(buffer.feed(b'1}\n{}\npart')), [b'{"a":1}', b'{}'])
        self.assertEqual(buffer.buffer, b'part')
        with self.assertRaises(ProtocolError):
            list(buffer.feed(b'x' * MAX_FRAME_BYTES))
        with self.assertRaises(ProtocolError):
            list(LineBuffer().feed(b'x' * (MAX_FRAME_BYTES + 1) + b'\n'))

    def test_requires_loopback(self):
        with self.assertRaises(ValueError):
            GatewayServer(('0.0.0.0', 0), FakeBackend())


class SocketTest(unittest.TestCase):
    def setUp(self):
        self.backend = FakeBackend()
        self.server = GatewayServer(('127.0.0.1', 0), self.backend)
        self.worker = threading.Thread(target=self.server.serve_forever,
                                       kwargs={'poll_interval': 0.02}, daemon=True)
        self.worker.start()
        self.clients = []

    def tearDown(self):
        for peer, stream in self.clients:
            stream.close()
            peer.close()
        self.server.shutdown()
        self.server.server_close()
        self.worker.join(2)

    def connect(self):
        peer = socket.create_connection(self.server.server_address, timeout=2)
        stream = peer.makefile('rb')
        self.clients.append((peer, stream))
        frame = json.loads(stream.readline())
        self.assertEqual(frame['server'], 'agri-onboard-ros')
        self.assertEqual(frame['mission']['frame_id'], 'world')
        return peer, stream

    def read_ack(self, stream):
        while True:
            frame = json.loads(stream.readline())
            if frame['type'] != 'telemetry':
                return frame

    def test_upload_ack_fragmented_commands_and_multiple_clients(self):
        peer, stream = self.connect()
        frame = json.dumps(upload()).encode() + b'\n'
        peer.sendall(frame[:15])
        peer.sendall(frame[15:] + frame)
        for _ in range(2):
            ack = self.read_ack(stream)
            self.assertEqual(ack['request_id'], 'request-1')
            self.assertTrue(ack['accepted'])
            self.assertIn('mission', ack)
        self.assertEqual(len(self.backend.commands), 2)
        other, other_stream = self.connect()
        other.sendall(frame)
        self.assertTrue(self.read_ack(other_stream)['accepted'])

    def test_bad_command_nacks_without_touching_ros(self):
        peer, stream = self.connect()
        obj = copy.deepcopy(upload())
        obj['waypoints'][0]['x'] = 'not a number'
        peer.sendall(json.dumps(obj).encode() + b'\n')
        ack = self.read_ack(stream)
        self.assertFalse(ack['accepted'])
        self.assertEqual(ack['request_id'], 'request-1')
        peer.sendall(b'{broken}\n')
        self.assertEqual(self.read_ack(stream)['type'], 'error')
        self.assertFalse(self.backend.commands)

    def test_oversized_client_does_not_stop_other_clients(self):
        peer, stream = self.connect()
        peer.sendall(b'x' * (MAX_FRAME_BYTES + 1))
        self.assertEqual(stream.readline(), b'')
        other, other_stream = self.connect()
        other.sendall(json.dumps(upload()).encode() + b'\n')
        self.assertTrue(self.read_ack(other_stream)['accepted'])


if __name__ == '__main__':
    unittest.main()
