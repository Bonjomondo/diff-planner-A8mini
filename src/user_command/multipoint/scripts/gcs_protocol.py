#!/usr/bin/env python3
"""Protocol 2 TCP transport; ROS is isolated in gcs_gateway_node.py."""
import ipaddress
import json
import math
import socket
import socketserver
import threading
import time

MAX_FRAME_BYTES = 65536
MAX_WAYPOINTS = 200
UINT32_MAX = 4294967295


class ProtocolError(ValueError):
    pass


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError('Duplicate JSON key: ' + key)
        result[key] = value
    return result


def decode_line(line):
    if len(line) > MAX_FRAME_BYTES:
        raise ProtocolError('Frame exceeds 64 KiB')
    def invalid_constant(value):
        raise ProtocolError('Non-finite JSON number: ' + value)
    try:
        value = json.loads(line.decode('utf-8'), object_pairs_hook=_unique_object,
                           parse_constant=invalid_constant)
    except (ValueError, UnicodeError, RecursionError) as error:
        raise ProtocolError(str(error)) from error
    if not isinstance(value, dict):
        raise ProtocolError('Expected a JSON object')
    return value


def _text(obj, key, empty=False):
    value = obj.get(key, '')
    if not isinstance(value, str) or len(value) > 80 or (not empty and not value):
        raise ProtocolError(key + ' must be a string of 1..80 characters')
    try:
        value.encode('utf-8')
    except UnicodeError as error:
        raise ProtocolError(key + ' contains invalid Unicode') from error
    return value


def _uint(obj, key, minimum=0):
    value = obj.get(key)
    if type(value) not in (int, float) or not minimum <= value <= UINT32_MAX or not math.isfinite(value) or int(value) != value:
        raise ProtocolError(key + ' must be an unsigned 32-bit integer')
    return int(value)


def validate_command(obj):
    if obj.get('type') != 'command' or type(obj.get('protocol')) is not int or obj['protocol'] != 2:
        raise ProtocolError('Expected type=command, protocol=2')
    operation = obj.get('operation')
    if operation not in ('upload', 'start', 'get_status', 'update'):
        raise ProtocolError('Unsupported operation')
    result = dict(request_id=_text(obj, 'request_id'), operation=operation,
                  map_session_id=_text(obj, 'map_session_id', operation == 'get_status'),
                  mission_id=_text(obj, 'mission_id', operation == 'get_status'),
                  revision=_uint(obj, 'revision', 0 if operation == 'get_status' else 1),
                  frame_id='', waypoints=[], replace_from=0)
    if operation in ('upload', 'update'):
        if obj.get('frame_id') != 'world':
            raise ProtocolError('frame_id must be world')
        result['frame_id'] = 'world'
        points = obj.get('waypoints')
        if not isinstance(points, list) or len(points) > MAX_WAYPOINTS or (operation == 'upload' and not points):
            raise ProtocolError('Expected at most 200 waypoints (upload must be nonempty)')
        for point in points:
            if not isinstance(point, dict):
                raise ProtocolError('Each waypoint must be an object')
            normalized = {'id': _uint(point, 'id', 1)}
            for key in ('x', 'y', 'z', 'hover_sec'):
                value = point.get(key)
                try:
                    valid = type(value) in (int, float) and math.isfinite(value)
                except OverflowError:
                    valid = False
                if not valid:
                    raise ProtocolError(key + ' must be a finite number')
                normalized[key] = float(value)
            result['waypoints'].append(normalized)
        if operation == 'update':
            result['replace_from'] = _uint(obj, 'replace_from')
    return result


class LineBuffer:
    def __init__(self):
        self.buffer = b''

    def feed(self, data):
        self.buffer += data
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            if len(line) > MAX_FRAME_BYTES:
                raise ProtocolError('Frame exceeds 64 KiB')
            yield line
        if len(self.buffer) > MAX_FRAME_BYTES:
            raise ProtocolError('Frame exceeds 64 KiB')


class GatewayServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, backend):
        # The ground station uses an SSH tunnel. No unauthenticated LAN listener.
        host = address[0]
        if host != 'localhost' and not ipaddress.ip_address(host).is_loopback:
            raise ValueError('Gateway must bind to loopback; connect through SSH forwarding')
        if ':' in host:
            self.address_family = socket.AF_INET6
        self.backend = backend
        self.command_lock = threading.Lock()
        self.clients = threading.BoundedSemaphore(4)
        super().__init__(address, GatewayHandler)

    def process_request(self, request, client_address):
        if not self.clients.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self.clients.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.clients.release()


class GatewayHandler(socketserver.BaseRequestHandler):
    def send(self, message):
        encoded = json.dumps(message, ensure_ascii=False, allow_nan=False,
                             separators=(',', ':')).encode('utf-8')
        if len(encoded) > MAX_FRAME_BYTES:
            raise ProtocolError('Outgoing frame exceeds 64 KiB')
        self.request.sendall(encoded + b'\n')

    def command(self, line):
        obj = {}
        try:
            obj = decode_line(line)
            command = validate_command(obj)
            with self.server.command_lock:
                accepted, reason, mission = self.server.backend.command(command)
            self.send(dict(type='ack', protocol=2, request_id=command['request_id'],
                           accepted=accepted, reason=reason, mission=mission))
        except ProtocolError as error:
            request_id = obj.get('request_id')
            if isinstance(request_id, str) and 0 < len(request_id) <= 80:
                self.send(dict(type='ack', protocol=2, request_id=request_id,
                               accepted=False, reason=str(error)[:300],
                               mission=self.server.backend.telemetry()['mission']))
            else:
                self.send(dict(type='error', protocol=2, reason=str(error)[:300]))

    def handle(self):
        self.request.settimeout(0.2)
        lines = LineBuffer()
        last_telemetry = 0.0
        try:
            while True:
                now = time.monotonic()
                if now - last_telemetry >= 0.5:
                    self.send(self.server.backend.telemetry())
                    last_telemetry = now
                try:
                    data = self.request.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    return
                for line in lines.feed(data):
                    self.command(line)
        except (OSError, ProtocolError, ValueError):
            # A broken/slow client never stops the onboard mission or other clients.
            return
