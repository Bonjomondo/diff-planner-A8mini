#!/usr/bin/env python3
"""User-owned module supervisor. SSH carries one JSON request; no LAN control port.
Connecting starts only this supervisor. It never arms, takes off or starts a route.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import socketserver
import subprocess
import sys
import threading
import time
import xmlrpc.client

MODULES = ('flight', 'gimbal', 'yolo', 'rviz')
OPTIONS = ('flight', 'gimbal', 'yolo', 'auto_record', 'save_video', 'rviz')
NODE_NAMES = {'flight': {'/mavros', '/laserMapping', '/ekf', '/px4ctrl', '/multipointplan', '/gcs_gateway'},
              'gimbal': {'/a8mini_gimbal'}, 'yolo': {'/a8mini_detection'}, 'rviz': {'/rviz'}}


def normalize(options):
    if not isinstance(options, dict) or set(options) - set(OPTIONS):
        raise ValueError('Unknown module selection')
    if any(type(v) is not bool for v in options.values()):
        raise ValueError('Module selections must be booleans')
    result = {key: options.get(key, False) for key in OPTIONS}
    if result['save_video']:
        result['yolo'] = True
    if result['auto_record']:
        result['gimbal'] = True
    return result


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.0
        return connection


def registered_nodes():
    try:
        master = xmlrpc.client.ServerProxy('http://127.0.0.1:11311', transport=TimeoutTransport())
        code, _, groups = master.getSystemState('/gcs_module_manager')
        return {node for group in groups for _, nodes in group for node in nodes} if code == 1 else set()
    except (OSError, xmlrpc.client.Error):
        return set()


def runtime_dir(workspace):
    key = hashlib.sha256(str(workspace).encode()).hexdigest()[:12]
    directory = Path.home() / '.local/state/agri-gcs-manager' / key
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    if directory.is_symlink() or directory.stat().st_uid != os.getuid():
        raise ValueError('Unsafe manager runtime directory')
    os.chmod(str(directory), 0o700)
    return directory


def child_environment():
    env = dict(os.environ)
    env.update(ROS_MASTER_URI='http://127.0.0.1:11311', ROS_IP='127.0.0.1')
    env.pop('ROS_HOSTNAME', None)
    return env


class Manager:
    def __init__(self, workspace, directory):
        self.workspace, self.directory = Path(workspace), Path(directory)
        self.children = {}
        self.options = normalize({})
        self.lock = threading.RLock()

    def command(self, name, options):
        base = str(self.workspace)
        if name == 'flight':
            return ['/bin/zsh', base + '/sh_files/one_click_takeoff.sh', '--start-only'], {
                'MISSION_SOURCE': 'remote', 'A8MINI_START_GIMBAL_NODE': 'false',
                'A8MINI_START_DETECTION': 'false', 'UAV_START_RVIZ': 'false'}
        if name == 'gimbal':
            return ['roslaunch', 'multipoint', 'gcs_payload.launch', 'module:=gimbal',
                    'auto_record:=' + str(options['auto_record']).lower()], {}
        if name == 'yolo':
            return ['roslaunch', 'multipoint', 'gcs_payload.launch', 'module:=yolo',
                    'save_video:=' + str(options['save_video']).lower()], {}
        if name == 'rviz':
            display = os.environ.get('DISPLAY', '')
            if not display:
                displays = sorted(Path('/tmp/.X11-unix').glob('X[0-9]*'))
                display = ':' + displays[0].name[1:] if displays else ''
            if not display:
                raise ValueError('RViz needs a graphical desktop; other modules run headless')
            env = {'DISPLAY': display}
            authority = Path.home() / '.Xauthority'
            if authority.exists():
                env['XAUTHORITY'] = str(authority)
            return ['roslaunch', 'diff_planner', 'exp_rviz.launch'], env
        if name == 'roscore':
            return ['roscore'], {}
        raise ValueError('Unknown module')

    def alive(self, name):
        child = self.children.get(name)
        return child is not None and child['process'].poll() is None

    def spawn(self, name, options):
        args, extra = self.command(name, options)
        # argv is passed through zsh positional parameters; no request text is shell code.
        command = ['/bin/zsh', '-c',
                   'source /opt/ros/noetic/setup.zsh && source "$1/devel/setup.zsh" && shift && exec "$@"',
                   'gcs-module', str(self.workspace)] + args
        env = child_environment()
        env.update(extra)
        log_path = self.directory / (name + '.log')
        with log_path.open('ab', buffering=0) as log:
            log.write(('\n--- launch ' + time.strftime('%Y-%m-%d %H:%M:%S') + ' ---\n').encode())
            process = subprocess.Popen(command, cwd=str(self.workspace), env=env,
                                       stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        self.children[name] = dict(process=process, started=time.monotonic(), options=dict(options))

    def ensure_ros(self):
        if registered_nodes() or self.master_reachable():
            return
        if not self.alive('roscore'):
            self.spawn('roscore', {})
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            if self.master_reachable():
                return
            time.sleep(.1)
        raise ValueError('ROS master failed to start; see roscore log')

    @staticmethod
    def master_reachable():
        try:
            with socket.create_connection(('127.0.0.1', 11311), timeout=.2):
                return True
        except OSError:
            return False

    def status(self):
        nodes = registered_nodes()
        rows = []
        for name in MODULES:
            child = self.children.get(name)
            alive = self.alive(name)
            external = bool(NODE_NAMES[name] & nodes) and not alive
            if alive:
                ready = NODE_NAMES[name].issubset(nodes)
                timeout = 180 if name == 'flight' else 20
                state = 'stopping' if child.get('stopping') else 'running' if ready else 'starting' if time.monotonic() - child['started'] < timeout else 'degraded'
            else:
                ready = False
                state = 'external' if external else 'exited' if child else 'stopped'
            rows.append(dict(name=name, state=state, ready=ready,
                             exit_code=child['process'].poll() if child else None,
                             options=child['options'] if child else {},
                             missing=sorted(NODE_NAMES[name] - nodes) if alive else [],
                             recording=('waiting_flight_state' if '/mavros' not in nodes else 'watching_flight_events')
                             if name == 'gimbal' and alive and child['options'].get('auto_record') else 'disabled'))
        return dict(modules=rows, options=dict(self.options),
                    flight_external=bool(NODE_NAMES['flight'] & nodes) and not self.alive('flight'),
                    note='Running means nodes registered; flight/position readiness comes from live telemetry')

    def start(self, options):
        options = normalize(options)
        nodes = registered_nodes()
        results = []
        # Do not alter/restart existing modules merely because a different preset was selected.
        for name in MODULES:
            if not options[name]:
                continue
            relevant = ('auto_record',) if name == 'gimbal' else ('save_video',) if name == 'yolo' else ()
            if self.alive(name):
                if self.children[name].get('stopping'):
                    results.append(dict(module=name, ok=False, message='Module still stopping; wait for cleanup'))
                elif any(self.children[name]['options'][key] != options[key] for key in relevant):
                    results.append(dict(module=name, ok=False, message='Configuration changed; stop this module before applying'))
                else:
                    results.append(dict(module=name, ok=True, message='Already running'))
                continue
            if NODE_NAMES[name] & nodes:
                results.append(dict(module=name, ok=False, message='Externally started nodes exist; manager will not duplicate or stop them'))
                continue
            try:
                # Check graphical availability before creating dependencies for RViz.
                self.command(name, options)
                self.ensure_ros()
                self.spawn(name, options)
                results.append(dict(module=name, ok=True, message='Launch requested; check status and logs'))
            except Exception as error:
                results.append(dict(module=name, ok=False, message=str(error)))
        self.options = options
        return results

    def ground_guard(self):
        # Obtain new messages, rather than accepting a cached last-known landed flag.
        source = '''import json,rospy
from mavros_msgs.msg import State,ExtendedState
rospy.init_node('gcs_stop_guard',anonymous=True,disable_signals=True)
s=rospy.wait_for_message('/mavros/state',State,timeout=2)
e=rospy.wait_for_message('/mavros/extended_state',ExtendedState,timeout=2)
now=rospy.Time.now()
fresh=all(not m.header.stamp.is_zero() and -.05 <= (now-m.header.stamp).to_sec() <= 2.5 for m in (s,e))
print(json.dumps(dict(connected=s.connected,armed=s.armed,ground=e.landed_state==1,fresh=fresh)))'''
        result = subprocess.run(['/bin/zsh', '-c',
                                 'source /opt/ros/noetic/setup.zsh && source "$1/devel/setup.zsh" && exec /usr/bin/python3 -c "$2"',
                                 'gcs-guard', str(self.workspace), source], env=child_environment(),
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=7, text=True)
        try:
            state = json.loads(result.stdout.strip().splitlines()[-1])
        except (ValueError, IndexError):
            raise ValueError('Fresh flight state unavailable; stop refused')
        if not state.get('fresh') or not state['connected'] or state['armed'] or not state['ground']:
            raise ValueError('Flight stop requires connected FCU, ON_GROUND and disarmed')

    def stop(self, name):
        if name not in MODULES:
            raise ValueError('Unknown module')
        if not self.alive(name):
            return 'No owned running process; external nodes are never stopped'
        if name == 'flight':
            self.ground_guard()
        process = self.children[name]['process']
        os.killpg(process.pid, signal.SIGINT)
        self.children[name]['stopping'] = True
        # Cleanup stays asynchronous: core supervisor needs time to finalize rosbag.
        return 'Stop requested; wait for process cleanup before starting again'

    def logs(self, name):
        if name not in MODULES and name != 'roscore':
            raise ValueError('Unknown module')
        path = self.directory / (name + '.log')
        if not path.exists():
            return 'No log for this module yet'
        with path.open('rb') as stream:
            stream.seek(max(0, path.stat().st_size - 16000))
            return stream.read().decode('utf-8', errors='replace')

    def dispatch(self, request):
        with self.lock:
            action = request.get('action')
            response = dict(ok=True)
            if action == 'start':
                response['results'] = self.start(request.get('options'))
                response['ok'] = all(row['ok'] for row in response['results'])
            elif action == 'stop':
                response['message'] = self.stop(request.get('module'))
            elif action == 'logs':
                response['logs'] = self.logs(request.get('module'))
            elif action != 'status':
                raise ValueError('Only status/start/stop/logs are supported')
            response['status'] = self.status()
            return response


def serve(workspace, directory):
    lock = (directory / 'daemon.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return
    endpoint = directory / 'control.sock'
    endpoint.unlink(missing_ok=True)
    manager = Manager(workspace, directory)

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            self.connection.settimeout(20)
            try:
                line = self.rfile.readline(65537)
                if len(line) > 65536:
                    raise ValueError('Request too large')
                request = json.loads(line)
                if not isinstance(request, dict):
                    raise ValueError('Expected a JSON object')
                response = manager.dispatch(request)
            except Exception as error:
                response = dict(ok=False, message=str(error))
            response['manager_pid'] = os.getpid()
            self.wfile.write(json.dumps(response, ensure_ascii=False).encode() + b'\n')

    with socketserver.ThreadingUnixStreamServer(str(endpoint), Handler) as server:
        os.chmod(str(endpoint), 0o600)
        server.daemon_threads = True
        server.serve_forever(poll_interval=.2)


def client(workspace, directory, request):
    endpoint = str(directory / 'control.sock')
    peer = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    peer.settimeout(25)
    try:
        peer.connect(endpoint)
    except (FileNotFoundError, ConnectionRefusedError):
        peer.close()
        with (directory / 'manager.log').open('ab') as output:
            subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--serve'],
                             stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                             start_new_session=True, close_fds=True)
        deadline = time.monotonic() + 5
        while True:
            peer = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            peer.settimeout(25)
            try:
                peer.connect(endpoint)
                break
            except (FileNotFoundError, ConnectionRefusedError):
                peer.close()
                if time.monotonic() >= deadline:
                    raise ValueError('Module manager did not start; check manager.log')
                time.sleep(.1)
    with peer:
        peer.sendall(json.dumps(request).encode() + b'\n')
        with peer.makefile('rb') as stream:
            return json.loads(stream.readline(131073))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serve', action='store_true')
    parser.add_argument('--request', action='store_true')
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    directory = runtime_dir(workspace)
    if args.serve:
        serve(workspace, directory)
    elif args.request:
        try:
            request = json.loads(sys.stdin.buffer.readline(65537))
            print(json.dumps(client(workspace, directory, request), ensure_ascii=False))
        except Exception as error:
            print(json.dumps(dict(ok=False, message=str(error)), ensure_ascii=False))
            return 1
    else:
        parser.error('Use --request with a JSON object on stdin')
    return 0


if __name__ == '__main__':
    sys.exit(main())
