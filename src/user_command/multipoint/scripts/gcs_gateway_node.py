#!/usr/bin/env python3
"""ROS1 adapter for the ground station's TCP JSON-lines protocol 2."""
import math
from pathlib import Path
import sys
import threading
import time

import rospy
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from multipoint.msg import MissionStatus, RemoteWaypoint
from multipoint.srv import MissionCommand, MissionCommandRequest

# Catkin's devel relay changes __file__ but not sys.path; import the source helper.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gcs_protocol import GatewayServer


STATUS_FIELDS = ('frame_id', 'map_session_id', 'mission_id', 'revision', 'state',
                 'current_waypoint_id', 'completed', 'total', 'reason', 'odom_fresh',
                 'flight_ready', 'world_x', 'world_y', 'world_z',
                 'max_abs_xy', 'min_z', 'max_z')


def status_dict(message):
    return {key: getattr(message, key) for key in STATUS_FIELDS}


class RosBackend:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.status_time = self.state_time = self.battery_time = 0.0
        self.state = State()
        self.battery = -1.0
        self.proxy = rospy.ServiceProxy('/mission/remote/command', MissionCommand)
        self.subscribers = [
            rospy.Subscriber('/mission/remote/status', MissionStatus, self.on_status, queue_size=1),
            rospy.Subscriber('/mavros/state', State, self.on_state, queue_size=1),
            rospy.Subscriber('/mavros/battery', BatteryState, self.on_battery, queue_size=1)]

    def on_status(self, message):
        with self.lock:
            self.status = status_dict(message)
            self.status_time = time.monotonic()

    def on_state(self, message):
        with self.lock:
            self.state = message
            self.state_time = time.monotonic()

    def on_battery(self, message):
        with self.lock:
            value = message.percentage
            self.battery = 100.0 * value if math.isfinite(value) and 0 <= value <= 1 else -1.0
            self.battery_time = time.monotonic()

    def wait_for_status(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.status is not None:
                    return
            rospy.sleep(0.1)

    def telemetry(self):
        with self.lock:
            mission = dict(self.status)
            now = time.monotonic()
            if now - self.status_time > 1.0:
                mission.update(odom_fresh=False, flight_ready=False,
                               reason='Mission executor status stale; execution result unknown')
            state_fresh = now - self.state_time <= 2.5
            return dict(type='telemetry', protocol=2, server='agri-onboard-ros',
                        fcu_connected=state_fresh and self.state.connected,
                        armed=state_fresh and self.state.armed,
                        mode=self.state.mode[:64] if state_fresh else '',
                        battery_percent=self.battery if now - self.battery_time <= 5 else -1.0,
                        mission_ready=mission['flight_ready'], odom_fresh=mission['odom_fresh'],
                        world_x=mission['world_x'], world_y=mission['world_y'],
                        world_z=mission['world_z'], mission=mission)

    def command(self, command):
        with self.lock:
            stale = time.monotonic() - self.status_time > 1.0
        if stale:
            return False, 'Mission executor status stale; query after recovery', self.telemetry()['mission']
        request = MissionCommandRequest()
        for key in ('request_id', 'operation', 'map_session_id', 'mission_id',
                    'revision', 'frame_id', 'replace_from'):
            setattr(request, key, command[key])
        request.waypoints = [RemoteWaypoint(**point) for point in command['waypoints']]
        try:
            rospy.wait_for_service('/mission/remote/command', timeout=1.0)
            response = self.proxy(request)
            # Telemetry is fed exclusively by the executor's ordered status topic.
            return response.accepted, response.reason, status_dict(response.mission)
        except (rospy.ROSException, rospy.ServiceException) as error:
            return False, 'ROS command result unknown: ' + str(error)[:200], self.telemetry()['mission']


def main():
    rospy.init_node('gcs_gateway')
    backend = RosBackend()
    backend.wait_for_status()
    if rospy.is_shutdown():
        return
    host = rospy.get_param('~bind_host', '127.0.0.1')
    port = int(rospy.get_param('~port', 8765))
    if not 1 <= port <= 65535:
        raise ValueError('Gateway port must be 1..65535')
    server = GatewayServer((host, port), backend)
    worker = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': 0.2}, daemon=True)
    worker.start()
    rospy.loginfo('GCS protocol 2 listening on %s:%d (SSH tunnel)', host, port)
    try:
        rospy.spin()
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=2.0)


if __name__ == '__main__':
    main()
