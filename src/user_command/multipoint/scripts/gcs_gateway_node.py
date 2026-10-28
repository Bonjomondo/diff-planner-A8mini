#!/usr/bin/env python3
"""ROS1 adapter for the ground station's TCP JSON-lines protocol 2."""
import math
import threading
import time
import sys
from pathlib import Path

import rospy
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import Float64MultiArray, Bool
from multipoint.msg import MissionStatus, RemoteWaypoint
from multipoint.srv import MissionCommand, MissionCommandRequest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gcs_protocol import GatewayServer
from gcs_map_guard import MapGuard, valid_bounds, inside


STATUS_FIELDS = ('frame_id', 'map_session_id', 'mission_id', 'revision', 'state',
                 'current_waypoint_id', 'completed', 'total', 'reason', 'odom_fresh',
                 'flight_ready', 'world_x', 'world_y', 'world_z',
                 'max_abs_xy', 'min_z', 'max_z', 'automatic_supported', 'automatic', 'takeoff_ready', 'takeoff_height')


def status_dict(message):
    return {key: getattr(message, key) for key in STATUS_FIELDS}


class RosBackend:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = None
        self.status_time = self.state_time = self.battery_time = 0.0
        self.state = State()
        self.battery = -1.0
        self.map_guard = MapGuard()
        self.fence = []
        self.map_route = None
        self.map_route_points = []
        self.map_ready_pub = rospy.Publisher('/indoor_mission/map_ready', Bool, queue_size=1)
        self.map_timer = rospy.Timer(rospy.Duration(.2), self.publish_map_ready)
        self.proxy = rospy.ServiceProxy('/mission/remote/command', MissionCommand)
        self.subscribers = [
            rospy.Subscriber('/mission/remote/status', MissionStatus, self.on_status, queue_size=1),
            rospy.Subscriber('/mavros/state', State, self.on_state, queue_size=1),
            rospy.Subscriber('/mavros/battery', BatteryState, self.on_battery, queue_size=1),
            rospy.Subscriber('/laserMapping/odometry', Odometry, lambda msg: self.on_odom('map', msg), queue_size=10),
            rospy.Subscriber('/ekf/ekf_odom', Odometry, lambda msg: self.on_odom('planner', msg), queue_size=50),
            rospy.Subscriber('/indoor_mission/fence', Float64MultiArray, self.on_fence, queue_size=1)]

    def on_odom(self, source, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        with self.lock:
            self.map_guard.observe(source, msg.header.stamp.to_sec(), msg.header.frame_id,
                                   (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), msg.header.seq)

    def on_fence(self, msg):
        with self.lock:
            self.fence = list(msg.data) if valid_bounds(msg.data) else []

    def publish_map_ready(self, _):
        with self.lock:
            ready = self.map_guard.snapshot(rospy.Time.now().to_sec())['map_coordinates_valid'] and bool(self.fence)
        self.map_ready_pub.publish(Bool(data=ready))

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
                mission.update(odom_fresh=False, flight_ready=False, takeoff_ready=False,
                               reason='Mission executor status stale; execution result unknown')
            state_fresh = now - self.state_time <= 2.5
            return dict(type='telemetry', protocol=2, server='agri-onboard-ros',
                        fcu_connected=state_fresh and self.state.connected,
                        armed=state_fresh and self.state.armed,
                        mode=self.state.mode[:64] if state_fresh else '',
                        battery_percent=self.battery if now - self.battery_time <= 5 else -1.0,
                        mission_ready=mission['flight_ready'], odom_fresh=mission['odom_fresh'],
                        world_x=mission['world_x'], world_y=mission['world_y'],
                        world_z=mission['world_z'], mission=mission, flight_bounds=list(self.fence),
                        **self.map_guard.snapshot(rospy.Time.now().to_sec()))

    def command(self, command):
        with self.lock:
            stale = time.monotonic() - self.status_time > 1.0
            guarded = command.get('require_map_alignment', False)
            verified = self.map_guard.snapshot(rospy.Time.now().to_sec())['map_coordinates_valid'] and bool(self.fence)
            bounds = tuple(self.fence)
            identity = (command['map_session_id'], command['mission_id'], command['revision'], bounds)
            route_matches = self.map_route == identity
            route_points = command['waypoints'] if command['operation'] == 'upload' else self.map_route_points
        if guarded and command['operation'] in ('upload', 'start', 'update'):
            if command['operation'] == 'update':
                return False, 'Upload a new bounded route while idle; guarded update is not supported', self.telemetry()['mission']
            if not verified or (command['operation'] == 'start' and not route_matches):
                return False, 'Map coordinates/fence changed; confirm and upload again', self.telemetry()['mission']
            if any(not inside((p['x'], p['y'], p['z']), bounds) for p in route_points):
                return False, 'Waypoint outside planner flight fence', self.telemetry()['mission']
            if command['operation'] in ('upload', 'start'):
                try:
                    rospy.wait_for_service('/indoor_mission/check_endpoints', timeout=1.)
                    check = rospy.ServiceProxy('/indoor_mission/check_endpoints', GetPlan)
                    for point in route_points:
                        query = GetPlanRequest()
                        query.start.header.frame_id = query.goal.header.frame_id = 'world'
                        query.start.pose.orientation.w = query.goal.pose.orientation.w = 1.
                        for key in ('x', 'y', 'z'):
                            setattr(query.start.pose.position, key, point[key])
                            setattr(query.goal.pose.position, key, point[key])
                        if not check(query).plan.poses:
                            return False, 'Goal/return point is occupied, unknown or outside local planner map', self.telemetry()['mission']
                except (rospy.ROSException, rospy.ServiceException):
                    return False, 'Planner endpoint check unavailable', self.telemetry()['mission']
        if stale:
            return False, 'Mission executor status stale; query after recovery', self.telemetry()['mission']
        request = MissionCommandRequest()
        for key in ('request_id', 'operation', 'map_session_id', 'mission_id',
                    'revision', 'frame_id', 'replace_from'):
            setattr(request, key, command[key])
        request.automatic = command.get('automatic', False)
        if request.automatic:
            ground = command.get('ground_origin', {})
            for key in ('x', 'y', 'z'):
                setattr(request.ground_origin, key, ground.get(key, 0.))
        request.waypoints = [RemoteWaypoint(**point) for point in command['waypoints']]
        try:
            rospy.wait_for_service('/mission/remote/command', timeout=1.0)
            response = self.proxy(request)
            if response.accepted and guarded and command['operation'] in ('upload', 'update'):
                with self.lock:
                    self.map_route = identity
                    self.map_route_points = [dict(point) for point in command['waypoints']]
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
