#!/usr/bin/env python3
"""TCP -> ROS service -> real mission executor, with mock flight/planner topics."""
import json
import socket
import time
import unittest
import uuid

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from multipoint.msg import MissionStatus
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import TakeoffLand
from std_msgs.msg import Bool, Empty, String


def point(ident, x, hover=0):
    return dict(id=ident, x=x, y=0., z=1., hover_sec=hover)


class RemoteIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.position = (0., 0., 1.)
        self.airborne = False
        self.publish_odom = True
        self.goals, self.stops, self.takeoff_land = [], [], []
        self.status = None
        self.subscribers = [
            rospy.Subscriber('/goal', PoseStamped, self.goals.append),
            rospy.Subscriber('/planning/stop', Empty, self.stops.append),
            rospy.Subscriber('/px4ctrl/takeoff_land', TakeoffLand, self.takeoff_land.append),
            rospy.Subscriber('/mission/remote/status', MissionStatus, self.on_status)]
        self.odom = rospy.Publisher('/test/odom', Odometry, queue_size=1)
        self.fcu = rospy.Publisher('/mavros/state', State, queue_size=1)
        self.extended = rospy.Publisher('/mavros/extended_state', ExtendedState, queue_size=1)
        self.controller = rospy.Publisher('/px4ctrl/state', String, queue_size=1)
        self.ready = rospy.Publisher('/px4ctrl/mission_ready', Bool, queue_size=1)
        self.heartbeat = rospy.Publisher('/drone_0_traj_server/heartbeat', Empty, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.03), self.tick)
        self.until(lambda: self.status is not None and self.status.odom_fresh)
        self.peer = None
        self.connect()

    def tearDown(self):
        self.timer.shutdown()
        if self.peer:
            self.stream.close()
            self.peer.close()

    def on_status(self, status):
        self.status = status

    def until(self, condition, timeout=6):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            rospy.sleep(.03)
        self.assertTrue(condition())

    def connect(self):
        if self.peer:
            self.stream.close()
            self.peer.close()
        deadline = time.monotonic() + 6
        while True:
            try:
                self.peer = socket.create_connection(('127.0.0.1', 18765), timeout=2)
                self.stream = self.peer.makefile('rb')
                telemetry = json.loads(self.stream.readline())
                self.assertEqual(telemetry['server'], 'agri-onboard-ros')
                return
            except OSError:
                if time.monotonic() > deadline:
                    raise
                rospy.sleep(.1)

    def tick(self, _):
        state = State()
        state.connected, state.armed = True, self.airborne
        state.mode = 'OFFBOARD' if self.airborne else 'ALTCTL'
        self.fcu.publish(state)
        extended = ExtendedState()
        extended.landed_state = ExtendedState.LANDED_STATE_IN_AIR if self.airborne else ExtendedState.LANDED_STATE_ON_GROUND
        self.extended.publish(extended)
        self.controller.publish(String(data='AUTO_HOVER' if self.airborne else 'MANUAL_CTRL'))
        self.ready.publish(Bool(data=self.airborne))
        self.heartbeat.publish(Empty())
        if self.publish_odom:
            odom = Odometry()
            odom.header.frame_id, odom.header.stamp = 'world', rospy.Time.now()
            odom.pose.pose.orientation.w = 1
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = self.position
            self.odom.publish(odom)

    def command(self, operation, mission='route-1', revision=1, request_id=None, **fields):
        command = dict(type='command', protocol=2, request_id=request_id or str(uuid.uuid4()),
                       operation=operation, mission_id=mission, revision=revision,
                       map_session_id=self.status.map_session_id)
        command.update(fields)
        self.peer.sendall(json.dumps(command).encode() + b'\n')
        while True:
            result = json.loads(self.stream.readline())
            if result['type'] == 'ack' and result['request_id'] == command['request_id']:
                return result

    def test_upload_manual_start_dynamic_replan_progress_and_failure(self):
        route = [point(1, 2, 1.5), point(2, 4), point(3, 6)]
        ack = self.command('upload', frame_id='world', waypoints=route)
        self.assertTrue(ack['accepted'])
        self.assertEqual(ack['mission']['state'], 'READY')
        self.assertFalse(self.goals)
        self.assertFalse(self.takeoff_land)  # Upload must never arm/take off.
        self.assertFalse(self.command('start')['accepted'])  # Ground trigger rejected.
        self.airborne = True
        self.until(lambda: self.status.flight_ready)
        self.assertFalse(self.command('start', revision=2)['accepted'])
        self.assertFalse(self.command('start', map_session_id='old-session')['accepted'])
        start_id = str(uuid.uuid4())
        self.assertTrue(self.command('start', request_id=start_id)['accepted'])
        self.until(lambda: len(self.goals) == 1)
        self.assertEqual(self.goals[-1].pose.position.x, 2)
        self.connect()  # Retry through a fresh TCP connection: executor deduplicates.
        self.assertTrue(self.command('start', request_id=start_id)['accepted'])
        rospy.sleep(.15)
        self.assertEqual(len(self.goals), 1)
        self.assertFalse(self.command('upload', mission='another', frame_id='world', waypoints=route)['accepted'])
        update_id = str(uuid.uuid4())
        update = dict(frame_id='world', replace_from=1, waypoints=[point(4, 4)])
        self.assertTrue(self.command('update', revision=2, request_id=update_id, **update)['accepted'])
        self.assertEqual(len(self.goals), 1)  # Future work does not interrupt current goal.
        self.assertTrue(self.command('update', revision=2, request_id=update_id, **update)['accepted'])
        changed = dict(update, waypoints=[point(5, 5)])
        self.assertFalse(self.command('update', revision=2, request_id=update_id, **changed)['accepted'])
        self.assertTrue(self.command('update', revision=3, frame_id='world', replace_from=0,
                                     waypoints=[point(1, 3, 1.5), point(4, 4)])['accepted'])
        self.until(lambda: len(self.goals) == 2)
        self.assertEqual(self.goals[-1].pose.position.x, 3)  # Immediate planner retarget.
        self.position = (3., 0., 1.)
        self.until(lambda: self.status.state == 'HOVERING')
        self.assertFalse(self.command('update', revision=4, frame_id='world', replace_from=0,
                                      waypoints=[point(1, 3)])['accepted'])
        self.until(lambda: self.status.completed == 1)
        self.until(lambda: len(self.goals) == 3)
        self.assertEqual(self.goals[-1].pose.position.x, 4)
        self.assertFalse(self.command('update', revision=4, frame_id='world', replace_from=0,
                                      waypoints=route)['accepted'])
        self.position = (4., 0., 1.)
        self.until(lambda: self.status.state == 'COMPLETED')
        ack = self.command('get_status')
        self.assertEqual(ack['mission']['completed'], 2)
        self.assertEqual(ack['mission']['total'], 2)
        self.assertEqual(ack['mission']['revision'], 3)
        self.assertEqual(ack['mission']['current_waypoint_id'], 0)
        self.assertFalse(self.command('start', revision=3)['accepted'])
        self.assertTrue(self.command('upload', mission='route-2', frame_id='world', waypoints=[point(8, 6)])['accepted'])
        self.assertTrue(self.command('start', mission='route-2')['accepted'])
        old_session = self.status.map_session_id
        self.publish_odom = False
        self.until(lambda: self.status.state == 'FAILED')
        self.until(lambda: self.status.map_session_id != old_session)
        self.assertTrue(self.stops)
        self.assertFalse(self.takeoff_land)
        self.assertFalse(self.command('start', mission='route-2')['accepted'])


if __name__ == '__main__':
    rospy.init_node('test_remote_integration')
    rostest.rosrun('multipoint', 'remote_integration', RemoteIntegrationTest)
