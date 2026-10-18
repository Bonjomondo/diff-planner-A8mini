#!/usr/bin/env python3
"""Real mission/planner/trajectory reset with isolated, mocked FCU/RC topics."""
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import ExtendedState, RCIn, State
from multipoint.msg import MissionStatus, RemoteWaypoint
from multipoint.srv import MissionCommand, MissionCommandRequest
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from traj_utils.msg import PolyTraj
from visualization_msgs.msg import MarkerArray


class FlightResetTest(unittest.TestCase):
    def setUp(self):
        self.remote = rospy.get_param('/multipointplan/mission_source') == 'remote'
        self.connected, self.armed = True, False
        self.landed = ExtendedState.LANDED_STATE_ON_GROUND
        self.rc8, self.channels = 1999, 8
        self.rc9 = 999
        self.send_fcu = self.send_extended = self.send_rc = True
        self.goals, self.commands, self.takeoffs, self.plans = [], [], [], []
        self.status = None
        self.markers = []
        self.subs = [
            rospy.Subscriber('/goal', PoseStamped, self.goals.append),
            rospy.Subscriber('/position_cmd', PositionCommand, self.commands.append),
            rospy.Subscriber('/px4ctrl/takeoff_land', TakeoffLand, self.takeoffs.append),
            rospy.Subscriber('/drone_0_planning/trajectory', PolyTraj, self.plans.append),
            rospy.Subscriber('/mission/remote/status', MissionStatus, self.on_status)]
        self.subs.append(rospy.Subscriber('/mission/clicked_waypoints', MarkerArray, self.markers.append))
        self.fcu = rospy.Publisher('/mavros/state', State, queue_size=1)
        self.extended = rospy.Publisher('/mavros/extended_state', ExtendedState, queue_size=1)
        self.rc = rospy.Publisher('/mavros/rc/in', RCIn, queue_size=1)
        self.odom = rospy.Publisher('/drone_0_test_odom', Odometry, queue_size=1)
        self.controller = rospy.Publisher('/px4ctrl/state', String, queue_size=1)
        self.ready = rospy.Publisher('/px4ctrl/mission_ready', Bool, queue_size=1)
        self.land = rospy.Publisher('/mission/land', Empty, queue_size=1)
        self.raw_takeoff = rospy.Publisher('/px4ctrl/takeoff_land', TakeoffLand, queue_size=1)
        self.trajectory = rospy.Publisher('/test/trajectory', PolyTraj, queue_size=1)
        self.goal = rospy.Publisher('/goal', PoseStamped, queue_size=1)
        self.clicked = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
        self.start = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.mandatory = rospy.Publisher('/mandatory_stop_to_planner', Empty, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.03), self.tick)
        for service in ('/mission/reset', '/planning/reset_planner', '/planning/reset_trajectory'):
            rospy.wait_for_service(service, 12)
        self.reset = rospy.ServiceProxy('/mission/reset', Trigger)
        self.reset_traj = rospy.ServiceProxy('/planning/reset_trajectory', Trigger)
        if self.remote:
            rospy.wait_for_service('/mission/remote/command', 5)
            self.remote_command = rospy.ServiceProxy('/mission/remote/command', MissionCommand)
            self.until(lambda: self.status is not None and self.status.odom_fresh)
        self.until(lambda: all(pub.get_num_connections() for pub in
                   (self.fcu, self.extended, self.rc, self.land, self.trajectory, self.goal)))
        rospy.sleep(.3)

    def tearDown(self):
        self.timer.shutdown()

    def on_status(self, status):
        self.status = status

    def until(self, condition, timeout=6):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            rospy.sleep(.03)
        self.assertTrue(condition())

    def tick(self, _):
        stamp = rospy.Time.now()
        if self.send_fcu:
            self.fcu.publish(State(connected=self.connected, armed=self.armed,
                                   mode='OFFBOARD' if self.armed else 'ALTCTL'))
        if self.send_extended:
            self.extended.publish(ExtendedState(landed_state=self.landed))
        if self.send_rc:
            self.rc.publish(RCIn(channels=([1500] * 5 + [1999, 1500, self.rc8, self.rc9])[:self.channels]))
        self.controller.publish(String(data='AUTO_HOVER' if self.armed else 'MANUAL_CTRL'))
        self.ready.publish(Bool(data=self.armed))
        odom = Odometry()
        odom.header.frame_id, odom.header.stamp = 'world', stamp
        odom.pose.pose.orientation.w = 1.
        odom.pose.pose.position.z = 1.
        self.odom.publish(odom)

    def route(self, ident):
        if self.remote:
            request = MissionCommandRequest(request_id='upload-' + ident, operation='upload',
                map_session_id=self.status.map_session_id, mission_id=ident, revision=1,
                frame_id='world', waypoints=[RemoteWaypoint(id=1, x=2., z=1.)])
            self.assertTrue(self.remote_command(request).accepted)
            return request
        point = PointStamped()
        point.header.frame_id, point.header.stamp = 'world', rospy.Time.now()
        point.point.x, point.point.z = 2., 1.
        self.clicked.publish(point)
        rospy.sleep(.15)

    def assert_silent(self):
        rospy.sleep(.1)  # Drain in-flight ROS messages before measuring.
        count = len(self.commands)
        rospy.sleep(.15)
        self.assertEqual(len(self.commands), count)

    def send_trajectory(self, stamp=None):
        trajectory = PolyTraj()
        trajectory.order = 5
        trajectory.traj_id = 42
        trajectory.start_time = stamp if stamp is not None else rospy.Time.now()
        trajectory.duration = [30.]
        trajectory.coef_x = [0., 0., 0., 0., 0., 2.]
        trajectory.coef_y = [0.] * 6
        trajectory.coef_z = [0., 0., 0., 0., 0., 1.]
        self.trajectory.publish(trajectory)
        rospy.sleep(.1)

    def rejected_reset(self, direct=False):
        rospy.sleep(.15)
        result = (self.reset_traj if direct else self.reset)()
        self.assertFalse(result.success, result.message)
        self.send_trajectory()
        self.assert_silent()

    def test_explicit_reset_guards_old_route_and_next_flight(self):
        old_request = self.route('old-flight')
        old_session = self.status.map_session_id if self.remote else None
        # Reproduce rejected TAKEOFF -> RC8 DOWN -> LANDING while still on ground.
        self.rc8 = 1499
        self.until(lambda: any(cmd.takeoff_land_cmd == TakeoffLand.TAKEOFF for cmd in self.takeoffs))
        self.rc8 = 1999
        self.until(lambda: any(cmd.takeoff_land_cmd == TakeoffLand.LAND for cmd in self.takeoffs))
        if self.remote:
            self.until(lambda: self.status.state == 'LANDING')
        self.send_trajectory()
        self.assert_silent()
        self.rc8 = 1499
        self.rejected_reset()
        self.raw_takeoff.publish(TakeoffLand(takeoff_land_cmd=TakeoffLand.TAKEOFF))
        self.send_trajectory()
        self.assert_silent()  # Neither RC middle nor raw TAKEOFF clears LAND.
        self.rc8 = 1999
        self.armed = True
        self.rejected_reset()
        self.rejected_reset(direct=True)
        # A full SWB gesture while armed cannot clear the lock or fire on landing.
        self.channels = 9
        marker_count = len(self.markers)
        self.rc9 = 1999
        rospy.sleep(2.3)
        self.armed = False
        rospy.sleep(.35)
        self.rc9 = 999
        rospy.sleep(.4)
        self.assertEqual(len(self.markers), marker_count)
        if self.remote:
            self.assertEqual(self.status.state, 'LANDING')
        self.channels = 8  # Existing RC-only setups must still support the service guard.
        self.armed = False
        for state in (ExtendedState.LANDED_STATE_IN_AIR, ExtendedState.LANDED_STATE_UNDEFINED,
                      ExtendedState.LANDED_STATE_LANDING):
            self.landed = state
            self.rejected_reset()
        self.landed = ExtendedState.LANDED_STATE_ON_GROUND
        self.connected = False
        self.rejected_reset()
        self.connected = True
        for field, pause in (('send_fcu', .45), ('send_extended', .45), ('send_rc', .65)):
            setattr(self, field, False)
            rospy.sleep(pause)
            self.rejected_reset()
            if field != 'send_rc':
                rospy.sleep(2.1)  # Trajectory node's independent FCU timeout is 2.5 s.
            self.rejected_reset(direct=True)
            setattr(self, field, True)
            rospy.sleep(.15)
        self.channels = 7
        self.rejected_reset()
        self.channels = 8
        self.rc8 = 1800
        self.rejected_reset()
        self.rc8 = 1999
        rospy.sleep(.2)
        before = len(self.takeoffs)
        # Use SWB on CH9 for this reset; keep the service for the next cycle below.
        self.channels, self.rc9 = 9, 999
        rospy.sleep(.35)  # Establish the home endpoint after the missing-channel samples.
        marker_count = len(self.markers)
        self.rc9 = 1999
        rospy.sleep(.35)
        self.rc9 = 999
        rospy.sleep(.35)
        self.assertEqual(len(self.markers), marker_count)  # A short flick is ignored.
        self.rc9 = 1999
        rospy.sleep(2.3)
        self.assertEqual(len(self.markers), marker_count)  # Holding alone cannot reset.
        self.rc9 = 999
        self.until(lambda: len(self.markers) > marker_count)
        rospy.sleep(.2)
        self.assertEqual(len(self.takeoffs), before)  # Reset cannot take off or land.
        old_stops = []
        late_stop_sub = rospy.Subscriber('/planning/stop', Empty, old_stops.append)
        self.subs.append(late_stop_sub)
        self.until(lambda: late_stop_sub.get_num_connections() > 0)
        rospy.sleep(.1)
        self.assertFalse(old_stops)  # Reconnecting nodes must not receive the old latched stop.
        if self.remote:
            self.until(lambda: self.status.state == 'IDLE' and self.status.map_session_id != old_session)
            self.assertEqual(self.status.total, 0)
            # Cached retry acknowledges the old result without restoring its route.
            self.assertTrue(self.remote_command(old_request).accepted)
            self.assertEqual(self.remote_command(old_request).mission.total, 0)
            old_request.request_id = 'delayed-old-upload'
            self.assertFalse(self.remote_command(old_request).accepted)
        self.send_trajectory()
        self.assert_silent()  # Reset still waits for a separate TAKEOFF.
        self.rc8 = 1499
        self.until(lambda: len(self.takeoffs) > before)
        self.assertEqual(self.takeoffs[-1].takeoff_land_cmd, TakeoffLand.TAKEOFF)
        self.armed, self.landed = True, ExtendedState.LANDED_STATE_IN_AIR
        rospy.sleep(.2)
        goals_before = len(self.goals)
        self.start.publish(PoseStamped())
        rospy.sleep(.2)
        self.assertEqual(len(self.goals), goals_before)  # Old route was cleared.
        self.send_trajectory()
        self.assert_silent()  # Old planner output cannot run after TAKEOFF either.
        self.route('new-flight')
        self.start.publish(PoseStamped())
        self.until(lambda: len(self.goals) > goals_before)
        self.send_trajectory(rospy.Time(1))
        self.assert_silent()  # Queued old trajectory cannot cross the new goal epoch.
        self.send_trajectory()
        self.until(lambda: bool(self.commands))  # A newly confirmed route can fly.
        self.land.publish(Empty())
        rospy.sleep(.2)
        self.assert_silent()
        self.armed, self.landed, self.rc8 = False, ExtendedState.LANDED_STATE_ON_GROUND, 1999
        rospy.sleep(.25)
        self.assertTrue(self.reset().success)  # A second landing/reset cycle works.
        plans = len(self.plans)
        self.goal.publish(self.goals[-1])  # Delayed old goal must not rearm planner.
        rospy.sleep(.25)
        self.assertEqual(len(self.plans), plans)
        self.send_trajectory()
        self.assert_silent()
        # An emergency planner lock prevents partial reset from reporting success.
        self.mandatory.publish(Empty())
        rospy.sleep(.15)
        result = self.reset()
        self.assertFalse(result.success)
        self.assertIn('Planner reset failed', result.message)
        self.raw_takeoff.publish(TakeoffLand(takeoff_land_cmd=TakeoffLand.TAKEOFF))
        self.send_trajectory()
        self.assert_silent()


if __name__ == '__main__':
    rospy.init_node('test_flight_reset')
    rostest.rosrun('multipoint', 'flight_reset', FlightResetTest)
