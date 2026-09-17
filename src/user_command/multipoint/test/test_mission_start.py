#!/usr/bin/env python3
"""Regression of real mission callbacks using mock flight/planner topics only."""
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import ExtendedState, State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, String


class MissionStartTest(unittest.TestCase):
    def setUp(self):
        self.goals = []
        self.stops = []
        self.position = (0.0, 0.0, 0.0)
        self.airborne = False
        self.publish_odom = True
        self.subscribers = [
            rospy.Subscriber('/goal', PoseStamped, self.goals.append),
            rospy.Subscriber('/planning/stop', Empty, self.stops.append)]
        self.odom = rospy.Publisher('/test/odom', Odometry, queue_size=1)
        self.state = rospy.Publisher('/mavros/state', State, queue_size=1)
        self.extended = rospy.Publisher('/mavros/extended_state', ExtendedState, queue_size=1)
        self.controller = rospy.Publisher('/px4ctrl/state', String, queue_size=1)
        self.ready = rospy.Publisher('/px4ctrl/mission_ready', Bool, queue_size=1)
        self.heartbeat = rospy.Publisher('/drone_0_traj_server/heartbeat', Empty, queue_size=1)
        self.clicked = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
        self.start = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.clear = rospy.Publisher('/mission/clear_clicked_route', Empty, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)
        self.until(lambda: all(p.get_num_connections() for p in
                   (self.odom, self.state, self.ready, self.clicked, self.start, self.heartbeat)))
        rospy.sleep(0.3)

    def tearDown(self):
        self.timer.shutdown()

    def until(self, condition, timeout=5):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            rospy.sleep(0.05)
        self.assertTrue(condition())

    def tick(self, _event):
        stamp = rospy.Time.now()
        state = State()
        state.header.stamp = stamp
        state.connected = True
        state.armed = self.airborne
        state.mode = 'OFFBOARD' if self.airborne else 'ALTCTL'
        self.state.publish(state)
        extended = ExtendedState()
        extended.header.stamp = stamp
        extended.landed_state = ExtendedState.LANDED_STATE_IN_AIR if self.airborne else ExtendedState.LANDED_STATE_ON_GROUND
        self.extended.publish(extended)
        self.controller.publish(String(data='AUTO_HOVER' if self.airborne else 'MANUAL_CTRL'))
        self.ready.publish(Bool(data=self.airborne))
        self.heartbeat.publish(Empty())
        if self.publish_odom:
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'world'
            odom.pose.pose.orientation.w = 1
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = self.position
            self.odom.publish(odom)

    def click(self, x, y):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = 'world'
        point.point.x, point.point.y = x, y
        self.clicked.publish(point)
        rospy.sleep(0.2)

    def trigger_without_goal(self, count):
        self.start.publish(PoseStamped())
        rospy.sleep(0.3)
        self.assertEqual(len(self.goals), count)

    def test_ground_trigger_route_retention_consumed_route_and_stale_odom(self):
        self.trigger_without_goal(0)  # Empty click mode must not execute YAML.
        self.click(2.0, 0.0)
        self.trigger_without_goal(0)  # Disarmed, ground, ALTCTL (recorded bug).
        self.airborne = True
        rospy.sleep(0.3)
        self.start.publish(PoseStamped())
        self.until(lambda: len(self.goals) == 1)
        self.assertAlmostEqual(self.goals[0].pose.position.x, 2.0)  # Retained point.
        self.assertAlmostEqual(self.goals[0].pose.position.z, 1.0)
        self.position = (2.0, 0.0, 1.0)
        rospy.sleep(1.0)  # Finish the route after stable arrival.
        self.trigger_without_goal(1)  # Consumed route must not fall back to YAML.
        self.clear.publish(Empty())
        rospy.sleep(0.2)
        self.trigger_without_goal(1)  # Cleared route must not fall back either.
        self.click(4.0, 0.0)
        self.start.publish(PoseStamped())
        self.until(lambda: len(self.goals) == 2)
        self.publish_odom = False
        self.until(lambda: bool(self.stops))  # Stale odom cannot fake arrival.
        self.assertEqual(len(self.goals), 2)


if __name__ == '__main__':
    rospy.init_node('test_mission_start')
    rostest.rosrun('multipoint', 'mission_start', MissionStartTest)
