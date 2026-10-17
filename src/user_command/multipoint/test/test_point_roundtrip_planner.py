#!/usr/bin/env python3
"""Actual Diff-Planner + mission gateway with perfect simulated pose tracking.

No MAVROS, PX4, motor, camera or lidar driver is started. Use an isolated master.
"""
import time
import copy
import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest
from mavros_msgs.msg import State, ExtendedState
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs import point_cloud2
from std_msgs.msg import Bool, Header, String
from test_remote_integration import RemoteIntegrationTest, point


class PlannerRoundTrip(RemoteIntegrationTest):
    def setUp(self):
        self.command_pose = None
        self.command_count = 0
        self.violations = []
        self.last_cloud = 0
        self.allow_cloud = False
        self.map_divergence = False
        self.extra_odom = [rospy.Publisher(topic, Odometry, queue_size=1) for topic in
                           ('/laserMapping/odometry', '/ekf/ekf_odom', '/drone_0_sim_odom')]
        self.cloud_pub = rospy.Publisher('/drone_0_sim_cloud', point_cloud2.PointCloud2, queue_size=1)
        self.command_sub = rospy.Subscriber('/test/position_cmd', PositionCommand, self.on_position_cmd)
        # Synthetic room shell, including a ray along the selected target.
        values = [i * .1 for i in range(-30,31)]
        heights = [i * .1 for i in range(31)]
        self.shell = [(side,y,z) for side in (-3.,3.) for y in values for z in heights]
        self.shell += [(x,side,z) for side in (-3.,3.) for x in values for z in heights]
        self.shell += [(x,y,z) for z in (0.,3.) for x in values for y in values]
        super().setUp()

    def on_position_cmd(self, msg):
        self.command_pose = msg
        self.command_count += 1
        if not (-2 < msg.position.x < 2 and -2 < msg.position.y < 2 and .3 < msg.position.z < 2.5):
            self.violations.append((msg.position.x,msg.position.y,msg.position.z))

    def tick(self, _):
        stamp = rospy.Time.now()
        if self.command_pose and self.airborne:
            p = self.command_pose.position
            self.position = (p.x,p.y,p.z)
        state = State(connected=True, armed=self.airborne, mode='OFFBOARD' if self.airborne else 'ALTCTL')
        self.fcu.publish(state)
        extended = ExtendedState()
        extended.landed_state = ExtendedState.LANDED_STATE_IN_AIR if self.airborne else ExtendedState.LANDED_STATE_ON_GROUND
        self.extended.publish(extended)
        self.controller.publish(String(data='AUTO_HOVER' if self.airborne else 'MANUAL_CTRL'))
        self.ready.publish(Bool(data=self.airborne))
        # The real planner supplies its own heartbeat; never publish a fake one.
        odom = Odometry()
        odom.header = Header(stamp=stamp, frame_id='world')
        odom.pose.pose.orientation.w = 1
        odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z = self.position
        if self.command_pose:
            odom.twist.twist.linear = self.command_pose.velocity
        self.odom.publish(odom)
        for index,publisher in enumerate(self.extra_odom):
            if self.map_divergence and index == 1:
                shifted = copy.deepcopy(odom)
                shifted.pose.pose.position.x += 1.
                publisher.publish(shifted)
            else:
                publisher.publish(odom)
        if self.allow_cloud and time.monotonic()-self.last_cloud > .15:
            self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id='world'),self.shell))
            self.last_cloud = time.monotonic()

    def test_upload_manual_start_dynamic_replan_progress_and_failure(self):
        self.airborne = True  # Synthetic FCU state only, no hardware connection.
        self.until(lambda: self.status.flight_ready, timeout=12)
        rospy.wait_for_service('/indoor_mission/check_endpoints', timeout=6)
        check = rospy.ServiceProxy('/indoor_mission/check_endpoints', GetPlan)
        query = GetPlanRequest()
        query.start.header.frame_id = query.goal.header.frame_id = 'world'
        query.start.pose.position.z = query.goal.pose.position.z = 1.
        query.goal.pose.position.x = .8
        self.assertFalse(check(query).plan.poses)  # Empty buffers must not be considered observed free space.
        self.allow_cloud = True
        self.until(lambda: len(check(query).plan.poses) == 2, timeout=10)
        bad = self.command('upload', mission='outside', frame_id='world',
                           waypoints=[point(1,3), point(2,0)], require_map_alignment=True)
        self.assertFalse(bad['accepted'])
        ack = self.command('upload', mission='roundtrip', frame_id='world',
                           waypoints=[point(1,.8,.3), point(2,0)], require_map_alignment=True)
        self.assertTrue(ack['accepted'], ack['reason'])
        self.assertFalse(self.goals)
        self.assertFalse(self.takeoff_land)

        ack = self.command('start', mission='roundtrip', require_map_alignment=True)
        self.assertTrue(ack['accepted'], ack['reason'])
        self.until(lambda: self.status.state in ('COMPLETED','FAILED'), timeout=35)
        self.assertEqual(self.status.state,'COMPLETED', self.status.reason)
        self.assertEqual(self.status.completed,2)
        self.assertEqual(len(self.goals),2)
        self.assertAlmostEqual(self.goals[0].pose.position.x,.8)
        self.assertAlmostEqual(self.goals[1].pose.position.x,0.)
        self.assertGreater(self.command_count,10)
        self.assertFalse(self.violations)
        self.assertFalse(self.takeoff_land)

        ack = self.command('upload', mission='coordinate-loss', frame_id='world',
                           waypoints=[point(1,.6), point(2,0)], require_map_alignment=True)
        self.assertTrue(ack['accepted'], ack['reason'])
        self.assertTrue(self.command('start', mission='coordinate-loss', require_map_alignment=True)['accepted'])
        self.until(lambda: len(self.goals) >= 3)
        self.map_divergence = True
        self.until(lambda: self.status.state == 'FAILED')
        self.assertIn('coordinates', self.status.reason)
        self.assertTrue(self.stops)
        self.assertFalse(self.takeoff_land)

if __name__ == '__main__':
    rospy.init_node('test_point_roundtrip_planner')
    rostest.rosrun('multipoint','point_roundtrip_planner',PlannerRoundTrip)
