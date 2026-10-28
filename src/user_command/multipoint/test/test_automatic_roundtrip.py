#!/usr/bin/env python3
"""Real task executor and TCP gateway, simulated FCU/controller on isolated master."""
import json
import os
import socket
import threading
import time
import unittest
import uuid

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState
from multipoint.msg import MissionStatus
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetPlanResponse
from quadrotor_msgs.msg import TakeoffLand
from std_msgs.msg import Bool, Empty, String, Float64, Float64MultiArray


class AutomaticRoundtripTest(unittest.TestCase):
    def setUp(self):
        self.assertEqual(rospy.get_master().getUri()[2], 'http://127.0.0.1:11411/')
        self.scenario = rospy.get_param('~scenario', 'complete')
        self.preparation_delay = float(rospy.get_param('~preparation_delay', 0.))
        self.stage = 'ground'
        self.position = [.4, -.2, .2]
        self.status = None
        self.goals, self.takeoffs, self.lands, self.aborts = [], [], [], []
        self.clear, self.publish_odom, self.controls_ok = True, True, True
        self.output_ready, self.publish_output_ready = True, True
        self.deadline, self.goal_time, self.seq = 0., 0., 0
        self.subscribers = [
            rospy.Subscriber('/mission/remote/status', MissionStatus, self.on_status),
            rospy.Subscriber('/goal', PoseStamped, self.on_goal),
            rospy.Subscriber('/px4ctrl/takeoff_land', TakeoffLand, self.on_flight_command),
            rospy.Subscriber('/px4ctrl/abort_automatic', Empty, self.aborts.append)]
        self.publishers = {name: rospy.Publisher(topic, cls, queue_size=1) for name, topic, cls in [
            ('odom', '/ekf/ekf_odom', Odometry), ('map', '/laserMapping/odometry', Odometry),
            ('fcu', '/mavros/state', State), ('extended', '/mavros/extended_state', ExtendedState),
            ('controller', '/px4ctrl/state', String), ('ready', '/px4ctrl/mission_ready', Bool),
            ('preflight', '/px4ctrl/takeoff_ready', Bool), ('control', '/px4ctrl/automatic_control_ok', Bool),
            ('height', '/px4ctrl/takeoff_height', Float64),
            ('output', '/drone_0_traj_server/output_ready', Bool),
            ('heartbeat', '/drone_0_traj_server/heartbeat', Empty), ('fence', '/indoor_mission/fence', Float64MultiArray)]}
        self.services = [rospy.Service('/indoor_mission/check_endpoints', GetPlan, self.check)]
        self.timer = rospy.Timer(rospy.Duration(.03), self.tick)
        self.until(lambda: self.status is not None and self.status.takeoff_ready)
        self.peer = socket.create_connection(('127.0.0.1', 18765), timeout=4)
        self.stream = self.peer.makefile('rb')
        self.mission = str(uuid.uuid4())

    def tearDown(self):
        self.timer.shutdown()
        self.stream.close()
        self.peer.close()
        for service in self.services:
            service.shutdown()

    def until(self, condition, timeout=8):
        end = time.monotonic() + timeout
        while not condition() and time.monotonic() < end:
            rospy.sleep(.03)
        self.assertTrue(condition(), 'stage={} status={}'.format(self.stage, self.status))

    def on_status(self, message):
        self.status = message

    def check(self, request):
        result = GetPlanResponse()
        if self.clear:
            result.plan.poses = [request.start, request.goal]
        return result

    def on_goal(self, message):
        self.goals.append(message)
        self.goal_time = time.monotonic()

    def on_flight_command(self, message):
        if message.takeoff_land_cmd == TakeoffLand.TAKEOFF:
            self.takeoffs.append(message)
            if self.scenario != 'takeoff_rejected':
                self.stage = 'preparing' if self.preparation_delay else 'taking_off'
                self.deadline = time.monotonic()+self.preparation_delay
        else:
            self.lands.append(message)
            if self.stage not in ('landing', 'landed_armed', 'ground_done'):
                self.stage, self.deadline = 'landing', time.monotonic() + .8

    def tick(self, _):
        now = time.monotonic()
        if self.stage == 'preparing' and now >= self.deadline:
            self.stage = 'taking_off'
        if self.stage == 'taking_off':
            self.position[2] = min(1.2, self.position[2]+.035)
            if self.position[2] >= 1.2:
                self.stage = 'hover'
        if self.goals and self.stage == 'hover' and now-self.goal_time > .18:
            p = self.goals[-1].pose.position
            target = [p.x, p.y, p.z]
            delta = [target[i]-self.position[i] for i in range(3)]
            distance = sum(v*v for v in delta)**.5
            step = min(1., .025/max(distance, 1e-8))
            self.position = [self.position[i]+delta[i]*step for i in range(3)]
            if len(self.goals) == 2 and self.scenario == 'landing_blocked':
                self.clear = False
        if self.stage == 'landing' and self.scenario == 'complete':
            self.position[2] = max(.2, self.position[2]-.035)
            if self.position[2] <= .2:
                self.stage, self.deadline = 'landed_armed', now + .4
        if self.stage == 'landed_armed' and now >= self.deadline:
            self.stage = 'ground_done'
        ground = self.stage in ('ground', 'ground_done', 'preparing')
        landed = ground or self.stage == 'landed_armed'
        self.publishers['fcu'].publish(State(connected=True, armed=not ground, mode='STABILIZED' if ground else 'OFFBOARD'))
        self.publishers['extended'].publish(ExtendedState(landed_state=ExtendedState.LANDED_STATE_ON_GROUND if landed else ExtendedState.LANDED_STATE_IN_AIR))
        ctrl = 'PREPARING_TAKEOFF' if self.stage == 'preparing' else 'MANUAL_CTRL' if ground else 'AUTO_TAKEOFF' if self.stage == 'taking_off' else 'AUTO_LAND' if self.stage in ('landing', 'landed_armed') else 'AUTO_HOVER'
        self.publishers['controller'].publish(String(data=ctrl))
        self.publishers['ready'].publish(Bool(data=self.stage == 'hover'))
        self.publishers['preflight'].publish(Bool(data=ground and self.stage != 'preparing'))
        self.publishers['control'].publish(Bool(data=self.controls_ok))
        self.publishers['height'].publish(Float64(data=1.))
        self.publishers['heartbeat'].publish(Empty())
        if self.publish_output_ready:
            self.publishers['output'].publish(Bool(data=self.output_ready))
        self.publishers['fence'].publish(Float64MultiArray(data=[-2., -2., -.5, 2., 2., 2.5]))
        if self.publish_odom:
            odom = Odometry()
            self.seq += 1
            odom.header.seq, odom.header.frame_id, odom.header.stamp = self.seq, 'world', rospy.Time.now()
            odom.pose.pose.orientation.w = 1.
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = self.position
            self.publishers['odom'].publish(odom)
            self.publishers['map'].publish(odom)

    def command(self, operation, **fields):
        request = dict(type='command', protocol=2, request_id=str(uuid.uuid4()), operation=operation,
                       map_session_id=self.status.map_session_id, mission_id=self.mission,
                       revision=1, require_map_alignment=True)
        request.update(fields)
        self.peer.sendall(json.dumps(request).encode() + b'\n')
        while True:
            message = json.loads(self.stream.readline())
            if message['type'] == 'ack' and message['request_id'] == request['request_id']:
                return message

    def test_automatic_lifecycle_and_failure_guards(self):
        route = [dict(id=1, x=1.1, y=.3, z=1.2, hover_sec=.15),
                 dict(id=2, x=.4, y=-.2, z=1.2, hover_sec=0.)]
        upload = dict(frame_id='world', automatic=True, ground_origin=dict(x=.4, y=-.2, z=.2), waypoints=route)
        self.assertFalse(self.command('upload', **dict(upload, automatic=False))['accepted'])
        self.clear = False
        self.assertFalse(self.command('upload', **upload)['accepted'])
        self.clear = True
        reply = self.command('upload', **upload)
        self.assertTrue(reply['accepted'], reply['reason'])
        self.assertEqual(reply['mission']['state'], 'READY')
        rospy.sleep(.2)
        self.assertFalse(self.takeoffs)
        self.assertFalse(self.goals)
        # Planner heartbeat alone must never allow takeoff from a stop-latched
        # or dead trajectory server. Refused starts must emit no TAKEOFF.
        self.output_ready = False
        rospy.sleep(.15)
        self.assertFalse(self.command('start')['accepted'])
        self.assertFalse(self.takeoffs)
        self.output_ready = True
        rospy.sleep(.15)
        self.publish_output_ready = False
        rospy.sleep(.4)
        self.assertFalse(self.command('start')['accepted'])
        self.assertFalse(self.takeoffs)
        self.publish_output_ready = True
        rospy.sleep(.15)
        start_id = str(uuid.uuid4())
        reply = self.command('start', request_id=start_id)
        self.assertTrue(reply['accepted'], reply['reason'])
        self.assertEqual(reply['mission']['state'], 'TAKING_OFF')
        self.until(lambda: len(self.takeoffs) == 1)
        self.assertFalse(self.goals)
        self.assertTrue(self.command('start', request_id=start_id)['accepted'])
        self.assertFalse(self.command('start')['accepted'])
        self.assertEqual(len(self.takeoffs), 1)
        if self.scenario == 'position_limit_removed':
            # Force each formerly rejected position while inputs and endpoints remain valid.
            self.stage = 'hover'
            for pos in ([.65, -.2, 1.0], [.4, -.2, 1.36], [.4, -.2, .05]):
                self.position = list(pos)
                rospy.sleep(.35)
                self.assertEqual(self.status.state, 'TAKING_OFF')
                self.assertFalse(self.aborts)
                self.assertFalse(self.goals)
            self.position = [.4, -.2, 1.2]
            self.scenario = 'complete'
        if self.scenario == 'cancel_takeoff':
            self.assertTrue(self.command('cancel')['accepted'])
        elif self.scenario == 'takeoff_odom_loss':
            self.publish_odom = False
        elif self.scenario == 'takeoff_rc_loss':
            self.controls_ok = False
        elif self.scenario == 'takeoff_corridor_blocked':
            self.clear = False
        if self.scenario.startswith('takeoff_') or self.scenario == 'cancel_takeoff':
            self.until(lambda: self.status.state == 'FAILED')
            self.until(lambda: bool(self.aborts))
            self.assertFalse(self.goals)
            self.assertFalse(self.lands)
            return
        self.until(lambda: len(self.goals) == 2)
        self.assertEqual([round(p.pose.position.x, 2) for p in self.goals], [1.1, .4])
        if self.scenario == 'landing_blocked':
            self.until(lambda: self.status.state == 'FAILED')
            self.assertFalse(self.lands)
            return
        self.until(lambda: self.status.state == 'LANDING')
        self.until(lambda: bool(self.lands))
        self.assertNotEqual(self.status.state, 'COMPLETED')
        if self.scenario == 'cancel_landing':
            self.assertTrue(self.command('cancel')['accepted'])
            self.until(lambda: self.status.state == 'FAILED' and bool(self.aborts))
            count = len(self.lands)
            rospy.sleep(.6)
            self.assertEqual(len(self.lands), count)
        elif self.scenario == 'landing_timeout':
            self.until(lambda: self.status.state == 'FAILED' and bool(self.aborts))
        else:
            self.until(lambda: self.stage == 'landed_armed')
            self.assertEqual(self.status.state, 'LANDING')
            self.until(lambda: self.status.state == 'COMPLETED')
            self.assertEqual(self.status.completed, 2)
            self.assertFalse(self.command('start')['accepted'])
        self.assertEqual(len(self.takeoffs), 1)


if __name__ == '__main__':
    if os.environ.get('ROS_MASTER_URI', '').rstrip('/') != 'http://127.0.0.1:11411':
        raise SystemExit('Automatic roundtrip tests require isolated master 11411')
    rospy.init_node('test_automatic_roundtrip')
    rostest.rosrun('multipoint', 'automatic_roundtrip', AutomaticRoundtripTest)
