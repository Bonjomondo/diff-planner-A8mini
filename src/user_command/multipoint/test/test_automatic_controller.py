#!/usr/bin/env python3
"""Real px4ctrl FSM; all MAVROS services and sensors are test-owned mocks."""
import os
import time
import unittest
import rospy
import rostest
from mavros_msgs.msg import State, ExtendedState, RCIn, AttitudeTarget
from mavros_msgs.srv import SetMode, SetModeResponse, CommandBool, CommandBoolResponse
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import TakeoffLand
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import Bool, String, Empty


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.mode, self.armed, self.z, self.switches = 'STABILIZED', False, .2, False
        self.state, self.ready, self.arm_calls, self.mode_calls = '', False, [], []
        self.reject_mode = False
        self.abort_during_mode = False
        self.feedback_delay = float(rospy.get_param('~state_feedback_delay', 0.))
        self.feedback_resume_at = 0.
        self.freeze_offboard_feedback = False
        self.last_fcu_feedback = ('', 0.)
        self.offboard_request_at = 0.
        self.arm_feedback = []
        self.outputs = []
        self.services = [rospy.Service('/mavros/set_mode', SetMode, self.set_mode),
                         rospy.Service('/mavros/cmd/arming', CommandBool, self.set_arm)]
        self.subs = [rospy.Subscriber('/px4ctrl/state', String, self.on_state),
                     rospy.Subscriber('/px4ctrl/takeoff_ready', Bool, self.on_ready),
                     rospy.Subscriber('/mavros/setpoint_raw/attitude', AttitudeTarget,
                                      lambda msg: self.outputs.append((time.monotonic(), msg.thrust)))]
        self.pubs = {name: rospy.Publisher(topic, cls, queue_size=1) for name, topic, cls in [
            ('fcu', '/mavros/state', State), ('ext', '/mavros/extended_state', ExtendedState),
            ('rc', '/mavros/rc/in', RCIn), ('odom', '/ekf/ekf_odom', Odometry),
            ('imu', '/mavros/imu/data', Imu), ('bat', '/mavros/battery', BatteryState),
            ('takeoff', '/px4ctrl/takeoff_land', TakeoffLand), ('abort', '/px4ctrl/abort_automatic', Empty)]}
        self.timer = rospy.Timer(rospy.Duration(.01), self.tick)
        self.until(lambda: bool(self.state))

    def tearDown(self):
        self.timer.shutdown()
        for service in self.services:
            service.shutdown()

    def on_state(self, message):
        self.state = message.data

    def on_ready(self, message):
        self.ready = message.data

    def until(self, condition, timeout=7):
        end = time.monotonic()+timeout
        while not condition() and time.monotonic()<end:
            rospy.sleep(.02)
        self.assertTrue(condition(), 'controller state='+self.state)

    def set_mode(self, request):
        self.mode_calls.append(request.custom_mode)
        if not self.reject_mode:
            self.mode = request.custom_mode
            if request.custom_mode == 'OFFBOARD':
                self.offboard_request_at = time.monotonic()
                self.feedback_resume_at = time.monotonic()+self.feedback_delay
        if self.abort_during_mode and request.custom_mode == 'OFFBOARD':
            self.pubs['abort'].publish(Empty())
        return SetModeResponse(mode_sent=not self.reject_mode)

    def set_arm(self, request):
        self.arm_calls.append(request.value)
        self.arm_feedback.append((self.last_fcu_feedback, self.offboard_request_at))
        self.armed = request.value
        return CommandBoolResponse(success=True, result=0)

    def tick(self, _):
        stamp = rospy.Time.now()
        if time.monotonic() >= self.feedback_resume_at:
            observed_mode = 'STABILIZED' if self.freeze_offboard_feedback and self.mode == 'OFFBOARD' else self.mode
            self.last_fcu_feedback = (observed_mode, time.monotonic())
            self.pubs['fcu'].publish(State(connected=True, armed=self.armed, mode=observed_mode))
        self.pubs['ext'].publish(ExtendedState(landed_state=ExtendedState.LANDED_STATE_ON_GROUND if self.z < .3 else ExtendedState.LANDED_STATE_IN_AIR))
        channels = [1500]*4 + [1999 if self.switches else 999]*2 + [999, 999]
        self.pubs['rc'].publish(RCIn(channels=channels))
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'world'
        odom.pose.pose.position.z, odom.pose.pose.orientation.w = self.z, 1.
        self.pubs['odom'].publish(odom)
        imu = Imu()
        imu.header.stamp, imu.orientation.w, imu.linear_acceleration.z = stamp, 1., 9.81
        self.pubs['imu'].publish(imu)
        self.pubs['bat'].publish(BatteryState(voltage=25.2, cell_voltage=[4.2]*6, percentage=.9))

    def test_explicit_takeoff_mode_failure_and_abort(self):
        self.assertFalse(self.arm_calls)
        self.assertFalse(self.mode_calls)
        self.assertFalse(self.ready)
        self.switches = True
        self.until(lambda: self.ready)
        rospy.sleep(.25)
        self.assertEqual(self.state, 'MANUAL_CTRL')
        self.assertFalse(self.arm_calls)  # Switch movement alone cannot arm or enter OFFBOARD.
        self.assertFalse(self.mode_calls)
        self.reject_mode = True
        self.pubs['takeoff'].publish(TakeoffLand(takeoff_land_cmd=TakeoffLand.TAKEOFF))
        self.until(lambda: bool(self.mode_calls))
        rospy.sleep(.2)
        self.assertFalse(self.arm_calls)
        self.assertEqual(self.state, 'MANUAL_CTRL')
        self.reject_mode = False
        self.until(lambda: self.ready)
        self.abort_during_mode = True
        count = len(self.mode_calls)
        self.pubs['takeoff'].publish(TakeoffLand(takeoff_land_cmd=TakeoffLand.TAKEOFF))
        self.until(lambda: len(self.mode_calls) >= count+2)
        rospy.sleep(.2)
        self.assertFalse(self.arm_calls)  # Cancel while mode changes must be rechecked before arming.
        self.assertEqual(self.state, 'MANUAL_CTRL')
        self.abort_during_mode = False
        self.until(lambda: self.ready)
        # A service accepting the mode request is insufficient without FCU feedback.
        self.freeze_offboard_feedback = True
        count = len(self.mode_calls)
        self.pubs['takeoff'].publish(TakeoffLand(takeoff_land_cmd=TakeoffLand.TAKEOFF))
        self.until(lambda: self.state == 'PREPARING_TAKEOFF')
        rospy.sleep(.3)
        self.assertFalse(self.arm_calls)
        recent = [thrust for stamp,thrust in self.outputs if stamp > time.monotonic()-.2]
        self.assertGreater(len(recent), 10)  # Waiting must not block the setpoint stream.
        self.assertTrue(all(thrust <= .041 for thrust in recent))
        self.until(lambda: len(self.mode_calls) >= count+2, timeout=5)
        self.until(lambda: self.state == 'MANUAL_CTRL')
        self.assertFalse(self.arm_calls)
        self.freeze_offboard_feedback = False
        self.until(lambda: self.ready)
        if self.feedback_delay >= .3:
            # RC changes during the delayed mode handshake must still stop arming.
            self.pubs['takeoff'].publish(TakeoffLand(takeoff_land_cmd=TakeoffLand.TAKEOFF))
            self.until(lambda: self.state == 'PREPARING_TAKEOFF')
            self.switches = False
            self.until(lambda: self.state == 'MANUAL_CTRL')
            self.assertFalse(self.arm_calls)
            self.switches = True
            self.until(lambda: self.ready)
        self.pubs['takeoff'].publish(TakeoffLand(takeoff_land_cmd=TakeoffLand.TAKEOFF))
        self.until(lambda: self.armed and self.state == 'AUTO_TAKEOFF')
        self.assertEqual(self.arm_calls, [True])
        feedback, requested = self.arm_feedback[0]
        self.assertEqual(feedback[0], 'OFFBOARD')
        self.assertGreaterEqual(feedback[1], requested)
        self.z = .6
        self.pubs['abort'].publish(Empty())
        self.until(lambda: self.state == 'AUTO_HOVER')
        self.assertTrue(self.armed)
        self.assertEqual(self.arm_calls, [True])  # Never disarm in air on cancellation.
        self.switches = False
        self.until(lambda: self.state == 'MANUAL_CTRL')
        self.assertEqual(self.arm_calls, [True])


if __name__ == '__main__':
    if os.environ.get('ROS_MASTER_URI', '').rstrip('/') != 'http://127.0.0.1:11411':
        raise SystemExit('Automatic controller tests require isolated master 11411')
    rospy.init_node('test_automatic_controller')
    rostest.rosrun('multipoint', 'automatic_controller', ControllerTest)
