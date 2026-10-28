#!/usr/bin/env python3
"""Actual px4ctrl binary with noisy ground odometry on isolated ROS master."""
import math, os, time, unittest
import rospy, rostest
from mavros_msgs.msg import State, ExtendedState, RCIn, AttitudeTarget
from mavros_msgs.srv import SetMode, SetModeResponse, CommandBool, CommandBoolResponse
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import Bool, String, Empty
from quadrotor_msgs.msg import TakeoffLand

class SpoolTest(unittest.TestCase):
    def until(self, predicate, timeout=8):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:rospy.sleep(.01)
        self.assertTrue(predicate(), 'state={} armed={} hold={}'.format(self.state,self.armed,self.hold))
    def on_mode(self, req):
        self.mode=req.custom_mode;self.mode_calls.append(self.mode)
        return SetModeResponse(mode_sent=True)
    def on_arm(self, req):
        self.armed=req.value;self.arm_calls.append(req.value)
        if req.value:self.arm_time=time.monotonic()
        return CommandBoolResponse(success=True,result=0)
    def tick(self, _):
        stamp=rospy.Time.now();dt=time.monotonic()-self.arm_time if self.armed else 0.
        self.pubs['fcu'].publish(State(connected=True,armed=self.armed,mode=self.mode))
        self.pubs['ext'].publish(ExtendedState(landed_state=2 if self.armed else 1))
        self.pubs['rc'].publish(RCIn(channels=[1500]*4+[1999 if self.switches else 999]*2+[999,999]))
        odom=Odometry();odom.header.frame_id='world';odom.header.stamp=stamp;odom.pose.pose.orientation.w=1.
        odom.pose.pose.position.z=self.z
        if self.armed:
            odom.pose.pose.position.x=.025*math.sin(11*dt)
            odom.pose.pose.position.y=.025*math.cos(13*dt)
            odom.twist.twist.linear.x=.25*math.sin(17*dt)
            odom.twist.twist.linear.y=.20*math.cos(19*dt)
        self.pubs['odom'].publish(odom)
        imu=Imu();imu.header.stamp=stamp
        imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w=self.q
        imu.linear_acceleration.z=9.81;self.pubs['imu'].publish(imu)
        self.pubs['bat'].publish(BatteryState(voltage=24.,cell_voltage=[4.]*6,percentage=.8))
    def setUp(self):
        self.assertEqual(rospy.get_master().getUri()[2],'http://127.0.0.1:11411/')
        self.enabled=rospy.get_param('~enabled');self.mode='STABILIZED';self.armed=False
        self.z=.2;self.switches=False;self.arm_time=0.;self.state='';self.ready=False;self.hold=False
        self.outputs=[];self.arm_calls=[];self.mode_calls=[]
        self.q=(.01,-.005,.15,math.sqrt(1-.01**2-.005**2-.15**2))
        self.services=[rospy.Service('/mavros/set_mode',SetMode,self.on_mode),rospy.Service('/mavros/cmd/arming',CommandBool,self.on_arm)]
        def state(m):self.state=m.data
        def ready(m):self.ready=m.data
        def hold(m):self.hold=m.data
        self.subs=[rospy.Subscriber('/px4ctrl/state',String,state),rospy.Subscriber('/px4ctrl/takeoff_ready',Bool,ready),
          rospy.Subscriber('/px4ctrl/ground_spool_attitude_held',Bool,hold),
          rospy.Subscriber('/mavros/setpoint_raw/attitude',AttitudeTarget,lambda m:self.outputs.append((time.monotonic(),m)))]
        self.pubs={k:rospy.Publisher(t,c,queue_size=1) for k,t,c in [
          ('fcu','/mavros/state',State),('ext','/mavros/extended_state',ExtendedState),('rc','/mavros/rc/in',RCIn),
          ('odom','/ekf/ekf_odom',Odometry),('imu','/mavros/imu/data',Imu),('bat','/mavros/battery',BatteryState),
          ('takeoff','/px4ctrl/takeoff_land',TakeoffLand),('abort','/px4ctrl/abort_automatic',Empty)]}
        self.timer=rospy.Timer(rospy.Duration(.01),self.tick)
        self.until(lambda:bool(self.state))
    def tearDown(self):
        self.timer.shutdown()
        for s in self.services:s.shutdown()
    def angular_error(self,m):
        q=m.orientation;dot=abs(sum(a*b for a,b in zip(self.q,[q.x,q.y,q.z,q.w])))
        return 2*math.acos(min(1,dot))
    def test_no_ground_tilt_and_restore_control(self):
        self.assertFalse(self.arm_calls);self.assertFalse(self.mode_calls)
        self.switches=True;self.until(lambda:self.ready)
        rospy.sleep(.2)
        self.assertEqual(self.state,'MANUAL_CTRL');self.assertFalse(self.arm_calls)
        self.assertEqual(rospy.get_param('/px4ctrl/auto_takeoff_land/hold_attitude_during_spool'),self.enabled)
        self.pubs['takeoff'].publish(TakeoffLand(takeoff_land_cmd=TakeoffLand.TAKEOFF))
        self.until(lambda:self.armed and self.state=='AUTO_TAKEOFF')
        if rospy.get_param('~cancel_during_spool',False):
            self.until(lambda:time.monotonic()-self.arm_time>.6)
            self.assertTrue(self.hold)
            self.z=.6;self.pubs['abort'].publish(Empty())
            self.until(lambda:self.state=='AUTO_HOVER' and not self.hold)
            self.assertEqual(self.arm_calls,[True]);self.assertTrue(self.armed)
            return
        self.until(lambda:time.monotonic()-self.arm_time>2.7)
        window=[m for t,m in self.outputs if .4<t-self.arm_time<2.6]
        self.assertGreater(len(window),150)
        errors=[self.angular_error(m) for m in window]
        self.assertTrue(all(.10<m.thrust<.3 for m in window))
        if self.enabled:
            self.assertTrue(self.hold)
            self.assertLess(max(errors),.001)
        else:
            self.assertFalse(self.hold)
            self.assertGreater(max(errors),.02)  # Reproduces noisy-odometry tilt output.
        self.until(lambda:time.monotonic()-self.arm_time>3.5)
        self.assertFalse(self.hold)  # The spool hold cannot persist into the ascent.
        after=[m for t,m in self.outputs if 3.2<t-self.arm_time<3.5]
        self.assertGreater(len(after),20)
        self.assertGreater(max(self.angular_error(m) for m in after),.02)
        self.z=.6;self.pubs['abort'].publish(Empty())
        self.until(lambda:self.state=='AUTO_HOVER')
        self.assertEqual(self.arm_calls,[True]);self.assertTrue(self.armed)
        self.assertFalse(self.hold)
        self.switches=False;self.until(lambda:self.state=='MANUAL_CTRL')
        self.assertEqual(self.arm_calls,[True])

if __name__=='__main__':
    if os.environ.get('ROS_MASTER_URI','').rstrip('/')!='http://127.0.0.1:11411':raise SystemExit('Isolated master 11411 only')
    rospy.init_node('test_spool_attitude')
    rostest.rosrun('multipoint','spool_attitude',SpoolTest)
