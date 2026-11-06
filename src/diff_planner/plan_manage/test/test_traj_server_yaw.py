#!/usr/bin/env python3
"""Exercise the real trajectory node with mocked inputs and no flight controller."""
import math
import time
import unittest

import rospy
import rostest
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
from std_msgs.msg import Empty
from traj_utils.msg import PolyTraj


def yaw_error(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class TrajectoryYawTest(unittest.TestCase):
    def test_startup_wrap_reversal_limits_and_stop(self):
        commands = []
        subscriber = rospy.Subscriber('/test/yaw/command', PositionCommand, commands.append)
        odom = rospy.Publisher('/test/yaw/odom', Odometry, queue_size=1)
        trajectory = rospy.Publisher('/test/yaw/trajectory', PolyTraj, queue_size=1)
        heartbeat = rospy.Publisher('/test/yaw/heartbeat', Empty, queue_size=1)
        override = rospy.Publisher('/test/yaw/override', PositionCommand, queue_size=1)
        stop = rospy.Publisher('/test/yaw/stop', Empty, queue_size=1)
        takeoff = rospy.Publisher('/test/yaw/takeoff_land', TakeoffLand, queue_size=1)
        publishers = (odom, trajectory, heartbeat, override, stop, takeoff)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not all(p.get_num_connections() for p in publishers):
            rospy.sleep(.02)
        self.assertTrue(all(p.get_num_connections() for p in publishers))

        def pump(seconds, with_odom=True, yaw_override=None):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                heartbeat.publish(Empty())
                if with_odom:
                    message = Odometry()
                    message.header.stamp = rospy.Time.now()
                    message.pose.pose.orientation.w = math.cos(math.radians(175) / 2)
                    message.pose.pose.orientation.z = math.sin(math.radians(175) / 2)
                    odom.publish(message)
                if yaw_override is not None:
                    override.publish(PositionCommand(yaw=yaw_override))
                rospy.sleep(.01)

        # At 0.25 m/s, the 1-second preview is shorter than the low-speed
        # route-distance threshold. It must still produce a usable heading.
        plan = PolyTraj(order=5, traj_id=1, start_time=rospy.Time.now(), duration=[20.],
                        coef_x=[0., 0., 0., 0., -.25, 0.], coef_y=[0.] * 6,
                        coef_z=[0., 0., 0., 0., 0., 1.])
        trajectory.publish(plan)
        pump(.25, with_odom=False)
        self.assertFalse(commands)
        pump(1.8)
        self.assertGreater(len(commands), 80)
        self.assertLess(abs(yaw_error(commands[0].yaw, math.radians(175))), .003)
        self.assertLess(abs(yaw_error(commands[-1].yaw, math.pi)), math.radians(1))
        self.assertLess(max(abs(yaw_error(c.yaw, math.pi)) for c in commands), math.radians(6))

        pump(.15, yaw_override=float('nan'))
        turn_start = len(commands)
        pump(5.2, yaw_override=0.)
        self.assertLess(abs(yaw_error(commands[-1].yaw, 0.)), math.radians(5))
        self.assertGreater(abs(yaw_error(commands[turn_start].yaw, 0.)), math.radians(170))
        for command in commands:
            self.assertTrue(math.isfinite(command.yaw) and math.isfinite(command.yaw_dot))
            self.assertLessEqual(abs(command.yaw_dot), math.radians(45) + 1e-7)
        for first, second in zip(commands, commands[1:]):
            dt = (second.header.stamp - first.header.stamp).to_sec()
            if 0 < dt <= .05:
                self.assertLessEqual(abs(second.yaw_dot - first.yaw_dot) / dt,
                                     math.radians(92))
        # Header timestamps are taken at publication, after the timer samples its
        # integration time. Check the accumulated angle without amplifying that
        # microsecond scheduling jitter into a per-message derivative.
        integrated_yaw = sum(
            .5 * (a.yaw_dot + b.yaw_dot) * (b.header.stamp - a.header.stamp).to_sec()
            for a, b in zip(commands, commands[1:]))
        self.assertLess(abs(yaw_error(commands[-1].yaw - commands[0].yaw, integrated_yaw)), .01)

        stop.publish(Empty())
        pump(.12)
        count = len(commands)
        takeoff.publish(TakeoffLand(takeoff_land_cmd=TakeoffLand.TAKEOFF))
        plan.start_time = rospy.Time.now()
        trajectory.publish(plan)
        pump(.3)
        self.assertEqual(len(commands), count)
        subscriber.unregister()


if __name__ == '__main__':
    rospy.init_node('trajectory_yaw_test')
    rostest.rosrun('diff_planner', 'traj_server_yaw', TrajectoryYawTest)
