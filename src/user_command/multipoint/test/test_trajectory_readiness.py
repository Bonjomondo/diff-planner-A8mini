#!/usr/bin/env python3
"""Exercise the real trajectory server, exclusively on isolated ROS master."""
import os
import subprocess
import time
import unittest
import rospy
import rostest
from std_msgs.msg import Bool, Empty
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
from traj_utils.msg import PolyTraj

class ReadinessTest(unittest.TestCase):
    def until(self, predicate, timeout=5):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            rospy.sleep(.02)
        self.assertTrue(predicate())

    def launch(self):
        self.ready = None
        self.child = subprocess.Popen(['rosrun', 'diff_planner', 'traj_server',
                                      '__name:=drone_0_traj_server'], env=os.environ.copy())
        self.until(lambda: self.ready is not None)

    def stop(self):
        if self.child is not None:
            self.child.terminate()
            self.child.wait(timeout=5)
            self.child = None

    def test_real_output_and_latched_stop(self):
        self.assertEqual(rospy.get_master().getUri()[2], 'http://127.0.0.1:11411/')
        self.child, self.ready, self.count = None, None, 0
        self.heartbeat_enabled = False
        heartbeat = rospy.Publisher('/drone_0_traj_server/heartbeat', Empty, queue_size=1)
        trajectory = rospy.Publisher('/drone_0_traj_server/planning/trajectory', PolyTraj, queue_size=1)
        stop = rospy.Publisher('/planning/stop', Empty, queue_size=1, latch=True)
        flight = rospy.Publisher('/px4ctrl/takeoff_land', TakeoffLand, queue_size=1)
        def ready_cb(msg): self.ready = msg.data
        def cmd_cb(msg): self.count += 1
        ready_sub = rospy.Subscriber('/drone_0_traj_server/output_ready', Bool, ready_cb)
        cmd_sub = rospy.Subscriber('/position_cmd', PositionCommand, cmd_cb)
        timer = rospy.Timer(rospy.Duration(.03), lambda _: heartbeat.publish(Empty()) if self.heartbeat_enabled else None)
        try:
            self.launch()
            self.assertFalse(self.ready)  # No planner heartbeat.
            self.heartbeat_enabled = True
            self.until(lambda: self.ready is True)
            self.heartbeat_enabled = False
            self.until(lambda: self.ready is False)  # Heartbeat expiry.
            self.heartbeat_enabled = True
            self.until(lambda: self.ready is True)
            self.until(lambda: trajectory.get_num_connections() > 0)
            def send_traj():
                msg = PolyTraj(order=5, traj_id=1, start_time=rospy.Time.now(), duration=[100.])
                msg.coef_x = msg.coef_y = [0.] * 6
                msg.coef_z = [0.] * 5 + [1.]
                trajectory.publish(msg)
            send_traj()
            self.until(lambda: self.count >= 5)  # Real PositionCommand emitted.
            stop.publish(Empty())
            self.until(lambda: self.ready is False)
            rospy.sleep(.15)
            count = self.count
            flight.publish(TakeoffLand(takeoff_land_cmd=TakeoffLand.TAKEOFF))
            send_traj()
            rospy.sleep(.2)
            self.assertFalse(self.ready)
            self.assertEqual(self.count, count)
            self.stop()
            self.launch()  # Reproduces a new subscriber receiving the old stop.
            rospy.sleep(.2)
            self.assertFalse(self.ready)
            stop.unregister()
            self.stop()
            self.launch()  # No old publisher, new server, fresh heartbeat.
            self.until(lambda: self.ready is True)
            self.until(lambda: flight.get_num_connections() > 0)
            flight.publish(TakeoffLand(takeoff_land_cmd=TakeoffLand.LAND))
            self.until(lambda: self.ready is False)
        finally:
            self.stop()
            timer.shutdown()
            ready_sub.unregister()
            cmd_sub.unregister()

if __name__ == '__main__':
    rospy.init_node('trajectory_readiness_test')
    rostest.rosrun('multipoint', 'trajectory_readiness', ReadinessTest)
