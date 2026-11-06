#!/usr/bin/env python3
"""Isolated real-executor joint test endpoint; never connects to flight hardware.

Start ONLY with ROS_MASTER_URI=http://127.0.0.1:11351. A private roscore must
already run on that port. This fixture publishes synthetic airborne telemetry;
it does not invoke arming, takeoff, or the physical flight stack/camera.
"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11351':
    raise SystemExit('Refusing to use a non-test ROS master; expected port11351')

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, String
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from gcs_gateway_node import RosBackend
from gcs_protocol import GatewayServer


class Backend(RosBackend):
    def telemetry(self):
        result = super().telemetry()
        result['mission']['test_backend'] = 'gimbal-dry-run'
        return result

    def command(self, command):
        accepted, reason, mission = super().command(command)
        mission['test_backend'] = 'gimbal-dry-run'
        return accepted, reason, mission


def main():
    rospy.init_node('gimbal_joint_harness', anonymous=True, disable_signals=True)
    children = []
    server = None
    def stop(*unused):
        rospy.signal_shutdown('joint fixture stopped')
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        children.append(subprocess.Popen(['rosrun', 'multipoint', 'multipointplan',
            '__name:=joint_executor', '_mission_source:=remote', '_enable_rc:=false',
            '_enable_start_trigger:=0', '_enable_back_trigger:=0', '_allow_gimbal_dry_run:=true',
            '_arrival_stable_sec:=0.1', '_gimbal_retry_sec:=12', 'odom_topic:=/joint/odom']))
        children.append(subprocess.Popen(['rosrun', 'multipoint', 'a8mini_gimbal_node.py',
            '__name:=joint_gimbal', '_dry_run:=true', '_enable_auto_recording:=false']))
        pose = [0., 0., 1.]
        target = [0., 0., 1.]
        goal_lock = threading.Lock()
        def goal(message):
            with goal_lock:
                target[:] = [message.pose.position.x, message.pose.position.y, message.pose.position.z]
        subscriber = rospy.Subscriber('/goal', PoseStamped, goal, queue_size=1)
        odom = rospy.Publisher('/joint/odom', Odometry, queue_size=1)
        fcu = rospy.Publisher('/mavros/state', State, queue_size=1)
        extended = rospy.Publisher('/mavros/extended_state', ExtendedState, queue_size=1)
        controller = rospy.Publisher('/px4ctrl/state', String, queue_size=1)
        ready = rospy.Publisher('/px4ctrl/mission_ready', Bool, queue_size=1)
        heartbeat = rospy.Publisher('/drone_0_traj_server/heartbeat', Empty, queue_size=1)
        start = time.monotonic()
        def tick(event):
            if any(child.poll() is not None for child in children):
                rospy.signal_shutdown('fixture child exited')
                return
            with goal_lock:
                for i in range(3):
                    error = target[i] - pose[i]
                    pose[i] += max(-.06, min(.06, error))
                current = list(pose)
            sample = Odometry()
            sample.header.frame_id, sample.header.stamp = 'world', rospy.Time.now()
            sample.pose.pose.orientation.w = 1.
            sample.pose.pose.position.x, sample.pose.pose.position.y, sample.pose.pose.position.z = current
            odom.publish(sample)
            fcu.publish(State(connected=True, armed=True, mode='OFFBOARD'))
            extended.publish(ExtendedState(landed_state=ExtendedState.LANDED_STATE_IN_AIR))
            controller.publish(String(data='AUTO_HOVER'))
            ready.publish(Bool(data=True))
            heartbeat.publish(Empty())
            if time.monotonic() - start > 900:
                rospy.signal_shutdown('fixture TTL reached')
        timer = rospy.Timer(rospy.Duration(.03), tick)
        backend = Backend()
        backend.wait_for_status()
        if rospy.is_shutdown():
            return
        server = GatewayServer(('127.0.0.1', 18765), backend)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        print('JOINT_TEST_READY master11351 endpoint127.0.0.1:18765 marker=gimbal-dry-run', flush=True)
        rospy.spin()
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
        for child in children:
            child.terminate()
        for child in children:
            try:
                child.wait(timeout=4)
            except subprocess.TimeoutExpired:
                child.kill()


if __name__ == '__main__':
    main()
