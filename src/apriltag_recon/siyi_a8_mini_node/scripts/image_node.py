#!/usr/bin/env python3
"""Publish fresh decoded images for ROS 1 consumers such as AprilTag."""

import math
import os
import time

import rospy
from sensor_msgs.msg import Image

from siyi_a8_mini_node.capture import Capture, source_with_credentials


def main():
    rospy.init_node("siyi_a8_mini")
    source_override = os.environ.get("SIYI_RTSP_URL")
    source = source_override or rospy.get_param("~source", "rtsp://192.168.144.25:8554/main.264")
    credentials_file = rospy.get_param("~credentials_file", "")
    if credentials_file and not source_override:
        source = source_with_credentials(source, credentials_file)
    frame_id = rospy.get_param("~frame_id", "siyi_a8_mini_optical_frame")
    max_fps = float(rospy.get_param("~max_fps", 0.0))
    max_age = float(rospy.get_param("~max_frame_age_sec", 1.0))
    startup_timeout = float(rospy.get_param("~startup_timeout_sec", 30.0))
    stall_timeout = float(rospy.get_param("~stall_timeout_sec", 3.0))
    quiet_time = float(rospy.get_param("~error_quiet_sec", 1.0))
    file_loop = bool(rospy.get_param("~file_loop", False))
    if not math.isfinite(max_fps) or max_fps < 0 or any(
            not math.isfinite(x) or x <= 0 for x in
            (max_age, startup_timeout, stall_timeout, quiet_time)):
        raise ValueError("FPS must be nonnegative and timeouts must be positive finite values")

    def log(level, text):
        (rospy.logwarn if level == "warning" else rospy.loginfo)(text)

    publisher = rospy.Publisher("/siyi_a8_mini/image_raw", Image, queue_size=1)
    capture = Capture(source, log, file_loop=file_loop, startup_timeout=startup_timeout,
                      stall_timeout=stall_timeout, quiet_time=quiet_time).start()
    previous = None
    next_publish = 0.0
    next_status = time.monotonic() + 10.0
    try:
        while not rospy.is_shutdown():
            frame = capture.read_latest(previous, timeout=0.2, max_age=max_age)
            now = time.monotonic()
            if frame is not None:
                previous, width, height, pixels = frame
                if now >= next_publish:
                    message = Image()
                    message.header.stamp = rospy.Time.now()
                    message.header.frame_id = frame_id
                    message.width, message.height = width, height
                    message.encoding = "rgb8"
                    message.is_bigendian = False
                    message.step = width * 3
                    message.data = pixels
                    publisher.publish(message)
                    next_publish = now + 1.0 / max_fps if max_fps else 0.0
            if now >= next_status:
                snapshot = capture.snapshot()
                rospy.loginfo("video state=%s frames=%d corruptions=%d reference_notes=%d reconnects=%d",
                              snapshot["state"], snapshot["accepted"],
                              snapshot["corruptions"], snapshot["reference_notes"], snapshot["restarts"])
                next_status = now + 10.0
    finally:
        capture.close()


if __name__ == "__main__":
    main()
