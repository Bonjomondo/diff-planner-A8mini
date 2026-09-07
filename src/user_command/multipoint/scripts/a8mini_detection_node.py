#!/usr/bin/env python3
"""Supervise the RTSP-only detector without importing ML libraries into ROS."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import threading

import rospy
# Catkin's devel relay changes __file__ but not sys.path; import the source helper.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from a8mini_diagnostics import log


def relay_output(stream):
    """Timestamp native FFmpeg/TensorRT stderr too, at the local receive boundary."""
    try:
        for line in stream:
            log("[DETECTION_PROCESS] " + line.rstrip())
    finally:
        stream.close()


def build_command(params, script):
    command = [str(Path(params.get("python_executable", sys.executable)).expanduser()),
               "-u", str(script)]
    for key in ("repo_path", "model", "source", "backend", "codec", "imgsz", "conf",
                "max_fps", "cpu_threads", "video_dir", "video_fps", "latency_ms",
                "open_timeout_ms", "read_timeout_ms", "reconnect_delay",
                "max_frame_age_ms"):
        if key in params:
            command.extend(["--" + key.replace("_", "-"), str(params[key])])
    for key, default in (("save_detection_video", False), ("display", True)):
        value = params.get(key, default)
        if not isinstance(value, bool):
            raise ValueError(key + " must be a YAML boolean (true/false)")
        if key == "save_detection_video" and value:
            command.append("--save-video")
        if key == "display" and not value:
            command.append("--no-display")
    return command


def main():
    rospy.init_node("a8mini_detection")
    params = rospy.get_param("~", {})
    enabled = params.get("enable_realtime_detection", False)
    if not isinstance(enabled, bool):
        raise ValueError("enable_realtime_detection must be a YAML boolean (true/false)")
    if not enabled:
        rospy.loginfo("A8 mini realtime detection disabled by mission configuration")
        return 0
    command = build_command(params,
                            Path(__file__).resolve().with_name("a8mini_detection.py"))
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    # Keep capture grandchildren in a dedicated group for bounded shutdown, even
    # when the inference process is stuck inside a native library.
    process = subprocess.Popen(command, env=env, start_new_session=True,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, errors="replace", bufsize=1)
    relay = threading.Thread(target=relay_output, args=(process.stdout,), daemon=True)
    relay.start()

    def signal_group(sig):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass

    def stop():
        signal_group(signal.SIGINT)
        try:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                signal_group(signal.SIGTERM)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    signal_group(signal.SIGKILL)
                    process.wait()
        finally:
            # A dead parent may still have an orphaned RTSP capture process.
            signal_group(signal.SIGKILL)

    rospy.on_shutdown(stop)
    rospy.loginfo("A8 mini RTSP detection started, pid=%d", process.pid)
    try:
        while not rospy.is_shutdown() and process.poll() is None:
            rospy.rostime.wallsleep(0.2)
        if not rospy.is_shutdown() and process.returncode:
            rospy.logerr("A8 mini detection exited with code %d; check detector output",
                         process.returncode)
            return process.returncode
    finally:
        stop()
        relay.join(timeout=2)
        rospy.loginfo("A8 mini detection worker exit: pid=%d returncode=%s output_drained=%s",
                      process.pid, process.returncode, not relay.is_alive())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
