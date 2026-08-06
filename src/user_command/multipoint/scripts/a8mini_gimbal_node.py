#!/usr/bin/env python3
"""Execute waypoint gimbal tasks through the SIYI Ethernet SDK."""

import binascii
import math
import queue
import socket
import struct
import threading
import time

import rospy
from std_msgs.msg import Float64MultiArray, UInt32


SIYI_HEADER = b"\x55\x66"
CMD_GIMBAL_MODE = 0x0C
CMD_REQUEST_ATTITUDE = 0x0D
CMD_SET_ATTITUDE = 0x0E
LOCK_MODE = 3


class SiyiProtocolError(RuntimeError):
    pass


def crc16_ccitt(data):
    """SIYI CRC16: CCITT polynomial 0x1021 with an initial value of zero."""
    return binascii.crc_hqx(data, 0)


def encode_packet(sequence, command_id, payload=b"", need_ack=True):
    control = 0x01 if need_ack else 0x00
    body = (
        SIYI_HEADER
        + bytes((control,))
        + struct.pack("<H", len(payload))
        + struct.pack("<H", sequence & 0xFFFF)
        + bytes((command_id,))
        + payload
    )
    return body + struct.pack("<H", crc16_ccitt(body))


def decode_packet(packet):
    if len(packet) < 10 or packet[:2] != SIYI_HEADER:
        raise SiyiProtocolError("invalid SIYI packet header or length")

    payload_length = struct.unpack_from("<H", packet, 3)[0]
    expected_length = 8 + payload_length + 2
    if len(packet) != expected_length:
        raise SiyiProtocolError(
            "invalid SIYI packet length: expected {}, received {}".format(
                expected_length, len(packet)
            )
        )

    expected_crc = struct.unpack_from("<H", packet, len(packet) - 2)[0]
    actual_crc = crc16_ccitt(packet[:-2])
    if actual_crc != expected_crc:
        raise SiyiProtocolError("invalid SIYI packet CRC")

    return {
        "control": packet[2],
        "sequence": struct.unpack_from("<H", packet, 5)[0],
        "command_id": packet[7],
        "payload": packet[8:-2],
    }


class SiyiUdpClient:
    def __init__(self, camera_ip, camera_port, timeout_sec, retries):
        self.camera_address = (camera_ip, camera_port)
        self.timeout_sec = timeout_sec
        self.retries = retries
        self.sequence = 0
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(timeout_sec)
        self.lock = threading.Lock()

    def close(self):
        self.socket.close()

    def _next_sequence(self):
        sequence = self.sequence
        self.sequence = (self.sequence + 1) & 0xFFFF
        return sequence

    def _drain_receive_buffer(self):
        self.socket.setblocking(False)
        try:
            while True:
                self.socket.recvfrom(2048)
        except (BlockingIOError, socket.timeout):
            pass
        finally:
            self.socket.settimeout(self.timeout_sec)

    def send_command(self, command_id, payload=b"", expect_ack=True):
        with self.lock:
            sequence = self._next_sequence()
            # SIYI's official frames set need_ack even for commands documented as
            # having no response (for example lock mode 0x0C/03).
            packet = encode_packet(sequence, command_id, payload, need_ack=True)
            self._drain_receive_buffer()

            for attempt in range(1, self.retries + 1):
                self.socket.sendto(packet, self.camera_address)
                if not expect_ack:
                    return b""

                deadline = time.monotonic() + self.timeout_sec
                while time.monotonic() < deadline:
                    try:
                        response, source = self.socket.recvfrom(2048)
                    except socket.timeout:
                        break

                    if source[0] != self.camera_address[0]:
                        continue
                    try:
                        decoded = decode_packet(response)
                    except SiyiProtocolError as error:
                        rospy.logwarn("Discarding malformed SIYI response: %s", error)
                        continue

                    if decoded["command_id"] != command_id:
                        continue
                    # Some firmware revisions do not echo sequence consistently, so the
                    # command ID and CRC are treated as the authoritative ACK match.
                    return decoded["payload"]

                rospy.logwarn(
                    "SIYI command 0x%02X timed out (attempt %d/%d)",
                    command_id,
                    attempt,
                    self.retries,
                )

            raise SiyiProtocolError(
                "no response from {}:{} for command 0x{:02X}".format(
                    self.camera_address[0], self.camera_address[1], command_id
                )
            )

    def set_lock_mode(self):
        # 0x0C mode commands intentionally have no ACK in the SIYI protocol.
        self.send_command(CMD_GIMBAL_MODE, bytes((LOCK_MODE,)), expect_ack=False)

    def set_attitude(self, yaw_deg, pitch_deg):
        payload = struct.pack(
            "<hh", int(round(yaw_deg * 10.0)), int(round(pitch_deg * 10.0))
        )
        response = self.send_command(CMD_SET_ATTITUDE, payload, expect_ack=True)
        if len(response) < 4:
            raise SiyiProtocolError("set-attitude ACK payload is too short")
        current_yaw, current_pitch = struct.unpack_from("<hh", response, 0)
        return current_yaw / 10.0, current_pitch / 10.0

    def request_attitude(self):
        response = self.send_command(CMD_REQUEST_ATTITUDE, expect_ack=True)
        if len(response) < 6:
            raise SiyiProtocolError("attitude response payload is too short")
        yaw, pitch, roll = struct.unpack_from("<hhh", response, 0)
        return yaw / 10.0, pitch / 10.0, roll / 10.0


class A8MiniGimbalNode:
    def __init__(self):
        self.camera_ip = rospy.get_param("~camera_ip", "192.168.144.25")
        self.camera_port = int(rospy.get_param("~camera_port", 37260))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", 0.6))
        self.command_retries = int(rospy.get_param("~command_retries", 3))
        self.move_wait_sec = float(rospy.get_param("~move_wait_sec", 2.0))
        self.max_settle_sec = float(rospy.get_param("~max_settle_sec", 60.0))
        self.dry_run = bool(rospy.get_param("~dry_run", False))

        if self.command_timeout_sec <= 0.0 or self.command_retries <= 0:
            raise ValueError("command timeout and retry count must be positive")
        if self.move_wait_sec < 0.0 or self.max_settle_sec < 0.0:
            raise ValueError("gimbal timing parameters cannot be negative")
        if not 1 <= self.camera_port <= 65535:
            raise ValueError("camera_port must be in the range 1..65535")

        self.completed_tasks = set()
        self.pending_tasks = set()
        self.task_lock = threading.Lock()
        self.task_queue = queue.Queue()
        self.client = None
        if not self.dry_run:
            self.client = SiyiUdpClient(
                self.camera_ip,
                self.camera_port,
                self.command_timeout_sec,
                self.command_retries,
            )

        self.done_pub = rospy.Publisher("/mission/gimbal_done", UInt32, queue_size=10)
        self.task_sub = rospy.Subscriber(
            "/mission/gimbal_task",
            Float64MultiArray,
            self.gimbal_task_callback,
            queue_size=10,
        )
        self.worker = threading.Thread(target=self._worker_loop, name="a8mini-gimbal-worker")
        self.worker.daemon = True
        self.worker.start()
        rospy.on_shutdown(self.shutdown)

        if self.dry_run:
            rospy.logwarn("A8 mini gimbal node is using the dry-run backend")
        else:
            rospy.loginfo(
                "A8 mini Ethernet SDK target is udp://%s:%d",
                self.camera_ip,
                self.camera_port,
            )

    @staticmethod
    def _parse_task(msg):
        if len(msg.data) != 4:
            raise ValueError("gimbal_task must contain exactly four values")
        if not all(math.isfinite(value) for value in msg.data):
            raise ValueError("gimbal_task values must be finite")

        raw_id, yaw_deg, pitch_deg, settle_sec = msg.data
        waypoint_id = int(raw_id)
        if raw_id != waypoint_id or not 1 <= waypoint_id <= 0xFFFFFFFF:
            raise ValueError("waypoint_id must be a positive UInt32 integer")
        if not -135.0 <= yaw_deg <= 135.0:
            raise ValueError("yaw is outside the A8 mini range [-135, 135]")
        if not -90.0 <= pitch_deg <= 25.0:
            raise ValueError("pitch is outside the A8 mini range [-90, 25]")
        return waypoint_id, float(yaw_deg), float(pitch_deg), float(settle_sec)

    def gimbal_task_callback(self, msg):
        try:
            task = self._parse_task(msg)
        except ValueError as error:
            rospy.logerr_throttle(5.0, "Rejected gimbal task: {}".format(error))
            return

        waypoint_id, _, _, settle_sec = task
        if settle_sec < 0.0 or settle_sec > self.max_settle_sec:
            rospy.logerr_throttle(
                5.0,
                "Rejected waypoint {}: settle_sec must be in [0, {}]".format(
                    waypoint_id, self.max_settle_sec
                ),
            )
            return

        with self.task_lock:
            if waypoint_id in self.completed_tasks:
                self.publish_done(waypoint_id)
                return
            if waypoint_id in self.pending_tasks:
                return
            self.pending_tasks.add(waypoint_id)
        self.task_queue.put(task)

    def _worker_loop(self):
        while not rospy.is_shutdown():
            try:
                task = self.task_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            waypoint_id, yaw_deg, pitch_deg, settle_sec = task
            succeeded = False
            try:
                rospy.loginfo(
                    "Executing waypoint %d gimbal task: yaw %.1f, pitch %.1f, settle %.2f s",
                    waypoint_id,
                    yaw_deg,
                    pitch_deg,
                    settle_sec,
                )
                current_angles = self.set_gimbal_angle(yaw_deg, pitch_deg)
                if current_angles is not None:
                    rospy.loginfo(
                        "A8 mini accepted target; ACK attitude yaw %.1f, pitch %.1f",
                        current_angles[0],
                        current_angles[1],
                    )

                rospy.sleep(self.move_wait_sec)
                rospy.sleep(settle_sec)
                if rospy.is_shutdown():
                    return

                stable_time = rospy.Time.now().to_sec()
                rospy.loginfo(
                    "Waypoint %d gimbal stable, ros_time=%.3f", waypoint_id, stable_time
                )
                with self.task_lock:
                    self.completed_tasks.add(waypoint_id)
                self.publish_done(waypoint_id)
                succeeded = True
            except (OSError, SiyiProtocolError, rospy.ROSInterruptException) as error:
                rospy.logerr("Waypoint %d gimbal task failed: %s", waypoint_id, error)
            finally:
                with self.task_lock:
                    self.pending_tasks.discard(waypoint_id)
                self.task_queue.task_done()
                if not succeeded and not rospy.is_shutdown():
                    rospy.logwarn(
                        "Waypoint %d remains incomplete; multipointplan may retry it",
                        waypoint_id,
                    )

    def set_gimbal_angle(self, yaw_deg, pitch_deg):
        if self.dry_run:
            rospy.loginfo(
                "Dry run: lock mode, yaw %.1f, pitch %.1f", yaw_deg, pitch_deg
            )
            return None

        self.client.set_lock_mode()
        # Leave a short gap because the lock-mode command has no protocol ACK.
        time.sleep(0.05)
        return self.client.set_attitude(yaw_deg, pitch_deg)

    def publish_done(self, waypoint_id):
        self.done_pub.publish(UInt32(data=waypoint_id))

    def shutdown(self):
        if self.client is not None:
            self.client.close()


def main():
    rospy.init_node("a8mini_gimbal")
    try:
        A8MiniGimbalNode()
    except (ValueError, OSError) as error:
        rospy.logfatal("Failed to start A8 mini gimbal node: %s", error)
        raise SystemExit(1)
    rospy.spin()


if __name__ == "__main__":
    main()
