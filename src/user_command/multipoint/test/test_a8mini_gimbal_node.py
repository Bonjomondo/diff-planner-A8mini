#!/usr/bin/env python3

import importlib.util
import pathlib
import sys
import threading
import types
import unittest
from unittest.mock import Mock, patch


class _Message:
    def __init__(self, data=None, **fields):
        self.data = data
        self.__dict__.update(fields)


class _ExtendedState:
    LANDED_STATE_UNDEFINED = 0
    LANDED_STATE_ON_GROUND = 1
    LANDED_STATE_IN_AIR = 2
    LANDED_STATE_TAKEOFF = 3
    LANDED_STATE_LANDING = 4


def _install_ros_stubs():
    rospy = types.ModuleType("rospy")
    for name in ("loginfo", "logwarn", "logerr", "logfatal", "logwarn_throttle"):
        setattr(rospy, name, lambda *args, **kwargs: None)
    sys.modules.setdefault("rospy", rospy)

    mavros_msgs = types.ModuleType("mavros_msgs")
    mavros_msgs_msg = types.ModuleType("mavros_msgs.msg")
    mavros_msgs_msg.ExtendedState = _ExtendedState
    mavros_msgs.msg = mavros_msgs_msg
    sys.modules.setdefault("mavros_msgs", mavros_msgs)
    sys.modules.setdefault("mavros_msgs.msg", mavros_msgs_msg)

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = _Message
    std_msgs_msg.Bool = _Message
    std_msgs_msg.Float64MultiArray = _Message
    std_msgs_msg.UInt32 = _Message
    std_msgs.msg = std_msgs_msg
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

    multipoint = types.ModuleType('multipoint')
    messages = types.ModuleType('multipoint.msg')
    messages.GimbalTask = messages.GimbalResult = _Message
    multipoint.msg = messages
    sys.modules.setdefault('multipoint', multipoint)
    sys.modules.setdefault('multipoint.msg', messages)


_install_ros_stubs()
SCRIPT_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "a8mini_gimbal_node.py"
SPEC = importlib.util.spec_from_file_location("a8mini_gimbal_node", str(SCRIPT_PATH))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeSiyiClient(MODULE.SiyiUdpClient):
    def __init__(self, statuses):
        self.lock = threading.RLock()
        self.statuses = list(statuses)
        self.commands = []

    def send_command(self, command_id, payload=b"", expect_ack=True):
        self.commands.append((command_id, payload, expect_ack))
        if command_id == MODULE.CMD_REQUEST_SYSTEM_INFO:
            if len(self.statuses) > 1:
                status = self.statuses.pop(0)
            else:
                status = self.statuses[0]
            return bytes((0, 0, 0, status, 0, 0, 0, 0))
        return b""


class SiyiRecordingTest(unittest.TestCase):
    def test_concatenated_udp_ack_attitude_and_feedback_frames(self):
        import struct
        ack = MODULE.encode_packet(1, MODULE.CMD_ROTATE_RATE, bytes((1,)))
        attitude = MODULE.encode_packet(2, MODULE.CMD_REQUEST_ATTITUDE,
            struct.pack('<hhhhhh', 1163, 0, 0, 0, 0, 0))
        photo = MODULE.encode_packet(3, MODULE.CMD_CAMERA_FEEDBACK, bytes((0,)))
        frame = ack + ack + attitude
        self.assertEqual(len(frame), 44)  # Observed from the actual A8 mini.
        self.assertEqual([p['command_id'] for p in MODULE.decode_packets(frame)], [7, 7, 13])
        damaged = bytearray(ack); damaged[-1] ^= 1
        recovered = list(MODULE.decode_packets(bytes(damaged) + attitude + photo))
        self.assertEqual([p['command_id'] for p in recovered], [13, 11])
        client = MODULE.SiyiUdpClient.__new__(MODULE.SiyiUdpClient)
        client.sequence, client.retries, client.timeout_sec = 0, 1, .1
        client.camera_address = ('127.0.0.1', 37260)
        client.lock, client.socket = threading.RLock(), Mock()
        client._drain_receive_buffer = Mock()
        client.socket.recvfrom.return_value = (frame, client.camera_address)
        self.assertEqual(client.request_attitude(), (116.3, 0., 0.))
        client.socket.recvfrom.return_value = (ack + photo + attitude, client.camera_address)
        client.take_photo_feedback()

    def test_generation_mismatch_rejects_delayed_task_without_motion(self):
        node = MODULE.A8MiniGimbalNode.__new__(MODULE.A8MiniGimbalNode)
        node.remote_instance_id = 'new-generation'
        node.remote_executor = Mock()
        node.remote_executor.result.return_value = dict(phase='failed', success=False)
        node.remote_result_pub = Mock()
        fields = {key: 0 for key in MODULE.FIELDS + ('lease_sec',)}
        fields['executor_id'] = 'old-generation'
        node.remote_task_callback(_Message(**fields))
        node.remote_executor.submit.assert_not_called()
        self.assertTrue(node.remote_result_pub.publish.called)

    def test_photo_feedback_success_and_failure_each_send_shutter_once(self):
        import socket
        for info in (0, 1):
            client = MODULE.SiyiUdpClient.__new__(MODULE.SiyiUdpClient)
            client.sequence = 0
            client.camera_address = ('192.168.144.25', 37260)
            client.lock = threading.RLock()
            client.socket = Mock()
            client.socket.recvfrom.return_value = (
                MODULE.encode_packet(42, MODULE.CMD_CAMERA_FEEDBACK, bytes((info,))),
                client.camera_address)
            client._drain_receive_buffer = Mock()
            if info == 0:
                client.take_photo_feedback()
            else:
                with self.assertRaisesRegex(MODULE.SiyiProtocolError, 'failed'):
                    client.take_photo_feedback()
            self.assertEqual(client.socket.sendto.call_count, 1)
            sent = MODULE.decode_packet(client.socket.sendto.call_args[0][0])
            self.assertEqual(sent['command_id'], MODULE.CMD_CAMERA_FUNCTION)
            self.assertEqual(sent['payload'], bytes((0,)))
        client.socket.reset_mock()
        client.socket.recvfrom.side_effect = socket.timeout()
        with self.assertRaisesRegex(MODULE.SiyiProtocolError, 'unknown'):
            client.take_photo_feedback(timeout_sec=.001)
        self.assertEqual(client.socket.sendto.call_count, 1)

    def test_disabled_auto_recording_does_not_subscribe_or_control_recording(self):
        for enabled in (False, True):
            ros = Mock()
            ros.get_param.side_effect = lambda name, default=None: {
                "~enable_auto_recording": enabled, "~dry_run": True,
            }.get(name, default)
            with patch.object(MODULE, "rospy", ros), patch.object(MODULE.threading, "Thread"):
                node = MODULE.A8MiniGimbalNode()
                topics = [call.args[0] for call in ros.Subscriber.call_args_list]
                self.assertEqual("/mavros/extended_state" in topics, enabled)
                self.assertEqual(node.recording_worker is not None, enabled)
                self.assertIsNone(node.recording_desired)

    def test_record_toggle_frame_matches_sdk_example(self):
        packet = MODULE.encode_packet(
            0,
            MODULE.CMD_CAMERA_FUNCTION,
            bytes((MODULE.CAMERA_FUNCTION_RECORD,)),
        )
        self.assertEqual(packet.hex(" "), "55 66 01 01 00 00 00 0c 02 76 ee")

    def test_start_queries_toggles_and_verifies(self):
        client = FakeSiyiClient(
            [
                MODULE.RECORD_STATUS_STOPPED,
                MODULE.RECORD_STATUS_STOPPED,
                MODULE.RECORD_STATUS_RECORDING,
            ]
        )
        status = client.set_recording(True, verify_timeout_sec=0.2, verify_poll_sec=0.0)

        self.assertEqual(status, MODULE.RECORD_STATUS_RECORDING)
        toggles = [
            command
            for command in client.commands
            if command[0] == MODULE.CMD_CAMERA_FUNCTION
        ]
        self.assertEqual(
            toggles,
            [(MODULE.CMD_CAMERA_FUNCTION, bytes((MODULE.CAMERA_FUNCTION_RECORD,)), False)],
        )

    def test_already_recording_does_not_toggle(self):
        client = FakeSiyiClient([MODULE.RECORD_STATUS_RECORDING])

        status = client.set_recording(True, verify_timeout_sec=0.2, verify_poll_sec=0.0)

        self.assertEqual(status, MODULE.RECORD_STATUS_RECORDING)
        self.assertFalse(
            any(command[0] == MODULE.CMD_CAMERA_FUNCTION for command in client.commands)
        )

    def test_stop_toggles_even_when_camera_reports_data_loss(self):
        client = FakeSiyiClient(
            [MODULE.RECORD_STATUS_DATA_LOSS, MODULE.RECORD_STATUS_STOPPED]
        )

        status = client.set_recording(False, verify_timeout_sec=0.2, verify_poll_sec=0.0)

        self.assertEqual(status, MODULE.RECORD_STATUS_STOPPED)
        self.assertEqual(client.commands[1][0], MODULE.CMD_CAMERA_FUNCTION)

    def test_start_without_tf_card_fails_without_toggle(self):
        client = FakeSiyiClient([MODULE.RECORD_STATUS_NO_TF_CARD])

        with self.assertRaisesRegex(MODULE.SiyiProtocolError, "no TF card"):
            client.set_recording(True, verify_timeout_sec=0.2, verify_poll_sec=0.0)

        self.assertFalse(
            any(command[0] == MODULE.CMD_CAMERA_FUNCTION for command in client.commands)
        )

    def test_flight_state_mapping_keeps_recording_during_landing(self):
        self.assertFalse(
            MODULE.recording_desired_for_landed_state(
                _ExtendedState.LANDED_STATE_ON_GROUND
            )
        )
        self.assertTrue(
            MODULE.recording_desired_for_landed_state(
                _ExtendedState.LANDED_STATE_TAKEOFF
            )
        )
        self.assertTrue(
            MODULE.recording_desired_for_landed_state(_ExtendedState.LANDED_STATE_IN_AIR)
        )
        self.assertTrue(
            MODULE.recording_desired_for_landed_state(_ExtendedState.LANDED_STATE_LANDING)
        )


if __name__ == "__main__":
    unittest.main()
