#!/usr/bin/env python3

import importlib.util
import pathlib
import sys
import threading
import types
import unittest


class _Message:
    def __init__(self, data=None):
        self.data = data


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
    std_msgs_msg.Bool = _Message
    std_msgs_msg.Float64MultiArray = _Message
    std_msgs_msg.UInt32 = _Message
    std_msgs.msg = std_msgs_msg
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)


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
