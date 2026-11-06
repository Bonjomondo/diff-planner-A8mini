#!/usr/bin/env python3
"""Real closed-loop and action-dedup logic against a deterministic camera model."""
from collections import OrderedDict
from pathlib import Path
import sys
import threading
import time
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from remote_gimbal import ActionError, RemoteGimbalExecutor, validate_task


def task(**changes):
    result = dict(map_session_id='map', mission_id='route', action_id='action', executor_id='executor', waypoint_id=1,
                  mode='angle', yaw_deg=15., pitch_deg=-20., yaw_min_deg=-15., yaw_max_deg=15.,
                  speed=12, settle_sec=.1, photo=True, lease_sec=5.)
    result.update(changes)
    return result


class ClockCamera:
    def __init__(self):
        self.now, self.yaw, self.pitch, self.rates = 0., -10., 10., (0, 0)
        self.commands, self.queries, self.photos = [], [], 0
        self.locked = False
        self.lock = threading.RLock()
        self.on_sleep = lambda: None

    def set_lock_mode(self):
        self.locked = True

    def request_attitude(self):
        self.queries.append(self.rates)
        return -self.yaw, self.pitch, 0.

    def set_rotate_rate(self, yaw, pitch):
        self.rates = (yaw, pitch)
        self.commands.append(self.rates)

    def sleep(self, duration):
        self.now += duration
        self.yaw += self.rates[0] * duration * 2
        self.pitch += self.rates[1] * duration * 2
        self.on_sleep()

    def take_photo_feedback(self, cancelled):
        if cancelled():
            raise ActionError('cancelled')
        self.photos += 1


def executor(camera, output):
    node = RemoteGimbalExecutor.__new__(RemoteGimbalExecutor)
    node.client, node.publish, node.shutdown = camera, output.append, lambda: False
    node.dry_run, node.clock, node.sleep, node.timeout = False, lambda: camera.now, camera.sleep, 60.
    node.condition, node.pending, node.current = threading.Condition(), None, None
    node.cached, node.cancelled, node.closed = OrderedDict(), OrderedDict(), False
    node.deadline = 500.
    return node


class MotionTest(unittest.TestCase):
    def test_feedback_converges_both_axes_in_normalized_signs_and_caps_speed(self):
        camera, output = ClockCamera(), []
        node = executor(camera, output)
        result = node.execute(task())
        self.assertTrue(result['success'])
        self.assertAlmostEqual(camera.yaw, 15., delta=1.)
        self.assertAlmostEqual(camera.pitch, -20., delta=1.)
        self.assertTrue(camera.locked)
        self.assertEqual(camera.photos, 1)
        self.assertEqual(camera.rates, (0, 0))
        self.assertTrue(all(query == (0, 0) for query in camera.queries))
        self.assertTrue(all(abs(rate) <= 12 for pair in camera.commands for rate in pair))
        self.assertTrue(any(y > 0 and p < 0 for y, p in camera.commands))
        self.assertEqual([x['phase'] for x in output], ['moving', 'settling', 'photo_sent'])

    def test_drift_during_settle_fails_without_correction_or_shutter(self):
        camera, output = ClockCamera(), []
        node = executor(camera, output)
        at_settle = []
        def drift():
            if output and output[-1]['phase'] == 'settling' and not at_settle:
                at_settle.append(len(camera.commands))
                camera.yaw += 4
        camera.on_sleep = drift
        with self.assertRaisesRegex(ActionError, 'drifted during settle'):
            node.execute(task())
        self.assertEqual(camera.photos, 0)
        self.assertTrue(at_settle)
        self.assertTrue(all(pair == (0, 0) for pair in camera.commands[at_settle[0]:]))

    def test_recording_contender_cannot_interleave_before_final_stop(self):
        camera, output = ClockCamera(), []
        node = executor(camera, output)
        attempted = threading.Event()
        finished = threading.Event()
        recorder_started = []
        contender = [None]
        def recorder():
            attempted.set()
            with camera.lock:
                recorder_started.append((camera.rates, output[-1]['phase']))
            finished.set()
        def pulse():
            if camera.rates != (0, 0) and contender[0] is None:
                contender[0] = threading.Thread(target=recorder)
                contender[0].start()
                self.assertTrue(attempted.wait(1))
                self.assertFalse(finished.is_set())
        camera.on_sleep = pulse
        result = node.execute(task())
        contender[0].join(1)
        self.assertTrue(result['success'])
        self.assertTrue(finished.is_set())
        self.assertEqual(recorder_started, [((0, 0), 'photo_sent')])

    def test_sweep_reaches_start_then_end_and_takes_only_one_photo(self):
        camera, output = ClockCamera(), []
        camera.yaw = 0
        node = executor(camera, output)
        node.execute(task(mode='sweep'))
        self.assertTrue(any(y < 0 for y, _ in camera.commands))
        self.assertTrue(any(y > 0 for y, _ in camera.commands))
        self.assertAlmostEqual(camera.yaw, 15., delta=1.)
        self.assertEqual(camera.photos, 1)

    def test_cancellation_during_pulse_stops_and_does_not_shoot(self):
        camera, output = ClockCamera(), []
        node = executor(camera, output)
        camera.on_sleep = lambda: node.cancel('action') if camera.rates != (0, 0) else None
        with self.assertRaisesRegex(ActionError, 'cancelled'):
            node.execute(task())
        self.assertEqual(camera.rates, (0, 0))
        self.assertEqual(camera.photos, 0)

    def test_expired_lease_and_unknown_photo_do_not_complete(self):
        camera, output = ClockCamera(), []
        node = executor(camera, output)
        node.deadline = -1
        with self.assertRaisesRegex(ActionError, 'lease expired'):
            node.execute(task())
        self.assertEqual(camera.photos, 0)
        node.deadline = 500
        def failed(**kwargs):
            camera.photos += 1
            raise ActionError('photo result unknown')
        camera.take_photo_feedback = failed
        with self.assertRaisesRegex(ActionError, 'unknown'):
            node.execute(task())
        self.assertEqual(camera.photos, 1)
        self.assertFalse(any(x['success'] for x in output))

    def test_task_validation_including_nonfinite_and_speed_type(self):
        for changes in (dict(speed=0), dict(speed=True), dict(speed=101), dict(pitch_deg=26),
                        dict(yaw_deg=float('nan')), dict(settle_sec=11), dict(photo=1),
                        dict(mode='range'), dict(yaw_min_deg=60), dict(lease_sec=0)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_task(task(**changes))


class CorrelationTest(unittest.TestCase):
    def test_retry_replays_cached_result_but_new_mission_reuses_waypoint(self):
        output = []
        node = RemoteGimbalExecutor(None, output.append, dry_run=True)
        self.addCleanup(node.close)
        action = task(settle_sec=0)
        node.submit(action)
        until = time.monotonic() + 3
        while not any(x['terminal'] for x in output) and time.monotonic() < until:
            time.sleep(.01)
        self.assertTrue(output[-1]['success'])
        phases = len(output)
        node.submit(action)
        self.assertEqual(len(output), phases + 1)
        self.assertTrue(output[-1]['terminal'])
        node.submit(dict(action, speed=11))
        self.assertFalse(output[-1]['success'])
        self.assertIn('reused', output[-1]['reason'])
        node.submit(dict(action, action_id='next-action', mission_id='next-route'))
        until = time.monotonic() + 3
        while not any(x['terminal'] and x['action_id'] == 'next-action' for x in output) and time.monotonic() < until:
            time.sleep(.01)
        self.assertTrue(output[-1]['success'])
        self.assertEqual(output[-1]['mission_id'], 'next-route')

    def test_cancelled_id_cannot_be_started_by_delayed_request(self):
        output = []
        node = RemoteGimbalExecutor(None, output.append, dry_run=True)
        self.addCleanup(node.close)
        node.cancel('action')
        node.submit(task())
        self.assertFalse(output[-1]['success'])
        self.assertIn('cancelled', output[-1]['reason'])


if __name__ == '__main__':
    unittest.main()
