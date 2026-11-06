#!/usr/bin/env python3
"""Correlated remote waypoint actions. No flight commands or ROS dependencies.

Yaw is right-positive and pitch up-positive, as in the ground station. SIYI
0x0D reports the opposite yaw sign. Speed is a 1..100 SDK level, not deg/s.
The motor is stopped before every potentially blocking attitude query.
"""
from collections import OrderedDict
from contextlib import nullcontext
import math
import threading
import time


class ActionError(RuntimeError):
    pass


FIELDS = ('map_session_id', 'mission_id', 'action_id', 'executor_id', 'waypoint_id', 'mode',
          'yaw_deg', 'pitch_deg', 'yaw_min_deg', 'yaw_max_deg', 'speed',
          'settle_sec', 'photo')


def validate_task(task):
    for key in ('map_session_id', 'mission_id', 'action_id', 'executor_id'):
        if not isinstance(task.get(key), str) or not 1 <= len(task[key]) <= 80:
            raise ValueError('invalid ' + key)
    if type(task.get('waypoint_id')) is not int or not 1 <= task['waypoint_id'] <= 0xffffffff:
        raise ValueError('invalid waypoint_id')
    if task.get('mode') not in ('angle', 'sweep'):
        raise ValueError('invalid mode')
    for key in ('yaw_deg', 'pitch_deg', 'yaw_min_deg', 'yaw_max_deg', 'settle_sec', 'lease_sec'):
        if type(task.get(key)) not in (int, float) or not math.isfinite(task[key]):
            raise ValueError('non-finite ' + key)
    if any(not -135 <= task[key] <= 135 for key in ('yaw_deg', 'yaw_min_deg', 'yaw_max_deg')):
        raise ValueError('yaw outside camera range')
    if not -90 <= task['pitch_deg'] <= 25 or not 0 <= task['settle_sec'] <= 10:
        raise ValueError('pitch or settle outside camera range')
    if task['yaw_min_deg'] >= task['yaw_max_deg']:
        raise ValueError('invalid sweep endpoints')
    if type(task.get('speed')) is not int or not 1 <= task['speed'] <= 100:
        raise ValueError('speed must be an integer SDK level 1..100')
    if type(task.get('photo')) is not bool or not 2 <= task['lease_sec'] <= 6:
        raise ValueError('invalid photo or lease_sec')


class RemoteGimbalExecutor:
    def __init__(self, client, publish, shutdown=lambda: False, dry_run=False,
                 clock=time.monotonic, sleep=time.sleep, timeout=60.):
        self.client, self.publish, self.shutdown = client, publish, shutdown
        self.dry_run, self.clock, self.sleep, self.timeout = dry_run, clock, sleep, timeout
        self.condition = threading.Condition()
        self.pending = None
        self.current = None
        self.cached = OrderedDict()
        self.cancelled = OrderedDict()
        self.closed = False
        self.deadline = 0.
        self.thread = threading.Thread(target=self._loop, name='remote-gimbal-worker', daemon=True)
        self.thread.start()

    @staticmethod
    def identity(task):
        return (task['map_session_id'], task['mission_id'], task['action_id'], task['waypoint_id'])

    @staticmethod
    def signature(task):
        return tuple(task[key] for key in FIELDS)

    def result(self, task, phase, reason='', terminal=False, success=False):
        result = {key: task[key] for key in ('map_session_id', 'mission_id', 'action_id', 'waypoint_id')}
        result.update(phase=phase, reason=reason[:300], terminal=terminal, success=success)
        return result

    def submit(self, task):
        validate_task(task)
        task = dict(task)
        key = self.identity(task)
        with self.condition:
            if key in self.cached:
                signature, result = self.cached[key]
                if self.signature(task) != signature:
                    self.publish(self.result(task, 'failed', 'action identity reused with different parameters', True))
                else:
                    self.publish(dict(result))
                return
            if task['action_id'] in self.cancelled:
                self.publish(self.result(task, 'failed', 'action already cancelled', True))
                return
            existing = self.current or self.pending
            if existing is not None:
                if self.identity(existing) == key and self.signature(existing) == self.signature(task):
                    self.deadline = self.clock() + task['lease_sec']
                    return
                self.publish(self.result(task, 'failed', 'gimbal executor busy; action not queued', True))
                return
            self.deadline = self.clock() + task['lease_sec']
            self.pending = task
            self.condition.notify_all()

    def busy(self):
        with self.condition:
            return self.current is not None or self.pending is not None

    def cancel(self, action_id):
        with self.condition:
            self.cancelled[action_id] = None
            while len(self.cancelled) > 256:
                self.cancelled.popitem(last=False)
            self.condition.notify_all()

    def check(self, task, started):
        if self.closed or self.shutdown() or task['action_id'] in self.cancelled:
            raise ActionError('action cancelled')
        now = self.clock()
        if now > self.deadline:
            raise ActionError('mission executor lease expired')
        if now - started > self.timeout:
            raise ActionError('gimbal motion timeout')

    def wait(self, duration, task, started):
        until = self.clock() + duration
        while self.clock() < until:
            self.check(task, started)
            remaining = until - self.clock()
            if remaining <= 0:
                break
            self.sleep(min(.02, remaining))
        self.check(task, started)

    def stop(self):
        if not self.dry_run:
            self.client.set_rotate_rate(0, 0)

    @staticmethod
    def rate(error, cap):
        if abs(error) <= 1.:
            return 0
        # Actual A8 mini needed level10 to overcome near-target static friction;
        # a smaller user cap remains authoritative and may time out mechanically.
        return (1 if error > 0 else -1) * min(cap, max(10, int(abs(error) * 2.)))

    def move(self, yaw, pitch, task, started):
        if self.dry_run:
            self.wait(.35, task, started)
            return
        stable = 0
        while True:
            self.check(task, started)
            # Query only with zero motor rates: a lost response cannot prolong motion.
            self.stop()
            raw_yaw, current_pitch, _ = self.client.request_attitude()
            current_yaw = -raw_yaw
            if not -145 <= current_yaw <= 145 or not -100 <= current_pitch <= 35:
                raise ActionError('camera attitude outside expected range; check mounting/orientation')
            dy, dp = yaw - current_yaw, pitch - current_pitch
            self.check(task, started)
            if abs(dy) <= 1. and abs(dp) <= 1.:
                stable += 1
                if stable >= 3:
                    return
                self.wait(.08, task, started)
                continue
            stable = 0
            self.client.set_rotate_rate(self.rate(dy, task['speed']), self.rate(dp, task['speed']))
            try:
                self.wait(.08, task, started)
            finally:
                self.stop()

    def execute(self, task):
        # Recording query/toggle/verify and legacy actions use the same RLock.
        # Hold ownership through the final stop, so a recorder cannot postpone
        # the zero-rate frame while the gimbal is moving.
        with (self.client.lock if not self.dry_run else nullcontext()):
            try:
                return self._execute_owned(task)
            finally:
                self.stop()

    def _execute_owned(self, task):
        started = self.clock()
        self.check(task, started)
        self.publish(self.result(task, 'moving'))
        if not self.dry_run:
            self.client.set_lock_mode()
            self.wait(.05, task, started)
        poses = [task['yaw_deg']] if task['mode'] == 'angle' else [task['yaw_min_deg'], task['yaw_max_deg']]
        for yaw in poses:
            self.move(yaw, task['pitch_deg'], task, started)
        self.stop()
        self.publish(self.result(task, 'settling'))
        self.wait(task['settle_sec'], task, started)
        # A correction after the settling window would require a new full settle.
        # Instead verify read-only and fail without releasing the shutter on drift.
        if not self.dry_run:
            self.check(task, started)
            raw_yaw, pitch, _ = self.client.request_attitude()
            self.check(task, started)
            if abs(poses[-1] + raw_yaw) > 1. or abs(task['pitch_deg'] - pitch) > 1.:
                raise ActionError('gimbal drifted during settle; photo not sent')
        if task['photo']:
            self.publish(self.result(task, 'photo_sent', 'single photo at final pose; awaiting camera feedback'))
            self.check(task, started)
            if not self.dry_run:
                self.client.take_photo_feedback(cancelled=lambda: self.closed or self.shutdown() or task['action_id'] in self.cancelled)
            self.check(task, started)
        return self.result(task, 'complete', 'dry-run simulated completion' if self.dry_run else
                           ('camera photo feedback succeeded' if task['photo'] else 'gimbal target confirmed'), True, True)

    def _loop(self):
        while not self.closed and not self.shutdown():
            with self.condition:
                while self.pending is None and not self.closed and not self.shutdown():
                    self.condition.wait(.2)
                if self.closed or self.shutdown():
                    return
                task, self.pending = self.pending, None
                self.current = task
            try:
                result = self.execute(task)
            except Exception as error:
                result = self.result(task, 'failed', str(error), True)
            with self.condition:
                self.cached[self.identity(task)] = (self.signature(task), result)
                while len(self.cached) > 256:
                    self.cached.popitem(last=False)
                self.current = None
            self.publish(result)

    def close(self):
        self.closed = True
        with self.condition:
            self.condition.notify_all()
        self.thread.join(timeout=3.)
        self.stop()
