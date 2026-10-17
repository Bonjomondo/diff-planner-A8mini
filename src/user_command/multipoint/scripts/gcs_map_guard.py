"""Same-session identity-frame verification, without ROS dependencies.

This release supports the existing zero-offset LIO -> EKF configuration only.
It rejects mismatched frames instead of estimating a transform from one pose.
"""
from collections import deque
import math
import time


def valid_bounds(bounds):
    return (isinstance(bounds, (list, tuple)) and len(bounds) == 6
            and all(type(v) in (int, float) and math.isfinite(v) for v in bounds)
            and all(bounds[i] < bounds[i + 3] for i in range(3)))


def inside(point, bounds):
    return valid_bounds(bounds) and all(bounds[i] < point[i] < bounds[i + 3] for i in range(3))


class MapGuard:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.map = None
        self.planner = deque(maxlen=100)
        self.last_pair = None
        self.good = 0
        self.last_good = 0
        self.sequences = {}

    def observe(self, source, stamp, frame, position, orientation, sequence=None):
        if sequence is not None:
            if source in self.sequences and sequence < self.sequences[source]:
                self.good = 0
                self.last_pair = None
                self.planner.clear()
                self.map = None
            self.sequences[source] = sequence
        values = [stamp] + list(position) + list(orientation)
        if (len(position) != 3 or len(orientation) != 4 or not all(math.isfinite(v) for v in values)
                or stamp <= 0 or frame != 'world'):
            self.good = 0
            return
        norm = math.sqrt(sum(v * v for v in orientation))
        if norm < 1e-8:
            self.good = 0
            return
        sample = (stamp, tuple(position), tuple(v / norm for v in orientation), self.clock())
        if source == 'map':
            if self.map and stamp <= self.map[0]:
                self.good = 0
                self.last_pair = None
            self.map = sample
        else:
            if self.planner and stamp < self.planner[-1][0]:
                self.good = 0
                self.planner.clear()
            self.planner.append(sample)
        if not self.map or not self.planner:
            return
        candidate = min(self.planner, key=lambda p: abs(p[0] - self.map[0]))
        if abs(candidate[0] - self.map[0]) > .05 or self.map[0] == self.last_pair:
            return
        self.last_pair = self.map[0]
        distance = sum((a - b) ** 2 for a, b in zip(candidate[1], self.map[1])) ** .5
        cosine = min(1., abs(sum(a * b for a, b in zip(candidate[2], self.map[2]))))
        if distance > .15 or 2 * math.acos(cosine) > math.radians(5):
            self.good = 0
        else:
            self.good += 1
            self.last_good = self.clock()

    def snapshot(self, ros_now):
        stamp = self.map[0] if self.map else 0.
        valid = (self.good >= 10 and self.clock() - self.last_good < .5
                 and self.map is not None and self.clock() - self.map[3] < .5
                 and -.05 <= ros_now - stamp < .5)
        return dict(map_coordinates_valid=bool(valid), map_odom_stamp=stamp)
