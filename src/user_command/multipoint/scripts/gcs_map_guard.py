"""Same-session identity-frame verification, without ROS dependencies.

This release supports the existing zero-offset LIO -> EKF configuration only.
It rejects mismatched frames instead of estimating a transform from one pose.
"""
from collections import deque
import math
import numbers
import time

# 两条流允许的最大配对时间差（秒）；沿用既有门限。
PAIR_TOLERANCE = .05
# 位置 / 姿态门限与连续合格对数，同样沿用既有值。
DISTANCE_TOLERANCE = .15
ANGLE_TOLERANCE = math.radians(5)
# 判定阈值：连续合格对数低于它即判无效。
MIN_GOOD_PAIRS = 10
# 失败时的衰减方式：从"清零"改为"减半"（漏桶）。孤立的一帧毛刺只会让计数
# 掉一半（长途飞行时计数接近上限，仍远高于阈值）；持续失真则每帧减半，
# 约 log2(计数/阈值) 帧内跌到阈值以下（10 Hz 下通常 < 1 s）。
FRESH_SECONDS = .5
# 计数上限：只用于避免整数无限增长，不影响判定（阈值是 MIN_GOOD_PAIRS）。
MAX_GOOD_PAIRS = 1000

_REAL = numbers.Real


def valid_bounds(bounds):
    return (isinstance(bounds, (list, tuple)) and len(bounds) == 6
            and all(isinstance(v, _REAL) and not isinstance(v, bool) and math.isfinite(v)
                    for v in bounds)
            and all(bounds[i] < bounds[i + 3] for i in range(3)))


def inside(point, bounds):
    return valid_bounds(bounds) and all(bounds[i] < point[i] < bounds[i + 3] for i in range(3))


class MapGuard:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.map = None
        self.planner = deque(maxlen=100)
        self.pending = deque(maxlen=16)
        self.good = 0
        self.last_good = 0
        # 最近一次拒绝的实测量值，便于现场判断差多少（类别仍是 last_reject）。
        self.last_reject_detail = ''
        # 最近一次把连续计数清零的原因（成功配对不覆盖），用于定位周期性掉闸。
        self.last_reset = ''
        self.last_reject = "none"
        self.sequences = {}

    def observe(self, source, stamp, frame, position, orientation, sequence=None):
        if sequence is not None:
            if source in self.sequences and sequence < self.sequences[source]:
                self.good = 0
                self.pending.clear()
                self.planner.clear()
                self.map = None
                self.last_reject = "restart"
                self.last_reset = "restart:sequence"
            self.sequences[source] = sequence
        values = [stamp] + list(position) + list(orientation)
        if (len(position) != 3 or len(orientation) != 4
                or not all(math.isfinite(v) for v in values) or stamp <= 0):
            self.good = 0
            self.last_reject = "nonfinite"
            self.last_reset = "nonfinite"
            return
        if frame != 'world':
            self.good = 0
            self.last_reject = "frame"
            self.last_reset = "frame"
            return
        norm = math.sqrt(sum(v * v for v in orientation))
        if norm < 1e-8:
            self.good = 0
            self.last_reject = "quaternion"
            self.last_reset = "quaternion"
            return
        sample = (stamp, tuple(position), tuple(v / norm for v in orientation), self.clock())
        if source == 'map':
            if self.map is not None and stamp < self.map[0]:
                self.good = 0
                self.pending.clear()
                self.last_reject = "restart"
                self.last_reset = "restart:map_stamp"
            elif self.map is not None and stamp == self.map[0]:
                # 同一帧重复发布：既不是新场次也不是倒退，忽略即可，
                # 不能像旧实现那样把连续计数清零。
                return
            self.map = sample
            self.pending.append(sample)
        else:
            if self.planner and stamp < self.planner[-1][0]:
                self.good = 0
                self.planner.clear()
                self.pending.clear()
                self.last_reject = "restart"
                self.last_reset = "restart:planner_stamp"
            self.planner.append(sample)
        self._flush()

    def _flush(self):
        """判定时机：等 EKF 流越过该 LIO 时间戳后再配对一次。

        时间最近的那个 EKF 样本，只可能在该时间戳之前或之后；一旦看到
        不早于它的 EKF 样本，前后两个候选就都在缓存里了，"最近的一对"
        此时才唯一确定。旧实现收到 LIO 就立刻拿当时缓存里最新的 EKF 配对，
        并把这帧 LIO 记为已配对；更贴近时间戳的 EKF 稍后到达也不会重新
        校验，高速转向时姿态差被放大（2026-11-05 实际飞行把计数从 296
        清零，map_ready 掉线 1.2 s，任务因此中止）。
        """
        while self.pending and self.planner and self.planner[-1][0] >= self.pending[0][0]:
            self._evaluate(self.pending.popleft())

    def _evaluate(self, map_sample):
        candidate = min(self.planner, key=lambda p: abs(p[0] - map_sample[0]))
        if abs(candidate[0] - map_sample[0]) > PAIR_TOLERANCE:
            self.last_reject = "no_counterpart"
            return                      # 没有可配对样本：不改变计数，也不判失败
        distance = sum((a - b) ** 2 for a, b in zip(candidate[1], map_sample[1])) ** .5
        cosine = min(1., abs(sum(a * b for a, b in zip(candidate[2], map_sample[2]))))
        angle = 2 * math.acos(cosine)
        if distance > DISTANCE_TOLERANCE:
            self.good //= 2
            self.last_reject = "distance"
            self.last_reject_detail = "%.3f m (门限 %.2f)" % (distance, DISTANCE_TOLERANCE)
        elif angle > ANGLE_TOLERANCE:
            self.good //= 2
            self.last_reject = "angle"
            self.last_reject_detail = "%.1f° (门限 %.1f°)" % (
                math.degrees(angle), math.degrees(ANGLE_TOLERANCE))
        else:
            self.good = min(self.good + 1, MAX_GOOD_PAIRS)
            self.last_good = self.clock()
            self.last_reject = "none"
            self.last_reject_detail = ""

    def _invalid_reason(self, ros_now, stamp):
        if self.map is None:
            return "no_map"
        if self.clock() - self.map[3] >= FRESH_SECONDS:
            return "map_not_arriving"
        age = ros_now - stamp
        if not math.isfinite(age):
            return "map_stamp_invalid"
        if age < -PAIR_TOLERANCE:
            return "map_stamp_ahead"
        if age >= FRESH_SECONDS:
            return "map_stamp_late"
        if self.good < MIN_GOOD_PAIRS:
            return "pairs_below_threshold"
        if self.clock() - self.last_good >= FRESH_SECONDS:
            return "pairs_stale"
        return ""

    def snapshot(self, ros_now):
        stamp = self.map[0] if self.map else 0.
        reason = self._invalid_reason(ros_now, stamp)
        return dict(map_coordinates_valid=not reason, map_odom_stamp=stamp,
                    map_reason=reason, map_last_reject=self.last_reject,
                    map_last_reject_detail=self.last_reject_detail,
                    map_last_reset=self.last_reset,
                    map_good=self.good,
                    map_pending=len(self.pending),
                    map_planner=len(self.planner),
                    map_good_age=round(self.clock() - self.last_good, 3) if self.last_good else -1.)
