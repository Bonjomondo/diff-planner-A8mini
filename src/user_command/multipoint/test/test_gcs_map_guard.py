import math
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from gcs_map_guard import MapGuard, valid_bounds, inside

class GuardTests(unittest.TestCase):
    def samples(self, shift=0, quat=(0,0,0,1)):
        self.now = 100.
        guard = MapGuard(clock=lambda: self.now)
        for i in range(12):
            stamp = 99.8 + i * .01
            guard.observe('planner', stamp, 'world', (1+shift,2,1), quat)
            guard.observe('map', stamp, 'world', (1,2,1), (0,0,0,1))
        return guard
    def test_same_session_fresh_matching_coordinates_only(self):
        guard = self.samples()
        self.assertTrue(guard.snapshot(100.)['map_coordinates_valid'])
        self.now += 1
        self.assertFalse(guard.snapshot(101.)['map_coordinates_valid'])
        self.assertTrue(self.samples(shift=1).snapshot(100.)['map_coordinates_valid'])
        # 2026-11-08 起姿态差不判失效，只记录数值
        self.assertTrue(self.samples(quat=(0,0,1,0)).snapshot(100.)['map_coordinates_valid'])
    def test_position_threshold_50_cm(self):
        self.assertTrue(self.samples(shift=0.49).snapshot(100.)['map_coordinates_valid'])
        report = self.samples(shift=0.51).snapshot(100.)
        self.assertTrue(report['map_coordinates_valid'])
        self.assertEqual(report['map_last_reject'], 'distance')

    def test_restart_or_bad_frame_revokes_verification(self):
        guard = self.samples()
        guard.observe('map', 99., 'world', (1,2,1), (0,0,0,1))
        self.assertFalse(guard.snapshot(100.)['map_coordinates_valid'])
        guard = self.samples()
        guard.observe('map', 100., 'other', (1,2,1), (0,0,0,1))
        self.assertFalse(guard.snapshot(100.)['map_coordinates_valid'])
    def test_attitude_difference_does_not_invalidate(self):
        """姿态差不再参与判定：30° 偏差仍应视为坐标有效。"""
        half = math.sin(math.radians(15))
        cos = math.cos(math.radians(15))
        guard = self.samples(quat=(0, 0, half, cos))
        report = guard.snapshot(100.)
        self.assertTrue(report['map_coordinates_valid'])
        self.assertGreater(report['map_last_angle'], 29.0)

    def test_fence_strict_bounds(self):
        b = [-2,-2,.3,2,2,2]
        self.assertTrue(valid_bounds(b))
        self.assertTrue(inside((0,0,1), b))
        self.assertFalse(inside((2,0,1), b))
        self.assertFalse(valid_bounds([0]*6))
    def test_publisher_restart_revokes_verification_even_with_forward_clock(self):
        guard = self.samples()
        guard.observe('map', 99.92, 'world', (1,2,1), (0,0,0,1), 100)
        guard.observe('map', 99.93, 'world', (1,2,1), (0,0,0,1), 1)
        self.assertFalse(guard.snapshot(100.)['map_coordinates_valid'])


    def test_late_counterpart_is_used_instead_of_stale_ekf(self):
        """2026-11-05 实飞场景：LIO 帧先到，时间最近的 EKF 样本几毫秒后才到。"""
        now = [100.]
        guard = MapGuard(clock=lambda: now[0])
        for i in range(12):
            stamp = 99.8 + i * .01
            guard.observe('planner', stamp, 'world', (1, 2, 1), (0, 0, 0, 1))
            guard.observe('map', stamp, 'world', (1, 2, 1), (0, 0, 0, 1))
        self.assertTrue(guard.snapshot(100.)['map_coordinates_valid'])

        target = 100.02
        stale = math.radians(6.0)          # 旧实现会用这个样本配对：6° > 5° 门限
        guard.observe('planner', target - .025, 'world', (1, 2, 1),
                      (0, 0, math.sin(stale / 2), math.cos(stale / 2)))
        guard.observe('map', target, 'world', (1, 2, 1), (0, 0, 0, 1))
        close = math.radians(.5)           # 0.5 ms 后到达的真实配对方
        guard.observe('planner', target + .0005, 'world', (1, 2, 1),
                      (0, 0, math.sin(close / 2), math.cos(close / 2)))
        self.assertTrue(guard.snapshot(100.03)['map_coordinates_valid'])

    def test_no_pairing_without_counterpart(self):
        """只有 LIO 流、没有 EKF 时不应凭旧样本判定通过。"""
        guard = MapGuard(clock=lambda: 100.)
        for i in range(12):
            guard.observe('map', 99.8 + i * .01, 'world', (1, 2, 1), (0, 0, 0, 1))
        self.assertFalse(guard.snapshot(100.)['map_coordinates_valid'])



    def test_duplicate_map_stamp_does_not_reset(self):
        """同一 stamp 重复发布不是新场次，不应清零连续计数。"""
        now = [100.]
        guard = MapGuard(clock=lambda: now[0])
        for i in range(12):
            stamp = 99.8 + i * .01
            guard.observe('planner', stamp, 'world', (1, 2, 1), (0, 0, 0, 1))
            guard.observe('map', stamp, 'world', (1, 2, 1), (0, 0, 0, 1))
        self.assertTrue(guard.snapshot(100.)['map_coordinates_valid'])
        before = guard.good
        guard.observe('map', 99.91, 'world', (1, 2, 1), (0, 0, 0, 1))   # 重复
        self.assertEqual(guard.good, before)
        self.assertTrue(guard.snapshot(100.)['map_coordinates_valid'])

    def test_reason_codes_are_reported(self):
        guard = MapGuard(clock=lambda: 100.)
        self.assertEqual(guard.snapshot(100.)['map_reason'], 'no_map')
        guard.observe('map', 99.9, 'other', (1, 2, 1), (0, 0, 0, 1))    # 坏 frame
        self.assertEqual(guard.snapshot(100.)['map_reason'], 'no_map')
        for i in range(3):
            t = 99.8 + i * .01
            guard.observe('planner', t, 'world', (1, 2, 1), (0, 0, 0, 1))
            guard.observe('map', t, 'world', (1, 2, 1), (0, 0, 0, 1))
        self.assertEqual(guard.snapshot(100.01)['map_reason'], 'pairs_below_threshold')

        shifted = MapGuard(clock=lambda: 100.)
        for i in range(12):
            t = 99.8 + i * .01
            shifted.observe('planner', t, 'world', (1.6, 2, 1), (0, 0, 0, 1))
            shifted.observe('map', t, 'world', (1, 2, 1), (0, 0, 0, 1))
        report = shifted.snapshot(100.)
        self.assertTrue(report['map_coordinates_valid'])
        self.assertEqual(report['map_last_reject'], 'distance')

    def test_bounds_accept_numpy_scalars(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest('numpy unavailable')
        b = [np.float64(-2), np.float64(-2), np.float64(-.01),
             np.float64(2), np.float64(2), np.float64(2)]
        self.assertTrue(valid_bounds(b))
        self.assertTrue(inside((np.float64(0), np.float64(0), np.float64(1)), b))



    def test_leaky_counter_survives_isolated_glitch(self):
        """孤立位置偏差只记录诊断，配对计数继续增长。"""
        now = [100.]
        guard = MapGuard(clock=lambda: now[0])
        for i in range(30):
            stamp = 99.6 + i * .01
            guard.observe('planner', stamp, 'world', (1, 2, 1), (0, 0, 0, 1))
            guard.observe('map', stamp, 'world', (1, 2, 1), (0, 0, 0, 1))
        self.assertGreaterEqual(guard.good, 10)
        before = guard.good
        guard.observe('planner', 100.0, 'world', (1.6, 2, 1), (0, 0, 0, 1))
        guard.observe('map', 100.0, 'world', (1, 2, 1), (0, 0, 0, 1))
        self.assertEqual(guard.good, before + 1)
        self.assertTrue(guard.snapshot(100.01)['map_coordinates_valid'])

    def test_leaky_counter_still_detects_sustained_mismatch(self):
        """持续位置偏差不关闭地图门，但保留诊断。"""
        now = [100.]
        guard = MapGuard(clock=lambda: now[0])
        for i in range(30):
            stamp = 99.6 + i * .01
            guard.observe('planner', stamp, 'world', (1, 2, 1), (0, 0, 0, 1))
            guard.observe('map', stamp, 'world', (1, 2, 1), (0, 0, 0, 1))
        for i in range(10):
            stamp = 100.0 + i * .01
            guard.observe('planner', stamp, 'world', (1.6, 2, 1), (0, 0, 0, 1))
            guard.observe('map', stamp, 'world', (1, 2, 1), (0, 0, 0, 1))
        self.assertGreaterEqual(guard.good, 10)
        self.assertTrue(guard.snapshot(100.2)['map_coordinates_valid'])
        self.assertEqual(guard.snapshot(100.2)['map_last_reject'], 'distance')


if __name__ == '__main__': unittest.main()
