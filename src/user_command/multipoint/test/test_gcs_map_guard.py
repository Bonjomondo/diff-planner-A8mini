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
        self.assertFalse(self.samples(shift=1).snapshot(100.)['map_coordinates_valid'])
        self.assertFalse(self.samples(quat=(0,0,1,0)).snapshot(100.)['map_coordinates_valid'])
    def test_restart_or_bad_frame_revokes_verification(self):
        guard = self.samples()
        guard.observe('map', 99., 'world', (1,2,1), (0,0,0,1))
        self.assertFalse(guard.snapshot(100.)['map_coordinates_valid'])
        guard = self.samples()
        guard.observe('map', 100., 'other', (1,2,1), (0,0,0,1))
        self.assertFalse(guard.snapshot(100.)['map_coordinates_valid'])
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

if __name__ == '__main__': unittest.main()
