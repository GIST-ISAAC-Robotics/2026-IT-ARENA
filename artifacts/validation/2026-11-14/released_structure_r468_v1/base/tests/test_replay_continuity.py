"""기본 기능 성공과 주행 비교 구간의 정지 없는 명령을 구분한다."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/arena_vehicle_interface'))
from arena_vehicle_interface.bag_contract import active_window_continuity


class ContinuityTest(unittest.TestCase):
    original = [(0, 0., 0.), (1, .2, 0.), (3, .5, 0.), (4, 0., 0.)]

    def test_start_and_eof_zeros_are_outside_reference_window(self):
        result = active_window_continuity(self.original, [(0, 0., 0.), (2, .2, 0.), (4, 0., 0.)])
        self.assertTrue(result['no_interruption'])
        self.assertEqual(result['samples'], 1)

    def test_stop_reverse_and_nan_reject_continuity(self):
        for command in ((2, 0., 0.), (2, -.1, 0.), (2, float('nan'), 0.), (2, .1, float('inf'))):
            with self.subTest(command=command):
                self.assertFalse(active_window_continuity(self.original, [command])['no_interruption'])

    def test_empty_reference_and_no_observed_commands_are_not_passed(self):
        self.assertFalse(active_window_continuity([], [])['no_interruption'])
        self.assertFalse(active_window_continuity(self.original, [])['no_interruption'])


if __name__ == '__main__':
    unittest.main()
