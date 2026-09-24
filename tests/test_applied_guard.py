"""가상 MCU 뒤 최종 출력 감시의 ROS 독립 회귀."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/arena_vehicle_interface'))
from arena_vehicle_interface.applied_guard import AppliedGuard


def status(t=1000, *, generation=1, state='ARMED', speed=0., boot='a', **extra):
    return dict(now_us=t, boot=boot, generation=generation, state=state,
                target_speed_mps=speed, steering_rad=.1, deadline_us=t+100_000, reason='test', **extra)


class AppliedGuardTests(unittest.TestCase):
    def armed(self):
        g = AppliedGuard()
        self.assertTrue(g.update(status(), 1000, 1.))
        self.assertTrue(g.update(status(2000, state='ACTIVE', speed=1.), 2000, 1.001))
        return g

    def test_no_active_boot(self):
        g = AppliedGuard()
        self.assertFalse(g.update(status(state='ACTIVE', speed=1.), 1000, 1.))
        self.assertEqual(g.speed, 0.)

    def test_lost_stream_no_auto_restart(self):
        g = self.armed()
        g.watch(103000, 1.102)
        self.assertEqual(g.speed, 0.)
        self.assertFalse(g.update(status(104000, state='ACTIVE', speed=1.), 104000, 1.103))
        self.assertTrue(g.update(status(105000, generation=2), 105000, 1.104))

    def test_shorter_command_deadline_wins(self):
        g = self.armed()
        s = status(3000, state='ACTIVE', speed=1.)
        s['deadline_us'] = 4000
        self.assertTrue(g.update(s, 3000, 1.002))
        g.watch(4000, 1.003)
        self.assertEqual(g.speed, 0.)

    def test_clock_pause_wall_watchdog(self):
        g = self.armed()
        g.watch(2000, 4.002)
        self.assertTrue(g.blocked)

    def test_clock_regression(self):
        g = self.armed()
        g.watch(0, 1.002)
        self.assertEqual(g.speed, 0.)

    def test_duplicate_cannot_refresh_wall_watchdog(self):
        g = self.armed()
        self.assertFalse(g.update(status(2000, state='ACTIVE', speed=1.), 2000, 3.))
        g.watch(2000, 4.002)
        self.assertEqual(g.speed, 0.)

    def test_bad_values_stop(self):
        for bad in [float('nan'), float('inf'), -1., 3., True, 10**400]:
            g = self.armed()
            self.assertFalse(g.update(status(3000, state='ACTIVE', speed=bad), 3000, 1.002))
            self.assertEqual(g.speed, 0.)

    def test_malformed_and_future(self):
        for s in [{}, {'now_us': 0}, status(5000)]:
            g = self.armed()
            self.assertFalse(g.update(s, 3000, 1.002))
            self.assertEqual(g.speed, 0.)

    def test_new_boot_needs_arm(self):
        g = self.armed()
        self.assertFalse(g.update(status(3000, boot='b', state='DISARMED'), 3000, 1.002))
        self.assertTrue(g.update(status(4000, boot='b', generation=2), 4000, 1.003))

    def test_fault_holds_steering(self):
        g = self.armed()
        self.assertFalse(g.update(status(3000, state='FAULT_LATCHED'), 3000, 1.002))
        self.assertEqual(g.speed, 0.)
        self.assertEqual(g.steering, .1)

    def test_old_generation_arm_cannot_clear_output_latch(self):
        g = self.armed()
        g.update(status(3000, generation=2, state='FAULT_LATCHED'), 3000, 1.002)
        self.assertFalse(g.update(status(4000, generation=1), 4000, 1.003))
        self.assertEqual(g.speed, 0.)

    def test_old_boot_cannot_return(self):
        g = self.armed()
        g.update(status(3000, boot='b', state='DISARMED'), 3000, 1.002)
        self.assertFalse(g.update(status(4000, boot='a', generation=3), 4000, 1.003))

    def test_invalid_clock_cannot_unlock_even_new_arm(self):
        g = self.armed()
        self.assertFalse(g.update(status(3000, generation=2), 3000, float('nan')))
        self.assertEqual(g.speed, 0.)


if __name__ == '__main__':
    unittest.main()
