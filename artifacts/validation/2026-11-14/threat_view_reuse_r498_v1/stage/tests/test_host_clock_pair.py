import unittest
from arena_vehicle_interface.host_clock_pair import sample_clocks


class ClockTests(unittest.TestCase):
    def call(self,mono,ros,attempts=3):
        m=iter(mono);r=iter(ros)
        return sample_clocks(lambda:next(m),lambda:next(r),attempts)

    def test_narrowest_bracket_removes_read_deschedule_as_offset(self):
        mono,ros,d=self.call([10,1010,1020,1030,1040,1044],[1000510,1001025,1001042])
        self.assertEqual((mono,ros,d['selected']),(1042,1001042,2))
        self.assertEqual(ros-mono,1000000)
        self.assertEqual(d['read_interval_half_width_ns'],2)
        self.assertEqual(d['initial_receipt_monotonic_ns'],10)

    def test_large_epoch_uses_integer_arithmetic(self):
        a=2**60
        mono,_,d=self.call([a,a+3],[2*a],1)
        self.assertEqual(mono,a+1);self.assertEqual(d['read_interval_half_width_ns'],2)

    def test_real_clock_step_is_not_smoothed(self):
        first=self.call([10,12],[1000011],1)
        second=self.call([20,22],[900021],1)
        self.assertEqual((second[1]-second[0])-(first[1]-first[0]),-100000)

    def test_backward_monotonic_rejected(self):
        with self.assertRaises(ValueError):self.call([20,19],[30],1)

    def test_invalid_read_type_rejected(self):
        with self.assertRaises(ValueError):self.call([True,5],[30],1)

    def test_unbounded_attempts_rejected(self):
        with self.assertRaises(ValueError):self.call([],[],6)


if __name__=='__main__':unittest.main()
