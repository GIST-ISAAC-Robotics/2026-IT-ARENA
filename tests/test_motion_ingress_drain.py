import unittest
from arena_autonomy.lidar_motion_ros import drain_ready_motion as drain_ready


class FakeSubscription:
    def __init__(self,values,callback=None):
        self.queue=list(values);self.accepted=[];self.handle=self
        self.msg_type=object;self.raw=False
        self.callback=callback or self.accepted.append
    def __enter__(self):return self
    def __exit__(self,*_):return False
    def take_message(self,*_):return (self.queue.pop(0),{}) if self.queue else None


class IngressTests(unittest.TestCase):
    def test_all_pending_preserves_stamps_and_per_stream_order(self):
        a=FakeSubscription([1.,1.005,1.01]);b=FakeSubscription([1.002,1.012])
        result=drain_ready([a,b],clock=lambda:0.)
        self.assertEqual(a.accepted,[1.,1.005,1.01]);self.assertEqual(b.accepted,[1.002,1.012])
        self.assertEqual(result['counts'],[3,2]);self.assertFalse(result['bounded_stop'])
    def test_count_limit_leaves_unread_messages(self):
        a=FakeSubscription(range(100));result=drain_ready([a],max_messages=4,clock=lambda:0.)
        self.assertEqual(a.accepted,list(range(4)));self.assertEqual(a.queue,list(range(4,100)))
        self.assertTrue(result['bounded_stop'])
    def test_time_limit_leaves_unread_messages(self):
        times=iter([0.,0.,.001,.003,.003])
        a=FakeSubscription(range(5));result=drain_ready([a],clock=lambda:next(times))
        self.assertEqual(a.accepted,[0]);self.assertEqual(a.queue,[1,2,3,4]);self.assertTrue(result['bounded_stop'])
    def test_empty_returns_without_waiting(self):
        result=drain_ready([FakeSubscription([])],clock=lambda:0.)
        self.assertEqual(result['total'],0);self.assertFalse(result['bounded_stop'])
    def test_validation_callback_is_not_bypassed_or_suppressed(self):
        def rejected(_):raise ValueError('existing validation')
        with self.assertRaisesRegex(ValueError,'existing validation'):
            drain_ready([FakeSubscription([1.],rejected)],clock=lambda:0.)


if __name__=='__main__':unittest.main()
