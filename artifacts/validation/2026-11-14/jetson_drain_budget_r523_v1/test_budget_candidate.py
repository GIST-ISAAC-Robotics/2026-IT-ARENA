"""r522 원 stamp·실측 표본당 비용 모델로 제한 소비의 경계를 확인한다."""
import unittest
from motion_ingress_candidate import drain_ready


class Clock:
    def __init__(self,cost=.00023335666648765457):self.value=0.;self.cost=cost
    def __call__(self):return self.value


class Subscription:
    def __init__(self,stamps,clock):
        self.queue=list(stamps);self.accepted=[];self.handle=self;self.clock=clock
        self.msg_type=object;self.raw=False;self.callback=self.accepted.append
    def __enter__(self):return self
    def __exit__(self,*_):return False
    def take_message(self,*_):
        if not self.queue:self.clock.value+=.00002;return None
        self.clock.value+=self.clock.cost
        return self.queue.pop(0),{}


class BudgetTests(unittest.TestCase):
    def replay(self,budget):
        c=Clock();w=Subscription([22.074+i*.01 for i in range(9)],c)
        g=Subscription([22.070+i*.005 for i in range(18)],c)
        result=drain_ready([w,g],budget_s=budget,clock=c)
        now=22.148689607
        ages=[now-max([initial]+[t for t in s.accepted if t<=now]) for initial,s in [(22.064,w),(22.065,g)]]
        return result,w,g,ages

    def test_recorded_burst_stale_at2ms_and_fresh_at6ms_without_restamping(self):
        old,w1,g1,ages1=self.replay(.002);new,w2,g2,ages2=self.replay(.006)
        self.assertEqual(old['counts'],[5,4]);self.assertTrue(all(a>.03 for a in ages1))
        self.assertTrue(all(a<=.03 for a in ages2));self.assertTrue(new['bounded_stop'])
        self.assertEqual(w2.accepted+w2.queue,[22.074+i*.01 for i in range(9)])
        self.assertEqual(g2.accepted+g2.queue,[22.070+i*.005 for i in range(18)])
        self.assertGreater(len(g2.queue),0)

    def test_sixtyfour_limit_is_unchanged(self):
        c=Clock(cost=0.);s=Subscription(range(100),c)
        result=drain_ready([s],budget_s=.006,clock=c)
        self.assertEqual(result['total'],64);self.assertEqual(s.queue,list(range(64,100)))

    def test_unbacklogged_stream_is_identical_and_exits_early(self):
        outputs=[]
        for budget in (.002,.006):
            c=Clock();a=Subscription([1.,1.01],c);b=Subscription([1.,1.005,1.01],c)
            r=drain_ready([a,b],budget_s=budget,clock=c)
            self.assertFalse(r['bounded_stop']);self.assertLess(r['duration_ms'],2.)
            outputs.append((a.accepted,b.accepted))
        self.assertEqual(outputs[0],outputs[1])


if __name__=='__main__':unittest.main()
