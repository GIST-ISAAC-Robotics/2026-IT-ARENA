from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from handoff_prototype import BlockingPersistence, HandoffPrototype
from arena_autonomy.lidar_motion import MotionHistory, MotionConfig


def scene(t, epoch=0):
    return SimpleNamespace(t_ref_s=t, source_first_s=t-.1, source_epoch=epoch,
                           source_frame_id='laser_frame')


class PersistenceTest(unittest.TestCase):
    def trial(self, target_speed=0., ego_speed=0., missing_at=None):
        monitor=BlockingPersistence()
        history=MotionHistory(MotionConfig(wheel_radius_m=.0325, history_s=3.))
        for t in np.arange(0.,2.,.01):
            history.add_mean_wheel(float(t),ego_speed/.0325);history.add_gyro(float(t),0.)
        results=[]
        with patch('handoff_prototype.source_valid',return_value=True):
            for k in range(11):
                t=round(.5+.1*k,9)
                cloud=np.c_[np.ones(7)*(1.+(target_speed-ego_speed)*(t-.5)),np.linspace(-.18,-.1,7)]
                with patch('handoff_prototype.front_patch',return_value=(None,'missing') if k==missing_at else (cloud,'face')):
                    results.append(monitor.update(scene(t),history))
        return results, monitor, history

    def test_measured_ego_motion_does_not_look_like_target_motion(self):
        results,_,_=self.trial(ego_speed=.35)
        self.assertTrue(results[-1]['ready'])
        self.assertLess(results[-1]['max_compensated_displacement_m'],1e-8)

    def test_moving_lead_not_ready(self):
        results,_,_=self.trial(target_speed=.25,ego_speed=.35)
        self.assertFalse(any(r['ready'] for r in results))

    def test_missing_patch_breaks_persistence(self):
        results,_,_=self.trial(missing_at=5)
        self.assertFalse(any(r['ready'] for r in results))

    def test_repeated_source_does_not_accumulate(self):
        results,m,h=self.trial()
        with patch('handoff_prototype.source_valid',return_value=True):
            r=m.update(scene(1.5),h)
        self.assertFalse(r['fresh']);self.assertEqual(r['sample_count'],results[-1]['sample_count'])

    def test_epoch_and_staleness_invalidate(self):
        _,m,h=self.trial()
        cloud=np.c_[np.ones(7),np.linspace(-.18,-.1,7)]
        with patch('handoff_prototype.source_valid',return_value=True),patch('handoff_prototype.front_patch',return_value=(cloud,'face')):
            self.assertFalse(m.update(scene(1.6,1),h)['ready'])
            stale=scene(1.7,1);stale.source_first_s=1.
            self.assertFalse(m.update(stale,h)['ready'])

    def test_slow_motion_remains_an_explicit_limit(self):
        results,_,_=self.trial(target_speed=.03)
        self.assertTrue(results[-1]['ready'])
        self.assertFalse(results[-1]['identity_or_stationarity_proven'])


class ModeTest(unittest.TestCase):
    def subject(self,offset=.16,seen=False):
        h=HandoffPrototype()
        h.normal.command=lambda *a,**kw:(.3,0.,dict(selector_choice='baseline',selector_sides={
            'left':dict(seen=seen,present=False),'right':dict(seen=False,present=False)}))
        h.monitor.update=lambda *a:dict(ready=True,observed=True,fresh=True)
        h.bypass.command=lambda *a,**kw:(.35,.03,dict(selected=dict(offset_m=offset),entry_phase='entry'))
        return h

    def test_zero_offset_does_not_start_bypass(self):
        h=self.subject(0.)
        with patch('handoff_prototype.source_valid',return_value=True):
            self.assertEqual(h.command(scene(.5),None,.3,'left',0.,.02,raw_scan=None)[2]['mode'],'FOLLOW')

    def test_seen_target_side_vetoes_entry(self):
        h=self.subject(seen=True)
        with patch('handoff_prototype.source_valid',return_value=True):
            self.assertEqual(h.command(scene(.5),None,.3,'left',0.,.02,raw_scan=None)[2]['mode'],'FOLLOW')

    def test_epoch_resets_active_bypass(self):
        h=self.subject()
        with patch('handoff_prototype.source_valid',return_value=True):
            self.assertEqual(h.command(scene(.5),None,.3,'left',0.,.02,raw_scan=None)[2]['mode'],'BYPASS')
            result=h.command(scene(.6,1),None,.3,'left',0.,.02,raw_scan=None)
        self.assertEqual(result[0],0.);self.assertEqual(h.mode,'FOLLOW')

    def test_fault_prevents_bypass_output(self):
        h=self.subject();h.mode='BYPASS'
        h.normal.command=lambda *a,**kw:(0.,0.,dict(selector_choice='hold_fault'))
        result=h.command(scene(.5),None,.3,'left',0.,.02,raw_scan=None)
        self.assertEqual(result[0],0.);self.assertEqual(h.mode,'FOLLOW')

    def test_quantized_measured_speed_is_not_clipped_to_output_cap(self):
        h=self.subject();captured=[]
        def candidate(sc,history,speed,*args,**kwargs):
            captured.append(speed)
            return .35,.03,dict(selected=dict(offset_m=.16),entry_phase='entry')
        h.bypass.command=candidate
        with patch('handoff_prototype.source_valid',return_value=True):
            result=h.command(scene(.5),None,.364649,'left',0.,.02,raw_scan=None)
        self.assertEqual(captured,[.364649]);self.assertEqual(result[0],.35)
        self.assertEqual(h.mode,'BYPASS')

    def test_speed_outside_existing_g2_contract_cannot_enter(self):
        h=self.subject()
        with patch('handoff_prototype.source_valid',return_value=True):
            result=h.command(scene(.5),None,1.01,'left',0.,.02,raw_scan=None)
        self.assertEqual(h.mode,'FOLLOW');self.assertLessEqual(result[0],.35)


if __name__=='__main__': unittest.main()
