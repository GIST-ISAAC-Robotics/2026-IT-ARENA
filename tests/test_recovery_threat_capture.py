"""전체 위협과 원래 스캔 기준을 보존하고 진단 절단으로 해제하지 않는다."""
import numpy as np
import pytest
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from arena_autonomy.surface_anchor_trial import SurfaceAnchorTrial
from test_partial_surface_alignment import room


def fixture():
    p,ids=room();ctl=RecoveryPolicy(True,True,True)
    previous=(p,1.,.9)
    ctl.point_ids[(.9,1.)]=ids.copy()
    # 아홉 광선/점 쌍이지만 여섯 개 고유점이다. 마지막 점도 보존한다.
    threats=np.zeros((3,len(p)),dtype=bool)
    threats[0,[20,21,22]]=True;threats[1,[20,21,23]]=True;threats[2,[24,25,20]]=True
    ctl._latch('previous_point_in_missing_sweep',1.16)
    return ctl,previous,threats,ids


def test_complete_union_keeps_all_points_and_original_anchor_reference():
    ctl,previous,threats,ids=fixture()
    ctl._capture_latched_threat(threats,previous,1.16)
    assert ctl.anchor_threat_summary==dict(pairs=9,unique_points=6,source_first_s=.9,reference_s=1.,complete=True)
    assert ctl.anchor_latch_time==1.16
    assert ctl.anchor_probe.anchor[1:]==(1.,.9)
    np.testing.assert_array_equal(ctl.anchor_probe.anchor[0],previous[0])
    np.testing.assert_array_equal(ctl.anchor_probe.retained,previous[0][[20,21,22,23,24,25]])
    retained=ctl.anchor_probe.retained.copy();anchor=ctl.anchor_probe.anchor[0].copy()
    previous[0][:]=99.;threats[:]=False;ids[:]=0
    np.testing.assert_array_equal(ctl.anchor_probe.retained,retained)
    np.testing.assert_array_equal(ctl.anchor_probe.anchor[0],anchor)


def test_missing_source_metadata_keeps_latch_closed():
    ctl,previous,threats,_=fixture();ctl.point_ids.clear()
    with pytest.raises(ValueError,match='acquisition metadata'):
        ctl._capture_latched_threat(threats,previous,1.16)
    assert ctl.latched and ctl.anchor_threat_summary is None


@pytest.mark.parametrize('mutation',['empty','wrong_shape','non_bool','bad_id'])
def test_bad_complete_set_cannot_initialize_anchor(mutation):
    ctl,previous,threats,_=fixture()
    if mutation=='empty':threats[:]=False
    elif mutation=='wrong_shape':threats=threats[:,:-1]
    elif mutation=='non_bool':threats=threats.astype(float)
    else:ctl.point_ids[(.9,1.)][:]=1
    with pytest.raises(ValueError):ctl._capture_latched_threat(threats,previous,1.16)
    assert ctl.latched and ctl.anchor_threat_summary is None


def test_reset_clears_complete_evidence_and_point_metadata():
    ctl,previous,threats,_=fixture();ctl._capture_latched_threat(threats,previous,1.16)
    ctl.reset()
    assert ctl.point_ids=={} and ctl.current_ids is None
    assert ctl.anchor_probe.anchor is None and ctl.anchor_threat_summary is None


def test_every_retained_point_must_reappear_not_just_first_eight_pairs(monkeypatch):
    import arena_autonomy.surface_anchor_trial as module
    p,ids=room();retained=np.vstack([p[[20,21,22,23,24]],[[.35,.02]]])
    probe=SurfaceAnchorTrial();probe.start(retained,p,ids,1.,.9)
    monkeypatch.setattr(module,'transport_points',lambda points,*args:np.array(points,copy=True))
    for t in (1.1,1.2):
        result=probe.observe(p,ids,None,t,t-.1)
        assert not result['matched'] and not result['observed'][-1]
        assert result['consecutive_fresh_sources']==0


def test_current_recheck_can_reject_even_after_complete_reobservation(monkeypatch):
    import arena_autonomy.traffic_recovery_trial as module
    from types import SimpleNamespace
    ctl,previous,threats,ids=fixture();ctl._capture_latched_threat(threats,previous,1.16)
    old=module.TimedPolicy.evaluate
    calls=[]
    def evaluate(self,*args):
        calls.append(self.latched)
        return dict(allowed=False,reason='known_obstacle_latched' if self.latched else 'near_gap_boundary')
    monkeypatch.setattr(module.TimedPolicy,'evaluate',evaluate)
    monkeypatch.setattr(ctl.anchor_probe,'observe',lambda *args:dict(matched=True,reason='reobserved'))
    raw=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=100000000)),
        ranges=[],angle_min=0.,angle_increment=.01,range_min=.05,range_max=12.,time_increment=.0002)
    h=SimpleNamespace(project=lambda *args,**kwargs:(previous[0],ids,None))
    out=ctl.evaluate(raw,previous[0],h,1.2,.2)
    assert calls==[True,False] and ctl.latched
    assert not out['allowed'] and not out['anchor_diagnostic']['released']
    assert out['anchor_diagnostic']['current_recheck_reason']=='near_gap_boundary'
