"""명시 선택형 재관측 연결의 독립 상태·두 스캔·입력 거절을 확인한다."""
import numpy as np
import pytest
from arena_autonomy.surface_anchor_trial import SurfaceAnchorTrial
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from test_surface_reobservation import patch


def test_default_anchor_and_basic_policy_are_unchanged():
    assert not SurfaceAnchorTrial().extended_surface
    assert RecoveryPolicy(True,True,True).anchor_probe.extended_surface
    with pytest.raises(ValueError):SurfaceAnchorTrial(extended_surface=1)


@pytest.mark.parametrize('enabled', [False,True])
def test_all_retained_points_and_two_distinct_scans_are_required(monkeypatch,enabled):
    import arena_autonomy.surface_anchor_trial as module
    p=patch();ids=np.arange(377,381);old=np.array([[.03237031136,.51284943630]])
    monkeypatch.setattr(module,'transport_points',lambda points,*args:np.array(points,copy=True))
    monkeypatch.setattr(module,'surface_translation',lambda *args:dict(accepted=True,translation_m=[0,0]))
    probe=SurfaceAnchorTrial(extended_surface=enabled);probe.start(old,p,ids,1.,.9)
    first=probe.observe(p,ids,None,1.1,1.)
    assert not first['matched']
    assert first['consecutive_fresh_sources']==int(enabled)
    assert not probe.observe(p,ids,None,1.12,1.)['matched']
    assert probe.observe(p,ids,None,1.2,1.1)['matched']==enabled
    probe.start(np.vstack([old,[[.3,.1]]]),p,ids,2.,1.9)
    assert not probe.observe(p,ids,None,2.1,2.)['matched']
    assert not probe.observe(p,ids,None,2.2,2.1)['matched']


def test_extended_surface_does_not_rescue_rejected_alignment(monkeypatch):
    import arena_autonomy.surface_anchor_trial as module
    p=patch();ids=np.arange(377,381)
    probe=SurfaceAnchorTrial(extended_surface=True)
    probe.start([[.03237031136,.51284943630]],p,ids,1.,.9)
    monkeypatch.setattr(module,'transport_points',lambda points,*args:np.array(points,copy=True))
    monkeypatch.setattr(module,'surface_translation',lambda *args:dict(accepted=False,reason='weak_improvement'))
    for t in (1.1,1.2):
        result=probe.observe(p,ids,None,t,t-.1)
        assert not result['matched'] and result['reason']=='surface_rejected'
        assert result['consecutive_fresh_sources']==0
