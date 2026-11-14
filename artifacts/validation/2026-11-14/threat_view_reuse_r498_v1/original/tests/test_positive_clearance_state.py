"""양성 반환 확인의 독립 상태·새 스캔 수·최종 재검사 계약."""
from dataclasses import replace
import numpy as np
import pytest
from arena_autonomy.anchor_clearance_trial import PositiveClearanceTrial
from test_anchor_clearance_probe import clearance_fixture


def later(scene,dt):
    return replace(scene,t_ref_s=scene.t_ref_s+dt,source_first_s=scene.source_first_s+dt,
        source_last_s=scene.source_last_s+dt,
        rays=tuple(replace(r,acquired_at_s=r.acquired_at_s+dt) for r in scene.rays))


def fixture():
    retained,background,ref,scene,h,p,ids=clearance_fixture()
    probe=PositiveClearanceTrial()
    # 전체 원래 배경에는 소실 점도 포함되며 원래 인덱스는 별도 검증된다.
    probe.start(retained,background,np.arange(len(background)),ref,.4)
    return probe,scene,h,p,ids


def test_requires_two_distinct_positive_scans_and_preserves_original():
    probe,scene,h,p,ids=fixture();original=tuple(x.copy() if hasattr(x,'copy') else x for x in probe.original)
    first=probe.observe(scene,h,p,ids)
    duplicate=probe.observe(scene,h,p,ids)
    second=probe.observe(later(scene,.1),h,p,ids)
    assert first['accepted'] and not first['matched']
    assert duplicate['reason']=='same_scan_no_confirmation' and duplicate['consecutive_fresh_sources']==1
    assert second['matched'] and second['consecutive_fresh_sources']==2
    for a,b in zip(original,probe.original):np.testing.assert_equal(a,b)


def test_new_unconfirmed_scan_resets_consecutive():
    probe,scene,h,p,ids=fixture()
    assert probe.observe(scene,h,p,ids)['consecutive_fresh_sources']==1
    # 유효 반환을 하나씩 건너뛴 위조 인덱스/좌표 대응은 확인으로 세지 않는다.
    assert not probe.observe(later(scene,.1),h,p,ids[::-1])['matched']
    good=probe.observe(later(scene,.2),h,p,ids)
    assert good['consecutive_fresh_sources']==1 and not good['matched']


@pytest.mark.parametrize('delta',[-.11,1.01])
def test_regression_or_expiry_never_releases(delta):
    probe,scene,h,p,ids=fixture()
    result=probe.observe(later(scene,delta),h,p,ids)
    assert not result['matched'] and probe.ended


def test_reset_discards_history_without_claiming_clearance():
    probe,scene,h,p,ids=fixture();probe.reset()
    assert probe.original is None and not probe.observe(scene,h,p,ids)['matched']


@pytest.mark.parametrize('current_allowed',[False,True])
def test_complete_clearance_still_needs_current_policy_recheck(monkeypatch,current_allowed):
    import arena_autonomy.traffic_recovery_trial as module
    from test_recovery_threat_capture import fixture as captured
    from types import SimpleNamespace
    ctl,previous,threats,ids=captured();ctl._capture_latched_threat(threats,previous,1.16)
    def evaluate(self,*args):
        return dict(allowed=False if self.latched else current_allowed,
            reason='known_obstacle_latched' if self.latched else 'bounded_side_missing' if current_allowed else 'near_gap_boundary')
    monkeypatch.setattr(module.TimedPolicy,'evaluate',evaluate)
    monkeypatch.setattr(ctl.anchor_probe,'observe',lambda *args:dict(matched=False,reason='inactive_or_ended'))
    monkeypatch.setattr(ctl.clearance_probe,'observe',lambda *args:dict(matched=True,reason='all_old_locations_supported'))
    monkeypatch.setattr(module,'motion_scene',lambda *args,**kw:object())
    raw=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=100000000)),
        ranges=[],angle_min=0.,angle_increment=.01,range_min=.05,range_max=12.,time_increment=.0002)
    h=SimpleNamespace(project=lambda *args,**kwargs:(previous[0],ids,None))
    out=ctl.evaluate(raw,previous[0],h,1.2,.2)
    assert out['allowed'] is current_allowed
    assert out['clearance_diagnostic']['released'] is current_allowed
    assert ctl.latched is not current_allowed
    assert not out['anchor_diagnostic']['released']


def test_planner_and_guard_clearance_state_are_independent():
    from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
    a,b=RecoveryPolicy(True,True,True),RecoveryPolicy(True,True,True)
    a.clearance_probe.ended=True
    assert a.clearance_probe is not b.clearance_probe and not b.clearance_probe.ended
