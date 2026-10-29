"""연속 확인의 원본 연결·두 스캔·만료 및 실제 반환 반례. 주행 검증은 아니다."""
from copy import deepcopy
from dataclasses import replace
import numpy as np
import pytest

from rolling_clearance_candidate import RollingClearanceCandidate, RollingRecoveryCandidate
from arena_autonomy.surface_anchor_trial import SurfaceAnchorTrial
from test_anchor_clearance_probe import clearance_fixture
from test_positive_clearance_state import later


def fixture(retained=None, modify=None):
    retained, background, reference, scene, history, points, ids = clearance_fixture(retained, modify)
    # 최초~현재의 10cm 차이는 기존 단일 정합 한계를 넘는 합성 조건이다.
    background = background + np.array([.10, 0.])
    old_ids = np.arange(len(background))
    anchor = SurfaceAnchorTrial()
    anchor.start(retained, background, old_ids, reference, .4)
    probe = RollingClearanceCandidate()
    probe.start(retained, background, old_ids, reference, .4)
    probe.bind(anchor)
    return probe, anchor, scene, history, points, ids


def current_anchor(anchor, scene, points):
    # 상태 계약 검사는 정합 승인 뒤 필드만 재현한다. 실제 정합은 저장 입력에서 검사한다.
    anchor.anchor = (points.copy(), scene.t_ref_s, scene.source_first_s)
    anchor.last_reference = scene.t_ref_s
    anchor.last_source = scene.source_first_s


def test_two_distinct_current_scans_and_deepcopy_keep_independent_bound_state():
    probe, anchor, scene, h, p, ids = fixture()
    current_anchor(anchor, scene, p)
    first = probe.observe(scene, h, p, ids)
    assert not first['matched'] and first['rolling_clearance']['accepted']
    assert first['rolling_clearance']['consecutive_fresh_sources'] == 1
    duplicate = probe.observe(scene, h, p, ids)
    assert not duplicate['matched'] and duplicate['rolling_clearance']['consecutive_fresh_sources'] == 1
    copied_anchor, copied_probe = deepcopy((anchor, probe))
    assert copied_probe.rolling_anchor is copied_anchor and copied_anchor is not anchor
    next_scene = later(scene,.1);current_anchor(copied_anchor,next_scene,p)
    second = copied_probe.observe(next_scene,h,p,ids)
    assert second['matched'] and second['reason'] == 'rolling_old_locations_supported'
    assert second['complete_points'] == 1 and not second['unknown_filled']
    assert probe.rolling_consecutive == 1


@pytest.mark.parametrize('fault',['different_points','different_time','missing','ended'])
def test_cannot_bind_unrelated_original_evidence(fault):
    probe, anchor, scene, h, p, ids = fixture()
    if fault == 'different_points':anchor.retained[0,0]+=.01
    if fault == 'different_time':anchor.created+=.1
    if fault == 'missing':anchor.anchor=None
    if fault == 'ended':anchor.ended=True
    with pytest.raises(ValueError,match='complete original'):probe.bind(anchor)


@pytest.mark.parametrize('fault',['old_source','old_reference','changed_current','lost_point','ended'])
def test_only_accepted_current_complete_anchor_can_supply_coordinates(fault):
    probe,anchor,scene,h,p,ids=fixture();current_anchor(anchor,scene,p)
    if fault=='old_source':anchor.anchor=(p.copy(),scene.t_ref_s,scene.source_first_s-.1)
    if fault=='old_reference':anchor.anchor=(p.copy(),scene.t_ref_s-.1,scene.source_first_s)
    if fault=='changed_current':anchor.anchor=(p+.001,scene.t_ref_s,scene.source_first_s)
    if fault=='lost_point':anchor.retained=np.empty((0,2))
    if fault=='ended':anchor.ended=True
    result=probe.observe(scene,h,p,ids)
    assert not result['matched'] and result['rolling_clearance']['reason']=='current_complete_anchor_required'


@pytest.mark.parametrize('kind',['one_occupied','missing','occluded','rear','age_margin'])
def test_real_returns_must_confirm_every_old_location(kind):
    retained = [[.35,.1],[.35,.425]] if kind=='one_occupied' else [[-.2,0.]] if kind=='rear' else [[.35,.31]] if kind=='age_margin' else None
    def modify(values, angles):
        if kind=='missing':values[(angles>.22)&(angles<.44)]=np.inf
        if kind=='occluded':values[(angles>.20)&(angles<.5)]=.20
    probe,anchor,scene,h,p,ids=fixture(retained,modify)
    for delta in (0.,.1):
        now=later(scene,delta);current_anchor(anchor,now,p)
        result=probe.observe(now,h,p,ids)
        assert not result['matched'] and not result['rolling_clearance']['accepted']


@pytest.mark.parametrize('delta',[-.11,1.01])
def test_original_time_reversal_and_one_second_expiry_still_reject(delta):
    probe,anchor,scene,h,p,ids=fixture();now=later(scene,delta);current_anchor(anchor,now,p)
    result=probe.observe(now,h,p,ids)
    assert not result['matched'] and probe.ended


def test_failed_new_scan_resets_then_requires_two_confirmations_again():
    probe,anchor,scene,h,p,ids=fixture();current_anchor(anchor,scene,p)
    assert probe.observe(scene,h,p,ids)['rolling_clearance']['consecutive_fresh_sources']==1
    for delta,valid,expected in ((.1,False,0),(.2,True,1),(.3,True,2)):
        now=later(scene,delta)
        if valid:current_anchor(anchor,now,p)
        result=probe.observe(now,h,p,ids)
        assert result['rolling_clearance']['consecutive_fresh_sources']==expected
        assert result['matched'] is (expected==2)


def test_candidate_uses_the_complete_policy_capture_and_reset_removes_binding():
    from test_recovery_threat_capture import fixture as capture
    _, previous, threats, ids=capture()
    policy=RollingRecoveryCandidate(True,True,True)
    policy.point_ids[(previous[2],previous[1])]=ids.copy()
    policy._latch('previous_point_in_missing_sweep',1.16)
    policy._capture_latched_threat(threats,previous,1.16)
    assert policy.clearance_probe.rolling_anchor is policy.anchor_probe
    assert len(policy.clearance_probe.original[0])==6
    policy.reset()
    assert policy.clearance_probe.rolling_anchor is None
