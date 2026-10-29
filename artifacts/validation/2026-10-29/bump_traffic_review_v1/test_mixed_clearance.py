"""점별 근거·최근 반환을 함께 쓰는 미연결 후보의 상태/광선 반례."""
from copy import deepcopy
from dataclasses import replace
import numpy as np
import pytest

from mixed_clearance_candidate import MixedClearanceCandidate, MixedRecoveryCandidate, aligned_previous, combine_points
from arena_autonomy.anchor_clearance_trial import fresh_clearance
from arena_autonomy.surface_anchor_trial import SurfaceAnchorTrial
from test_anchor_clearance_probe import clearance_fixture
from test_positive_clearance_state import later
from test_rolling_clearance_state import current_anchor


RETAINED=[[.35,.1],[.35,.425]]


def fixture(modify=None):
    retained,bg,ref,scene,h,p,ids=clearance_fixture(RETAINED,modify)
    bg=bg+[.1,0.]
    anchor=SurfaceAnchorTrial(extended_surface=True,small_alignment=True)
    anchor.start(retained,bg,np.arange(len(bg)),ref,.4)
    probe=MixedClearanceCandidate();probe.start(retained,bg,np.arange(len(bg)),ref,.4);probe.bind(anchor)
    return probe,anchor,scene,h,p,ids


def evaluate(probe,anchor,scene,h,p,ids):
    current_anchor(anchor,scene,p)
    return probe.observe(scene,h,p,ids)


def gap(angle):
    def modify(values,angles):values[np.argmin(abs(angles-angle))]=np.inf
    return modify


def changed_epoch(scene):
    return replace(scene,source_epoch=7,rays=tuple(replace(r,source_epoch=7) for r in scene.rays))


def test_mixed_reobserved_and_clear_complete_set_needs_two_new_sources():
    probe,anchor,s,h,p,ids=fixture()
    first=evaluate(probe,anchor,s,h,p,ids)
    detail=first['mixed_clearance'];a,b=detail['evidence']
    assert a['current_clear'] and not a['observed']
    assert b['observed'] and b['occupied'] and not b['current_clear']
    assert not first['matched'] and detail['consecutive_fresh_sources']==1
    assert not evaluate(probe,anchor,s,h,p,ids)['matched']
    second=evaluate(probe,anchor,later(s,.1),h,p,ids)
    assert second['matched'] and second['reason']=='mixed_locations_accounted'
    assert second['mixed_clearance']['complete_points']==2
    assert not second['mixed_clearance']['independent_complete_scans']


def test_edge_unknown_uses_recent_real_returns_without_changing_scan():
    probe,anchor,s,h,p,ids=fixture()
    assert not evaluate(probe,anchor,s,h,p,ids)['matched']
    _,_,next_s,h,next_p,next_ids=fixture(gap(.42));next_s=later(next_s,.1)
    original=next_s
    second=evaluate(probe,anchor,next_s,h,next_p,next_ids)
    first_point=second['mixed_clearance']['evidence'][0]
    assert second['matched'] and first_point['recent_clear']
    assert first_point['current_center_supported'] and first_point['current_supported']<53
    assert first_point['combined_supported']==53 and first_point['previous_age_s']>.19
    assert next_s==original and any(r.status=='unknown' for r in next_s.rays)


def test_old_only_center_does_not_release():
    probe,anchor,s,h,p,ids=fixture();evaluate(probe,anchor,s,h,p,ids)
    _,_,next_s,h,next_p,next_ids=fixture(gap(np.arctan2(.1,.29)));next_s=later(next_s,.1)
    result=evaluate(probe,anchor,next_s,h,next_p,next_ids)
    point=result['mixed_clearance']['evidence'][0]
    assert not point['current_center_supported'] and not result['matched']
    assert result['mixed_clearance']['consecutive_fresh_sources']==0


def test_unknown_in_both_sources_has_no_positive_history():
    probe,anchor,s,h,p,ids=fixture(gap(.42))
    for delta in (0.,.1):
        result=evaluate(probe,anchor,later(s,delta),h,p,ids)
        assert not result['matched']
    assert not result['mixed_clearance']['evidence'][0]['accounted']


def test_current_new_return_blocks_old_clearance():
    probe,anchor,s,h,p,ids=fixture();evaluate(probe,anchor,s,h,p,ids)
    def closer(values,angles):values[(angles>.2)&(angles<.5)]=.25
    _,_,next_s,h,next_p,next_ids=fixture(closer);next_s=later(next_s,.1)
    result=evaluate(probe,anchor,next_s,h,next_p,next_ids)
    point=result['mixed_clearance']['evidence'][0]
    assert point['occupied'] and not point['observed'] and not point['accounted']
    assert not result['matched']


@pytest.mark.parametrize('fault',['expired','epoch','same_source','lost_point','nonrigid'])
def test_recent_scene_requires_same_fresh_bounded_complete_geometry(fault):
    _,anchor,s,h,p,ids=fixture();old_points=anchor.retained.copy();retained=old_points.copy()
    now=later(s,.1)
    if fault=='expired':now=later(s,.2001)
    if fault=='epoch':now=changed_epoch(now)
    if fault=='same_source':now=s
    if fault=='lost_point':retained=retained[:1]
    if fault=='nonrigid':retained[0,0]+=.001
    assert aligned_previous((s,old_points),now,retained,h) is None


def test_alignment_preserves_unknowns_and_original_ray_timestamps():
    _,a,s,h,p,ids=fixture(gap(.42));retained=a.retained.copy()
    now=later(s,.1);transformed=aligned_previous((s,retained),now,retained+[.01,-.02],h)
    assert transformed.t_ref_s==now.t_ref_s and transformed.source_first_s==s.source_first_s
    assert [r.acquired_at_s for r in transformed.rays]==[r.acquired_at_s for r in s.rays]
    assert [r.status for r in transformed.rays]==[r.status for r in s.rays]
    np.testing.assert_allclose(transformed.rays[0].ray_origin_xy_m,np.array(s.rays[0].ray_origin_xy_m)+[.01,-.02])


@pytest.mark.parametrize('fault',['ended','invalid_anchor','epoch','time_reverse','expired'])
def test_failed_new_state_cannot_use_prior_confirmation(fault):
    probe,anchor,s,h,p,ids=fixture();evaluate(probe,anchor,s,h,p,ids)
    now=later(s,.1);current_anchor(anchor,now,p)
    if fault=='ended':anchor.ended=True
    if fault=='invalid_anchor':anchor.anchor=(p.copy(),s.t_ref_s,s.source_first_s)
    if fault=='epoch':now=changed_epoch(now)
    if fault=='time_reverse':now=later(s,-.11)
    if fault=='expired':now=later(s,1.01)
    result=probe.observe(now,h,p,ids)
    assert not result['matched'] and probe.mixed_count==0 and probe.mixed_previous is None


def test_deepcopy_and_reset_have_independent_recent_evidence():
    probe,anchor,s,h,p,ids=fixture();evaluate(probe,anchor,s,h,p,ids)
    copied_anchor,copied_probe=deepcopy((anchor,probe))
    assert copied_probe.rolling_anchor is copied_anchor
    assert copied_probe.mixed_previous[1] is not probe.mixed_previous[1]
    copied_probe.reset()
    assert copied_probe.mixed_previous is None and probe.mixed_previous is not None


def test_policy_uses_candidate_without_modifying_default_class():
    from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
    a=MixedRecoveryCandidate(True,True,True);b=RecoveryPolicy(True,True,True)
    assert isinstance(a.clearance_probe,MixedClearanceCandidate)
    assert not isinstance(b.clearance_probe,MixedClearanceCandidate)
