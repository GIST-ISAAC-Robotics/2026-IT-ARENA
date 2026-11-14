"""우회 목표선이 막혔을 때 현재 센서로만 조기 복귀하는 후보의 회귀."""
from dataclasses import replace
from unittest.mock import patch
import numpy as np
import pytest

from arena_autonomy.bypass_observed_return import ObservedEntryReturn
from arena_autonomy.bypass_entry_trial import EntryGuide, entry_path, _compare_entries, current_outer_wall_padding
from probe_bypass_entry import sensor_scene


def subject():
    guide = EntryGuide(stationary_outer_boundary=True, rejoin_enabled=True)
    guide.config = replace(guide.config,piecewise_wall_trial=True)
    guide.rear_wall_witness_enabled = True
    guide.wall_confirmations = 2
    guide.side, guide.offset, guide.transition = 'right', -.24, .9
    guide.padding_provider = current_outer_wall_padding
    return guide, ObservedEntryReturn()


def observed(guide, *, now=.6, targets=((.8,-.24),), missing=()):
    scene,_ = sensor_scene(targets=targets, now=now, first=now-.1, missing=missing)
    wall = dict(slope=0., intercept_m=0., observed_x_m=[-.15,1.5])
    guide.path = entry_path((0.,0.,-.15,1.5),-.24,guide.config,guide.entry,.9)
    ref = dict(reason='current_outer_wall_rechecked_for_remembered_guide', walls={'right':wall})
    out = _compare_entries(scene, now, 0., 0., .02, config=guide.config, entry=guide.entry,
        padding_provider=guide.padding_provider,
        guide=dict(reference=ref,path=guide.path,offset_m=-.24,transition_m=.9,
                   outer_side='right',stationary_outer_boundary=True))
    return scene,out


def test_current_ray_geometry_allows_better_center_after_distinct_observations():
    guide,choice=subject()
    scene,primary=observed(guide)
    path=guide.path.copy(); rays=scene.rays
    first=choice.evaluate(guide,scene,0.,0.,.02,primary)
    assert first['observed_entry_return']['confirmations']==1
    assert guide.phase=='entry'
    np.testing.assert_array_equal(path,guide.path)
    assert choice.evaluate(guide,scene,0.,0.,.02,primary) is primary
    scene,primary=observed(guide,now=.7)
    out=choice.evaluate(guide,scene,0.,0.,.02,primary)
    assert out['observed_entry_return']['applied'] and out['selected']['speed_mps']>0
    assert guide.phase=='rejoin' and guide.offset==0 and out['entry_phase']=='rejoin'
    assert not out['control_authorized'] and not out['unknown_filled']
    assert rays[0].acquired_at_s==pytest.approx(.5)  # 입력 시각을 갱신하지 않는다.


@pytest.mark.parametrize('kind',['legacy','witness_off','rejoin_off','reverse','stale','phase','no_wall','reverse_wait'])
def test_unavailable_evidence_and_legacy_do_not_change_route(kind):
    guide,choice=subject();scene,primary=observed(guide);speed=0.
    if kind=='legacy':guide.config=replace(guide.config,piecewise_wall_trial=False)
    elif kind=='witness_off':guide.rear_wall_witness_enabled=False
    elif kind=='rejoin_off':guide.rejoin_enabled=False
    elif kind=='reverse':speed=-.1
    elif kind=='stale':scene=replace(scene,t_ref_s=1.)
    elif kind=='phase':guide.phase='rejoin'
    elif kind=='no_wall':guide.wall_confirmations=0
    elif kind=='reverse_wait':guide.reverse_wait=True
    phase=guide.phase;path=guide.path.copy()
    assert choice.evaluate(guide,scene,speed,0.,.02,primary) is primary
    assert guide.phase==phase
    np.testing.assert_array_equal(guide.path,path)


@pytest.mark.parametrize('kind',['occupied_inner_band','unknown_front','all_comparisons_reject'])
def test_existing_occupancy_and_unknown_checks_are_not_overridden(kind):
    guide,choice=subject()
    for now in (.6,.7):
        targets=((.8,-.24),(.18,.1)) if kind=='occupied_inner_band' else ((.8,-.24),)
        missing=tuple(range(245,300)) if kind=='unknown_front' else ()
        scene,primary=observed(guide,now=now,targets=targets,missing=missing)
        if kind=='all_comparisons_reject':
            with patch('arena_autonomy.bypass_observed_return._compare_entries',return_value=dict(
                    candidates=[dict(reason='large_motion_region_unobserved')],selected=None)):
                result=choice.evaluate(guide,scene,0.,0.,.02,primary)
        else:result=choice.evaluate(guide,scene,0.,0.,.02,primary)
        assert not result.get('observed_entry_return',{}).get('applied',False)
        assert guide.phase=='entry'


def test_scene_gap_and_failed_new_scan_clear_confirmation():
    guide,choice=subject();scene,p=observed(guide)
    choice.evaluate(guide,scene,0.,0.,.02,p)
    scene,p=observed(guide,now=.7,targets=((.8,-.24),(.18,.1)))
    choice.evaluate(guide,scene,0.,0.,.02,p)
    assert choice.confirmations==0
    for t in (.8,2.):
        scene,p=observed(guide,now=t)
        result=choice.evaluate(guide,scene,0.,0.,.02,p)
        assert result['observed_entry_return']['confirmations']==1 and guide.phase=='entry'


def test_complete_clear_current_path_does_not_start_new_comparisons():
    guide,choice=subject();scene,p=observed(guide,targets=())
    # 외벽에 닿지 않는 기존 중앙 경로는 간섭 후보 비교 대상이 아니다.
    row=p['candidates'][0]
    row['preview_return_progress_m']=row['path_points'][-1][0]-row['path_points'][0][0]
    with patch('arena_autonomy.bypass_observed_return._compare_entries') as cmp:
        assert choice.evaluate(guide,scene,0.,0.,.02,p) is p
        cmp.assert_not_called()


def test_partial_transition_cannot_be_remembered_as_completed_return():
    guide,choice=subject()
    for t in (.6,.7):
        scene,p=observed(guide,now=t)
        p['reference']['walls']['right']['observed_x_m']=[-.15,.4]
        with patch('arena_autonomy.bypass_observed_return._compare_entries') as cmp:
            out=choice.evaluate(guide,scene,0.,0.,.02,p)
            assert not out.get('observed_entry_return',{}).get('applied',False)
            assert guide.phase=='entry'
            cmp.assert_not_called()
