"""정합은 대응 가설일 뿐이다. 사라진 독립 물체와 퇴화 관측을 따로 검사한다."""
import json
from pathlib import Path
import math
import numpy as np
import pytest
from arena_autonomy.partial_surface_alignment import surface_translation
from arena_autonomy.partial_observation import PartialObservationPolicy
from test_local_follow import history,scan


def room():
    x=np.linspace(-.1,1.5,81);y=np.linspace(-.425,.425,51)
    points=np.concatenate([np.column_stack((x,x*0+.425)),np.column_stack((x,x*0-.425)),
                           np.column_stack((y*0+1.5,y))])
    return points,np.r_[np.arange(81),np.arange(100,181),np.arange(200,251)]


def test_common_small_motion_residual_is_recovered_without_modifying_input():
    current,ids=room();shift=np.array([.025,.012]);old=current-shift
    before=old.copy();result=surface_translation(old,current,ids)
    assert result['accepted']
    np.testing.assert_allclose(result['translation_m'],shift,atol=1e-5)
    np.testing.assert_array_equal(old,before)
    assert result['scope']=='correspondence_only_not_independent_motion'
    assert result['static_background_assumption']


@pytest.mark.parametrize('fault',['parallel','large_shift','few','missing_between_every_ray','one_side'])
def test_ambiguous_or_unsupported_geometry_cannot_authorize_correspondence(fault):
    current,ids=room();old=current-np.array([.025,.012])
    if fault=='parallel':old,current,ids=old[:162],current[:162],ids[:162]
    if fault=='large_shift':old=current-np.array([.055,.055])
    if fault=='few':old,current,ids=old[:20],current[:20],ids[:20]
    if fault=='missing_between_every_ray':ids=ids*2
    if fault=='one_side':old,current,ids=old[:81],current[:81],ids[:81]
    assert not surface_translation(old,current,ids)['accepted']


def test_disappeared_independent_object_does_not_become_a_surface_match():
    current,ids=room();shift=np.array([.025,.012])
    car=np.column_stack((np.full(12,.35),np.linspace(-.07,.07,12)))
    fit=surface_translation(np.concatenate([current,car])-shift,current,ids)
    assert fit['accepted']
    corrected=car-shift+fit['translation_m']
    assert np.min(np.linalg.norm(corrected[:,None]-current[None,:],axis=2))>.3


def test_recorded_braking_failure_has_shared_surface_offset_without_truth_input():
    path=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-10-27/marker_visibility_v1/run5_latch_context.json'
    data=json.loads(path.read_text(encoding='utf-8'))['evidence']
    context=data['correspondence_context']
    current=np.array(context['current_return_points_xy'])
    fit=surface_translation(context['previous_points_now_xy'],current,context['current_return_source_indices'])
    assert fit['accepted']
    assert .015<np.linalg.norm(fit['translation_m'])<.03
    for witness in data['witnesses']:
        moved=np.array(witness['previous_point_now_xy'])+fit['translation_m']
        assert np.min(np.linalg.norm(current-moved,axis=1))<.02


def observed_room(first=.5):
    raw=scan(first);angle=raw.angle_min+np.arange(500)*raw.angle_increment
    ranges=[]
    for a in angle:
        c,s=math.cos(a),math.sin(a)
        distances=[(1.5-.06)/c if c>1e-8 else (-.5-.06)/c if c< -1e-8 else math.inf,
                   .425/abs(s) if abs(s)>1e-8 else math.inf]
        ranges.append(min(distances))
    raw.ranges=ranges
    for i in (344,345,346):raw.ranges[i]=math.inf
    h=history()
    points,_,_=h.project(raw.ranges,raw.angle_min,raw.angle_increment,raw.range_min,raw.range_max,
                         first,raw.time_increment,first+.1,'both',True)
    return raw,points,h


def test_policy_keeps_independent_lost_obstacle_after_common_surface_correspondence():
    raw,current,h=observed_room()
    p=PartialObservationPolicy(True,True)
    old=np.vstack([current-np.array([.03,.025]),[.06,.15]])
    p._remember(old,.5,.4,True)
    result=p.evaluate(raw,current,h,.6,1.,.37,.37)
    assert result['surface_correspondence']['accepted'],result['surface_correspondence']
    assert result['reason']=='known_obstacle_latched'
    assert result['latch_evidence']['pairs']>0


def test_policy_reuses_fit_for_same_scan_pair_without_changing_current_points():
    raw,current,h=observed_room()
    p=PartialObservationPolicy(True,True)
    p._remember(current-np.array([.03,.025]),.5,.4,True)
    original=current.copy()
    first=p.evaluate(raw,current,h,.6,1.,.37,.37)
    second=p.evaluate(raw,current,h,.62,1.,.37,.37)
    assert first['allowed'] and second['allowed']
    assert first['surface_correspondence']['accepted'],first['surface_correspondence']
    assert not first['surface_correspondence']['cached']
    assert second['surface_correspondence']['cached']
    np.testing.assert_array_equal(current,original)
