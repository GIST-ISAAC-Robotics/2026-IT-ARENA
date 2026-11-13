"""원 저장 국소 비교와 명시 합성 점유 반례. 실제 주행/전체 추적 재생은 아니다."""
from dataclasses import replace
import importlib.util
import math
import sys
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest
import validate_stage as probe
from arena_autonomy.bypass_entry_trial import _compare_entries,entry_path
from arena_autonomy.follow_trace import equal_output
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig


def apply(guide,d,history,scene,result):
    return guide._current_follow_return(scene,history,d['speed_mps'],
        d['previous_steering_rad'],d['dt_s'],result)


def visible_box(scene, center, yaw=0., size=(.2,.15)):
    # 평가용 합성 직사각형을 현재 광선에 ray cast. 원 unknown/mask는 바꾸지 않는다.
    c,s=math.cos(yaw),math.sin(yaw)
    rot=np.array([[c,s],[-s,c]]);half=np.asarray(size)/2;rays=[];changed=0
    for ray in scene.rays:
        if ray.status!='return':rays.append(ray);continue
        origin=rot@(np.asarray(ray.ray_origin_xy_m)-center)
        direction=rot@np.asarray(ray.direction_xy);low,high=0.,math.inf
        for axis in range(2):
            if abs(direction[axis])<1e-12:
                if abs(origin[axis])>half[axis]:low=math.inf;break
            else:
                a,b=sorted(((-half[axis]-origin[axis])/direction[axis],(half[axis]-origin[axis])/direction[axis]))
                low=max(low,a);high=min(high,b)
        old=np.linalg.norm(np.asarray(ray.hit_xy_m)-ray.ray_origin_xy_m)
        if 0.<low<=high and low<old:
            point=np.asarray(ray.ray_origin_xy_m)+low*np.asarray(ray.direction_xy)
            ray=replace(ray,hit_xy_m=tuple(map(float,point)));changed+=1
        rays.append(ray)
    assert changed>0
    return replace(scene,rays=tuple(rays))


def test_actual_speed_kept_and_headway_applied_before_region():
    guide,row,d,scan,history,scene,result=probe.at_entry(524)
    out=apply(guide,d,history,scene,result)
    assert d['speed_mps']==pytest.approx(.9183756302160638)
    assert out['selected']['speed_mps']==pytest.approx(.35)
    candidate=out['candidates'][0]
    assert candidate['speed_policy']=='follow_reaction_braking_and_headway'
    assert candidate['motion_region']['distance_m']>.4
    assert guide.phase=='rejoin' and guide.returning_behind and guide.follow_headway
    evidence=out['current_follow_return']
    assert evidence['target_rear_bound_m']>evidence['required_target_rear_bound_m']
    assert abs(out['selected']['steering_rad']-d['previous_steering_rad'])<=3*d['dt_s']


def test_next_actual_input_keeps_rejoin_path_and_follow_headway():
    guide,row,d,scan,history,scene,result=probe.at_entry(524)
    apply(guide,d,history,scene,result)
    row,d,scan,history,scene=probe.captured(527)
    probe.observed_tracker(guide,row,scene);guide.raw_scan=scan
    out=guide.evaluate(scene,history,d['speed_mps'],d['previous_steering_rad'],d['dt_s'])
    assert out['selected']['speed_mps']==pytest.approx(.35)
    assert guide.phase=='rejoin' and guide.follow_headway
    assert out['candidates'][0]['speed_policy']=='follow_reaction_braking_and_headway'


@pytest.mark.parametrize('seq',[560,563])
def test_late_input_cannot_be_forced_positive(seq):
    guide,row,d,scan,history,scene,result=probe.at_entry(seq)
    out=apply(guide,d,history,scene,result)
    assert out is result and guide.phase=='entry' and not guide.follow_headway


@pytest.mark.parametrize('kind',['missing','alongside'])
def test_current_target_not_ahead_cannot_start_return(kind):
    guide,row,d,scan,history,scene,result=probe.at_entry(524)
    if kind=='missing':guide.tracker.current=None
    else:guide.tracker.current=np.array([[.1,-.25],[.2,-.25]])
    assert apply(guide,d,history,scene,result) is result


@pytest.mark.parametrize('center,yaw',[((.22,0.),0.),((.30,-.10),.65),((.25,-.16),0.)])
def test_front_diagonal_and_near_side_returns_reject_new_return(center,yaw):
    guide,row,d,scan,history,scene,result=probe.at_entry(524)
    changed=visible_box(scene,np.asarray(center),yaw)
    assert apply(guide,d,history,changed,result) is result
    assert guide.phase=='entry' and not guide.follow_headway


def test_side_return_outside_return_path_does_not_erase_current_free_path():
    guide,row,d,scan,history,scene,result=probe.at_entry(524)
    changed=visible_box(scene,np.array([-.45,.30]),0.)
    out=apply(guide,d,history,changed,result)
    assert out is not result and out['selected']['speed_mps']>0.


def test_published_proposal_contains_only_new_center_path():
    guide,row,d,scan,history,scene,result=probe.at_entry(524)
    out=apply(guide,d,history,scene,result)
    ctrl=BypassHandoffFollow(probe.CFG,HandoffExecutionConfig.from_mapping(probe.HEADER['execution_config']),
        allow_simulation_trial=True)
    ctrl.moving_pass.guide=guide;ctrl.moving_pass.tracker=guide.tracker
    output=ctrl.moving_pass._output(out['selected'],out,'moving_pass_entry')
    published=output[2]['moving_pass']['entry']
    assert published['selected']['offset_m']==0.
    assert len(published['candidates'])==1
    assert published['candidates'][0]['offset_m']==0.
    assert np.array_equal(published['candidates'][0]['path_points'],guide.path)
    assert published['reference']==result['reference']  # 현재 재확인 벽은 동일, 경로만 중심으로 변경.
    assert not np.array_equal(result['candidates'][0]['path_points'],guide.path)
    from arena_autonomy.current_path_recheck import scope
    assert scope(output) is None  # 기존 MOVING_PASS는 이 current 재검사 처리 범위 밖이다.


def test_epoch_change_resets_return_state():
    guide,row,d,scan,history,scene,result=probe.at_entry(524)
    apply(guide,d,history,scene,result)
    changed=replace(scene,source_epoch=1,rays=tuple(replace(r,source_epoch=1) for r in scene.rays))
    out=guide.evaluate(changed,history,d['speed_mps'],d['previous_steering_rad'],d['dt_s'])
    assert out['selected'] is None and not guide.follow_headway and not guide.returning_behind


def test_current_front_path_loss_keeps_return_stopped():
    guide,row,d,scan,history,scene,result=probe.at_entry(524)
    apply(guide,d,history,scene,result)
    row,d,scan,history,scene=probe.captured(527)
    changed=replace(scene,rays=tuple(replace(r,status='unknown',hit_xy_m=None)
        if r.status=='return' else r for r in scene.rays))
    out=guide.evaluate(changed,history,d['speed_mps'],d['previous_steering_rad'],d['dt_s'])
    assert out['selected'] is None


def test_reapproaching_lead_blocks_an_already_started_return():
    guide,row,d,scan,history,scene,result=probe.at_entry(524)
    apply(guide,d,history,scene,result)
    row,d,scan,history,scene=probe.captured(527)
    changed=visible_box(scene,np.array([.22,0.]),0.)
    guide.tracker.current=np.array([[.12,-.075],[.12,.075]])
    guide.tracker.detail['observed']=True
    out=guide.evaluate(changed,history,d['speed_mps'],d['previous_steering_rad'],d['dt_s'])
    assert out['selected'] is None


@pytest.mark.parametrize('prepared_speed',[0.,.4])
def test_centered_handoff_keeps_prepared_follow_state_and_its_stop(prepared_speed):
    # 기존 test_moving_pass의 같은 인계 회귀를 ROS fixture import 없이 수행한다.
    ctrl=BypassHandoffFollow(probe.CFG,HandoffExecutionConfig.from_mapping(probe.HEADER['execution_config']),
        allow_simulation_trial=True)
    adapter=ctrl.moving_pass
    row,d,scan,history,scene=probe.captured(524)
    adapter.active=True;adapter.tracker.epoch=scene.source_epoch
    adapter.guide.returning_behind=True;adapter.guide.follow_headway=True
    prepared=SimpleNamespace();monitor=SimpleNamespace()
    adapter.handoff_follow=prepared;adapter.handoff_monitor=monitor
    normal=(prepared_speed,.01,dict(reason='handoff_follow',normal=dict(reason='follow_gap_hold')))
    with patch.object(adapter.tracker,'update',return_value={}), \
         patch.object(adapter.guide,'evaluate',return_value=dict(entry_phase='centered',
             selected=dict(speed_mps=.65,steering_rad=.02))), \
         patch.object(adapter,'_prepare_follow_handoff'), \
         patch.object(ctrl,'_base_command',return_value=normal):
        out=adapter.command(scene,history,d['speed_mps'],d['side'],d['previous_steering_rad'],d['dt_s'],raw_scan=scan)
    assert ctrl.normal is prepared and ctrl.monitor is monitor
    assert out[:2]==normal[:2]
    assert out[2]['moving_pass']['reason']=='aborted_to_observed_follow'
    assert not adapter.active and not adapter.guide.follow_headway


def test_invalid_input_cannot_bypass_reset():
    ctrl=BypassHandoffFollow(probe.CFG,HandoffExecutionConfig.from_mapping(probe.HEADER['execution_config']),
        allow_simulation_trial=True)
    ctrl.moving_pass.active=True;ctrl.moving_pass.guide.follow_headway=True
    row,d,scan,history,scene=probe.captured(524)
    out=ctrl.command(scene,history,d['speed_mps'],d['side'],d['previous_steering_rad'],d['dt_s'],raw_scan=None)
    assert out[0]==0. and not ctrl.moving_pass.active and not ctrl.moving_pass.guide.follow_headway


@pytest.mark.parametrize('seq',[524,563])
def test_default_entry_comparison_output_preserved(seq):
    path=probe.FROZEN/'src/arena_autonomy/arena_autonomy/bypass_entry_trial.py'
    spec=importlib.util.spec_from_file_location('r486_original_entry',path)
    base=importlib.util.module_from_spec(spec);sys.modules[spec.name]=base;spec.loader.exec_module(base)
    guide,row,d,scan,history,scene,result=probe.at_entry(seq)
    reference=result['reference'];wall=reference['walls']['left']
    fit=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
    kwargs=dict(config=guide.config,entry=guide.entry,guide=dict(reference=reference,
        path=entry_path(fit,0.,guide.config,guide.entry,1.2),offset_m=0.,transition_m=1.2,
        outer_side='left',stationary_outer_boundary=True))
    args=(scene,scene.t_ref_s,d['speed_mps'],d['previous_steering_rad'],d['dt_s'])
    assert equal_output(base._compare_entries(*args,**kwargs),_compare_entries(*args,**kwargs))
