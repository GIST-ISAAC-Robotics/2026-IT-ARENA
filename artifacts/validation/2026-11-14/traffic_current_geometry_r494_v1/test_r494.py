"""r490 소수 원 입력과 명시 합성 점유 반례. 폐루프/실물 증거 아님."""
from dataclasses import replace
import copy,importlib.util,json,math,sys
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import common_probe as probe
from arena_autonomy.follow_trace import freeze
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig

def apply(guide,data,history,scene,original):
    result={**original,**probe.compare(guide,scene,data,original)}
    return guide._current_follow_return(scene,history,data['speed_mps'],
        data['previous_steering_rad'],data['dt_s'],result)


def visible_box(scene,center,size=(.20,.15)):
    # 현재 유효 반환에만 실제 광선-직사각형 교차를 넣는다. unknown을 채우지 않는다.
    half=np.asarray(size)/2;rays=[];hits=[]
    for ray in scene.rays:
        if ray.status!='return':rays.append(ray);continue
        origin=np.asarray(ray.ray_origin_xy_m)-center
        direction=np.asarray(ray.direction_xy);low,high=0.,math.inf
        for axis in range(2):
            if abs(direction[axis])<1e-12:
                if abs(origin[axis])>half[axis]:low=math.inf;break
            else:
                a,b=sorted(((-half[axis]-origin[axis])/direction[axis],(half[axis]-origin[axis])/direction[axis]))
                low=max(low,a);high=min(high,b)
        old=np.linalg.norm(np.asarray(ray.hit_xy_m)-ray.ray_origin_xy_m)
        if 0.<low<=high and low<old:
            point=np.asarray(ray.ray_origin_xy_m)+low*np.asarray(ray.direction_xy)
            ray=replace(ray,hit_xy_m=tuple(map(float,point)));hits.append(point)
        rays.append(ray)
    assert len(hits)>0
    return replace(scene,rays=tuple(rays)),np.asarray(hits)


def test_saved_geometry_positive_rejoin_and_real_late_stop():
    rows=[]
    for seq in (525,528,567):
        g,row,d,raw,h,scene,original=probe.at_entry(seq)
        points=scene.occupied_endpoints_xy_m
        out=apply(g,d,h,scene,original)
        assert scene.occupied_endpoints_xy_m==points
        if seq==567:
            assert out['selected'] is None
            assert out['candidates'][0]['command_clearance_m']==.04
            assert not g.returning_to_current_follow
        else:
            assert out['selected']['speed_mps']>0
            assert abs(out['selected']['steering_rad']-d['previous_steering_rad'])<=3*d['dt_s']
            assert out['candidates'][0]['motion_region']['measured_speed_mps']==d['speed_mps']
            if seq==528:
                assert out['selected']['offset_m']==0. and g.phase=='rejoin'
                assert g.returning_to_current_follow and not g.returning_behind
                assert not out['current_follow_return']['target_currently_ahead']
                follow_g=g;follow_out=out
        rows.append(dict(seq=seq,measured_speed_mps=d['speed_mps'],output=freeze(out),
            phase=g.phase,returning_behind=g.returning_behind,returning_to_current_follow=g.returning_to_current_follow))
    row,d,raw,h,scene=probe.captured(531)
    probe.observed_tracker(follow_g,row,scene);follow_g.raw_scan=raw
    out=follow_g.evaluate(scene,h,d['speed_mps'],d['previous_steering_rad'],d['dt_s'])
    assert out['selected'] is not None and out['selected']['offset_m']==0.
    assert follow_g.follow_headway and follow_g.returning_to_current_follow
    rows.append(dict(seq=531,scope='original subsequent motion; not candidate closed loop',output=freeze(out)))
    (probe.A/'saved_comparison.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')


def test_visible_front_and_alongside_objects_block_crossing_return():
    rows=[]
    for label,center in (('front',(.22,0.)),('alongside_crossing',(.20,-.15))):
        g,row,d,raw,h,scene,original=probe.at_entry(528)
        changed,hits=visible_box(scene,np.asarray(center))
        g.tracker.current=hits;g.tracker.detail['observed']=True
        proposal=probe.compare(g,changed,d,original,center=True)
        assert proposal['selected'] is None
        candidate=proposal['candidates'][0]
        assert candidate['reason']=='known_occupied_command_hold'
        out=apply(g,d,h,changed,original)
        assert not g.returning_to_current_follow
        rows.append(dict(case=label,synthetic_current_returns=len(hits),center_candidate=freeze(candidate)))
    (probe.A/'occupied_counterexamples.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')


def test_target_loss_is_not_free_and_prepared_follow_stop_survives_handoff():
    g,row,d,raw,h,scene,original=probe.at_entry(528)
    g.tracker.current=None;g.tracker.detail['observed']=False
    out=apply(g,d,h,scene,original)
    assert not g.returning_to_current_follow and g.phase=='entry'
    ctrl=BypassHandoffFollow(probe.CFG,HandoffExecutionConfig.from_mapping(probe.HEADER['execution_config']),
        allow_simulation_trial=True)
    adapter=ctrl.moving_pass;adapter.active=True;adapter.tracker.epoch=scene.source_epoch
    adapter.guide.returning_to_current_follow=True;adapter.guide.follow_headway=True
    prepared=SimpleNamespace();monitor=SimpleNamespace()
    adapter.handoff_follow=prepared;adapter.handoff_monitor=monitor
    stopped=(0.,.01,dict(reason='handoff_follow',normal=dict(reason='follow_gap_hold')))
    with patch.object(adapter.tracker,'update',return_value={}), \
         patch.object(adapter.guide,'evaluate',return_value=dict(entry_phase='centered',
             selected=dict(speed_mps=.65,steering_rad=.02))), \
         patch.object(adapter,'_prepare_follow_handoff'), \
         patch.object(ctrl,'_base_command',return_value=stopped):
        out=adapter.command(scene,h,d['speed_mps'],d['side'],d['previous_steering_rad'],d['dt_s'],raw_scan=raw)
    assert out[:2]==stopped[:2] and ctrl.normal is prepared and ctrl.monitor is monitor
    assert out[2]['moving_pass']==dict(reason='returned_to_current_follow',completed=False)
    assert not adapter.active and not adapter.guide.returning_to_current_follow


def test_default_entry_outputs_exact():
    path=probe.A/'base/src/arena_autonomy/arena_autonomy/bypass_entry_trial.py'
    spec=importlib.util.spec_from_file_location('r494_original_entry',path)
    original=importlib.util.module_from_spec(spec);sys.modules[spec.name]=original;spec.loader.exec_module(original)
    exact=[]
    for seq in (525,528,567):
        g,row,d,raw,h,scene,result=probe.at_entry(seq)
        g.config=replace(g.config,guard_observation_policy='original_v1',released_body_gap_trial=False)
        reference=result['reference']
        kwargs=dict(config=g.config,entry=g.entry,guide=dict(reference=reference,path=g.path,
            offset_m=g.offset,transition_m=g.transition,outer_side=g.side,
            stationary_outer_boundary=g.stationary_outer_boundary and g.wall_confirmations>=2),
            padding_provider=g.padding_provider)
        args=(scene,scene.t_ref_s,d['speed_mps'],d['previous_steering_rad'],d['dt_s'])
        a=original._compare_entries(*args,**kwargs);b=probe.entry._compare_entries(*args,**kwargs)
        assert freeze(a)==freeze(b)
        exact.append(dict(seq=seq,output_exact=True,numeric_tolerance=0))
    (probe.A/'default_exact.json').write_text(json.dumps(exact,indent=2),encoding='utf-8')
