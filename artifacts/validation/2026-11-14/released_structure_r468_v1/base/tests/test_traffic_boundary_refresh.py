"""현재의 짧은 벽 갱신. 실제 센서/차량의 성능 증명은 아니다."""
from dataclasses import replace
import numpy as np
import pytest
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.local_scene import LocalScene, SensorRay
from test_local_follow import history, scene
from test_boundary_bridge import short_scene

CONFIG = dict(partial_observation_policy='bounded_local_v4',
              boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate',
              corridor_fallback_enabled=True, parallel_hold_trial=True, pass_release_trial=True,
              guard_curve_trial=True, traffic_recovery_trial=True, traffic_path_recovery_trial=True)


def controller(on=True):
    return LocalFollow(FollowConfig(**CONFIG, traffic_boundary_refresh_trial=on))


def baseline(c, s, h=None, side='left'):
    return c._baseline_command(s, history() if h is None else h, .35, side)


def settle(c,h):
    for t in (.4,.5,.6,.7,.8):
        result=baseline(c,scene(t),h)
    assert result[0]>0


def test_requires_explicit_traffic_profile():
    assert FollowConfig().traffic_boundary_refresh_trial is False
    for changes in ({'solo_route_only':True}, {'traffic_path_recovery_trial':False}, {'traffic_boundary_refresh_trial':1}):
        with pytest.raises(ValueError):
            FollowConfig(**{**CONFIG, 'traffic_boundary_refresh_trial':True, **changes})


def test_short_current_wall_alone_no_longer_refreshes():
    # r29의 짧은 벽 연속 갱신은 실주행 개선이 없어 r30에서 제외했다.
    for c in (controller(False),controller()):
        settle(c,history())
        for t in (.9,1.,1.1):
            result=baseline(c,short_scene(t))
        assert result[0]==0 and result[2]['reason']=='follow_boundary_unconfirmed'


@pytest.mark.parametrize('case', ['no_memory','expired','disagrees','anchor','epoch','side'])
def test_current_short_wall_cannot_start_or_replace_unrelated_memory(case):
    c=controller(); h=history()
    if case!='no_memory':settle(c,h)
    if case=='expired':c.path_source=.5
    s=short_scene(.9,offset=.11 if case=='disagrees' else .2 if case=='anchor' else 0.)
    if case=='epoch':s=replace(s,source_epoch=1,rays=tuple(replace(r,source_epoch=1) for r in s.rays))
    side='right' if case=='side' else 'left'
    first=baseline(c,s,h,side)
    assert first[2].get('boundary_selection')!='short_current_history_agreement'
    for t in (1.,1.1):
        result=baseline(c,short_scene(t,offset=.11 if case=='disagrees' else .2 if case=='anchor' else 0.),h,side)
    assert result[0]==0


def test_refresh_does_not_exempt_large_motion_or_stale_scene():
    c=controller();settle(c,history())
    c.path=np.column_stack((np.linspace(-.05,2.75,36),np.zeros(36)))
    c.path_ref,c.path_source=.8,.7
    assert baseline(c,short_scene(.92),history(0.,1.8))[0]==0
    c=controller();settle(c,history())
    result=baseline(c,scene(1.2,source=.7))
    assert result[0]==0 and result[2]['reason']=='follow_scene_stale'


def test_occupied_and_unobserved_input_still_stop():
    c=controller();settle(c,history())
    current=short_scene(.9)
    rays=list(current.rays)
    for j,y in enumerate(np.linspace(-.075,.075,20)):
        point=(.10,float(y));d=np.array(point)-(.06,0.);d/=np.linalg.norm(d)
        rays.append(SensorRay((.06,0.),point,.8,'return','laser_frame',0,1000+j,tuple(d)))
    blocked=replace(current,rays=tuple(rays))
    result=baseline(c,blocked)
    assert result[0]==0 and result[2]['reason']=='follow_gap_hold'
    c=controller();settle(c,history())
    missing=replace(current,rays=tuple(replace(r,status='unknown',hit_xy_m=None) for r in current.rays))
    result=baseline(c,missing)
    assert result[0]==0 and result[2]['reason']=='follow_front_unobserved'


def test_path_only_hold_keeps_existing_release_wait():
    c=controller();settle(c,history())
    c.held,c.hold_reason,c.release_count=True,'follow_boundary_unconfirmed',0
    result=baseline(c,short_scene(.9))
    assert result[0]==0 and result[2]['reason']=='follow_release_wait'


def test_explicit_launch_and_header_agree(tmp_path):
    import json
    from pathlib import Path
    import g1_handoff_eval as ev
    root=Path(__file__).resolve().parents[1]
    launch=json.loads((root/'artifacts/validation/2026-11-01/race_follow_pair_v1/stop_restart.json').read_bytes())['launch']
    off=ev.launch_arguments('g1g2','paced',launch,True)
    on=ev.launch_arguments('g1g2','paced',launch,True,boundary_refresh=True)
    assert on==['g1_traffic_boundary_refresh_trial:=true']+off
    with pytest.raises(ValueError):ev.launch_arguments('g1',None,launch,True,boundary_refresh=True)
    with pytest.raises(ValueError):ev.launch_arguments('g1g2','paced',launch,False,boundary_refresh=True)
    for actual in (False,True):
        p=tmp_path/'header.jsonl'
        p.write_text(json.dumps(dict(kind='header',data=dict(controller_mode='g1_g2_handoff_trial',
                     follow_config={'traffic_boundary_refresh_trial':actual},execution_config={})))+'\n')
        for requested in (False,True):
            assert ev.inspect_header(p,'paced',boundary_refresh=requested)['checks']['boundary_refresh']==(actual==requested)
