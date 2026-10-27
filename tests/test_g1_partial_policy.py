"""G1의 경로 후보와 부분 관측 허가를 연결하되 현재 장애물/기억을 보존한다."""
from dataclasses import replace
import math
import numpy as np
import pytest
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import scan,history
from arena_autonomy.follow_trace import FollowTrace,replay_events,comparable_decision,equal_output
import json


def corridor(first,missing=(),obstacle=None):
    raw=scan(first)
    angles=raw.angle_min+np.arange(500)*raw.angle_increment
    values=[]
    for a in angles:
        c,s=math.cos(a),math.sin(a)
        value=min(.425/max(abs(s),1e-12),8.)
        if obstacle is not None and c>0:
            distance=(obstacle-.06)/c
            if abs(distance*s)<=.075:value=min(value,distance)
        if abs(a)>math.radians(150)+1e-12:value=math.inf
        values.append(value)
    raw.ranges=values
    for i in missing:raw.ranges[i]=math.inf
    return raw


def call(ctl,raw,h=None,speed=0.,side='left'):
    h=h or history()
    now=raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9+.1
    scene=motion_scene(raw,h,now,timing_verified=True)
    return ctl.command(scene,h,speed,side,raw_scan=raw)


def warmed(policy='bounded_local_v2'):
    ctl=LocalFollow(FollowConfig(partial_observation_policy=policy))
    for first in (.3,.4,.5,.6):out=call(ctl,corridor(first))
    assert out[0]>0
    return ctl


def test_strict_keeps_single_missing_stop_but_explicit_candidate_caps_progress():
    old=LocalFollow();new=warmed()
    raw=corridor(.7,[375])
    assert call(old,raw)[2]['reason']=='follow_front_unobserved'
    out=call(new,raw)
    assert 0<out[0]<=.5
    assert out[2]['partial_observation']['allowed']
    assert not out[2]['partial_observation']['unknown_filled']
    assert not math.isfinite(raw.ranges[375])


@pytest.mark.parametrize('indices',[range(300,310),range(500),range(249,252)])
def test_wide_or_front_excessive_gaps_remain_hold(indices):
    out=call(warmed(),corridor(.7,indices))
    assert out[0]==0 and out[2]['behavior']=='HOLD'


@pytest.mark.parametrize('policy',['bounded_local_v2','bounded_local_v3','bounded_local_v4'])
def test_front_obstacle_is_kept_when_side_gap_is_permitted(policy):
    out=call(warmed(policy),corridor(.7,[375],obstacle=.25),speed=.4)
    assert out[0]==0
    assert out[2]['unfiltered_occupancy']


def test_narrow_front_gap_has_lower_speed_and_under_range_stays_blocked():
    ctl=warmed()
    out=call(ctl,corridor(.7,[250]))
    assert 0<out[0]<=.2
    raw=corridor(.8);raw.ranges[375]=.01
    out=call(ctl,raw)
    assert out[0]==0 and out[2]['partial_observation']['reason']=='invalid_near_return'


@pytest.mark.parametrize('policy',['bounded_local_v2','bounded_local_v3','bounded_local_v4'])
def test_new_policy_trace_replays_its_raw_scan_and_state(tmp_path,policy):
    h=history();ctl=LocalFollow(FollowConfig(partial_observation_policy=policy))
    path=tmp_path/'new_policy.jsonl'
    trace=FollowTrace(str(path),h.config,ctl.config)
    for first in (.3,.4,.5,.6,.7,.8):
        raw=corridor(first,[375] if first>=.7 else [])
        now=raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9+.1
        out=call(ctl,raw,h)
        trace.record_call(raw,h,now,0,0.,'left',0.,.02,True,30.,out)
    trace.close()
    result=replay_events(json.loads(line) for line in path.read_text().splitlines())
    assert result['passed'] and result['calls']==6


def test_path_or_preferred_side_reset_does_not_erase_observation_latch():
    ctl=warmed();ctl.partial_policy._latch('test_known_lost',.65)
    out=call(ctl,corridor(.7,[375]),side='right')
    assert out[0]==0 and ctl.partial_policy.latched
    assert out[2]['partial_observation']['latch_reason']=='test_known_lost'
    ctl.reset()
    assert not ctl.partial_policy.latched


def test_raw_scan_required_and_source_pair_must_match():
    ctl=warmed();h=history();raw=corridor(.7,[375]);scene=motion_scene(raw,h,.8,timing_verified=True)
    assert ctl.command(scene,h,0.,'left')[2]['reason']=='follow_partial_scan_missing'
    assert ctl.command(scene,h,0.,'left',raw_scan=corridor(.6))[2]['reason']=='follow_partial_scan_scene_mismatch'


def test_motion_transport_cannot_import_uncovered_raw_gap_into_front():
    ctl=warmed();h=history(w=1.)
    raw=corridor(.7,[390])  # 약100.8도, 취득 후 회전으로 전방 창에 들어온다.
    out=call(ctl,raw,h)
    assert out[0]==0
    assert out[2]['reason']=='follow_partial_transformed_gap_uncovered'


@pytest.mark.parametrize('change',[{'max_speed_mps':1.1},{'partial_observation_policy':'bounded_side_v1'},
                                  {'recent_observation_enabled':True}])
def test_unsupported_trial_combinations_rejected(change):
    with pytest.raises(ValueError):replace(FollowConfig(partial_observation_policy='bounded_local_v2'),**change)


def test_v3_is_explicit_and_does_not_replace_current_occupancy_or_path_gate():
    from test_partial_surface_alignment import observed_room
    raw,points,h=observed_room()
    angles=raw.angle_min+np.arange(len(raw.ranges))*raw.angle_increment
    for index in np.flatnonzero(np.abs(angles)>math.radians(150)+1e-12):
        raw.ranges[index]=math.inf
    scene=motion_scene(raw,h,.6,timing_verified=True)
    points=np.asarray(scene.occupied_endpoints_xy_m)
    ctl=LocalFollow(FollowConfig(partial_observation_policy='bounded_local_v3'))
    assert ctl.partial_policy.surface_translation_enabled
    assert not warmed().partial_policy.surface_translation_enabled
    ctl.partial_epoch=0
    ctl.partial_policy._remember(points-np.array([.03,.025]),.5,.4,True)
    out=ctl.command(scene,h,1.,'left',.37,.02,raw_scan=raw)
    assert 'partial_observation' in out[2],out[2]
    assert out[2]['partial_observation']['surface_correspondence']['accepted']
    # 표면 대응은 경로 경계가 확인되지 않은 후보를 양수로 만들지 않는다.
    assert out[0]==0 and out[2]['reason']=='follow_boundary_unconfirmed'
    assert not out[2]['partial_observation']['unknown_filled']


def test_replay_ignores_only_surface_wall_clock_cost_not_geometry_or_permission():
    import copy
    out=[.2,0.,{'partial_observation':{'allowed':True,'surface_correspondence':{
        'processing_ms':3.,'accepted':True,'translation_m':[.01,.02]}}}]
    different=copy.deepcopy(out);different[2]['partial_observation']['surface_correspondence']['processing_ms']=8.
    assert equal_output(comparable_decision(out),comparable_decision(different))
    assert out[2]['partial_observation']['surface_correspondence']['processing_ms']==3.
    different[2]['partial_observation']['surface_correspondence']['translation_m'][0]=.011
    assert not equal_output(comparable_decision(out),comparable_decision(different))
    different=copy.deepcopy(out);different[2]['partial_observation']['allowed']=False
    assert not equal_output(comparable_decision(out),comparable_decision(different))
    different=copy.deepcopy(out);different[2]['partial_observation']['surface_correspondence']['processing_ms']=-1.
    with pytest.raises(ValueError):comparable_decision(different)


def test_v4_lateral_gap_is_capped_and_real_obstacle_remains_hold():
    ctl=warmed('bounded_local_v4')
    raw=corridor(.7,range(361,366))
    raw.ranges[360]=3.;raw.ranges[366]=3.
    # 벽 한가운데 임의로 구멍을 만든 것이므로 이전 벽 소실로 거절되는 것도 유지한다.
    result=call(ctl,raw,speed=.5)
    assert result[0]<=.2
    assert result[2]['partial_observation_policy']=='bounded_local_v4'
    raw=corridor(.8,range(361,366),obstacle=.25)
    raw.ranges[360]=3.;raw.ranges[366]=3.
    result=call(ctl,raw,speed=.5)
    assert result[0]==0 and result[2]['unfiltered_occupancy']
