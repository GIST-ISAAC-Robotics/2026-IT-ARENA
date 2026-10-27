"""시제품의 상태 계약 반례. 주입 view는 상태 분기만 검증하며 센서 인식 증명이 아니다."""
import math
from types import SimpleNamespace
import numpy as np
import pytest
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import scan, history


def config():
    return FollowConfig(partial_observation_policy='bounded_local_v4',
        boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate')


def output(speed=1., angle=0., reason='local_follow'):
    return speed, angle, {'reason': reason, 'path_points': [[0., 0.], [1., 0.]]}


def view(present=False, rear=.25):
    return {'frame':None, 'state':None, 'left':{'present':present,'points':4 if present else 0,
        'rearmost_ds_m':rear if present else None,'frontmost_ds_m':rear+.2 if present else None},
        'right':{'present':False,'points':0,'rearmost_ds_m':None,'frontmost_ds_m':None}}


@pytest.fixture
def setup(monkeypatch):
    # 늘린 측방 창(rear/unknown 해제용)은 기본으로 미관측이 남는다. 띠만으로 rear를 풀지 않는 기존 반례를 유지한다.
    state={'view':view(), 'band':{'free':True,'samples':10,'observed_free':10,'masked':0,'unobserved':0},
           'window':{'free':False,'samples':40,'observed_free':30,'masked':0,'unobserved':10}, 'window_calls':[]}
    monkeypatch.setattr(ps,'threat_view',lambda *a:state['view'])
    def band(*a):
        if len(a)>5:
            state['window_calls'].append(a[5])
            return state['window']
        return state['band']
    monkeypatch.setattr(ps,'band_check',band)
    ctl=ps.PassReleaseSelector(config())
    def call(t, *,speed=.3, previous=0., dt=.02, epoch=0, base=None, parallel=None):
        scene=SimpleNamespace(source_first_s=t-.1,t_ref_s=t,source_epoch=epoch)
        return ctl.select(scene,output() if base is None else base,
            output(.2) if parallel is None else parallel,speed,previous,dt)
    return ctl,state,call


def test_never_seen_solo_does_not_wait_for_impossible_pass(setup):
    ctl,state,call=setup
    assert call(1.)[0]==1. and ctl.mode=='SOLO'


def test_two_distinct_clear_scans_after_forward_observation_allow_release(setup):
    ctl,state,call=setup;state['view']=view(True,.25)
    assert call(1.)[0]==.2
    state['view']=view()
    assert call(1.1)[0]==.2
    assert call(1.1)[0]==.2
    assert call(1.2)[0]==1. and ctl.mode=='RELEASED'


def test_disappearance_behind_never_becomes_forward_pass(setup):
    ctl,state,call=setup;state['view']=view(True,-.2);call(1.)
    state['view']=view()
    for t in (1.1,1.2,1.3):out=call(t)
    assert ctl.sides['left']['vanish']=='rear'
    assert ctl.mode=='GUARD' and out[0]==.2


def test_second_object_passing_does_not_erase_first_unresolved_disappearance(setup):
    ctl,state,call=setup;state['view']=view(True,-.2);call(1.)
    state['view']=view();call(1.1)
    state['view']=view(True,.25);call(1.2)
    state['view']=view();call(1.3);out=call(1.4)
    assert ctl.mode=='GUARD' and out[0]==.2


def test_remaining_observed_object_prevents_release(setup):
    ctl,state,call=setup;state['view']=view(True,-.2)
    for t in (1.,1.1,1.2):out=call(t)
    assert ctl.mode=='GUARD' and out[0]==.2


def test_reapproaching_object_reenters_guard(setup):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();call(1.1);call(1.2)
    state['view']=view(True,-.15)
    assert call(1.3)[0]==.2 and ctl.mode=='GUARD'


@pytest.mark.parametrize('fault',['nan_speed','reverse','nan_previous','bad_dt','nan_baseline','reverse_parallel',
    'inf_steering','excess_baseline','excess_previous','zero_dt'])
def test_invalid_motion_or_output_must_hold_even_in_solo(setup,fault):
    ctl,state,call=setup
    kwargs={'nan_speed':{'speed':math.nan},'reverse':{'speed':-.01},
        'nan_previous':{'previous':math.nan},'bad_dt':{'dt':.3},
        'nan_baseline':{'base':output(math.nan)},'reverse_parallel':{'parallel':output(-.1)},
        'inf_steering':{'base':output(angle=math.inf)},'excess_baseline':{'base':output(1.01)},
        'excess_previous':{'previous':.38},'zero_dt':{'dt':0.}}[fault]
    out=call(1.,**kwargs)
    assert out[0]==0. and math.isfinite(out[1])


@pytest.mark.parametrize('fault',['rewind','epoch','reset'])
def test_time_discontinuity_never_turns_seen_threat_into_solo(setup,fault):
    ctl,state,call=setup;state['view']=view(True,-.2);call(1.)
    state['view']=view()
    if fault=='reset':ctl.invalidate('sensor_reset');out=call(1.1)
    elif fault=='rewind':out=call(.8)
    else:out=call(1.1,epoch=1)
    assert out[0]<=.2
    for t in (1.2,1.3,1.4):out=call(t,epoch=1 if fault=='epoch' else 0)
    assert ctl.mode=='GUARD' and out[0]<=.2


def test_unobserved_band_cannot_clear_seen_side(setup):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();state['band'].update(free=False,unobserved=1)
    for t in (1.1,1.2,1.3):out=call(t)
    assert ctl.mode=='GUARD' and out[0]==.2


def test_front_stop_stays_stop_after_release(setup):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();call(1.1);call(1.2)
    assert call(1.3,base=output(0.,reason='follow_gap_hold'))[0]==0.


def test_release_wait_does_not_presteer_into_rejected_return(setup):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();call(1.1);call(1.2)
    out=call(1.3,base=output(0.,.25,reason='follow_release_wait'),previous=.01)
    assert out[:2]==(0.,.01)
    assert out[2]['selector_held_baseline_steering_rad']==.25


def test_parallel_fallback_reduces_speed_instead_of_freezing_turned_wheels():
    from dataclasses import replace
    from arena_autonomy.parallel_lane_trial import proposal
    from test_parallel_lane_trial import scene,base
    cfg=replace(config(),parallel_hold_trial=True,pass_release_trial=True)
    r=proposal(scene(),base(),0.,.25,.02,cfg)
    assert r['eligible'] and 0.<r['target_speed_mps']<.35
    assert r['lateral_speed_limited'] and r['maximum_body_corner_lateral_m']<=.01
    assert 0.<r['steering_rad']<.25
    # 감속을 이미 마쳤다고 가정하면 안 된다.
    moving=proposal(scene(),base(),1.,.25,.02,cfg)
    assert not moving['eligible'] and moving['reason']=='lateral_motion_not_small'
    # 기존 명시 직선 유지 옵션은 바뀌지 않는다.
    old=proposal(scene(),base(),0.,.25,.02,replace(cfg,pass_release_trial=False))
    assert not old['eligible'] and old['reason']=='lateral_motion_not_small'


def test_straight_road_is_not_a_curve_when_ego_has_heading_error():
    from dataclasses import replace
    from arena_autonomy.parallel_lane_trial import proposal
    from test_parallel_lane_trial import scene,base
    cfg=replace(config(),parallel_hold_trial=True,pass_release_trial=True)
    b=base(slope=math.tan(math.radians(8.)))
    current=proposal(scene(),b,0.,0.,.02,cfg)
    old=proposal(scene(),b,0.,0.,.02,replace(cfg,pass_release_trial=False))
    assert current['eligible'] and current['maximum_body_corner_lateral_m']<=.01
    assert not old['eligible'] and old['reason']=='reference_not_straight'


@pytest.mark.parametrize('extent,front,allow',[(.0015,0,True),(.01,0,True),(.01001,0,False),(.0015,1,False)])
def test_small_shadow_tail_is_explicit_risk_not_observed_free(setup,monkeypatch,extent,front,allow):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();call(1.1);call(1.2)
    region={'unobserved':0,'masked_left':0,'masked_right':3,
            'masked_tail_extent_m':extent,'masked_front_samples':front}
    monkeypatch.setattr(ps,'motion_region',lambda *a:region)
    out=call(1.3,base=output(angle=.2))
    assert (out[0]==1.)==allow
    if allow:
        assert out[2]['selector_masked_tail_risk_accepted']
        assert not out[2]['selector_unknown_filled']
        assert out[2]['selector_region']['masked_right']==3


def test_fault_reason_cannot_fall_back_to_positive_parallel(setup):
    ctl,state,call=setup;state['view']=view(True,-.2);call(1.)
    assert call(1.1,base=output(0.,reason='follow_scene_stale'))[0]==0.


def test_clear_confirmation_interrupted_by_no_path(setup):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();call(1.1)
    state['view']=None;call(1.2)
    state['view']=view();out=call(1.3)
    assert ctl.mode=='GUARD' and out[0]==.2
    assert call(1.4)[0]==1.


@pytest.mark.parametrize('age',[.3,.31,-.01,math.nan])
def test_invalid_time_age_preserves_unresolved_threat(setup,age):
    ctl,state,call=setup;state['view']=view(True,-.2);call(1.)
    scene=SimpleNamespace(source_first_s=2.-age,t_ref_s=2.,source_epoch=0)
    out=ctl.select(scene,output(),output(.2),.3,0.,.02)
    assert out[0]==0. and ctl.mode=='GUARD'


def test_large_motion_with_unobserved_region_falls_back(setup,monkeypatch):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();call(1.1);call(1.2)
    monkeypatch.setattr(ps,'motion_region',lambda *a:{'unobserved':1,'masked_left':0,'masked_right':0})
    out=call(1.3,base=output(angle=.2))
    assert out[0]==.2 and out[2]['selector_reason']=='large_motion_region_unobserved'


def test_previous_steer_counts_when_new_command_straight(setup,monkeypatch):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();call(1.1);call(1.2)
    monkeypatch.setattr(ps,'motion_region',lambda *a:{'unobserved':1,'masked_left':0,'masked_right':0})
    out=call(1.3,previous=.2)
    assert out[0]==.2 and out[2]['selector_reason']=='large_motion_region_unobserved'


def test_finite_reverse_holds_without_inventing_opposite_side_unknown(setup):
    ctl,state,call=setup;state['view']=view(True,-.1);call(1.)
    assert call(1.1,speed=-.05,base=output(0.,reason='follow_reverse_motion'))[0]==0.
    assert not ctl.sides['right']['seen']
    state['view']=view(True,.25);call(1.2)
    state['view']=view();call(1.3)
    assert call(1.4)[0]==1. and ctl.mode=='RELEASED'


def test_solo_reverse_keeps_existing_g1_stop_and_steering(setup):
    ctl,state,call=setup
    out=call(1.,speed=-.05,previous=.18,base=output(0.,0.,reason='follow_reverse_motion'))
    assert out[:2]==(0.,0.) and out[2]['reason']=='follow_reverse_motion'
    assert out[2]['selector_choice']=='baseline' and ctl.mode=='SOLO'
    # 비정상 양수 기준 출력은 SOLO에서도 정지로 막는다.
    assert call(1.1,speed=-.05,previous=.18)[:2]==(0.,.18)


def test_guard_reverse_still_holds_previous_steering(setup):
    ctl,state,call=setup;state['view']=view(True,-.1);call(1.)
    out=call(1.1,speed=-.05,previous=.18,base=output(0.,0.,reason='follow_reverse_motion'))
    assert out[:2]==(0.,.18) and out[2]['selector_reason']=='reverse_motion_hold'


def test_rear_record_releases_only_after_whole_side_window_is_observed(setup):
    ctl,state,call=setup;state['view']=view(True,-.2);call(1.)
    state['view']=view()
    for t in (1.1,1.2,1.3):out=call(t)
    assert ctl.mode=='GUARD' and out[0]==.2 and ctl.sides['left']['vanish']=='rear'
    lat=ps.lateral_config(config())
    assert state['window_calls'][-1]==(-lat.band_behind_m,lat.band_ahead_m)
    state['window']={'free':True,'samples':40,'observed_free':36,'masked':4,'unobserved':0}
    assert call(1.4)[0]==.2 and ctl.mode=='GUARD'
    out=call(1.5)
    assert out[0]==1. and ctl.mode=='RELEASED' and ctl.sides['left']['vanish']=='rear'
    assert out[2]['selector_evidence']['left']['side_window']['free']


def test_rear_window_needs_two_distinct_scans_and_resets_on_occlusion(setup):
    ctl,state,call=setup;state['view']=view(True,-.2);call(1.)
    state['view']=view();state['window']={'free':True,'samples':40,'observed_free':40,'masked':0,'unobserved':0}
    call(1.1);call(1.1)
    assert ctl.mode=='GUARD'
    state['window']={'free':False,'samples':40,'observed_free':39,'masked':0,'unobserved':1}
    call(1.2)
    state['window']={'free':True,'samples':40,'observed_free':40,'masked':0,'unobserved':0}
    assert call(1.3)[0]==.2 and ctl.mode=='GUARD'
    assert call(1.4)[0]==1. and ctl.mode=='RELEASED'


def test_rear_window_release_reenters_guard_on_new_side_return(setup):
    ctl,state,call=setup;state['view']=view(True,-.2);call(1.)
    state['view']=view();state['window']={'free':True,'samples':40,'observed_free':40,'masked':0,'unobserved':0}
    call(1.1);call(1.2)
    assert ctl.mode=='RELEASED'
    state['view']=view(True,-.3)
    assert call(1.3)[0]==.2 and ctl.mode=='GUARD'
    # 다시 앞으로 지나가도 이전 rear 기록은 지워지지 않아 창 전체 관측을 요구한다.
    state['view']=view(True,.25);call(1.4)
    state['view']=view();state['window']={'free':False,'samples':40,'observed_free':30,'masked':0,'unobserved':10}
    for t in (1.5,1.6,1.7):out=call(t)
    assert ctl.mode=='GUARD' and ctl.sides['left']['vanish']=='rear'


def test_unknown_after_sensor_reset_uses_whole_side_window(setup):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    ctl.invalidate('sensor_reset')
    state['view']=view()
    for t in (1.1,1.2,1.3):out=call(t)
    assert ctl.mode=='GUARD' and out[0]==.2
    state['window']={'free':True,'samples':40,'observed_free':40,'masked':0,'unobserved':0}
    call(1.4)
    assert call(1.5)[0]==1. and ctl.mode=='RELEASED'


def test_whole_window_band_geometry_extends_existing_band(monkeypatch):
    cfg=config();lat=ps.lateral_config(cfg);sel=ps.SelectorConfig()
    seen=[]
    class Frame:
        def project(self,q):
            seen.append(np.array(q));return q[:,0],q[:,1]
    monkeypatch.setattr(ps,'depth_support',lambda scene,q,pad:np.ones(len(q),dtype=bool))
    monkeypatch.setattr(ps.mx,'ray_padding',lambda *a:None)
    short=ps.band_check(SimpleNamespace(),Frame(),1.,lat,sel)
    wide=ps.band_check(SimpleNamespace(),Frame(),1.,lat,sel,(-lat.band_behind_m,lat.band_ahead_m))
    assert short['free'] and wide['free'] and wide['samples']>short['samples']
    xs_short,xs_wide=np.unique(seen[0][:,0]),np.unique(seen[1][:,0])
    assert np.isclose(xs_short.min(),-cfg.length_m/2) and np.isclose(xs_short.max(),cfg.length_m/2)
    assert np.isclose(xs_wide.min(),-lat.band_behind_m) and xs_wide.max()<=lat.band_ahead_m+1e-9
    assert np.allclose(np.unique(seen[0][:,1]),np.unique(seen[1][:,1]))


def test_finite_reverse_does_not_erase_unresolved_rear_object(setup):
    ctl,state,call=setup;state['view']=view(True,-.1);call(1.)
    state['view']=view();call(1.1)
    assert call(1.2,speed=-.05)[0]==0.
    state['view']=view(True,.25);call(1.3)
    state['view']=view();call(1.4);out=call(1.5)
    assert ctl.mode=='GUARD' and out[0]==.2


@pytest.mark.parametrize('masked',['masked_left','masked_right'])
def test_large_move_mask_is_not_free_even_on_previously_passed_or_unseen_side(setup,monkeypatch,masked):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();call(1.1);call(1.2)
    region={'unobserved':0,'masked_left':0,'masked_right':0};region[masked]=1
    monkeypatch.setattr(ps,'motion_region',lambda *a:region)
    assert call(1.3,base=output(angle=.2))[0]==.2


@pytest.mark.parametrize('angle',[-.37,-.12,.001,.12,.37])
def test_arc_contains_dense_corner_coordinate_extrema(angle):
    cfg=config();distance=.53
    poses=ps.extrema_arc(angle,distance,ps.lateral_config(cfg))
    k=math.tan(angle)/cfg.wheelbase_m
    theta=np.linspace(0.,distance*k,10001)
    for x in (-cfg.length_m/2,cfg.length_m/2):
        for y in (-cfg.width_m/2,cfg.width_m/2):
            a,b=1/k-y,cfg.wheelbase_m/2+x
            for sampled,dense in ((a*np.sin(poses[:,2])+b*np.cos(poses[:,2])-cfg.wheelbase_m/2,
                                  a*np.sin(theta)+b*np.cos(theta)-cfg.wheelbase_m/2),
                                 (1/k-a*np.cos(poses[:,2])+b*np.sin(poses[:,2]),
                                  1/k-a*np.cos(theta)+b*np.sin(theta))):
                assert sampled.min()<=dense.min()+1e-10
                assert sampled.max()>=dense.max()-1e-10


@pytest.mark.parametrize('kwargs',[{'pass_release_trial':1},{'pass_release_trial':True},
    {'parallel_hold_trial':True,'pass_release_trial':True,'partial_observation_policy':'strict'}])
def test_runtime_requires_explicit_compatible_trial(kwargs):
    with pytest.raises(ValueError):FollowConfig(**kwargs)
    assert not FollowConfig().pass_release_trial


def test_runtime_record_replay_and_front_stop(tmp_path):
    import json
    from arena_autonomy.local_follow import LocalFollow
    from arena_autonomy.follow_trace import FollowTrace,replay_events
    from test_g1_partial_policy import corridor,call
    from test_local_follow import history
    cfg=FollowConfig(partial_observation_policy='bounded_local_v4',parallel_hold_trial=True,
        pass_release_trial=True,boundary_evidence_mode='corner_distance_candidate',
        occupancy_timing_mode='per_ray_candidate')
    h=history();ctl=LocalFollow(cfg);path=tmp_path/'selector.jsonl'
    trace=FollowTrace(str(path),h.config,cfg);outputs=[]
    for first in (.3,.4,.5,.6,.7,.8,.9):
        raw=corridor(first,obstacle=.25 if first>=.8 else None)
        out=call(ctl,raw,h);outputs.append(out)
        trace.record_call(raw,h,first+.1,0,0.,'left',0.,.02,True,30.,out)
    trace.close()
    assert any(o[0]>0 for o in outputs) and outputs[-1][0]==0.
    assert all(o[2]['selector_mode']=='SOLO' for o in outputs)
    result=replay_events(json.loads(line) for line in path.read_text().splitlines())
    assert result['passed'] and result['calls']==7


def test_runtime_reset_keeps_seen_threat_unresolved():
    from dataclasses import replace
    from arena_autonomy.local_follow import LocalFollow
    cfg=replace(config(),parallel_hold_trial=True,pass_release_trial=True)
    ctl=LocalFollow(cfg)
    ctl.pass_selector.sides['left'].update(seen=True,present=True)
    ctl.reset()
    assert ctl.pass_selector.mode=='GUARD'
    assert ctl.pass_selector.sides['left']['vanish']=='unknown'


def test_selector_preserves_chosen_path_and_reason(setup):
    ctl,state,call=setup;state['view']=view(True,-.1)
    chosen=(.2,.01,{'reason':'parallel_hold_follow','path_points':[[0,.1],[1,.1]],'behavior':'FOLLOW'})
    out=call(1.,parallel=chosen)
    assert out[:2]==chosen[:2] and out[2]['reason']==chosen[2]['reason']
    assert out[2]['path_points']==chosen[2]['path_points']


# rear/unknown 해제의 실제 광선 기하 반례(벽 병합·두 번째 물체 통과·후방 원뿔) 및 해제 뒤 기존 거절 유지.
def box_scene(first, boxes=()):
    """벽 y=±0.425와 자차 좌표 축정렬 상자(x0,x1,y0,y1)의 광선 투사. 후방 ±30°는 기존 시험처럼 무반환."""
    raw = scan(first)
    angles = raw.angle_min+np.arange(500)*raw.angle_increment
    values = []
    for a in angles:
        c, s = math.cos(a), math.sin(a)
        value = min(.425/max(abs(s), 1e-12), 8.)
        for x0, x1, y0, y1 in boxes:
            lo, hi = 0., value
            for o, d, a0, a1 in ((.06, c, x0, x1), (0., s, y0, y1)):
                if abs(d) < 1e-12:
                    if not a0 <= o <= a1:
                        lo, hi = 1., 0.
                    continue
                t0, t1 = sorted(((a0-o)/d, (a1-o)/d))
                lo, hi = max(lo, t0), min(hi, t1)
            if lo <= hi and lo > 0:
                value = min(value, lo)
        if abs(a) > math.radians(150)+1e-12:
            value = math.inf
        values.append(value)
    raw.ranges = values
    return motion_scene(raw, history(), first+.1, timing_verified=True)


STRAIGHT = np.c_[np.linspace(-1., 2., 61), np.zeros(61)].tolist()


def baseline(speed=1., angle=0.):
    return speed, angle, {'reason': 'local_follow', 'path_points': STRAIGHT}


BESIDE = (-.05, .15, .15, .30)          # 옆, 벽과 떨어진 compact 차량 크기 상자
MERGED_BEHIND = (-.40, -.20, .15, .425)  # 옆뒤, 벽에 붙어 compact가 꺼지는 물체
PASSING = (.00, .20, .12, .27)           # 다른 물체가 옆을 지나감
PASSED = (.22, .42, .12, .27)           # 최후단이 앞면+0.10 m를 넘은 통과(단독이면 benign)
AHEAD = (.60, .80, .12, .27)             # 측방 창 앞으로 빠짐
HIDDEN = (-.60, -.40, -.05, .10)         # 후방 마스크 원뿔 안에 완전히 숨음


def view_side(scene):
    return ps.threat_view(scene, STRAIGHT, ps.lateral_config(config()))['left']


def test_wall_merged_object_behind_is_not_present_and_blocks_only_the_whole_window():
    cfg, sel = config(), ps.SelectorConfig()
    lat = ps.lateral_config(cfg)
    scene = box_scene(1., [MERGED_BEHIND])
    assert not view_side(scene)['present']
    frame = ps.mx.ExecutionFrame(np.asarray(STRAIGHT))
    short = ps.band_check(scene, frame, 1., lat, sel)
    whole = ps.band_check(scene, frame, 1., lat, sel, (-lat.band_behind_m, lat.band_ahead_m))
    assert short['free'] and not whole['free'] and whole['unobserved'] > 0
    hidden = box_scene(1., [HIDDEN])
    clear = ps.band_check(hidden, frame, 1., lat, sel, (-lat.band_behind_m, lat.band_ahead_m))
    # 원뿔 안 물체는 배제하지 못한다. 마스크 표본으로 따로 셀 뿐 자유 관측으로 바꾸지 않는다.
    assert clear['free'] and clear['masked'] > 0 and clear['observed_free'] < clear['samples']


def test_real_geometry_rear_record_survives_second_object_and_needs_observed_window():
    ctl = ps.PassReleaseSelector(config())
    t = [1.]

    def step(boxes, base=None, previous=0.):
        t[0] += .1
        return ctl.select(box_scene(t[0], boxes), base or baseline(), (.2, 0., {'reason': 'parallel_hold_follow'}),
                          .3, previous, .02)
    assert view_side(box_scene(1., [BESIDE]))['present']
    step([BESIDE])
    assert ctl.mode == 'GUARD'
    for _ in range(3):
        out = step([MERGED_BEHIND])
    assert ctl.sides['left']['vanish'] == 'rear' and ctl.mode == 'GUARD' and out[0] == .2
    assert out[2]['selector_evidence']['left']['band']['free']
    assert not out[2]['selector_evidence']['left']['side_window']['free']
    # 첫 물체가 옆뒤에 남은 채 다른 물체가 옆을 지나 앞으로 빠져도 rear 기록·GUARD 유지.
    step([MERGED_BEHIND, PASSING])
    assert ctl.sides['left']['present']
    step([MERGED_BEHIND, PASSED])
    assert ctl.sides['left']['present'] and ctl.sides['left']['last_present']['rearmost_ds_m'] >= .2
    for _ in range(3):
        out = step([MERGED_BEHIND, AHEAD])
    assert ctl.sides['left']['vanish'] == 'rear' and ctl.mode == 'GUARD' and out[0] == .2
    # 관측 가능한 측방 창이 비고 첫 물체가 마스크 원뿔에만 있으면 두 fresh 뒤 해제(잔여 위험 명시).
    step([HIDDEN, AHEAD])
    assert ctl.mode == 'GUARD'
    out = step([HIDDEN, AHEAD])
    assert ctl.mode == 'RELEASED' and ctl.sides['left']['vanish'] == 'rear' and out[0] == 1.
    assert out[2]['selector_hidden_rear_risk_unresolved'] and not out[2]['selector_unknown_filled']


@pytest.mark.parametrize('region,reason', [
    ({'unobserved': 1, 'masked_left': 0, 'masked_right': 0}, 'large_motion_region_unobserved'),
    ({'unobserved': 0, 'masked_left': 3, 'masked_right': 0, 'masked_tail_extent_m': .011, 'masked_front_samples': 0},
     'large_motion_masked_toward_left'),
    ({'unobserved': 0, 'masked_left': 3, 'masked_right': 0, 'masked_tail_extent_m': .002, 'masked_front_samples': 1},
     'large_motion_masked_toward_left')])
def test_rear_release_still_rejects_large_motion_without_observed_region(monkeypatch, region, reason):
    state = {'view': {'frame': None, 'state': None,
                      'left': {'present': True, 'points': 4, 'rearmost_ds_m': -.2, 'frontmost_ds_m': 0.},
                      'right': {'present': False, 'points': 0, 'rearmost_ds_m': None, 'frontmost_ds_m': None}}}
    free = {'free': True, 'samples': 40, 'observed_free': 36, 'masked': 4, 'unobserved': 0}
    monkeypatch.setattr(ps, 'threat_view', lambda *a: state['view'])
    monkeypatch.setattr(ps, 'band_check', lambda *a: free)
    ctl = ps.PassReleaseSelector(config())

    def call(t, base=output(), previous=0.):
        return ctl.select(SimpleNamespace(source_first_s=t-.1, t_ref_s=t, source_epoch=0), base,
                          output(.2), .3, previous, .02)
    call(1.)
    state['view'] = {**state['view'], 'left': {'present': False, 'points': 0, 'rearmost_ds_m': None,
                                               'frontmost_ds_m': None}}
    call(1.1)
    call(1.2)
    assert ctl.mode == 'RELEASED' and ctl.sides['left']['vanish'] == 'rear'
    monkeypatch.setattr(ps, 'motion_region', lambda *a: region)
    out = call(1.3, base=output(angle=.25))
    assert out[0] == .2 and out[2]['selector_choice'] == 'parallel' and out[2]['selector_reason'] == reason


@pytest.mark.parametrize('fault', ['rewind', 'epoch', 'stale', 'invalid_speed'])
def test_rear_release_keeps_time_and_input_fault_rejections(monkeypatch, fault):
    views = {'present': {'frame': None, 'state': None,
                         'left': {'present': True, 'points': 4, 'rearmost_ds_m': -.2, 'frontmost_ds_m': 0.},
                         'right': {'present': False, 'points': 0, 'rearmost_ds_m': None, 'frontmost_ds_m': None}},
             'empty': {'frame': None, 'state': None,
                       'left': {'present': False, 'points': 0, 'rearmost_ds_m': None, 'frontmost_ds_m': None},
                       'right': {'present': False, 'points': 0, 'rearmost_ds_m': None, 'frontmost_ds_m': None}}}
    state = {'view': views['present']}
    monkeypatch.setattr(ps, 'threat_view', lambda *a: state['view'])
    monkeypatch.setattr(ps, 'band_check', lambda *a: {'free': True, 'samples': 40, 'observed_free': 40,
                                                      'masked': 0, 'unobserved': 0})
    ctl = ps.PassReleaseSelector(config())

    def call(t, epoch=0, age=.1, speed=.3):
        return ctl.select(SimpleNamespace(source_first_s=t-age, t_ref_s=t, source_epoch=epoch), output(), output(.2),
                          speed, 0., .02)
    call(1.)
    state['view'] = views['empty']
    call(1.1)
    call(1.2)
    assert ctl.mode == 'RELEASED'
    kwargs = {'rewind': {'t': 1.0}, 'epoch': {'t': 1.3, 'epoch': 1}, 'stale': {'t': 1.3, 'age': .3},
              'invalid_speed': {'t': 1.3, 'speed': math.nan}}[fault]
    out = call(**kwargs)
    assert out[0] == 0. and out[2]['selector_choice'] == 'hold_fault'
    assert ctl.mode == 'GUARD' and all(s['vanish'] == 'unknown' for s in ctl.sides.values())
