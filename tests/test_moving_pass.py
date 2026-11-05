"""이동차 추월 후보의 현재 관측·복귀·오차 속도 선택 검사. 실제 주행 검증 아님."""
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from arena_autonomy.bypass_entry_trial import EntryConfig, EntryGuide, straight_reference
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.front_patch_motion import ObservedForwardPace
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.moving_pass import MovingEntryConfig, MovingPassAdapter, MovingPassGuide, MovingTargetTrack
from arena_autonomy.moving_pass import available_pass_offsets, rear_wall_witness
from probe_bypass_entry import sensor_scene
from test_bypass_handoff import context
from test_bypass_handoff_corridor_trial import subject as corridor_controller


def history(speed=0.):
    result = MotionHistory(MotionConfig(wheel_radius_m=.0325, history_s=3.))
    for t in np.arange(0., 2., .01):
        result.add_mean_wheel(float(t), speed/.0325)
        result.add_gyro(float(t), 0.)
    return result


def scene(t=.5, x=-.65, *, missing=False, epoch=0):
    # 현재 구성처럼 후방 반평면 마스크는 없다. 구조물 가림은 후속 장면에서 별도 적용.
    return sensor_scene(targets=() if missing else ((x,.25),), now=t, first=round(t-.1,9), rear_half=0.,epoch=epoch)[0]


def tracker():
    s=scene(); cfg=FollowConfig()
    _, ref=straight_reference(sensor_scene()[0],cfg)
    points=np.asarray([r.hit_xy_m for r in s.rays if r.status=='return'
                       and r.hit_xy_m[0]<-.4 and .15<r.hit_xy_m[1]<.35])
    assert len(points)>=3
    result=MovingTargetTrack(cfg); result.begin(s,points,ref['walls'])
    return result


def test_only_moving_candidate_gets_065_cap():
    assert EntryConfig().speed_cap_mps == .35
    assert MovingEntryConfig().speed_cap_mps == .65
    for value in (0., .66, True, float('nan')):
        with pytest.raises(ValueError): MovingEntryConfig(speed_cap_mps=value)
    with pytest.raises(ValueError): MovingEntryConfig(offsets_m=(0., .50))


def moving_controller():
    base=corridor_controller()
    execution=replace(base.execution,moving_follow_reserve=True,closing_follow_reserve=True,
        low_motion_reserve=True,moving_pace_floor=True,observed_forward_pace=True,
        continuous_forward_pace=True,moving_pass_trial=True,entry_policy='corridor_local_v1')
    return BypassHandoffFollow(replace(base.config,open_route_speed_mps=1.5),execution,allow_simulation_trial=True)


def test_explicit_activation_and_reset():
    assert not HandoffExecutionConfig().moving_pass_trial
    assert not HandoffExecutionConfig.from_mapping({'bypass':{}}).moving_pass_trial
    with pytest.raises(ValueError,match='moving pass'):
        HandoffExecutionConfig(moving_pass_trial=True)
    ctrl=moving_controller(); ctrl.moving_pass.active=True
    ctrl.reset()
    assert not ctrl.moving_pass.active and ctrl.moving_pass.tracker.points is None
    assert ctrl.moving_pass.handoff_follow is None and ctrl.moving_pass.handoff_monitor is None


@pytest.mark.parametrize('prepared_speed',[0.,.4])
def test_centered_handoff_preserves_current_g1_and_honors_its_stop(prepared_speed):
    ctrl=moving_controller();adapter=ctrl.moving_pass
    raw,h,s=context();adapter.active=True;adapter.tracker.epoch=s.source_epoch
    adapter.guide.returning_behind=True
    prepared=SimpleNamespace();monitor=SimpleNamespace()
    adapter.handoff_follow=prepared;adapter.handoff_monitor=monitor
    normal=(prepared_speed,.01,dict(reason='handoff_follow',normal=dict(reason='follow_gap_hold')))
    with patch.object(adapter.tracker,'update',return_value={}), \
         patch.object(adapter.guide,'evaluate',return_value=dict(entry_phase='centered',
             selected=dict(speed_mps=.65,steering_rad=.02))), \
         patch.object(adapter,'_prepare_follow_handoff'), \
         patch.object(ctrl,'_base_command',return_value=normal):
        out=adapter.command(s,h,.2,'right',0.,.02,raw_scan=raw)
    assert ctrl.normal is prepared and ctrl.monitor is monitor
    assert out[:2]==normal[:2]  # 준비된 G1이 정지를 요구하면 이전 양수 복귀 후보로 덮지 않는다.
    assert out[2]['moving_pass']['reason']=='aborted_to_observed_follow'
    assert not adapter.active and adapter.handoff_follow is None and adapter.handoff_monitor is None


def test_active_candidate_cannot_bypass_bad_input_reset():
    ctrl=moving_controller(); ctrl.moving_pass.active=True
    _,h,s=context()
    out=ctrl.command(s,h,0.,'right',0.,.02,raw_scan=None)
    assert out[0]==0 and not ctrl.moving_pass.active


def test_evaluation_arguments_are_opt_in():
    from g1_handoff_eval import launch_arguments
    launch=dict(actuation_mode='virtual_mcu',autonomy_mode='local_pursuit',speed_bump=False,
        lidar_rate_hz=10.,lidar_compensation='both',lidar_acquisition='sequential',
        speed_profile='g1_open_1p5',control_rate_hz=50.)
    with pytest.raises(ValueError,match='moving pass'):
        launch_arguments('g1g2','paced',launch,True,moving_pass=True,observed_motion=True)
    args=launch_arguments('g1g2','paced',launch,True,moving_pass=True,
                          continuous_pace=True,observed_motion=True)
    assert 'evaluation_moving_pass_trial:=true' in args


def test_rear_confirmation_needs_distinct_current_returns():
    track=tracker(); h=history()
    first=track.update(scene(.6),h)
    assert first['observed'] and not first['rear_clear']
    for _ in range(3):
        again=track.update(scene(.6),h)
        assert not again['rear_clear'] and again['rear_confirmations']==1
    assert track.update(scene(.7),h)['rear_clear']


def test_missing_target_is_not_passage_confirmation():
    track=tracker(); h=history()
    track.update(scene(.6),h); assert track.update(scene(.7),h)['rear_clear']
    absent=track.update(scene(.8, missing=True),h)
    assert not absent['observed'] and not absent['rear_clear'] and track.current is None
    assert not track.update(scene(.8, missing=True),h)['rear_clear']
    assert track.update(scene(1.2, missing=True),h)['reason']=='target_observation_expired'


def test_partial_visible_rear_surface_does_not_locate_front_bumper():
    track=tracker(); h=history()
    for t in (.6,.7):
        out=track.update(scene(t,x=-.45),h)
        assert out['observed'] and not out['rear_clear']
        assert out['front_bound_m'] > -.16


def test_same_scan_motion_and_age_can_revoke_clearance():
    track=tracker(); h=history()
    track.update(scene(.6),h); track.update(scene(.7),h)
    assert track.detail['rear_clear']
    # 같은 광선의 운동 이송 후 자차가 뒤로 움직인 경우. 확인 횟수 추가 없이 취소.
    with patch('arena_autonomy.moving_pass.transport_points', side_effect=lambda p,*a: np.asarray(p)+[.3,0.]):
        out=track.update(replace(scene(.7),t_ref_s=.72),h)
    assert not out['rear_clear'] and out['rear_confirmations']==0


@pytest.mark.parametrize('kind',['epoch','stale','history'])
def test_discontinuous_input_drops_target(kind):
    track=tracker(); s=scene(.6); h=history()
    if kind=='epoch': s=scene(.6,epoch=1)
    elif kind=='stale': s=replace(s,t_ref_s=.9)
    else: h=MotionHistory(h.config)
    out=track.update(s,h)
    assert not out['rear_clear'] and track.current is None and track.points is None


def test_rejoin_requires_target_and_current_free_band():
    track=tracker(); guide=MovingPassGuide(FollowConfig(),track)
    with patch.object(EntryGuide,'_rejoin_band', return_value={'supported':True}):
        assert not guide._rejoin_band(scene(),None)['supported']
        track.detail['rear_clear']=True
        assert guide._rejoin_band(scene(),None)['supported']
        track.current=None
        assert not guide._rejoin_band(scene(),None)['supported']
    track.current=np.array([[-1.,.25]])
    with patch.object(EntryGuide,'_rejoin_band', return_value={'supported':False}):
        assert not guide._rejoin_band(scene(),None)['supported']


@pytest.mark.parametrize('lead_speed,should_withdraw',[(.30,False),(.58,True)])
def test_speed_advantage_loss_needs_distinct_current_forward_observations(lead_speed,should_withdraw):
    track=tracker();h=history()
    # 센서 값의 상대 차체 점. 자차 운동은 0인 이력으로 분리한다.
    track.speed_rows.clear();track.speed_source=None
    for i in range(7):
        s=scene(.50+.1*i)
        track.current=np.array([[1.+lead_speed*.1*i,-.20],[1.01+lead_speed*.1*i,-.19]])
        track.source=s.source_first_s;track.detail['observed']=True
        track._observe_speed(s,h)
        count=track.lost_advantage_scans
        track._observe_speed(s,h)
        assert track.lost_advantage_scans==count
        if i<6:assert count<3
    assert (track.lost_advantage_scans>=3)==should_withdraw
    assert track.speed_upper==pytest.approx(lead_speed+.15)
    track.current=None
    track._observe_speed(scene(1.2),h)
    assert track.lost_advantage_scans==0 and not track.speed_rows


@pytest.mark.parametrize('change',['none','beside','few_scans','missing','occupied','stale'])
def test_return_behind_requires_whole_target_ahead_and_current_band(change):
    track=tracker();guide=MovingPassGuide(FollowConfig(),track)
    s=scene(.7);track.source=s.source_first_s
    track.current=np.array([[.9,-.2],[.91,-.19]]) if change!='missing' else None
    if change=='beside':track.current=np.array([[.2,-.2],[.21,-.19]])
    if change=='stale':track.source=0.
    track.lost_advantage_scans=2 if change=='few_scans' else 3
    track.detail=dict(observed=True,rear_clear=False)
    with patch.object(EntryGuide,'_rejoin_band',return_value={'supported':change!='occupied'}):
        band=guide._rejoin_band(s,None)
    assert band['supported']==(change=='none')
    assert not band['moving_target_rear_observed']


def test_return_confirmation_counts_observations_but_needs_current_support():
    guide=MovingPassGuide(FollowConfig(),tracker())
    yes=dict(supported=True,return_behind_lead=True)
    no=dict(supported=False,return_behind_lead=True)
    assert not guide._confirm_rejoin(yes,True)
    assert not guide._confirm_rejoin(yes,False)
    assert guide.rejoin_confirmations==1
    assert not guide._confirm_rejoin(no,False)
    assert guide.rejoin_confirmations==1
    assert guide._confirm_rejoin(yes,True)
    assert not guide._confirm_rejoin(no,False)
    assert not guide._confirm_rejoin(no,True)
    assert guide.rejoin_confirmations==0
    # 후방에서 추월을 마친 뒤 복귀하는 기존 분기는 그대로 초기화한다.
    guide._confirm_rejoin(dict(supported=True),True)
    guide._confirm_rejoin(dict(supported=False),False)
    assert guide.rejoin_confirmations==0

def test_lost_advantage_can_request_yield_before_whole_target_is_ahead():
    track=tracker();h=history();cfg=track.config
    for i in range(7):
        s=scene(.50+.1*i)
        # 상대 반환은 전방이나 규격 전체와 시간 여유까지 앞이라고 볼 수 없다.
        track.current=np.array([[.14+.058*i,-.20],[.15+.058*i,-.19]])
        track.source=s.source_first_s;track.detail['observed']=True
        track._observe_speed(s,h)
    assert track.lost_advantage_scans>=3
    assert not track.ahead_clear(s)
    guide=MovingPassGuide(cfg,track)
    with patch.object(EntryGuide,'_rejoin_band',return_value={'supported':True}):
        assert not guide._rejoin_band(s,None)['supported']


def test_yield_cap_is_gradual_keeps_path_checks_and_resets():
    adapter=moving_controller().moving_pass
    assert adapter._yield_before_return(.02).speed_cap_mps==.65
    adapter.tracker.lost_advantage_scans=3
    adapter.tracker.speed_lower=.44
    previous=.65
    for _ in range(40):
        cap=adapter._yield_before_return(.02).speed_cap_mps
        assert 0<=previous-cap<=.0100000001
        previous=cap
    assert cap==pytest.approx(.39) and adapter.withdrawal_requested
    # 실제 비교기가 이 상한으로 후보를 다시 고른다. 기본 Entry 설정은 유지한다.
    assert adapter.guide.entry.speed_cap_mps==.65
    adapter.tracker.speed_lower=None
    assert adapter._yield_before_return(.02).speed_cap_mps==pytest.approx(.39)
    adapter.reset()
    assert not adapter.withdrawal_requested and adapter.withdrawal_speed_cap==.65


def test_changing_visible_side_does_not_become_speed_loss_evidence():
    track=tracker();h=history()
    for i in range(7):
        s=scene(.50+.1*i)
        track.current=np.array([[-.02+.058*i,-.20],[-.01+.058*i,-.19]])
        track.source=s.source_first_s;track.detail['observed']=True
        track._observe_speed(s,h)
    assert track.speed_upper>.50
    assert track.rear_bound(s)<0. and track.lost_advantage_scans==0


def test_speed_selection_uses_observed_displacement_and_error_not_lower_rate():
    h=history(); pace=ObservedForwardPace()
    for k in range(5):
        source=.1+k*.1
        pace.observe(source,source+.1,[1.+.03*k,0.],h)
    adapter=MovingPassAdapter(SimpleNamespace(config=FollowConfig(),monitor=SimpleNamespace(forward_pace=pace)))
    s=scene(.6)
    assert adapter.eligibility(s,h)['ready']
    assert adapter.eligibility(s,h)['estimated_lead_upper_mps']==pytest.approx(.45)
    # 낮은 추종율만 남아도 실제 .50 m/s 상대는 이번 .65 m/s 후보에서 제외한다.
    pace.rows.clear()
    for k in range(5): pace.rows.append((.1+k*.1,.2+k*.1,np.array([1.+.05*k,0.])))
    pace.rate=.05
    assert not adapter.eligibility(s,h)['ready']


@pytest.mark.parametrize('changed',['none','ahead_only','short','missing','not_clear','entry','few','remembered'])
def test_current_forward_reference_is_only_for_an_observed_rejoin(changed):
    guide=MovingPassGuide(FollowConfig(),tracker())
    guide.phase='entry' if changed=='entry' else 'rejoin'
    guide.tracker.detail['rear_clear']=changed!='not_clear'
    guide.tracker.current=None if changed=='missing' else np.array([[-.75,.21]])
    fit=(0.,.039,.01 if changed=='ahead_only' else -.0256,.60 if changed=='short' else 1.86)
    wall=dict(reason='history' if changed=='remembered' else 'current_connected_straight_wall_fit',
              actual_return_count=11 if changed=='few' else 130)
    assert not EntryGuide._wall_covers_guide(guide,fit,wall)
    assert guide._wall_covers_guide(fit,wall)==(changed=='none')
    assert ('guide_extent' in wall)==(changed=='none')


def test_current_path_still_rejects_new_obstacle_and_unknown():
    guide=MovingPassGuide(FollowConfig(),tracker()); h=history()
    guide.evaluate(sensor_scene(targets=((1.2,-.12),))[0],h,0.,0.,.02)
    initial=guide.evaluate(sensor_scene(targets=((1.2,-.12),),now=.6,first=.5)[0],h,0.,0.,.02)
    assert initial['selected'] is not None
    for obstacle in (True, False):
        blocked=sensor_scene(targets=((.18,0.),) if obstacle else (),
            missing=() if obstacle else tuple(range(225,276)),now=.7,first=.6)[0]
        result=guide.evaluate(blocked,h,0.,initial['selected']['steering_rad'],.02)
        assert result['selected'] is None


@pytest.mark.parametrize('lead,side',[(0.,None),(-.12,'left'),(.12,'right')])
def test_initial_offset_reserves_room_on_both_sides_even_at_end_of_scan(lead,side):
    cfg=FollowConfig();s=sensor_scene(rear_half=0.)[0]
    _,ref=straight_reference(s,cfg)
    patch=np.array([[1.0,lead-.075],[1.0,lead],[1.0,lead+.075]])
    offsets,rows=available_pass_offsets(s,cfg,ref['walls'],patch)
    if side is None:
        assert offsets==(0.,) and all(not r['feasible'] for r in rows.values())
    else:
        assert len(offsets)==2 and offsets[1]*(1 if side=='left' else -1)>0
        assert rows[side]['low_m']<=cfg.wall_offset_m-abs(offsets[1])<=rows[side]['high_m']


@pytest.mark.parametrize('side',['left','right'])
@pytest.mark.parametrize('change',['none','unexplained','unknown','occupied','short','skew','wide'])
def test_rear_witness_changes_only_path_reference(side,change):
    cfg=FollowConfig()
    sign=1 if side=='left' else -1
    ends=[(1.20-.02*i,sign*.18) for i in range(63)]
    count=10 if change=='short' else 24
    start=-.12 if change=='wide' else -.072
    tail=[(start-.01*i,sign*.18+(.002*i if change=='skew' else 0.)) for i in range(count)]
    values=ends+[None]*5+tail
    if side=='right':values=values[::-1]
    rays=[SimpleNamespace(source_index=i,hit_xy_m=p,status='masked' if p is None else 'return')
          for i,p in enumerate(values)]
    gaps=frozenset(r.source_index for r in rays if r.status=='masked')
    if change=='unknown':rays[min(gaps)+1].status='unknown'
    if change=='occupied':
        ray=rays[min(gaps)+1];ray.status='return';ray.hit_xy_m=(0.,0.)
    indices=[r.source_index for r in rays if r.hit_xy_m in ends]
    fit=(0.,sign*(.18-cfg.wall_offset_m),-.04,1.2)
    wall=dict(reason='current_connected_straight_wall_fit',source_ray_indices=indices)
    result,evidence=rear_wall_witness(SimpleNamespace(rays=rays),cfg,side,fit,wall,
                                      set() if change=='unexplained' else {gaps})
    if change!='none':
        assert result==fit and evidence is wall
    else:
        assert result[:2]==fit[:2] and result[3]==fit[3] and result[2]<-.28
        assert evidence['source_ray_indices']==indices and 'rear_geometry_witness' not in wall
        proof=evidence['rear_geometry_witness']
        assert not proof['short_piece_static'] and not proof['unknown_filled']
