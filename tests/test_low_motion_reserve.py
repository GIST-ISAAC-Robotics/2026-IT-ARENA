"""고정 우회 여유를 낮은 이동 조각에만 쓰는 선택형 후보. 기본 OFF와 STOP 경계를 함께 확인한다."""
from dataclasses import replace
import json
from unittest.mock import patch

import numpy as np
import pytest

import g1_handoff_eval as handoff
from arena_autonomy.bypass_handoff import HandoffExecutionConfig
from test_bypass_handoff import context,fast_normal,follow_space,moving_monitor


def control(speeds,enabled=True,floor=False):
    ctrl=follow_space()
    ctrl.execution=replace(ctrl.execution,moving_follow_reserve=True,closing_follow_reserve=True,
                           low_motion_reserve=enabled,moving_pace_floor=floor)
    ctrl.monitor,_=moving_monitor(speeds)
    return ctrl


def limited(ctrl,extra,request):
    _,_,sc=context(.6)
    front=max(ctrl.bypass.guide.entry.transitions_m)+extra-ctrl.config.wheelbase_m/2
    cloud=np.c_[np.full(5,front),np.linspace(-.06,.06,5)]
    with patch('arena_autonomy.bypass_handoff.front_patch',return_value=(cloud,'front')),patch(
            'arena_autonomy.bypass_handoff.straight_reference',return_value=({},{})):
        return ctrl._follow_entry_space(sc,(request,0.,{}),0.,request)


def test_default_is_off_and_old_trace_mapping_keeps_it_off():
    assert HandoffExecutionConfig().low_motion_reserve is False
    values=dict(bypass={},preparation_policy='front_patch_v0')
    assert HandoffExecutionConfig.from_mapping(values).low_motion_reserve is False


@pytest.mark.parametrize('values',[
    dict(low_motion_reserve=True),
    dict(moving_follow_reserve=True,low_motion_reserve=True),
    dict(moving_follow_reserve=True,closing_follow_reserve=True,recent_forward_reserve=True,low_motion_reserve=True),
])
def test_option_requires_closing_policy_and_excludes_recent_forward(values):
    with pytest.raises(ValueError,match='low motion'):
        replace(follow_space().execution,**values)


def test_option_type_is_strict():
    ctrl=follow_space()
    with pytest.raises(ValueError,match='low motion'):
        replace(ctrl.execution,moving_follow_reserve=True,closing_follow_reserve=True,low_motion_reserve=1)


@pytest.mark.parametrize('extra,request_mps',[(.3,1.4),(-.1,1.4),(-.3,.9)])
def test_fast_patch_keeps_g1_request_instead_of_zero_pace_reserve(extra,request_mps):
    old=limited(control([.85]*12,False),extra,request_mps)
    new=limited(control([.85]*12),extra,request_mps)
    assert old[0]==pytest.approx(max(0.,extra/.8))
    assert new[0]==request_mps and new[1]['reason']=='reserve_waits_low_motion'
    assert new[1]['applied'] is False and new[1]['free_space_assumed'] is False


@pytest.mark.parametrize('extra,request_mps',[(.16,.9),(-.1,1.4),(.3,.2)])
def test_normal_hold_and_lower_g1_request_are_never_raised(extra,request_mps):
    for request in (request_mps,0.):
        assert limited(control([.85]*12),extra,request)[0]==request


def test_moving_patch_with_measured_pace_keeps_closing_limit():
    # 0.35 m/s 앞차처럼 전진율이 측정되면 기존 접근 상한이 유지되어 정지 대비 거리가 남는다.
    ctrl=control([.25]*12)
    speed,detail=limited(ctrl,.16,.9)
    assert detail['reason']=='advancing_patch_close' and speed==pytest.approx(.45,abs=.01)


@pytest.mark.parametrize('extra,expected',[(.16,.20),(-.1,0.)])
def test_three_low_motion_scans_restore_fixed_reserve_for_blocking_patch(extra,expected):
    ctrl=control([0.]*6)
    speed,detail=limited(ctrl,extra,1.2)
    assert speed==pytest.approx(expected) and detail['reason']=='observed_entry_distance_reserve'


def test_two_low_motion_scans_are_not_enough():
    new=limited(control([0.]*2),-.1,1.2)
    old=limited(control([0.]*2,False),-.1,1.2)
    assert new[0]==1.2 and new[1]['reason']=='reserve_waits_low_motion' and old[0]==0.


def test_lead_that_stops_regains_reserve_within_three_scans():
    speeds=[.85]*6
    results=[limited(control(speeds+[0.]*n),-.1,1.2)[0] for n in (1,2,3,4)]
    assert results[0]==1.2 and results[-1]==0.
    assert results==sorted(results,reverse=True)


def test_preparation_waits_for_three_low_motion_scans():
    short=control([0.]*2)._preparation(None,None,(.8,0.,{}),dict(observed=True),0.)
    assert short['applied'] is False and short['reason']=='short_low_motion_history'
    single=control([0.])._preparation(None,None,(.8,0.,{}),dict(observed=True),0.)
    assert single['reason']=='single_scan_patch'


def test_entry_observation_continues_without_separate_zero_hold():
    h=follow_space()
    h.execution=replace(h.execution,moving_follow_reserve=True,closing_follow_reserve=True,low_motion_reserve=True)
    fast_normal(h,.3)
    called=[]
    original=h._execution_entries
    h._execution_entries=lambda *args:(called.append(1) or original(*args))
    for now in (.5,.6,.7):
        scan,history,scene=context(now)
        output=h.command(scene,history,.3,'left',0.,.02,raw_scan=scan)
    assert called and output[2]['preparation']['applied'] is True
    # 0명령이 나와도 거리 상한의 값이다. 별도 대기로 상한보다 내리지 않는다.
    assert output[2]['entry_space']['hold'] is False
    limit=output[2]['follow_entry_space']
    assert limit['reason']=='observed_entry_distance_reserve'
    assert output[0]==pytest.approx(min(.3,limit['cap_mps']))


@pytest.mark.parametrize('requested,recorded,valid',[
    ('low_motion',True,True),('low_motion',False,False),('closing',True,False),('closing',False,True)])
def test_option_must_be_present_in_actual_trace(tmp_path,requested,recorded,valid):
    path=tmp_path/'header.jsonl'
    ex=dict(moving_follow_reserve=True,closing_follow_reserve=True,low_motion_reserve=recorded)
    path.write_text(json.dumps(dict(kind='header',data=dict(follow_config={},execution_config=ex))))
    assert handoff.inspect_header(path,requested)['checks']['low_motion'] is valid


def test_launch_and_evaluator_accept_only_the_explicit_option():
    import importlib.util
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    spec=importlib.util.spec_from_file_location('low_motion_launch_check',root/'src/arena_bringup/launch/simulation.launch.py')
    launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)
    normal=dict(guarded=True,actor='test_actor.sdf',evidence='test_run',track='official')
    assert launch._autonomy_executable('local_pursuit','low_motion',**normal)=='g1_g2_handoff_trial'
    assert launch._autonomy_executable('local_pursuit','off',**normal)=='local_pursuit'
    recorded=dict(actuation_mode='virtual_mcu',autonomy_mode='local_pursuit',speed_bump=False,lidar_rate_hz=10.,
                  lidar_compensation='both',lidar_acquisition='sequential',speed_profile='g1_open_1p5',control_rate_hz=50.)
    assert 'evaluation_handoff_reserve:=low_motion' in handoff.launch_arguments('g1g2','low_motion',recorded,True)


def test_pace_floor_is_off_by_default_and_requires_low_motion_reserve():
    assert HandoffExecutionConfig().moving_pace_floor is False
    with pytest.raises(ValueError,match='pace floor'):
        replace(follow_space().execution,moving_pace_floor=True)


@pytest.mark.parametrize('extra,request_mps,expected',[
    (.3,1.4,.875),  # 0.5 + (0.30 m / 0.8 s)
    (-.1,1.4,.5),   # 여유 안쪽이어도 0이 아니라 가정 속도까지
    (.3,.4,.4),     # G1 상한 준수
    (.3,0.,0.),     # 정상 HOLD를 전진으로 바꾸지 않는다
])
def test_pace_floor_limits_fast_patch_without_zero_command(extra,request_mps,expected):
    speed,detail=limited(control([.85]*12,floor=True),extra,request_mps)
    assert speed==pytest.approx(expected)
    assert detail['reason']=='reserve_waits_low_motion' and detail['free_space_assumed'] is False


def test_pace_floor_is_never_below_the_old_zero_pace_reserve():
    for extra in (-.3,-.1,0.,.1,.3):
        old=limited(control([.85]*12,False),extra,1.4)[0]
        assert limited(control([.85]*12,floor=True),extra,1.4)[0]>=old


def test_pace_floor_keeps_blocking_patch_rules():
    speed,detail=limited(control([0.]*6,floor=True),-.1,1.2)
    assert speed==0. and detail['reason']=='observed_entry_distance_reserve'
    assert limited(control([.25]*12,floor=True),.16,.9)[1]['reason']=='advancing_patch_close'


def test_pace_floor_must_be_present_in_actual_trace(tmp_path):
    path=tmp_path/'header.jsonl'
    ex=dict(moving_follow_reserve=True,closing_follow_reserve=True,low_motion_reserve=True,moving_pace_floor=True)
    path.write_text(json.dumps(dict(kind='header',data=dict(follow_config={},execution_config=ex))))
    assert handoff.inspect_header(path,'paced')['checks']['pace_floor'] is True
    assert handoff.inspect_header(path,'low_motion')['checks']['pace_floor'] is False
