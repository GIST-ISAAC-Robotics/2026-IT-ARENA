"""별도 중속 프로필: 기본 보존, 속도 이점·포기·현재 경로 검사. 주행 증명 아님."""
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest

from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.front_patch_motion import ObservedForwardPace
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.moving_pass import MovingEntryConfig, MovingPassAdapter, MovingTargetTrack
from test_moving_pass import moving_controller, history, scene


def medium_controller():
    slow = moving_controller()
    return BypassHandoffFollow(slow.config, replace(slow.execution, moving_pass_profile='medium_v1'),
                               allow_simulation_trial=True)


def test_registered_profile_needs_explicit_trial_and_fits_feedback_range():
    assert HandoffExecutionConfig().moving_pass_profile == 'slow_v1'
    for name in ('medium_v1', 'unknown'):
        with pytest.raises(ValueError, match='moving pass profile'):
            HandoffExecutionConfig(moving_pass_profile=name)
    with pytest.raises(ValueError): MovingEntryConfig(speed_cap_mps=.9)
    assert MovingEntryConfig(profile='medium_v1', speed_cap_mps=.9).speed_cap_mps == .9
    for v in (.9001, 0., True, float('nan')):
        with pytest.raises(ValueError): MovingEntryConfig(profile='medium_v1', speed_cap_mps=v)
    ctrl = medium_controller()
    with pytest.raises(ValueError, match='feedback envelope'):
        MovingPassAdapter(SimpleNamespace(config=replace(ctrl.config,max_speed_mps=.8),execution=ctrl.execution))


@pytest.mark.parametrize('lead,ready',[(.3,True),(.5,True),(.55,True),(.56,False),(.7,False)])
def test_observed_displacement_and_error_select_medium_not_future_truth(lead, ready):
    h = history(); pace = ObservedForwardPace()
    for k in range(5):
        source = .1 + k*.1
        pace.observe(source, source+.1, [1.+lead*k*.1,0.], h)
    ctrl = medium_controller(); ctrl.monitor.forward_pace = pace
    out = ctrl.moving_pass.eligibility(scene(.6),h)
    assert bool(out['ready']) == ready
    assert out['estimated_lead_upper_mps'] == pytest.approx(lead+.15)
    assert not out['future_speed_guaranteed']


@pytest.mark.parametrize('chosen,lead,ready',[(.9,.65,True),(.85,.65,True),(.84,.65,False),(.65,.65,False),(.9,float('nan'),False)])
def test_actual_candidate_must_have_advantage_even_when_profile_cap_is_high(chosen,lead,ready):
    assert medium_controller().moving_pass.entry_advantage_sufficient(chosen,lead) is ready
    # 옛 저속 기록의 양수 속도 차 조건은 기본 프로필에서 유지한다.
    assert moving_controller().moving_pass.entry_advantage_sufficient(.651,.65)


@pytest.mark.parametrize('lead,loss',[(.5,False),(.58,True)])
def test_fresh_front_observations_use_profile_withdrawal_threshold(lead,loss):
    track = MovingTargetTrack(FollowConfig(), 'medium_v1'); h = history()
    for i in range(7):
        s=scene(.5+.1*i)
        track.current=np.array([[1.+lead*.1*i,-.20],[1.01+lead*.1*i,-.19]])
        track.source=s.source_first_s; track.detail['observed']=True
        track._observe_speed(s,h)
    assert (track.lost_advantage_scans >= 3) is loss
    before=track.lost_advantage_scans
    track._observe_speed(s,h)
    assert track.lost_advantage_scans == before  # 같은 스캔을 재사용해 확인 횟수를 늘리지 않는다.


def test_medium_yields_gradually_and_reset_preserves_profile():
    adapter = medium_controller().moving_pass
    assert adapter.tracker.association_speed_mps == 1.
    adapter.tracker.lost_advantage_scans=3; adapter.tracker.speed_lower=.6
    previous=.9
    for _ in range(60):
        cap=adapter._yield_before_return(.02).speed_cap_mps
        assert 0 <= previous-cap <= .0100000001
        previous=cap
    assert cap == pytest.approx(.4)
    adapter.reset()
    assert adapter.guide.entry.speed_cap_mps == .9 and adapter.withdrawal_speed_cap == .9
    assert adapter.tracker.profile.lead_upper_mps == .7


def test_evaluation_arguments_cannot_silently_request_medium():
    from g1_handoff_eval import launch_arguments
    launch=dict(actuation_mode='virtual_mcu',autonomy_mode='local_pursuit',speed_bump=False,
        lidar_rate_hz=10.,lidar_compensation='both',lidar_acquisition='sequential',
        speed_profile='g1_open_1p5',control_rate_hz=50.)
    with pytest.raises(ValueError,match='profile'):
        launch_arguments('g1g2','paced',launch,True,moving_profile='medium_v1')
    args=launch_arguments('g1g2','paced',launch,True,moving_profile='medium_v1',moving_pass=True,
                          continuous_pace=True,observed_motion=True)
    assert 'evaluation_moving_pass_profile:=medium_v1' in args
