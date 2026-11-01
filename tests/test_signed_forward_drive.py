"""음수 피드백과 전진 명령을 분리한다. 실제 접지/후방 밀림 안전의 검증은 아니다."""
from dataclasses import asdict, replace
import json
import math
from pathlib import Path

import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig, LocalFollow, forward_planning_speed
from arena_autonomy.lidar_safety import LidarSafety
from arena_autonomy.follow_trace import FollowTrace, replay_events
from arena_autonomy.bypass_entry_follow import BypassEntryFollow, BypassExecutionConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import history, scene, settle, scan
from test_partial_observation import candidate_node
from test_bypass_handoff import context, stubbed
from test_bypass_signed_hold_trial import active, inputs


def config(**kw):
    return FollowConfig(signed_motion_policy='forward_drive', **kw)


@pytest.mark.parametrize('velocity', [-.001, -.08129882628142206, -.20])
def test_finite_reverse_keeps_forward_path_and_signed_history(velocity):
    h=history(velocity)
    controller=LocalFollow(config())
    settle(controller,h)
    before=list(h.wheels.values)
    for t in (.8,.9,1.):
        output=controller.command(scene(t),h,velocity,'left')
        assert output[0]>0
        assert output[2]['measured_speed_mps']==velocity
        assert output[2]['forward_planning_speed_mps']==abs(velocity)
        assert output[2]['signed_history_preserved']
        assert not controller.held and controller.path is not None
    assert list(h.wheels.values)==before and all(v<0 for v in before)
    # 부호가 바뀌었다는 이유로 두 스캔 재출발 대기를 추가하지 않는다.
    assert controller.command(scene(1.1),h,0.,'left')[0]>0


def test_near_obstacle_still_holds_during_reverse_feedback():
    controller=LocalFollow(config());h=history(-.08);settle(controller,h)
    result=controller.command(scene(.8,obstacle_x=.10),h,-.08,'left')
    assert result[0]==0 and result[2]['reason']=='follow_gap_hold'


@pytest.mark.parametrize('bad', [math.nan,math.inf,-math.inf])
def test_nonfinite_is_not_mapped_to_planning_speed(bad):
    controller=LocalFollow(config());h=history();settle(controller,h)
    assert controller.command(scene(.8),h,bad,'left')[0]==0


def test_legacy_recording_default_is_explicitly_preserved():
    old=asdict(config());old.pop('signed_motion_policy')
    cfg=FollowConfig(**old)
    assert cfg.signed_motion_policy=='legacy_hold'
    controller=LocalFollow(cfg);h=history(-.08);settle(controller,h)
    assert controller.command(scene(.8),h,-.08,'left')[2]['reason']=='follow_reverse_motion'
    with pytest.raises(ValueError):FollowConfig(signed_motion_policy='ignore_sensors')


@pytest.mark.parametrize('condition', ['clear','obstacle','zero','stale','legacy'])
def test_independent_guard_still_decides_from_current_path(condition):
    node,commands,status=candidate_node()
    node.signed_motion_policy='legacy_hold' if condition=='legacy' else 'forward_drive'
    node.motion.history=history(-.08)
    before=list(node.motion.history.wheels.values)
    if condition=='obstacle':
        node.motion.project=lambda raw:(np.tile([.1,0.],(60,1)),60)
    elif condition=='zero':node.command.drive.speed=0.
    elif condition=='stale':node.motion.history=history(-.08,end=.2)
    LidarSafety.control(node)
    if condition=='clear':
        assert commands[-1].drive.speed>0
        assert json.loads(status[-1].data)['partial_observation']['measured_speed_mps']==pytest.approx(-.08)
        assert list(node.motion.history.wheels.values)==before
    else:assert commands[-1].drive.speed==0.


def test_recent_observation_guard_does_not_stop_only_because_feedback_is_negative():
    from test_recent_observation_safety import candidate_node as recent_node
    node,commands,_=recent_node();node.motion.history=history(-.1)
    node.signed_motion_policy='forward_drive'
    LidarSafety.control(node)
    assert commands[-1].drive.speed==pytest.approx(.2)


def test_g2_keeps_actual_signed_transport_and_matches_speed_bound_reference():
    sc,h=inputs()
    def make():
        follow=BypassEntryFollow(config(),BypassExecutionConfig(corridor_policy='corridor_v2_signed_hold'),
                                 allow_simulation_trial=True)
        follow.guide=active();follow.guide.config=follow.config
        return follow
    subject,reference=make(),make()
    x=subject.guide.path[0,0]
    backward=subject.command(sc,h,-.0325,'left',0.,.02)
    expected=reference.command(sc,h,.0325,'left',0.,.02)
    assert backward[:2]==expected[:2]
    assert backward[2]['reason'] not in ('entry_reverse_motion_hold','entry_reverse_recovery_wait')
    assert backward[2]['measured_speed_mps']==-.0325
    if subject.guide.path is not None:assert subject.guide.path[0,0]>x
    assert all(v<0 for v in h.wheels.values)


def test_active_handoff_does_not_reset_on_negative_sign():
    h,captured=stubbed();h.config=replace(h.config,signed_motion_policy='forward_drive')
    raw,hist,sc=context()
    h.command(sc,hist,.3,'left',.01,.02,raw_scan=raw)
    assert h.mode=='BYPASS'
    previous_sides=h.normal.pass_selector.sides
    out=h.command(sc,hist,-.0813,'left',.01,.02,raw_scan=raw)
    assert out[0]>0 and h.mode=='BYPASS'
    assert out[2]['measured_speed_mps']==-.0813
    assert h.normal.pass_selector.sides is previous_sides


def test_new_signed_policy_is_recorded_and_replayed(tmp_path):
    h=history(-.08);raw=scan();controller=LocalFollow(config())
    destination=tmp_path/'signed.jsonl'
    writer=FollowTrace(str(destination),h.config,controller.config)
    sc=motion_scene(raw,h,.6,timing_verified=True,rear_half_angle_deg=30.)
    out=controller.command(sc,h,-.08,'left')
    writer.record_call(raw,h,.6,0,-.08,'left',0.,.02,True,30.,out)
    writer.close()
    events=[json.loads(line) for line in destination.read_text().splitlines()]
    assert events[0]['data']['follow_config']['signed_motion_policy']=='forward_drive'
    result=replay_events(events)
    assert result['passed']
