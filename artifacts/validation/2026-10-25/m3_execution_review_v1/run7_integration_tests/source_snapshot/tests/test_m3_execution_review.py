"""GPT 인계 검토의 독립 반례. 기존 후보의 지지/정지 주장 범위를 확인한다."""
from dataclasses import replace
import numpy as np
import pytest

import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.observation_lateral import reachable_from_unknown
from test_m3_execution_trial import two_scans, GRASS, EXEC, ctl, stub
from test_m3_observed_follow import timed_scene
from test_local_follow import history
from arena_autonomy.lateral_intent import PathFrame


def test_radial_history_support_rejects_old_lateral_unknown_reach():
    """거리방향 padding만으로 옆 unknown→질의 이동을 배제했다고 할 수 없다."""
    tail = np.array([[-.095, .0755], [-.09, .0752], [-.095, -.0755]])
    now, _, frames = two_scans(.7)
    old, (dx, dy, yaw) = frames[0]
    c, s = np.cos(yaw), np.sin(yaw)
    old_queries = (tail-[dx, dy])@np.array([[c, -s], [s, c]])
    radial = mx.depth_support(old, old_queries, mx.ray_padding(
        replace(old, t_ref_s=now.t_ref_s), GRASS, EXEC.history_motion_bound_mps))
    assert radial.all()
    reachable = reachable_from_unknown(replace(old, t_ref_s=now.t_ref_s), old_queries,
        FollowConfig(object_motion_bound_mps=EXEC.history_motion_bound_mps))
    # 자기몸체 밖에 있던 미관측 점의 도달 가능성이다. 실제 차량 존재를 뜻하지 않는다.
    assert reachable.any()
    guarded = mx.support(now, frames, tail, GRASS, EXEC)
    assert not guarded['supported'] and guarded['history_reachable_rejections'] > 0


@pytest.mark.parametrize('yaw_rate', [-.8, .8])
def test_encoder_zero_presteer_rejects_nonzero_gyro(yaw_rate):
    """모터축0·자이로 회전이 동시에 있으면 기존각을 유지한 정지로 남는다."""
    follow = ctl(mx.ExecutionConfig(masked_history_support=False))
    scene, path = timed_scene(.4)
    stub(follow, path, selected='straight')
    out = follow.command(scene, history(v=0., w=yaw_rate), 0., 'left', previous_steering=.2)
    assert out[:2] == (0., .2)
    assert out[2]['reason'] == 'm3_command_unobserved'
    assert not out[2]['m3_presteer']['motion_quiet']
    assert not out[2]['body_standstill_verified']


def test_presteer_requires_quiet_motor_history_not_just_latest_zero():
    moving = history(v=.2, w=0.)
    moving.add_mean_wheel(3.01, 0.)
    moving.add_gyro(3.01, 0.)
    assert not mx.presteer_motion_quiet(moving, 3.01)
    assert mx.presteer_motion_quiet(history(), .4)
    assert not mx.presteer_motion_quiet(history(end=.2), .4)


def test_history_can_support_distant_unknown_without_blanket_rejection():
    # 큰 좌표 이동의 기하 대조일 뿐 이 속도를 운용 목표로 사용하지 않는다.
    now, _, frames = two_scans(2.)
    tail = np.array([[-.095, .0755], [-.095, -.0755]])
    supported = mx.support(now, frames, tail, GRASS, EXEC)
    assert supported['supported'] and supported['history_supported'] == 2
    assert supported['history_reachable_rejections'] == 0


def test_exact_projection_cache_owns_inputs_and_outputs():
    t = np.linspace(-.3, 1.4, 40)
    path = np.column_stack((np.sin(t), 1.-np.cos(t)))
    frame, reference = mx.ExecutionFrame(path), PathFrame(path)
    q = np.random.default_rng(23).uniform([-.2, -.4], [1., .5], (300, 2))
    expected = reference.project(q)
    first = frame.project(q)
    for a, b in zip(first, expected):
        np.testing.assert_array_equal(a, b)
    first[0][:] = 100.
    for a, b in zip(frame.project(q.copy()), expected):
        np.testing.assert_array_equal(a, b)
    q += .01
    for a, b in zip(frame.project(q), reference.project(q)):
        np.testing.assert_array_equal(a, b)
