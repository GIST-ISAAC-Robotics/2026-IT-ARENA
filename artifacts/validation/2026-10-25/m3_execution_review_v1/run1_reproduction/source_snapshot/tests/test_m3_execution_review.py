"""GPT 인계 검토의 독립 반례. 기존 후보의 지지/정지 주장 범위를 확인한다."""
from dataclasses import replace
import numpy as np

import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.observation_lateral import reachable_from_unknown
from test_m3_execution_trial import two_scans, GRASS, EXEC, ctl, stub
from test_m3_observed_follow import timed_scene
from test_local_follow import history


def test_radial_history_support_can_overlap_old_lateral_unknown_reach():
    """거리방향 padding만으로 옆 unknown→질의 이동을 배제했다고 할 수 없다."""
    tail = np.array([[-.095, .0755], [-.09, .0752], [-.095, -.0755]])
    now, _, frames = two_scans(.7)
    radial = mx.support(now, frames, tail, GRASS, EXEC)
    assert radial['supported'] and radial['history_supported'] == 3
    old, (dx, dy, yaw) = frames[0]
    c, s = np.cos(yaw), np.sin(yaw)
    old_queries = (tail-[dx, dy])@np.array([[c, -s], [s, c]])
    reachable = reachable_from_unknown(replace(old, t_ref_s=now.t_ref_s), old_queries,
        FollowConfig(object_motion_bound_mps=EXEC.history_motion_bound_mps))
    # 자기몸체 밖에 있던 미관측 점의 도달 가능성이다. 실제 차량 존재를 뜻하지 않는다.
    assert reachable.any()


def test_encoder_zero_presteer_currently_ignores_nonzero_gyro():
    """모터축0·자이로 회전이 동시에 있어도 현재 후보가 정지정렬을 승인하는지 재현."""
    follow = ctl(mx.ExecutionConfig(masked_history_support=False))
    scene, path = timed_scene(.4)
    stub(follow, path, selected='straight')
    out = follow.command(scene, history(v=0., w=.8), 0., 'left', previous_steering=.2)
    assert out[0] == 0. and out[1] != .2
    assert out[2]['reason'] == 'm3_straight_presteer'
    assert not out[2]['body_standstill_verified']
