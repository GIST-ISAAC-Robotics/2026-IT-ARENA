"""활성 회랑 뒤 실제 입력 결측/복구 계약. 물리 제동이나 ROS 전달 시험은 아니다."""
import math
from dataclasses import replace
import numpy as np
import pytest

from arena_autonomy.local_follow import LocalFollow
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.lidar_observation import audit_front_observation
from probe_corner_observability import annular_scan
from test_corridor_follow import config
from test_local_follow import history


def observed(now, value=None, degree=86.4):
    ranges, *_ = annular_scan(.15)
    if value is not None:
        index = round((math.radians(degree)+math.pi)/(math.tau/500))
        ranges[index] = value
    scene = scene_from_scan(ranges.tolist(), angle_min=-math.pi, angle_increment=math.tau/500,
        range_min=.05, range_max=12., t_ref_s=now,
        acquired_at_s=(now-.1+np.arange(500)*.0002).tolist(), rear_half_angle_deg=30.)
    return ranges, scene


def enabled():
    ctl, h = LocalFollow(config()), history()
    for now in (.4, .5, .6):
        _, scene = observed(now)
        output = ctl.command(scene, h, 0., 'left', .2, .02)
    assert output[0] > 0 and output[2]['corridor_selected']
    return ctl, h


@pytest.mark.parametrize('missing', [math.inf, -math.inf, math.nan, .01, 12.1])
@pytest.mark.parametrize('degree', [0., 86.4, -86.4])
def test_current_unknown_stops_active_corridor_and_requires_new_confirmations(missing, degree):
    ctl, h = enabled()
    ranges, scene = observed(.7, missing, degree)
    original = ranges.copy()
    audit = audit_front_observation(ranges, -math.pi, math.tau/500, .05, 12.)
    output = ctl.command(scene, h, 0., 'left', .2, .02)
    assert audit.reason == 'front_unobserved' and audit.unknown_samples == 1
    assert output[0] == 0 and output[2]['reason'] == 'follow_front_unobserved'
    assert not output[2]['corridor_selected'] and not output[2]['corridor_candidate']['positive']
    assert ctl.corridor_observer.path is None and ctl.corridor_observer.confirmations == 0
    np.testing.assert_array_equal(original, ranges)
    for now in (.8, .82, .84, .9):
        # 동일 취득 스캔의 재사용은 새로운 확인 횟수가 아니다.
        source_now = .8 if now < .9 else .9
        _, fixed = observed(source_now)
        fixed = replace(fixed, t_ref_s=now)
        output = ctl.command(fixed, h, 0., 'left', .2, .02)
        assert output[0] == 0 and not output[2]['corridor_selected']
    _, recovered = observed(1.0)
    result = ctl.command(recovered, h, 0., 'left', .2, .02)
    assert result[0] > 0 and result[2]['corridor_selected']
    assert ctl.path is None and not result[2]['boundary_memory_from_corridor']


def test_exact_finite_range_max_is_not_equated_with_missing_return():
    values = [1.]*500
    values[370] = 12.
    assert audit_front_observation(values, -math.pi, math.tau/500, .05, 12.).reason == 'ok'
    values[370] = math.inf
    assert audit_front_observation(values, -math.pi, math.tau/500, .05, 12.).reason == 'front_unobserved'
