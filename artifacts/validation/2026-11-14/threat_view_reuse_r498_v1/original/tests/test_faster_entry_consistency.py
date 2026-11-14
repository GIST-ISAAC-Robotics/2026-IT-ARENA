"""추월 진입의 지속 가능 여유와 진입 거부 시 FOLLOW 출력 유지."""
from dataclasses import replace
from unittest.mock import patch

import pytest

from arena_autonomy.bypass_entry_trial import straight_reference
from arena_autonomy.bypass_handoff import BypassHandoffFollow, front_patch
from arena_autonomy.moving_pass import available_pass_offsets
from probe_bypass_entry import sensor_scene
from test_moving_pass import history
from test_moving_pass_faster import faster_controller


def controller(wall_half_width=.505):
    c = faster_controller()
    return BypassHandoffFollow(replace(c.config, wall_offset_m=wall_half_width),
                              c.execution, allow_simulation_trial=True)


def test_fixed_only_narrow_pass_is_not_admitted():
    ctrl = controller()
    s, _ = sensor_scene(targets=((1.2, -.12),), wall_half_width=.505)
    _, ref = straight_reference(s, ctrl.config)
    points, _ = front_patch(s, 'contiguous_three_v0')
    fixed, _ = available_pass_offsets(s, ctrl.config, ref['walls'], points,
                                      fixed_outer_candidate=True, maximum_offset=.35)
    assert len(fixed) > 1
    out = ctrl.moving_pass.guide.evaluate(s, history(), .7, 0., .02)
    assert out['reason'] == 'moving_pass_sustained_side_width_unavailable'
    assert out['selected'] is None and ctrl.moving_pass.guide.path is None
    assert not any(r['feasible'] for r in out['pass_width'].values())
    assert all(not r['fixed_outer_hypothesis'] for r in out['pass_width'].values())


def test_wider_sensor_corridor_can_still_propose_a_faster_pass():
    ctrl = controller(.6)
    s, _ = sensor_scene(targets=((1.2, -.20),), wall_half_width=.6)
    _, ref = straight_reference(s, ctrl.config)
    points, _ = front_patch(s, 'contiguous_three_v0')
    offsets, width = available_pass_offsets(s, ctrl.config, ref['walls'], points,
                                           maximum_offset=.35)
    assert len(offsets) > 1
    out = ctrl.moving_pass.guide.evaluate(s, history(), 0., 0., .02)
    assert out['reason'] != 'moving_pass_sustained_side_width_unavailable'
    assert out['selected'] is None  # 서로 다른 스캔의 벽 확인은 여전히 필요하다.
    for side in width:
        assert out['pass_width'][side]['low_m'] == width[side]['low_m']
        assert out['pass_width'][side]['high_m'] == width[side]['high_m']


@pytest.mark.parametrize('normal_speed', [0., .7, 1.1])
def test_unmaintainable_entry_preserves_normal_follow_or_actual_stop(normal_speed):
    ctrl = controller()
    s, _ = sensor_scene(targets=((1.2, -.12),), wall_half_width=.505)
    normal = (normal_speed, .01, {'reason': 'handoff_follow'})
    with patch.object(ctrl, '_base_command', return_value=normal), \
         patch.object(ctrl.moving_pass, 'eligibility', return_value={'ready': True}):
        out = ctrl.moving_pass.command(s, history(), .7, 'right', 0., .02, raw_scan=object())
    assert out[:2] == normal[:2]
    assert out[2]['moving_pass']['entry']['reason'] == 'moving_pass_sustained_side_width_unavailable'
    assert not ctrl.moving_pass.active and ctrl.moving_pass.guide.path is None
