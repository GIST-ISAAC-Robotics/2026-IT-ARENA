"""좁은 병렬 출발과 전방/측방 접근 반례. 실제 센서 정확도 보증은 아니다."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.parallel_lane_trial import ParallelLaneTrial, proposal
from arena_autonomy.parallel_side_departure import limited_clearance
from test_parallel_lane_trial import config, base


def cfg(**kw):
    return config(pass_release_trial=True, side_departure_trial=True, **kw)


def scene(first=1., side=-.16, extra=()):
    pts = np.array([[0., side], [0., .425], [1., .425], [1., -.425], *extra])
    return SimpleNamespace(t_ref_s=first+.1, source_first_s=first, source_epoch=0,
                           occupied_endpoints_xy_m=pts,
                           rays=[SimpleNamespace(status='return', acquired_at_s=first) for _ in pts])


def test_off_keeps_hold_and_on_moves_without_removing_side_return():
    s = scene(); pts = s.occupied_endpoints_xy_m.copy()
    off = proposal(s, base(), 0., 0., .02, replace(cfg(), side_departure_trial=False))
    on = proposal(s, base(), 0., 0., .02, cfg())
    assert off['reason'] == 'unchanged_gap_hold'
    assert on['eligible'] and on['target_speed_mps'] == pytest.approx(.3)
    assert on['side_departure']['applied'] and on['side_departure']['side_points'] == 1
    assert on['hidden_rear_risk_unresolved'] and not on['unknown_filled']
    np.testing.assert_array_equal(s.occupied_endpoints_xy_m, pts)


@pytest.mark.parametrize('side,extra', [(-.129, ()), (-.16, ((.15, 0.),)),
                                      (-.16, ((.18, -.16),)), (-.16, ((.30, 0.),))])
def test_close_side_front_corner_and_second_object_still_hold(side, extra):
    result = proposal(scene(side=side, extra=extra), base(), 0., 0., .02, cfg())
    assert not result['eligible'] and result['reason'] == 'unchanged_gap_hold'


@pytest.mark.parametrize('side,angle', [(-.16, -.03), (.16, .03)])
def test_previous_steering_toward_either_side_blocks_progress(side, angle):
    s = scene(side=side)
    clearance, info = limited_clearance(s.occupied_endpoints_xy_m, (0., angle), .105,
                                        0., .1, .02, cfg())
    assert clearance == 0 and info['reason'] == 'approaches_observed_side'


def test_actual_speed_stopping_distance_not_target_speed_controls_eligibility():
    low = proposal(scene(), base(), .4, 0., .02, cfg())
    high = proposal(scene(), base(), .8, 0., .02, cfg())
    assert low['eligible'] and low['target_speed_mps'] < .4
    assert not high['eligible'] and high['side_departure']['reason'] == 'motion_outside_window'


def test_two_fresh_confirmations_and_partial_rejection_remain():
    ctl = ParallelLaneTrial(cfg())
    assert ctl.apply(scene(1.), base(), 0., 0., .02)[0] == 0
    assert ctl.apply(scene(1.), base(), 0., 0., .02)[0] == 0
    assert ctl.apply(scene(1.1), base(), 0., 0., .02)[0] > 0
    partial = base(); partial[2]['partial_observation']['reason'] = 'bounded_side_missing'
    assert ctl.apply(scene(1.2), partial, 0., 0., .02)[0] == 0
    assert ctl.apply(scene(1.3), base(), 0., 0., .02)[0] == 0
    assert ctl.apply(scene(1.4), base(), 0., 0., .02)[0] > 0


def test_option_requires_traffic_selector_and_does_not_enable_solo():
    assert not FollowConfig().side_departure_trial
    for changes in ({'pass_release_trial': False}, {'solo_route_only': True}, {'side_departure_trial': 1}):
        with pytest.raises(ValueError, match='side departure'):
            replace(cfg(), **changes)


def test_eval_option_and_actual_header_must_agree(tmp_path):
    import json
    from pathlib import Path
    import g1_handoff_eval
    root = Path(__file__).resolve().parents[1]
    launch = json.loads((root/'config/tests/official_traffic_v1.json').read_bytes())['launch']
    args = g1_handoff_eval.launch_arguments('g1g2', 'paced', launch, True, side_departure=True)
    assert 'evaluation_side_departure_trial:=true' in args
    with pytest.raises(ValueError):
        g1_handoff_eval.launch_arguments('g1', None, launch, True, side_departure=True)
    p = tmp_path/'trace.jsonl'
    p.write_text(json.dumps(dict(kind='header', data=dict(follow_config={}, execution_config={}))))
    assert not g1_handoff_eval.inspect_header(p, 'paced', side_departure=True)['checks']['side_departure']
