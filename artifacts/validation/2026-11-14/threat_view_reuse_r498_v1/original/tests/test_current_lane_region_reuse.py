"""같은 proposal 호출의 영역만 재사용하고 다른 기하/미계산 영역은 재평가한다."""
from dataclasses import replace
import pytest
import arena_autonomy.current_lane_preference as mod
from test_current_lane_preference import setup


@pytest.mark.parametrize('change', ['same', 'missing', 'speed', 'geometry'])
def test_only_same_proposal_region_is_reused(setup, monkeypatch, change):
    ctl, _, call = setup
    region = dict(distance_m=.4, queries=200, unobserved=0, masked_left=0,
                  masked_right=0, masked_tail_extent_m=0., masked_front_samples=0)
    faster = dict(eligible=True, reason='parallel_current_position', target_speed_mps=.6,
                  steering_rad=.01, clearance_m=2., path_points=[[0.,0.],[1.,0.]],
                  measured_above_target_cap=True, measured_speed_mps=.3, measured_region=region)
    if change == 'missing': faster.pop('measured_region')
    if change == 'speed': faster['measured_speed_mps'] = .4
    if change == 'geometry': ctl.lat = replace(ctl.lat, width_m=ctl.lat.width_m+.01)
    monkeypatch.setattr(mod, 'faster_proposal', lambda *args: faster)
    calls = []
    def recalculate(*args):
        calls.append(args)
        return {**region, 'unobserved': 1}
    monkeypatch.setattr(mod, 'motion_region', recalculate)
    output = call()
    assert len(calls) == (0 if change == 'same' else 1)
    assert output[:2] == ((.6, .01) if change == 'same' else (.35, 0.))
    assert region['unobserved'] == 0
