from dataclasses import replace
import math
from types import SimpleNamespace

import numpy as np
import pytest

from candidate import PathProposal, current_candidate, proposal
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import scene_from_scan
from test_local_follow import history


def inputs(ranges=None):
    config = FollowConfig()
    data = dict(reference_s=.5, epoch=0, side='left', timing_verified=True,
                speed_mps=.2, previous_steering_rad=0.)
    scene = scene_from_scan([3.]*500 if ranges is None else ranges,
        angle_min=-math.pi, angle_increment=math.tau/500, range_min=.05, range_max=12.,
        acquired_at_s=(.4+np.arange(500)*.0002).tolist(), t_ref_s=.5)
    path = tuple((float(x), 0.) for x in np.linspace(-.1, 2., 43))
    plan = PathProposal(path, .4, .3, 0, 0, 'left', .35, 'baseline')
    return plan, data, SimpleNamespace(source_epoch=0), history(v=.2), scene, config


def test_current_tracking_is_finite_capped_and_preserves_original_time():
    out = current_candidate(*inputs())
    assert out['eligible'] and 0 < out['speed_mps'] <= .35
    assert out['steering_rad'] == pytest.approx(0.)
    assert out['original_source_first_s'] == .3 and out['current_source_first_s'] == .4
    assert out['path_points'][0][0] == pytest.approx(-.12)
    assert out['runtime_authorized'] is False and out['independent_guard_required']


@pytest.mark.parametrize('change', ['time', 'future', 'motion_epoch', 'sensor_epoch', 'side', 'timing'])
def test_boundaries_do_not_reuse_old_plan(change):
    p, d, scan, h, scene, cfg = inputs()
    if change == 'time': p = replace(p, reference_s=.19)
    if change == 'future': p = replace(p, reference_s=.6)
    if change == 'motion_epoch': d['epoch'] = 1
    if change == 'sensor_epoch': scan.source_epoch = 1
    if change == 'side': d['side'] = 'right'
    if change == 'timing': d['timing_verified'] = False
    assert not current_candidate(p, d, scan, h, scene, cfg)['eligible']


def test_new_front_obstacle_blocks_old_clear_path():
    ranges = [3.]*500
    ranges[245:256] = [.18]*11
    assert current_candidate(*inputs(ranges))['reason'] == 'current_gap_hold'


def test_new_front_unknown_is_not_replaced_by_old_path():
    ranges = [3.]*500
    ranges[247:254] = [math.nan]*7
    out = current_candidate(*inputs(ranges))
    assert not out['eligible'] and out['reason'] == 'large_motion_region_unobserved'


def test_zero_output_and_other_modes_are_not_positive_path_proposals():
    inp = dict(reference_s=.4, epoch=0, side='left', scan=dict(representation='native_measurements_v1',
        measurements=dict(source_epoch=0, acquired_at_s=[.3])))
    detail = dict(handoff_state='FOLLOW', normal=dict(reason='local_follow', selector_choice='baseline',
        path_points=[[i*.1, 0.] for i in range(20)]))
    assert proposal(inp, [.2, 0., detail]) is not None
    assert proposal(inp, [0., 0., detail]) is None
    assert proposal(inp, [.2, 0., {**detail, 'handoff_state':'BYPASS'}]) is None
    assert not current_candidate(None, *inputs()[1:])['eligible']
