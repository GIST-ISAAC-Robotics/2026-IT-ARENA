"""별개 측방 공백을 전방 폭으로 세지 않는다. 회전/연속 공백 전체는 포함한다."""
import math
from dataclasses import replace

import numpy as np
import pytest

from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.scan_input import scan_angles, project_scan
from test_local_follow import history
from test_partial_observation import sample
from test_native_scan_pipeline import native


def mixed(representation='uniform', side=32., *, front=28., connected=False):
    raw = sample(.5) if representation=='uniform' else native(clockwise=representation=='cw')
    angles = np.degrees(scan_angles(raw))
    indices = sorted(set(np.argsort(abs(angles-front))[:2].tolist()
                         + np.argsort(abs(angles-side))[:2].tolist()))
    if connected:
        indices = list(range(min(indices), max(indices)+1))
    values = list(raw.ranges)
    for index in indices:
        values[index] = math.inf
    if representation=='uniform':
        raw.ranges = values
    else:
        raw = replace(raw, ranges_m=tuple(values))
    return raw


def check(raw, *, w=0., legacy=False, previous=()):
    p = PartialObservationPolicy(True, not legacy, not legacy)
    h = history(w=w)
    p.evaluate(sample(.4), previous, h, .5, .2)
    points, _, _ = project_scan(raw, h, .62, timing_verified=True)
    return p.evaluate(raw, points, h, .62, .2)


@pytest.mark.parametrize('representation', ['uniform','cw','ccw'])
@pytest.mark.parametrize('sign', [-1.,1.])
def test_separate_side_does_not_spend_forward_budget(representation, sign):
    raw = mixed(representation, side=sign*32., front=sign*28.)
    before = tuple(raw.ranges)
    result = check(raw)
    assert result['allowed'] and result['reason']=='bounded_forward_missing', result
    assert result['forward_gap_deg'] <= 1.5 < result['total_gap_deg'] <= 6.
    assert result['speed_cap_mps']==.2 and not result['unknown_filled']
    assert tuple(raw.ranges)==before


def test_legacy_forward_policy_remains_unchanged():
    assert check(mixed(), legacy=True)['reason']=='forward_gap_too_wide'


def test_connected_gap_crossing_boundary_is_not_split():
    result = check(mixed(connected=True))
    assert not result['allowed']


def test_two_forward_groups_still_share_one_budget():
    result = check(mixed(side=-28.))
    assert result['reason']=='forward_gap_too_wide' and not result['allowed']
    assert result['forward_gap_deg']==pytest.approx(result['total_gap_deg'])


@pytest.mark.parametrize('w', [1.,-1.])
def test_rotation_of_either_side_into_forward_counts_whole_run(w):
    sign = 1 if w>0 else -1
    result = check(mixed(side=sign*32., front=sign*28.), w=w)
    assert not result['allowed'] and result['forward_gap_deg']>1.5, result


def test_prior_front_obstacle_cannot_be_erased_by_separate_side_gap():
    angle=math.radians(28.)
    result=check(mixed(), previous=[[.06+.25*math.cos(angle),.25*math.sin(angle)]])
    assert not result['allowed'] and result['reason']=='known_obstacle_latched'
