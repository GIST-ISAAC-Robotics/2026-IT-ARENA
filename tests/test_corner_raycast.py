"""정적 첫 반환/후방 가림의 코너 회귀. 실제 회전 취득·Gazebo 접촉 시험과 구별한다."""
from dataclasses import replace
import math

import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig, LocalFollow
from test_local_follow import box, cast_scene, history


def elbow(entry, kind):
    theta = np.linspace(-math.pi/2, 0., 121)
    arc = np.column_stack((entry+.85*np.cos(theta), .425+.85*np.sin(theta)))
    segments = [((-3., -.425), (entry, -.425)), *list(zip(arc[:-1], arc[1:])),
                ((entry+.85, .425), (entry+.85, 3.)), ((entry+.85, 3.), (entry, 3.))]
    if kind not in ('missing_inner', 'short_fake_inner'):
        segments += [((-3., .425), (entry, .425))]
    if kind != 'missing_inner':
        segments += [((entry, .425), (entry, 3.))]
    if kind == 'center_block':
        segments += box(.26, 0., .20, .20)
    if kind == 'side_block':
        segments += box(.35, .18, .20, .20)
    if kind == 'short_fake_inner':
        segments += box(0., .50, .20, .15)
    return segments


@pytest.mark.parametrize('kind', ['open', 'center_block', 'side_block', 'missing_inner', 'short_fake_inner'])
@pytest.mark.parametrize('entry', [.15, .30, .50])
@pytest.mark.parametrize('side,sign', [('left', 1), ('right', -1)])
def test_corner_actual_first_returns_keep_stops_and_required_bilateral_evidence(kind, entry, side, sign):
    world = (np.asarray(elbow(entry, kind))*[1, sign]).tolist()
    controller = LocalFollow(FollowConfig(boundary_evidence_mode='corner_anchor_candidate',
                                          occupancy_timing_mode='per_ray_candidate'))
    h = history()
    for now in (.4, .5, .6, .7):
        observed = cast_scene(now, world)
        observed = replace(observed, rays=tuple(
            replace(ray, status='masked', hit_xy_m=None)
            if abs(math.atan2(ray.direction_xy[1], ray.direction_xy[0])) > math.radians(150)
            else ray for ray in observed.rays))
        speed, _, detail = controller.command(observed, h, 0., side)
    if kind == 'open' and entry in (.30, .50):
        assert speed > 0, detail
        assert detail['unfiltered_occupancy']
        assert detail['occupied_points'] == len(observed.occupied_endpoints_xy_m)
        if entry == .30:
            assert detail['boundary_selection'] == 'bilateral_observed_corner_anchor'
    else:
        assert speed == 0 and detail['behavior'] == 'HOLD', detail
