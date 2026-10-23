import math
from dataclasses import replace

import numpy as np
import pytest
from shapely.geometry import Polygon

from arena_autonomy.local_follow import FollowConfig
from probe_observation_envelope import inspect_envelope, swept_region, unknown_sector, envelope_poses
from audit_corridor_sweep import sweep_poses
from test_corridor_visibility import scene


@pytest.mark.parametrize('steering', [0., .175, -.175, .37, -.37])
def test_cover_contains_dense_continuous_nominal_positions_outside_current_body(steering):
    from shapely.geometry import box
    cfg = FollowConfig()
    region, bound, _ = swept_region(steering, .8, cfg)
    original = box(-.1,-.075,.1,.075)
    corners = np.array([[-.1,-.075],[-.1,.075],[.1,.075],[.1,-.075]])
    assert bound > 0
    for _,x,y,a in sweep_poses(steering,.135,.8,.0017):
        c,s = math.cos(a),math.sin(a)
        physical = Polygon(corners@np.array([[c,s],[-s,c]])+[x,y]).difference(original)
        assert physical.difference(region).area < 1e-12


def unknown_at(angle):
    original = scene()
    rays = list(original.rays)
    i = round((angle+180)/.72)
    rays[i] = replace(rays[i], status='unknown', hit_xy_m=None)
    return replace(original, rays=tuple(rays))


def test_front_unknown_is_never_filled():
    result = inspect_envelope(unknown_at(0),0.,.5,math.tau/500,FollowConfig())
    assert not result['no_unknown_intersection']
    assert any(r['index']==250 for r in result['unknown_intersections'])
    assert not result['unknown_filled'] and not result['driving_authorized']


def test_recorded_ackermann_limit_is_used_without_rounding_or_clipping():
    limit = .3710583448410034
    cfg = replace(FollowConfig(), max_steering_rad=limit)
    p = envelope_poses(limit,.5,cfg)
    assert p[-1,3] == pytest.approx(math.tan(limit)/.135*.5)
    with pytest.raises(ValueError):
        envelope_poses(limit+.001,.5,cfg)


def test_side_unknown_cannot_be_globally_ignored_in_turn():
    a = inspect_envelope(unknown_at(90), .37,.7,math.tau/500,FollowConfig())
    assert any(r['index']==375 for r in a['unknown_intersections'])


def test_safety_margin_is_not_omitted_with_current_body():
    a = inspect_envelope(unknown_at(90), 0.,.5,math.tau/500,FollowConfig(),.055)
    assert any(r['index']==375 for r in a['unknown_intersections'])


def test_measurement_origin_is_used():
    original = scene().rays[250]
    region,_,_ = swept_region(0.,.5,FollowConfig())
    near = unknown_sector(original,.0063,region)
    distant = unknown_sector(replace(original,ray_origin_xy_m=(0.,5.)),.0063,region)
    assert near.intersects(region) and not distant.intersects(region)


@pytest.mark.parametrize('bad', [0., -.01, .04, math.nan])
def test_bad_cell_width_rejected(bad):
    region,_,_ = swept_region(0.,.5,FollowConfig())
    with pytest.raises(ValueError):
        unknown_sector(scene().rays[0],bad,region)
