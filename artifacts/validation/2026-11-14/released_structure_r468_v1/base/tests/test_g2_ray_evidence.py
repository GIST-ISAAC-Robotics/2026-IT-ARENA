import math

import pytest

from arena_autonomy.local_scene import LocalScene, SensorRay, scene_from_scan
from g2_ray_evidence import QueryBox, ray_interval, query_evidence


def scene(distance, **kwargs):
    defaults = dict(angle_min=0., angle_increment=.01, range_min=.05, range_max=12.,
                    acquired_at_s=[1.], t_ref_s=1.1, sensor_xy_m=(0., 0.))
    defaults.update(kwargs)
    return scene_from_scan([distance], **defaults)


@pytest.mark.parametrize('distance,kind', [(.5, 'occluded_before'), (1., 'hit_or_margin'),
                                         (2., 'return_beyond'), (math.inf, 'unknown')])
def test_distinguishes_hit_shadow_and_beyond_without_free_claim(distance, kind):
    result = query_evidence(scene(distance), QueryBox(1, 0), now_s=1.1)
    assert result['counts'][kind] == 1
    assert not result['free_area_certified']
    assert not result['angular_gaps_filled']
    assert not result['driving_authorized']


def test_mask_is_not_a_return_or_free_space():
    result = query_evidence(scene(2., angle_min=math.pi), QueryBox(-1, 0), now_s=1.1)
    assert result['counts']['masked'] == 1


def test_old_ray_not_refreshed_by_scene_reference():
    result = query_evidence(scene(2., t_ref_s=1.4), QueryBox(1, 0), now_s=1.4)
    assert result['counts']['stale'] == 1
    assert result['details'][0]['acquired_at_s'] == 1.


def test_no_beam_and_source_origin_inside_query_are_distinct():
    result = query_evidence(scene(2.), QueryBox(1, 1), now_s=1.1)
    assert result['no_intersecting_beam']
    assert not result['free_area_certified']
    assert ray_interval((1, 0), (1, 0), QueryBox(1, 0)) == pytest.approx((0, .1))


def test_rotated_query_and_behind_ray():
    assert ray_interval((0, 0), (1, 0), QueryBox(1, 0, math.pi/2)) == pytest.approx((.925, 1.075))
    assert ray_interval((0, 0), (-1, 0), QueryBox(1, 0)) is None


def test_incoherent_endpoint_cannot_be_clear_evidence():
    ray = SensorRay((0, 0), (2, .01), 1., 'return', 'laser_frame', 0, 0, (1., 0.))
    data = LocalScene(1.1, 1., 1., 'laser_frame', 0, (ray,))
    result = query_evidence(data, QueryBox(1, 0), now_s=1.1)
    assert result['counts']['invalid_ray'] == 1


def test_moving_ray_origins_are_used_separately():
    rays = (SensorRay((0, 0), (2, 0), 1., 'return', 'laser_frame', 0, 0, (1., 0.)),
            SensorRay((0, .5), (2, .5), 1.01, 'return', 'laser_frame', 0, 1, (1., 0.)))
    data = LocalScene(1.1, 1., 1.01, 'laser_frame', 0, rays)
    result = query_evidence(data, QueryBox(1, 0), now_s=1.1)
    assert result['intersecting_rays'] == 1


@pytest.mark.parametrize('kwargs', [{'now_s': 1.2}, {'now_s': 1.1, 'max_age_s': .5},
                                   {'now_s': 1.1, 'endpoint_margin_m': -1}])
def test_wrong_reference_or_bounds_rejected(kwargs):
    with pytest.raises(ValueError):
        query_evidence(scene(2.), QueryBox(1, 0), **kwargs)
