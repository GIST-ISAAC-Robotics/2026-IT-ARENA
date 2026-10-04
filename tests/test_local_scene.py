from dataclasses import FrozenInstanceError, fields
import math

import pytest

from arena_autonomy.local_scene import LocalScene, SensorRay, rear_masked, scene_from_scan, MAX_RAYS


def build(values=(1., 1.), **overrides):
    args = dict(angle_min=0., angle_increment=.1, range_min=.05, range_max=12.,
                acquired_at_s=[1. + i*.002 for i in range(len(values))], t_ref_s=1.1)
    args.update(overrides)
    return scene_from_scan(values, **args)


def test_scan_copies_input_and_only_endpoints_are_occupancy():
    values, times, sensor, poses = [1., math.inf, math.nan, .01, 13.], [1.]*5, [.06, 0.], [[0., 0., 0.]]*5
    scene = build(values, acquired_at_s=times, sensor_xy_m=sensor, acquisition_to_ref_se2=poses)
    values[0], times[0], sensor[0], poses[0][0] = 99., 99., 99., 99.
    assert [ray.status for ray in scene.rays] == ['return'] + ['unknown']*4
    assert scene.occupied_endpoints_xy_m == ((1.06, 0.),)
    assert scene.rays[0].ray_origin_xy_m == (.06, 0.)
    assert scene.source_first_s == scene.source_last_s == 1.
    with pytest.raises(FrozenInstanceError):
        scene.t_ref_s = 7.


def test_per_ray_origin_and_hit_transform_preserves_source_times():
    scene = build(acquisition_to_ref_se2=[(-.1, 0., 0.), (0., .2, math.pi/2)])
    assert scene.rays[0].ray_origin_xy_m == pytest.approx((-.04, 0.))
    assert scene.rays[0].hit_xy_m == pytest.approx((.96, 0.))
    assert scene.rays[1].ray_origin_xy_m == pytest.approx((0., .26))
    assert scene.rays[1].hit_xy_m == pytest.approx((-math.sin(.1), .26+math.cos(.1)))
    assert [r.acquired_at_s for r in scene.rays] == [1., 1.002]
    assert scene.source_last_s != scene.t_ref_s


def test_empty_has_no_new_observation_and_contract_has_no_truth_fields():
    scene = build(())
    assert scene.rays == () and scene.source_first_s is scene.source_last_s is None
    assert {f.name for f in fields(LocalScene)} == {
        't_ref_s', 'source_first_s', 'source_last_s', 'source_frame_id', 'source_epoch', 'rays', 'frame_id'}
    with pytest.raises(TypeError):
        LocalScene(1., None, None, 'laser', 0, (), actor_id=1)


@pytest.mark.parametrize('kwargs', [
    {'t_ref_s': math.inf}, {'t_ref_s': math.nan}, {'t_ref_s': .5}, {'t_ref_s': -1.},
    {'acquired_at_s': [1., .9]}, {'acquired_at_s': [1., math.nan]},
    {'acquired_at_s': [1.]}, {'source_epoch': -1}, {'source_epoch': 1.2},
    {'source_frame_id': ''}, {'angle_increment': 0.}, {'range_max': .01},
    {'acquisition_to_ref_se2': [(0., 0., 0.)]}, {'sensor_xy_m': (math.inf, 0.)},
])
def test_invalid_input_rejected(kwargs):
    with pytest.raises(ValueError):
        build(**kwargs)


def test_size_epoch_and_fabricated_time_rejected():
    with pytest.raises(ValueError):
        build([1.]*(MAX_RAYS+1))
    ray = SensorRay((0., 0.), (1., 0.), 1., 'return', 'laser', 0, 0, (1., 0.))
    with pytest.raises(ValueError):
        LocalScene(2., 2., 2., 'laser', 0, (ray,))
    with pytest.raises(ValueError):
        LocalScene(2., 1., 1., 'laser', 1, (ray,))
    with pytest.raises(ValueError):
        SensorRay((0., 0.), (1., 0.), 1., 'unknown', 'laser', 0, 0, (1., 0.))


def test_unknown_direction_transforms_with_per_ray_yaw_and_validates_unit():
    scene = build((math.inf,), sensor_yaw_rad=.3, acquisition_to_ref_se2=[(0., 0., .5)])
    assert scene.rays[0].hit_xy_m is None
    assert scene.rays[0].direction_xy == pytest.approx((math.cos(.8), math.sin(.8)))
    with pytest.raises(ValueError):
        SensorRay((0., 0.), None, 1., 'unknown', 'laser', 0, 0, (2., 0.))


def test_angle_wrap_mask_boundary_and_sensor_offset():
    assert not rear_masked(math.radians(150))
    assert not rear_masked(math.radians(-150))
    assert rear_masked(math.radians(150.0001))
    assert rear_masked(3*math.pi)
    scene = build((1.,), angle_min=2*math.pi)
    assert scene.rays[0].hit_xy_m == pytest.approx((1.06, 0.))
    masked = build((1.,), angle_min=math.pi)
    assert masked.rays[0].status == 'masked' and masked.rays[0].hit_xy_m is None

