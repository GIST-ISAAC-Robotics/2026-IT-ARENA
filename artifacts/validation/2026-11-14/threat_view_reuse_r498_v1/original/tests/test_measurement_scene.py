"""실물 연결 전, 비균일/시계 방향 입력을 재표본화하지 않는 투영 계약."""
import math

import pytest

from arena_autonomy.local_scene import MAX_RAYS, scene_from_measurements, scene_from_scan
from arena_autonomy.observation_support import source_valid


def build(values=(1., 2., 3.), **changes):
    params = dict(angles_rad=[.01, -.003, -.021], range_min=.05, range_max=12.,
                  acquired_at_s=[1., 1.0002, 1.0004], t_ref_s=1.01,
                  rear_half_angle_deg=0.)
    params.update(changes)
    return scene_from_measurements(values, **params)


def test_clockwise_nonuniform_wrap_keeps_original_index_time_and_range():
    angles = [-math.pi+.01, math.pi-.004, math.pi-.022]
    stamps = [1., 1.00019, 1.00041]
    result = build(angles_rad=angles, acquired_at_s=stamps)
    for i, (ray, angle) in enumerate(zip(result.rays, angles)):
        assert ray.source_index == i and ray.acquired_at_s == stamps[i]
        assert ray.direction_xy == pytest.approx((math.cos(angle), math.sin(angle)))
        assert ray.hit_xy_m == pytest.approx((.06+(i+1)*math.cos(angle), (i+1)*math.sin(angle)))
    assert result.source_first_s == stamps[0] and result.source_last_s == stamps[-1]


@pytest.mark.parametrize('count', [412, 424, 485, 500, 508, 524, 535, 640, 720])
def test_variable_count_is_neither_padded_nor_resampled(count):
    angles = [.017-i*math.tau/count+(i % 3)*.0001 for i in range(count)]
    stamps = [1.+i*.0002 for i in range(count)]
    result = build([1.]*count, angles_rad=angles, acquired_at_s=stamps, t_ref_s=1.2)
    assert len(result.rays) == count
    assert [r.source_index for r in result.rays] == list(range(count))
    assert [r.acquired_at_s for r in result.rays] == stamps
    assert [r.direction_xy for r in result.rays] == pytest.approx(
        [(math.cos(math.remainder(a, math.tau)), math.sin(math.remainder(a, math.tau))) for a in angles])
    # 장면 투영 성공을 기존 500점·반시계 방향의 주행 관문 승인으로 확대하지 않는다.
    assert not source_valid(result)


def test_equal_angles_remain_distinct_acquisitions_without_fake_new_points():
    result = build(angles_rad=[0., 0., -.015])
    assert len(result.rays) == 3
    assert result.rays[0].direction_xy == result.rays[1].direction_xy
    assert result.rays[0].source_index != result.rays[1].source_index
    assert result.rays[0].acquired_at_s != result.rays[1].acquired_at_s
    assert result.rays[0].hit_xy_m != result.rays[1].hit_xy_m


def test_nonuniform_unknown_and_mount_mask_keep_their_directions():
    result = build((math.inf, 2., 0.), angles_rad=[.2, .3, -.2],
                  rear_half_angle_deg={'sectors_deg': [[math.degrees(.3), 1.]]})
    assert [r.status for r in result.rays] == ['unknown', 'masked', 'unknown']
    assert all(r.hit_xy_m is None for r in result.rays)
    assert result.rays[0].direction_xy == pytest.approx((math.cos(.2), math.sin(.2)))


def test_explicit_per_ray_pose_transforms_each_origin_and_direction():
    result = build((1.,), angles_rad=[-.2], acquired_at_s=[1.], sensor_yaw_rad=.1,
                  sensor_xy_m=(.06, .02), acquisition_to_ref_se2=[(.3, -.1, math.pi/2)])
    ray = result.rays[0]
    assert ray.ray_origin_xy_m == pytest.approx((.28, -.04))
    assert ray.direction_xy == pytest.approx((math.sin(.1), math.cos(.1)))
    assert ray.hit_xy_m == pytest.approx((.28+math.sin(.1), -.04+math.cos(.1)))
    assert ray.acquired_at_s == 1. and result.t_ref_s == 1.01


@pytest.mark.parametrize('count', [0, 1, 424, 500, 508, 720])
def test_explicit_uniform_input_matches_existing_scan_exactly(count):
    ranges = [math.inf if i % 11 == 0 else .5+i*.01 for i in range(count)]
    start, step = -math.pi, math.tau/max(count, 1)
    params = dict(range_min=.05, range_max=12., acquired_at_s=[1.+i*.0002 for i in range(count)],
                  t_ref_s=1.2, sensor_yaw_rad=.12,
                  acquisition_to_ref_se2=[(-i*.0001, .02, i*.00003) for i in range(count)],
                  rear_half_angle_deg={'rear_half_angle_deg': 12., 'sectors_deg': [[90., 2.]]})
    legacy = scene_from_scan(ranges, angle_min=start, angle_increment=step, **params)
    explicit = scene_from_measurements(ranges, angles_rad=[start+i*step for i in range(count)], **params)
    assert explicit == legacy


@pytest.mark.parametrize('changes', [
    {'angles_rad': [0.]}, {'angles_rad': [0., math.nan, 1.]},
    {'angles_rad': [0., math.inf, 1.]}, {'angles_rad': [0., True, 1.]},
    {'acquired_at_s': [1., .99, 1.01]}, {'acquired_at_s': [1.]},
    {'t_ref_s': .9}, {'acquisition_to_ref_se2': [(0., 0., 0.)]},
    {'range_max': .01}, {'source_epoch': -1}, {'source_frame_id': ''},
])
def test_invalid_input_is_not_silently_sorted_or_filled(changes):
    with pytest.raises(ValueError):
        build(**changes)


def test_input_is_copied_and_maximum_size_still_applies():
    values, angles, stamps = [1.], [.1], [1.]
    result = build(values, angles_rad=angles, acquired_at_s=stamps)
    values[0], angles[0], stamps[0] = 9., 2., 5.
    assert result.rays[0].acquired_at_s == 1.
    assert result.rays[0].direction_xy == pytest.approx((math.cos(.1), math.sin(.1)))
    with pytest.raises(ValueError):
        build([1.]*(MAX_RAYS+1), angles_rad=[0.]*(MAX_RAYS+1), acquired_at_s=[1.]*(MAX_RAYS+1))
