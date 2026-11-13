"""명시 frame과 원 시각/기하의 계약. 실제 센서·ROS 노드·구동 실행 없음."""
from dataclasses import asdict, fields, replace
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from arena_autonomy.follow_trace import freeze, snapshot_call, restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable
from arena_autonomy.lidar_source_frame import REGISTERED_SOURCE_FRAMES, registered_source_frame
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.observation_support import RecentScenes, source_valid, sweep_support, transport_scene
from arena_autonomy.scene_motion import measurement_motion_scene, motion_scene
from arena_autonomy.scan_input import project_scan
from arena_autonomy.linear_observation import linear_observation
from arena_autonomy.corridor_observer import current_proposal, CorridorObserver
from test_native_measurement_motion import history, scan

FRAME = 'c1_unmounted_preview'
POLICY = 'host_estimated_v1'


def selected_history(frame=FRAME, policy=POLICY):
    result = history()
    result.config = replace(result.config, lidar_frame_id=frame)
    result.scan_time_policy = policy
    return result


def relabel_scene(scene, frame):
    return replace(scene, source_frame_id=frame,
        rays=tuple(replace(ray, source_frame_id=frame) for ray in scene.rays))


def round_scene(now=.4, frame='laser_frame', unknown=None, distance=2.):
    ranges = [distance]*500
    if unknown is not None: ranges[unknown] = math.nan
    return scene_from_scan(ranges, angle_min=-math.pi, angle_increment=math.tau/500,
        range_min=.05, range_max=12., t_ref_s=now,
        acquired_at_s=(now-.1+np.arange(500)*.0002).tolist(),
        source_frame_id=frame, rear_half_angle_deg=0.)


def test_new_field_is_last_and_legacy_mapping_and_position_keep_default():
    config = MotionConfig()
    names = [field.name for field in fields(config)]
    assert names[-1] == 'lidar_frame_id'
    legacy = {key:value for key,value in asdict(config).items() if key != 'lidar_frame_id'}
    assert MotionConfig(**legacy) == MotionConfig(*legacy.values()) == config
    assert config.lidar_frame_id == 'laser_frame'
    assert REGISTERED_SOURCE_FRAMES == ('laser_frame', FRAME)


@pytest.mark.parametrize('value', [None, False, 123, '', 'laser frame', '/laser_frame',
                                  'C1_unmounted_preview', 'c1_unmounted_preview ', 'other_laser', ['laser_frame']])
def test_unregistered_or_nonstring_configuration_is_not_coerced(value):
    assert not registered_source_frame(value)
    with pytest.raises(ValueError, match='registered lidar source frame'):
        MotionConfig(lidar_frame_id=value)


def test_default_frame_rejects_c1_even_with_explicit_time_permission():
    raw = scan(source_frame_id=FRAME, time_basis='unverified_estimate')
    with pytest.raises(MotionUnavailable, match='^scan_frame_mismatch$'):
        measurement_motion_scene(raw, history(), 1.12, timing_verified=False, timing_policy=POLICY)


@pytest.mark.parametrize('clockwise', [False, True])
def test_selected_c1_preserves_identical_geometry_and_source(clockwise):
    original = scan(clockwise=clockwise, source_epoch=4)
    physical = replace(original, source_frame_id=FRAME, time_basis='unverified_estimate')
    before = measurement_motion_scene(original, history(), 1.12, timing_verified=True,
                                      rear_half_angle_deg=0., motion_epoch=2)
    after = measurement_motion_scene(physical, selected_history(), 1.12, timing_verified=False,
                                     rear_half_angle_deg=0., motion_epoch=2)
    assert relabel_scene(before, FRAME) == after
    assert after.source_epoch == 23  # Cantor(4,2), 두 기존 세대 결합 유지.
    assert physical.source_epoch == 4 and physical.time_basis == 'unverified_estimate'
    assert source_valid(after)
    assert after.source_angles_rad == physical.angles_rad
    assert tuple(ray.acquired_at_s for ray in after.rays) == physical.acquired_at_s
    moved = transport_scene(after, selected_history(), 1.15)
    assert moved.source_frame_id == FRAME and moved.source_epoch == after.source_epoch
    assert tuple(ray.acquired_at_s for ray in moved.rays) == physical.acquired_at_s
    points, ids, meta = project_scan(physical, selected_history(), 1.12, timing_verified=False)
    baseline_points, baseline_ids, _ = project_scan(original, history(), 1.12, timing_verified=True)
    np.testing.assert_array_equal(points, baseline_points)
    np.testing.assert_array_equal(ids, baseline_ids)
    assert meta['hardware_timing_verified'] is False and meta['time_basis'] == 'unverified_estimate'
    assert meta['scan_stamp_s'] == physical.acquired_at_s[0]
    assert meta['last_ray_stamp_s'] == physical.acquired_at_s[-1]


@pytest.mark.parametrize('fault,reason', [
    ('other_registered_frame', 'scan_frame_mismatch'), ('unknown_frame', 'scan_frame_mismatch'),
    ('default_time_policy', 'scan_timing_unverified'), ('invalid_time_policy', 'scan_timing_policy_invalid'),
    ('simulation_basis', 'scan_timing_policy_basis_mismatch'), ('future', 'scan_future'), ('stale', 'scan_stale')])
def test_frame_selection_does_not_replace_existing_time_contract(fault, reason):
    raw = scan(source_frame_id=FRAME, time_basis='unverified_estimate')
    h = selected_history(); now = 1.12; policy = POLICY
    if fault == 'other_registered_frame': raw = replace(raw, source_frame_id='laser_frame')
    elif fault == 'unknown_frame': raw = replace(raw, source_frame_id='other_laser')
    elif fault == 'default_time_policy': policy = 'verified_only'
    elif fault == 'invalid_time_policy': policy = 'auto'
    elif fault == 'simulation_basis': raw = replace(raw, time_basis='simulation_acquisition')
    elif fault == 'future': now = 1.09
    elif fault == 'stale': now = 1.4
    with pytest.raises(MotionUnavailable, match='^'+reason+'$'):
        measurement_motion_scene(raw, h, now, timing_verified=False, timing_policy=policy)


def test_snapshot_json_roundtrip_preserves_config_frame_raw_arrays_and_false_verified():
    values = [2.]*508; values[20:23] = [math.nan, math.inf, -math.inf]
    raw = scan(source_frame_id=FRAME, time_basis='unverified_estimate', source_epoch=7, ranges_m=tuple(values))
    h = selected_history()
    captured = snapshot_call(raw, h, 1.12, 3, 1., 'left', .1, .02, False, 0.)
    wire = json.dumps(dict(config=asdict(h.config), call=captured), allow_nan=False)
    decoded = json.loads(wire)
    restored, same, history_again = restore_call(decoded['call'], MotionConfig(**decoded['config']))
    assert history_again.config == h.config and history_again.scan_time_policy == POLICY
    assert restored['timing_verified'] is False and restored['timing_policy'] == POLICY
    assert freeze(asdict(same)) == freeze(asdict(raw))
    before = motion_scene(raw, h, 1.12, timing_verified=False, epoch=3, rear_half_angle_deg=0.)
    after = motion_scene(same, history_again, 1.12, timing_verified=False, epoch=3, rear_half_angle_deg=0.)
    assert after == before and after.source_frame_id == FRAME
    assert all(after.rays[i].status == 'unknown' and after.rays[i].hit_xy_m is None for i in (20,21,22))
    # 옛 header를 새 frame으로 자동 해석하지 않는다.
    del decoded['config']['lidar_frame_id']
    _, same, legacy_history = restore_call(decoded['call'], MotionConfig(**decoded['config']))
    with pytest.raises(MotionUnavailable, match='scan_frame_mismatch'):
        motion_scene(same, legacy_history, 1.12, timing_verified=False)


@pytest.mark.parametrize('frame', ['laser_frame', FRAME])
def test_registered_downstream_geometry_is_unchanged(frame):
    original = round_scene(); selected = relabel_scene(original, frame)
    assert source_valid(selected)
    assert linear_observation(selected, .2) == linear_observation(original, .2)
    assert linear_observation(selected, .2)['supported']
    assert freeze(current_proposal(selected, 0., 'left', 0., .02, FollowConfig())) == freeze(current_proposal(
        original, 0., 'left', 0., .02, FollowConfig()))
    unknown = round_scene(frame=frame, unknown=250)
    assert not linear_observation(unknown, .2)['supported']
    assert current_proposal(unknown, 0., 'left', 0., .02, FollowConfig())['reason'] == 'front_unobserved'


def test_unknown_frame_rejected_by_each_downstream_gate():
    scene = round_scene(frame='other_laser')
    assert not source_valid(scene)
    assert not linear_observation(scene, .2)['supported']
    assert current_proposal(scene, 0., 'left', 0., .02, FollowConfig())['reason'] == 'source_frame_mismatch'


def test_cache_drops_old_registered_frame_even_when_epoch_and_time_are_valid():
    cache = RecentScenes(); h = history()
    cache.observe(round_scene(now=1.02), h, transport=False)
    cache.observe(round_scene(now=1.07), h, transport=False)
    assert len(cache.scenes) == 2
    current = round_scene(now=1.12, frame=FRAME)
    sources, _ = cache.observe(current, h)
    assert sources == [current] and list(cache.scenes) == [current]
    assert cache.epoch == current.source_epoch == 0 and cache.frame == FRAME


def test_direct_sweep_cannot_use_other_registered_frame_as_missing_support():
    current = round_scene(frame=FRAME, unknown=375)
    other_frame = round_scene()
    same_frame = relabel_scene(other_frame, FRAME)
    arguments = (0., .2, .175, .175, FollowConfig())
    unsupported = sweep_support([current], *arguments)
    assert not unsupported['supported']
    assert sweep_support([current, other_frame], *arguments) == unsupported
    allowed_same_frame = sweep_support([current, same_frame], *arguments)
    assert allowed_same_frame['supported'] and allowed_same_frame['earlier_supported_queries'] > 0


def test_corridor_existing_frame_identity_reset_is_preserved():
    observer = CorridorObserver(FollowConfig())
    observer.evaluate(round_scene(now=1.02), history(), 0., 'left')
    result = observer.evaluate(round_scene(now=1.12, frame=FRAME), selected_history(), 0., 'left')
    assert observer.key == (FRAME, 0, 'left')
    assert result['reset_cause'] == 'frame_epoch_or_side_changed'


def test_legacy_laserscan_matches_explicit_frame_and_keeps_verified_only_contract():
    raw = SimpleNamespace(header=SimpleNamespace(frame_id=FRAME,
        stamp=SimpleNamespace(sec=1, nanosec=0)), ranges=[2.]*500, angle_min=-math.pi,
        angle_increment=math.tau/500, range_min=.05, range_max=12., time_increment=.0002)
    h = selected_history(policy='verified_only')
    scene = motion_scene(raw, h, 1.12, timing_verified=True)
    assert scene.source_frame_id == FRAME
    with pytest.raises(MotionUnavailable, match='scan_frame_mismatch'):
        motion_scene(raw, history(), 1.12, timing_verified=True)
    with pytest.raises(MotionUnavailable, match='estimated_policy_requires_native_measurements'):
        motion_scene(raw, selected_history(), 1.12, timing_verified=False)
