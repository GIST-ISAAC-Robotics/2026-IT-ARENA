"""선택 저속 관측 정책의 경계/재검사. 실제 sweep이며 실차 안전 증명은 아니다."""
import copy
from dataclasses import replace
import math

import numpy as np
import pytest

from arena_autonomy import current_wall_reacquire as helper
from arena_autonomy.current_path_recheck import _geometry as current_geometry
from arena_autonomy.race_runtime_policy.retained_recheck import _geometry as retained_geometry
from test_current_wall_reacquire import history
from test_reacquire_radial_recheck import (
    CONFIG, expected_radius, make_current, native_scan, record_real_sweeps,
)


FIXED = 'fixed_box_low_speed_observation'
RADIAL = 'fixed_box_plus_euclidean_motion_ball'
GEOMETRIES = (current_geometry, retained_geometry)


def inputs(scan, motion, now, *, measured=0., previous=.1):
    data, scene = make_current(scan, motion, now)
    # 정책 경계용 독립 피드백 필드. 실제 차량 가감속의 동역학 시험은 아니다.
    data['speed_mps'] = measured
    data['previous_steering_rad'] = previous
    return data, scene


def acquired(*, measured=0., previous=.1):
    motion, scan = history(), native_scan()
    submitted, scene = inputs(scan, motion, .7, measured=measured, previous=previous)
    candidate = helper.current_wall_candidate(scene, measured, 'left', previous, .1, CONFIG)
    assert candidate is not None and 0 < candidate[0] <= .1
    normal = {**candidate[2], 'selector_mode': 'SOLO', 'selector_choice': 'baseline'}
    output = (candidate[0], candidate[1], {'handoff_state': 'FOLLOW', 'normal': normal})
    return output, submitted, scene, motion, scan


def check_calls(calls, scene, mode, previous):
    assert len(calls) == 2
    assert calls[1]['angle'] == previous
    for call in calls:
        np.testing.assert_array_equal(call['points'], scene.occupied_endpoints_xy_m)
        if mode == 'axis':
            assert 'radial_margin' not in call['options']
            np.testing.assert_array_equal(call['options']['margin'], .055+expected_radius(scene))
        else:
            assert call['options']['margin'] == pytest.approx(.055)
            if mode == 'fixed':
                assert 'radial_margin' not in call['options']
            else:
                np.testing.assert_array_equal(call['options']['radial_margin'], expected_radius(scene))


def test_helper_request_and_measured_boundaries_select_exact_model(monkeypatch):
    _, _, scene, _, _ = acquired()
    before = scene.rays
    calls = record_real_sweeps(monkeypatch, helper)
    for measured, cap, mode, shape in (
        (.20, .10, 'fixed', FIXED),
        (.200001, .10, 'radial', RADIAL),
        (0., .100001, 'axis', 'axis_padding'),
    ):
        calls.clear()
        result = helper.current_wall_candidate(scene, measured, 'left', .1, .1, CONFIG, cap_mps=cap)
        assert result is not None and 0 < result[0] <= cap
        assert result[2]['occupancy_motion_shape'] == shape
        assert result[2]['occupancy_motion_bound_applied'] is (mode != 'fixed')
        assert result[2]['residual_unobserved_object_motion_risk'] is (mode == 'fixed')
        if mode == 'fixed':
            assert result[2]['occupancy_padding_m'] == pytest.approx(.055)
        check_calls(calls, scene, mode, .1)
        assert calls[0]['angle'] == result[1] and scene.rays == before


def test_actual_new_front_obstacle_stops_helper_and_both_rechecks(monkeypatch):
    output, submitted, _, motion, _ = acquired()
    current, blocked = inputs(native_scan(.8, obstacle=True), motion, .8)
    assert helper.current_wall_candidate(blocked, 0., 'left', .1, .1, CONFIG) is None
    before = copy.deepcopy((output, submitted, current))
    rays_before = blocked.rays
    for geometry in GEOMETRIES:
        with monkeypatch.context() as scoped:
            calls = record_real_sweeps(scoped, geometry)
            result = geometry.rebase(output, submitted, current, blocked, motion, CONFIG)
        assert result is not None and result['speed_mps'] == 0.
        assert result['reason'] == 'rebase_current_gap_hold'
        check_calls(calls, blocked, 'fixed', .1)
        assert min(call['clearance'] for call in calls) == 0.
        assert (output, submitted, current) == before and blocked.rays == rays_before


def test_aged_current_recalculates_real_sweeps_with_fixed_observation_model(monkeypatch):
    output, submitted, old_scene, motion, scan = acquired()
    current, scene = inputs(scan, motion, .8)
    before = copy.deepcopy((output, submitted, current))
    assert scene.source_first_s == old_scene.source_first_s
    assert scene.t_ref_s-old_scene.t_ref_s == pytest.approx(.1)
    for geometry in GEOMETRIES:
        with monkeypatch.context() as scoped:
            calls = record_real_sweeps(scoped, geometry)
            result = geometry.rebase(output, submitted, current, scene, motion, CONFIG)
        assert result is not None and 0 < result['speed_mps'] <= .1
        assert result['reference_s'] == .8 and result['path_source_s'] == scene.source_first_s
        assert result['clearance_m'] == min(call['clearance'] for call in calls)
        assert result['independent_guard_required'] and not result['runtime_authorized']
        check_calls(calls, scene, 'fixed', .1)
        assert (output, submitted, current) == before


def test_current_measured_speed_crossing_restores_new_per_ray_radius(monkeypatch):
    output, submitted, old_scene, motion, scan = acquired()
    for measured, mode in ((.20, 'fixed'), (.200001, 'radial'), (-.200001, 'radial')):
        current, scene = inputs(scan, motion, .8, measured=measured)
        before = copy.deepcopy((output, submitted, current))
        for geometry in GEOMETRIES:
            with monkeypatch.context() as scoped:
                calls = record_real_sweeps(scoped, geometry)
                result = geometry.rebase(output, submitted, current, scene, motion, CONFIG)
            assert result is not None
            check_calls(calls, scene, mode, .1)
            if mode == 'radial':
                np.testing.assert_allclose(calls[0]['options']['radial_margin']-expected_radius(old_scene),
                    .05, rtol=0., atol=1e-14)
            assert (output, submitted, current) == before


def test_existing_radial_normal_does_not_silently_become_fixed(monkeypatch):
    output, submitted, scene, motion, _ = acquired(measured=.200001)
    assert output[2]['normal']['occupancy_motion_shape'] == RADIAL
    current = copy.deepcopy(submitted)
    current['speed_mps'] = 0.
    before = copy.deepcopy((output, submitted, current))
    for geometry in GEOMETRIES:
        with monkeypatch.context() as scoped:
            calls = record_real_sweeps(scoped, geometry)
            result = geometry.rebase(output, submitted, current, scene, motion, CONFIG)
        assert result is not None
        check_calls(calls, scene, 'radial', .1)
        assert (output, submitted, current) == before


def test_ordinary_unmarked_and_faster_requests_preserve_legacy_axis_padding(monkeypatch):
    source, submitted, scene, motion, _ = acquired()
    for kind in ('ordinary', 'unmarked', 'faster'):
        output = copy.deepcopy(source)
        if kind == 'ordinary':
            output[2]['normal'].pop('current_wall_reacquire')
        elif kind == 'unmarked':
            output[2]['normal'].pop('occupancy_motion_shape')
        else:
            output = (.100001, output[1], output[2])
        current = copy.deepcopy(submitted)
        before = copy.deepcopy((output, submitted, current))
        for geometry in GEOMETRIES:
            with monkeypatch.context() as scoped:
                calls = record_real_sweeps(scoped, geometry)
                result = geometry.rebase(output, submitted, current, scene, motion, CONFIG)
            assert result is not None
            check_calls(calls, scene, 'axis', .1)
            assert (output, submitted, current) == before


def test_stop_zero_expired_source_clock_epoch_and_timing_remain_rejected(monkeypatch):
    source, submitted, scene, motion, _ = acquired()
    for kind in ('STOP', 'zero', 'expired_source', 'clock', 'epoch', 'timing'):
        output = copy.deepcopy(source)
        current = copy.deepcopy(submitted)
        if kind == 'STOP':
            output[2]['handoff_state'] = 'STOP'
        elif kind == 'zero':
            output = (0., output[1], output[2])
        elif kind == 'expired_source':
            output[2]['normal']['path_source_s'] -= 1.
        elif kind == 'clock':
            current['reference_s'] -= .05
        elif kind == 'epoch':
            current['epoch'] += 1
        else:
            current['timing_verified'] = False
        before = copy.deepcopy((output, submitted, current))
        for geometry in GEOMETRIES:
            with monkeypatch.context() as scoped:
                calls = record_real_sweeps(scoped, geometry)
                result = geometry.rebase(output, submitted, current, scene, motion, CONFIG)
            if kind in ('STOP', 'zero'):
                assert result is None
            else:
                assert result is not None and result['speed_mps'] == 0.
            assert not calls and (output, submitted, current) == before


def test_previous_angle_real_occupied_sweep_still_limits_clear_current_angle(monkeypatch):
    output, submitted, _, motion, _ = acquired(previous=.2)
    scan = native_scan(.8)
    # 현재 직진 고정 상자 밖이지만 직전 좌조향 외곽에는 놓이는 실제 유한 반환.
    direction = math.atan2(.15, .25-.06)
    index = min(range(len(scan.angles_rad)), key=lambda i: abs(scan.angles_rad[i]-direction))
    ranges = list(scan.ranges_m)
    ranges[index] = math.hypot(.25-.06, .15)
    current, scene = inputs(replace(scan, ranges_m=tuple(ranges)), motion, .8, previous=.2)
    before = copy.deepcopy((output, submitted, current))
    rays_before = scene.rays
    for geometry in GEOMETRIES:
        with monkeypatch.context() as scoped:
            calls = record_real_sweeps(scoped, geometry)
            result = geometry.rebase(output, submitted, current, scene, motion, CONFIG)
        check_calls(calls, scene, 'fixed', .2)
        assert abs(calls[0]['angle']) < 1e-6
        assert calls[0]['clearance'] > CONFIG.hold_clearance_m
        assert calls[1]['clearance'] <= CONFIG.standstill_gap_m
        assert result is not None and result['speed_mps'] == 0.
        assert (output, submitted, current) == before and scene.rays == rays_before
