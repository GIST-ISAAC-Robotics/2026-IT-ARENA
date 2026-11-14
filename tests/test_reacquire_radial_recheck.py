"""재획득 helper→현재/retained 재검사 연결. 실제 sweep을 쓰는 합성 센서 계약 검사."""
import copy
from dataclasses import replace
import math

import numpy as np
import pytest

from arena_autonomy import current_wall_reacquire as helper
from arena_autonomy.current_path_recheck import _geometry as current_geometry
from arena_autonomy.follow_trace import snapshot_call
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.race_runtime_policy.retained_recheck import _geometry as retained_geometry
from arena_autonomy.scene_motion import motion_scene
from test_current_wall_reacquire import CFG, history, sensor


SHAPE = 'fixed_box_plus_euclidean_motion_ball'
CONFIG = replace(CFG, max_steering_rad=.28475436568260193)


def native_scan(now=.7, *, obstacle=False):
    # 기존 테스트의 교차광선 생성기만 재사용한다. artifact·정답 경로 입력은 없다.
    _, raw = sensor(now, closed=True, obstacle=obstacle)
    first = raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9
    return MeasurementScan(tuple(raw.ranges),
        tuple(-math.pi+index*math.tau/500 for index in range(500)),
        tuple(first+index*.0002 for index in range(500)),
        time_basis='simulation_acquisition')


def make_current(scan, motion, now):
    scene = motion_scene(scan, motion, now, timing_verified=True, epoch=0, rear_half_angle_deg=0.)
    data = snapshot_call(scan, motion, now, 0, 0., 'left', 0., .1, True, 0.)
    return data, scene


@pytest.fixture
def acquired():
    motion, scan = history(), native_scan()
    submitted, scene = make_current(scan, motion, .7)
    before = scene.rays
    # This fixture specifically exercises the retained radial policy above the crawl band.
    submitted['speed_mps'] = .200001
    candidate = helper.current_wall_candidate(scene, .200001, 'left', 0., .1, CONFIG)
    assert candidate is not None and 0 < candidate[0] <= .1
    assert candidate[2]['occupancy_motion_shape'] == SHAPE
    assert candidate[2]['current_wall_reacquire']['fixed_padding_m'] == pytest.approx(.055)
    assert scene.rays == before
    normal = {**candidate[2], 'selector_mode': 'SOLO', 'selector_choice': 'baseline'}
    output = (candidate[0], candidate[1], {'handoff_state': 'FOLLOW', 'normal': normal})
    return output, submitted, scene, motion, scan


@pytest.fixture(params=[current_geometry, retained_geometry], ids=['current', 'retained'])
def geometry(request):
    return request.param


def record_real_sweeps(monkeypatch, geometry):
    original = geometry.swept_limit
    calls = []

    def record(points, angle, *args, **kwargs):
        # 기록만 추가한다. clearance·반경·원시점 및 호출 횟수를 조작하지 않는다.
        result = original(points, angle, *args, **kwargs)
        calls.append(dict(points=np.array(points, copy=True), angle=angle,
                          options=copy.deepcopy(kwargs), clearance=result))
        return result

    monkeypatch.setattr(geometry, 'swept_limit', record)
    return calls


def expected_radius(scene):
    return np.array([CONFIG.object_motion_bound_mps*(scene.t_ref_s-ray.acquired_at_s)
                     for ray in scene.rays if ray.status == 'return'])


def assert_radial_calls(calls, scene):
    assert len(calls) == 2  # 실제 이번 조향과 직전 조향을 각각 검사한다.
    for call in calls:
        assert call['options']['margin'] == pytest.approx(.055)
        np.testing.assert_array_equal(call['options']['radial_margin'], expected_radius(scene))
        np.testing.assert_array_equal(call['points'], scene.occupied_endpoints_xy_m)


def test_actual_helper_output_rechecks_positive_without_mutating_input(acquired, geometry, monkeypatch):
    output, submitted, scene, motion, _ = acquired
    current = copy.deepcopy(submitted)
    before = copy.deepcopy((output, submitted, current))
    rays_before = scene.rays
    calls = record_real_sweeps(monkeypatch, geometry)
    result = geometry.rebase(output, submitted, current, scene, motion, CONFIG)
    assert result is not None and 0 < result['speed_mps'] <= output[0]
    assert result['path_source_s'] == output[2]['normal']['path_source_s']
    assert result['independent_guard_required'] and not result['runtime_authorized']
    assert (output, submitted, current) == before
    assert scene.rays == rays_before
    assert_radial_calls(calls, scene)
    assert calls[1]['angle'] == current['previous_steering_rad']


def test_new_actual_front_obstacle_stops_the_previously_positive_path(acquired, geometry, monkeypatch):
    output, submitted, _, motion, _ = acquired
    current, blocked = make_current(native_scan(.8, obstacle=True), motion, .8)
    before = copy.deepcopy((output, submitted, current))
    rays_before = blocked.rays
    calls = record_real_sweeps(monkeypatch, geometry)
    result = geometry.rebase(output, submitted, current, blocked, motion, CONFIG)
    assert result is not None and result['speed_mps'] == 0.
    assert result['reason'] == 'rebase_current_gap_hold'
    assert min(call['clearance'] for call in calls) == 0.
    assert_radial_calls(calls, blocked)
    assert (output, submitted, current) == before and blocked.rays == rays_before


def test_same_acquisition_at_later_reference_recomputes_each_radius(acquired, geometry, monkeypatch):
    output, submitted, old_scene, motion, scan = acquired
    current, later_scene = make_current(scan, motion, .8)
    assert later_scene.source_first_s == old_scene.source_first_s
    before = copy.deepcopy((output, submitted, current))
    calls = record_real_sweeps(monkeypatch, geometry)
    result = geometry.rebase(output, submitted, current, later_scene, motion, CONFIG)
    assert result is not None and result['speed_mps'] > 0.
    assert_radial_calls(calls, later_scene)
    np.testing.assert_allclose(calls[0]['options']['radial_margin']-expected_radius(old_scene),
        CONFIG.object_motion_bound_mps*.1, rtol=0., atol=1e-14)
    assert (output, submitted, current) == before


def test_unselected_or_faster_normal_keeps_legacy_sweep_contract(acquired, geometry, monkeypatch):
    source, submitted, scene, motion, _ = acquired
    calls = record_real_sweeps(monkeypatch, geometry)
    for kind in ('ordinary', 'no_shape', 'wrong_shape', 'over_crawl_cap'):
        output = copy.deepcopy(source)
        normal = output[2]['normal']
        if kind == 'ordinary':
            normal.pop('current_wall_reacquire')
        elif kind == 'no_shape':
            normal.pop('occupancy_motion_shape')
        elif kind == 'wrong_shape':
            normal['occupancy_motion_shape'] = 'legacy_axis_padding'
        else:
            output = (.100001, output[1], output[2])
        current = copy.deepcopy(submitted)
        before = copy.deepcopy((output, submitted, current))
        calls.clear()
        result = geometry.rebase(output, submitted, current, scene, motion, CONFIG)
        assert result is not None and result['speed_mps'] > 0.
        assert len(calls) == 2
        for call in calls:
            assert 'radial_margin' not in call['options']
            np.testing.assert_array_equal(call['options']['margin'], .055+expected_radius(scene))
            np.testing.assert_array_equal(call['points'], scene.occupied_endpoints_xy_m)
        assert (output, submitted, current) == before


def test_source_epoch_timing_and_clock_rejections_precede_sweeps(acquired, geometry, monkeypatch):
    source, submitted, scene, motion, _ = acquired
    calls = record_real_sweeps(monkeypatch, geometry)
    for kind in ('expired_path', 'input_epoch', 'scan_epoch', 'side', 'timing', 'reversed_clock',
                 'scene_reference', 'source_indices'):
        output = copy.deepcopy(source)
        current = copy.deepcopy(submitted)
        selected = scene
        if kind == 'expired_path':
            output[2]['normal']['path_source_s'] -= 1.
        elif kind == 'input_epoch':
            current['epoch'] += 1
        elif kind == 'scan_epoch':
            current['scan']['measurements']['source_epoch'] += 1
        elif kind == 'side':
            current['side'] = 'right'
        elif kind == 'timing':
            current['timing_verified'] = False
        elif kind == 'reversed_clock':
            current['reference_s'] -= .05
        elif kind == 'scene_reference':
            selected = replace(scene, t_ref_s=scene.t_ref_s+.01)
        else:
            selected = replace(scene, rays=scene.rays[1:], source_first_s=scene.rays[1].acquired_at_s,
                source_angles_rad=None if scene.source_angles_rad is None else scene.source_angles_rad[1:])
        before = copy.deepcopy((output, submitted, current))
        calls.clear()
        result = geometry.rebase(output, submitted, current, selected, motion, CONFIG)
        assert result is not None and result['speed_mps'] == 0., kind
        assert not calls, kind
        assert (output, submitted, current) == before


def test_zero_or_stop_output_cannot_be_revived_by_radial_metadata(acquired, geometry, monkeypatch):
    source, submitted, scene, motion, _ = acquired
    calls = record_real_sweeps(monkeypatch, geometry)
    for kind in ('zero', 'STOP'):
        output = copy.deepcopy(source)
        if kind == 'zero':
            output = (0., output[1], output[2])
        else:
            output[2]['handoff_state'] = 'STOP'
        current = copy.deepcopy(submitted)
        before = copy.deepcopy((output, submitted, current))
        assert geometry.rebase(output, submitted, current, scene, motion, CONFIG) is None
        assert not calls and (output, submitted, current) == before
