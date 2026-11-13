"""거리 한정 부분 성분과 기존 whole-sector 경로의 독립 검사. 장치 없음."""
import copy
from dataclasses import replace
import math

import numpy as np
import pytest

from arena_autonomy.current_path_recheck._gaps import bounded_adjacent_structure as legacy
from arena_autonomy.local_scene import scene_from_measurements
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.numeric_kernels import configure
from range_mask_component import bounded_adjacent_structure as candidate

configure('native_v1')


def fixture(*, whole=False):
    angles = tuple(math.radians(-180+i*.72) for i in range(500))
    values = [2.]*500
    values[410] = math.inf
    values[411] = values[412] = math.nan
    if not whole:
        values[419] = math.nan  # 같은 nominal sector의 다른 성분; 중간 실제 반환을 보존한다.
    profile = dict(sectors_deg=[[116.28, .72] if whole else [119.8, 4.]],
                   range_limits_m=[[.055, .080]])
    raw = MeasurementScan(values, angles, tuple(1+i*.0002 for i in range(500)),
                          time_basis='simulation_acquisition')
    scene = make_scene(raw, profile)
    radius = .080/math.sin(math.radians(115.5))
    q = np.array([[.0754+radius*math.cos(math.radians(115.5)), .080]])
    return dict(raw=raw, scene=scene, profile=profile, q=q, padding=np.full(500, .055),
                sectors=tuple(map(tuple, profile['sectors_deg'])), speed=0., target=.10)


def make_scene(raw, profile):
    return scene_from_measurements(raw.ranges_m, angles_rad=raw.angles_rad,
        acquired_at_s=raw.acquired_at_s, t_ref_s=1.12, range_min=raw.range_min_m,
        range_max=raw.range_max_m, source_frame_id=raw.source_frame_id,
        source_epoch=raw.source_epoch, sensor_xy_m=(.0754, 0.),
        rear_half_angle_deg=profile, preserve_sampling=True)


def args(f):
    return (f['scene'], f['raw'], f['q'], f['padding'], .207, .15,
            f['speed'], f['target'], f['sectors'])


def run(f, **options):
    return candidate(*args(f), **(dict(mask_profile=f['profile'], motion_epoch=0,
                                      timing_verified=True) | options))


def change_range(f, index, value, *, rebuild=True):
    values = list(f['raw'].ranges_m); values[index] = value
    f['raw'] = replace(f['raw'], ranges_m=values)
    if rebuild:
        f['scene'] = make_scene(f['raw'], f['profile'])


@pytest.mark.parametrize('case', ('accepted', 'near_return', 'bad_unknown', 'too_fast', 'outside_body'))
def test_whole_sector_exact_original_behavior(case):
    f = fixture(whole=True)
    if case == 'near_return': change_range(f, 409, .10)
    if case == 'bad_unknown': change_range(f, 410, math.nan)
    if case == 'too_fast': f['speed'] = .151
    if case == 'outside_body': f['q'][0, 1] = .086
    old = legacy(*args(f))
    assert candidate(*args(f)) == old
    assert run(f) == old
    if case == 'accepted': assert old['accepted']
    else: assert not old['accepted']


def test_partial_requires_profile_and_preserves_raw_scene_arrays():
    f = fixture(); before = copy.deepcopy(f)
    old = legacy(*args(f))
    assert old['reason'] == 'not_one_declared_mount_sector'
    assert candidate(*args(f)) == old
    result = run(f)
    assert result['accepted'] and result['policy'] == 'range_limited_mount_component_r396'
    assert result['range_mask_components'][0]['masked_indices'] == [411, 412]
    assert result['range_mask_components'][0]['distant_split_return_indices'] == [413]
    assert result['maximum_chord_m'] <= .05 and not result['unknown_filled']
    assert result['maximum_exterior_m'] == .01 and result['maximum_gap_rad'] == math.radians(6)
    assert result['minimum_neighbor_range_m'] == .6
    assert f['scene'] == before['scene'] and f['profile'] == before['profile']
    np.testing.assert_equal(f['raw'].ranges_m, before['raw'].ranges_m)
    np.testing.assert_array_equal(f['q'], before['q'])


@pytest.mark.parametrize('case', (
    'unknown_nan', 'unknown_near_invalid', 'known_near_return', 'far_flank_missing',
    'raw_scene_replaced', 'sector_mismatch', 'missing_range', 'profile_status_changed',
    'epoch', 'unverified', 'stale', 'missing_raw', 'speed', 'width'))
def test_partial_cannot_excuse_invalid_source_or_new_risk(case):
    f = fixture(); options = {}
    if case == 'unknown_nan': change_range(f, 410, math.nan)
    if case == 'unknown_near_invalid': change_range(f, 410, .01)
    if case == 'known_near_return': change_range(f, 409, .10)
    if case == 'far_flank_missing':
        values = list(f['raw'].ranges_m); values[413:] = [math.nan]*len(values[413:])
        f['raw'] = replace(f['raw'], ranges_m=values)
        f['scene'] = make_scene(f['raw'], f['profile'])
    if case == 'raw_scene_replaced': change_range(f, 400, math.nan, rebuild=False)
    if case == 'sector_mismatch': f['sectors'] = ((-119.8, 4.),)
    if case == 'missing_range': options['mask_profile'] = dict(sectors_deg=[[119.8, 4.]])
    if case == 'profile_status_changed':
        options['mask_profile'] = dict(sectors_deg=[[119.8, 4.]], range_limits_m=[[.055, 3.]])
    if case == 'epoch': options['motion_epoch'] = 1
    if case == 'unverified': options['timing_verified'] = False
    if case == 'stale': f['scene'] = replace(f['scene'], t_ref_s=1.4)
    if case == 'missing_raw': f['raw'] = None
    if case == 'speed': f['target'] = .151
    if case == 'width': f['q'][0, 1] = .086
    assert not run(f, **options)['accepted']


def test_arbitrary_subset_cannot_cross_second_mount_sector():
    f = fixture()
    f['profile']['sectors_deg'].append([116., .3])
    f['profile']['range_limits_m'].append([.055, .08])
    f['sectors'] = tuple(map(tuple, f['profile']['sectors_deg']))
    f['scene'] = make_scene(f['raw'], f['profile'])
    assert not run(f)['accepted']


def test_nonrange_near_return_cannot_be_used_as_distant_separator():
    # 분리 반환이 측정 최소값보다 크더라도 자기 구조 band 앞쪽이면 새 부분 근거가 아니다.
    from profile_evidence import partial_component
    assert partial_component({1}, [{1, 2}], 0, 2, [2., math.nan, .051], ((.055, .08),)) is None
