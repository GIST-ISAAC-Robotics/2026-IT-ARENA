import copy
import math
import xml.etree.ElementTree as ET

import pytest

from validate_bump_lidar_pose import build_fixture, expected_front_range, inspect_scan


def sample(pitch=.07413795):
    view = dict(sensor_height_m=.09538384, sdf_pitch_rad=pitch)
    row = dict(angle_min=-math.pi, angle_increment=2*math.pi/500,
               range_min=.05, range_max=12.)
    row['ranges'] = [expected_front_range(view['sensor_height_m'], pitch,
                       -math.pi+i*row['angle_increment'], 12.) for i in range(500)]
    return row, view


def test_fixture_copies_actual_sensor_and_contains_no_driving_system():
    sensor = ET.fromstring('<sensor name="c1" type="gpu_lidar"><topic>/original</topic><lidar><range><max>12</max></range></lidar></sensor>')
    before = ET.tostring(sensor)
    root, views = build_fixture(sensor, .095, .074)
    assert ET.tostring(sensor) == before
    assert len(root.findall('.//sensor')) == 3
    assert len(root.findall('.//model')) == 4
    assert all(m.findtext('static') == 'true' for m in root.findall('.//model'))
    assert views['nose_down']['sdf_pitch_rad'] > 0
    assert views['nose_up']['sdf_pitch_rad'] < 0
    assert len({v['topic'] for v in views.values()}) == 3
    assert not root.findall('.//joint')
    assert all('drive' not in p.attrib['filename'] for p in root.findall('.//plugin'))
    assert all(s.findtext('lidar/range/max') == '12' for s in root.findall('.//sensor'))


@pytest.mark.parametrize('height,pitch', [(0., .1), (.4, .1), (.1, 0.), (.1, -.1), (.1, math.nan)])
def test_bad_fixture_pose_rejected(height, pitch):
    with pytest.raises(ValueError):
        build_fixture(ET.Element('sensor'), height, pitch)


@pytest.mark.parametrize('pitch', [0., -.07413795, .07413795])
def test_analytic_front_samples_match(pitch):
    row, view = sample(pitch)
    result = inspect_scan(row, view)
    assert result['passed'] and result['front_rays_compared'] == 27
    if pitch > 0:
        assert 1.28 < row['ranges'][250] < 1.30
        assert result['max_range_difference_m'] == pytest.approx(0.)
    else:
        assert row['ranges'][250] is None


@pytest.mark.parametrize('measured', [None, 12., .01, math.nan, -math.inf, True, '1.3', 2.])
def test_wrong_downward_return_fails(measured):
    row, view = sample()
    row['ranges'][250] = measured
    assert not inspect_scan(row, view)['passed']


def test_level_fake_return_fails_and_nohit_encoding_is_accepted():
    row, view = sample(0.)
    row['ranges'][250] = 1.3
    assert not inspect_scan(row, view)['passed']
    row['ranges'][250] = 12.
    assert inspect_scan(row, view)['passed']


@pytest.mark.parametrize('key,value', [('angle_min', 0.), ('angle_increment', .01),
    ('range_min', 0.), ('range_max', 1.), ('range_max', math.nan), ('range_min', True)])
def test_wrong_metadata_cannot_hide_a_floor_return(key, value):
    row, view = sample()
    row[key] = value
    with pytest.raises(ValueError):
        inspect_scan(row, view)


def test_tolerance_is_bounded_and_scope_only_front():
    row, view = sample()
    with pytest.raises(ValueError):
        inspect_scan(row, view, 1.)
    row['ranges'][0] = 2.
    assert inspect_scan(row, view)['passed']
    short = copy.deepcopy(row)
    short['ranges'].pop()
    with pytest.raises(ValueError):
        inspect_scan(short, view)
