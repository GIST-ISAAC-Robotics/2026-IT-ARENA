"""시험 전용 C1 장착 구조(지주 자기반환·알려진 구조 unknown)의 기본 불변과 계산."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from arena_autonomy.local_path import mask_scan
from arena_autonomy.local_scene import blind_parameter, rear_masked, scene_from_scan
from test_low_speed_arc_reacquire import launch_module

ROOT = Path(__file__).resolve().parents[1]
SHARE = ROOT/'src/arena_description'
PROFILES = ROOT/'artifacts/validation/2026-11-04/lidar_mount_performance_v1/profiles'
MODEL = """<sdf version='1.10'><model name='arena_car'><link name='chassis'>
<visual name='visual'><geometry><box><size>0.188 0.13 0.06</size></box></geometry></visual>
</link></model></sdf>"""
MARKER = ([-0.1005, 0., 0.090], 0.05, True)


def scan(n=500):
    return SimpleNamespace(angle_min=-math.pi, angle_increment=math.tau/n, ranges=[1.]*n)


def test_default_parameter_is_the_same_float_and_mask():
    assert blind_parameter(30., '') == 30.
    plain = mask_scan(scan(), 30.)
    spec = mask_scan(scan(), {'rear_half_angle_deg': 30., 'sectors_deg': []})
    assert np.array_equal(np.isnan(plain.ranges), np.isnan(spec.ranges))
    assert int(np.isnan(plain.ranges).sum()) == 83


def test_sector_mask_matches_scene_status_and_keeps_rear_open():
    spec = blind_parameter(0., json.dumps([[51.1, 3.0], [-128.9, 2.0]]))
    masked = mask_scan(scan(), spec)
    angles = -math.pi + np.arange(500)*math.tau/500
    nan = np.isnan(masked.ranges)
    assert nan.any() and not nan[np.abs(angles) > math.radians(175)].any()
    assert all(rear_masked(a, spec) == bool(m) for a, m in zip(angles, nan))
    result = scene_from_scan([1.]*500, angle_min=-math.pi, angle_increment=math.tau/500,
        range_min=.05, range_max=12., acquired_at_s=[1.]*500, t_ref_s=1., rear_half_angle_deg=spec)
    assert [r.status == 'masked' for r in result.rays] == list(nan)


def test_invalid_sector_rejected():
    with pytest.raises(ValueError):
        blind_parameter(0., json.dumps([[10., 0.]]))
    with pytest.raises(ValueError):
        rear_masked(0., {'rear_half_angle_deg': 0., 'other': 1})


def test_purchase_profile_has_no_mount_trial():
    launch = launch_module()
    profile = launch._local_sensor_profile(SHARE, '', True)
    assert launch._mount_trial(MODEL, profile, .0325, .06, *MARKER) == (MODEL, '')


def test_jr_profile_posts_marker_and_chassis_top():
    launch = launch_module()
    profile = yaml.safe_load((PROFILES/'jr_unknown.yaml').read_text(encoding='utf-8'))
    model, sectors = launch._mount_trial(MODEL, profile, .0325, .06, *MARKER)
    chassis = ET.fromstring(model).find('model/link')
    assert len(chassis.findall('visual/geometry/cylinder')) == 4
    box = chassis.find("visual[@name='visual']")
    assert box.find('geometry/box/size').text.split()[2] == str(.065-.0325)
    assert math.isclose(float(box.find('pose').text.split()[2]) + .0625, (.0325+.065)/2)
    values = json.loads(sectors)
    assert len(values) == 5
    front, _, rear, _, marker = values
    assert math.isclose(front[0], math.degrees(math.atan2(.062, -.04)))
    assert math.isclose(front[1], math.degrees(math.asin(.0035/math.hypot(.04, .062))) + .5)
    assert math.isclose(rear[0], math.degrees(math.atan2(.052, -.13)))
    assert math.isclose(abs(marker[0]), 180.) and math.isclose(marker[1], math.degrees(math.atan(.025/.1505)) + .5)
    raw = yaml.safe_load((PROFILES/'jr_raw.yaml').read_text(encoding='utf-8'))
    assert launch._mount_trial(MODEL, raw, .0325, .06, *MARKER)[1] == ''


def test_c_profile_keeps_95mm_and_four_corner_posts():
    launch = launch_module()
    profile = yaml.safe_load((PROFILES/'c_unknown.yaml').read_text(encoding='utf-8'))
    assert profile['lidar']['xyz_m'] == [.03, 0., .095]
    model, sectors = launch._mount_trial(MODEL, profile, .0325, .06, *MARKER)
    assert ET.fromstring(model).find("model/link/visual[@name='visual']/pose") is None
    centers = [round(c, 3) for c, _ in json.loads(sectors)]
    assert len(centers) == 5 and abs(centers[-1]) == 180.
    for x, y in ((.05, .062), (.05, -.062), (-.11, .062), (-.11, -.062)):
        assert round(math.degrees(math.atan2(y, x)), 3) in centers
