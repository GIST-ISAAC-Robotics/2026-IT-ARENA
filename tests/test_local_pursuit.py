import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import subprocess
import json

import numpy as np
import pytest
import xacro
import yaml

from arena_autonomy.local_path import local_path, pursuit_command, mask_scan, swept_limit, braking_speed

ROOT = Path(__file__).resolve().parents[1]


def test_straight_and_signed_offset():
    x = np.linspace(-.1, 2., 200)
    points = np.column_stack((x, np.full_like(x, .425)))
    speed, steer, meta = pursuit_command(points, 'left', 1.)
    assert speed > 1.8
    assert steer == pytest.approx(0., abs=1e-8)
    _, steer, _ = pursuit_command(points + [0, .05], 'left', 1.)
    assert steer > 0
    _, right, _ = pursuit_command(points * [1, -1] - [0, .05], 'right', 1.)
    assert right < 0
    assert len(meta['path_points']) == 36


def test_no_wall_stops_without_path_fabrication():
    speed, _, details = pursuit_command(np.zeros((3, 2)), 'left', 1.)
    assert speed == 0 and details['reason'] == 'wall_lost'


def test_short_observation_and_old_scan_reduce_speed_before_corner_is_visible():
    def command(length, age):
        x = np.linspace(-.1, length, 100)
        return pursuit_command(np.column_stack((x, x*0+.425)), 'left', 2., scan_age=age)
    long, _, _ = command(1.8, .1)
    short, _, meta = command(.5, .1)
    old, _, _ = command(.5, .3)
    assert 0 < old < short < long
    assert short < 1.2
    assert meta['observed_path_ahead_m'] < .7


@pytest.mark.parametrize('run', ['lap_configured_video_v2'])
def test_observed_sharp_corner_has_forward_turning_path(run):
    source = ROOT/'artifacts/validation/2026-09-21/local_pursuit'/run/'last_scan.json'
    scan = json.loads(source.read_text())
    ranges = np.array([np.nan if r is None else r for r in scan['ranges']])
    angles = scan['angle_min'] + np.arange(len(ranges)) * scan['angle_increment']
    keep = np.isfinite(ranges) & (np.abs(angles) <= np.deg2rad(150))
    points = np.column_stack((ranges[keep]*np.cos(angles[keep])+.06, ranges[keep]*np.sin(angles[keep])))
    # 정지 상태에 저장된 스캔이다. 움직이는 스캔의 무보정 사용을 허용하는 검사가 아니다.
    speed, steer, meta = pursuit_command(points, 'left', 0.)
    assert 0 < speed <= 2.0 and steer > 0
    assert meta['target_xy_rear_m'][0] > .10


@pytest.mark.parametrize('run', ['lap_configured_v3', 'lap_configured_v5'])
def test_already_overshot_corner_is_not_given_a_fabricated_forward_path(run):
    source = ROOT/'artifacts/validation/2026-09-21/local_pursuit'/run/'last_scan.json'
    scan = json.loads(source.read_text())
    ranges = np.array([np.nan if r is None else r for r in scan['ranges']])
    angles = scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
    keep = np.isfinite(ranges) & (np.abs(angles) <= np.deg2rad(150))
    points = np.column_stack((ranges[keep]*np.cos(angles[keep])+.06, ranges[keep]*np.sin(angles[keep])))
    speed, _, meta = pursuit_command(points, 'left', 0.)
    # 이미 곡선의 전방 경로를 지나친 실패 자세: 후진 복구를 구현하지 않았으므로 정지.
    assert speed == 0 and meta['reason'] in ('path_too_short', 'wall_segment_short')
    assert meta['alternate_wall_failure'] in ('path_too_short', 'wall_segment_short')


def test_parametric_wall_supports_quarter_turn_without_y_of_x_assumption():
    theta = np.linspace(-.2, 1.5, 180)
    wall = np.column_stack((.575*np.sin(theta), 1.-.575*np.cos(theta)))
    speed, steer, meta = pursuit_command(wall, 'left', 1.)
    assert 0 < speed < 2.
    assert .06 < steer < .3
    assert meta['target_xy_rear_m'][1] > 0


def test_observed_side_post_does_not_hide_remaining_long_wall():
    source = ROOT/'artifacts/validation/2026-09-21/local_pursuit/lap_configured_v4/last_scan.json'
    scan = json.loads(source.read_text())
    ranges = np.array([np.nan if r is None else r for r in scan['ranges']])
    angles = scan['angle_min'] + np.arange(len(ranges))*scan['angle_increment']
    keep = np.isfinite(ranges) & (np.abs(angles) <= np.deg2rad(150))
    points = np.column_stack((ranges[keep]*np.cos(angles[keep])+.06, ranges[keep]*np.sin(angles[keep])))
    speed, steer, meta = pursuit_command(points, 'left', 0.)
    assert speed > 0 and abs(steer) < .05
    assert meta['path_wall'] == 'left'


def test_real_folded_preferred_path_uses_observed_opposite_wall():
    source = ROOT/'artifacts/validation/2026-09-21/local_pursuit/lap_configured_v6_repeat2/last_scan.json'
    scan = json.loads(source.read_text())
    ranges = np.array([np.nan if r is None else r for r in scan['ranges']])
    angles = scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
    keep = np.isfinite(ranges) & (np.abs(angles) <= np.deg2rad(150))
    points = np.column_stack((ranges[keep]*np.cos(angles[keep])+.06, ranges[keep]*np.sin(angles[keep])))
    speed, steer, meta = pursuit_command(points, 'left', 0.)
    assert 0 < speed <= 1.2 and abs(steer) < .08
    assert meta['wall_fallback'] and meta['path_wall'] == 'right'


def test_valid_preferred_wall_is_not_overridden_by_other_side():
    x = np.linspace(-.1, .8, 100)
    points = np.vstack((np.column_stack((x, x*0+.425)), np.column_stack((x, x*0-.50))))
    _, steer, meta = pursuit_command(points, 'left', 1.)
    assert steer == pytest.approx(0., abs=1e-8)
    assert not meta['wall_fallback'] and meta['path_wall'] == 'left'


def test_rear_mask_is_unknown_not_infinite_and_preserves_input():
    scan = SimpleNamespace(angle_min=-np.pi, angle_increment=2*np.pi/360, ranges=[1.]*360)
    masked = mask_scan(scan)
    assert np.isnan(masked.ranges[0])
    assert masked.ranges[180] == 1.
    assert scan.ranges[0] == 1.


@pytest.mark.parametrize('side', ['left', 'right'])
def test_straight_wall_small_pose_and_noise_perturbations(side):
    # 고정 seed의 기하 회귀일 뿐, C1 실물 오차 분포나 동적 반복성 검증은 아니다.
    rng = np.random.default_rng(20260921)
    x = np.linspace(-1., 3., 350)
    walls = np.vstack((np.column_stack((x, x*0+.425)), np.column_stack((x, x*0-.425))))
    for lateral in (-.06, 0., .06):
        for yaw in np.deg2rad([-10., 0., 10.]):
            c, s = np.cos(yaw), np.sin(yaw)
            points = (walls - [0., lateral]) @ np.array([[c, -s], [s, c]])
            points += rng.normal(0., .005, points.shape)
            speed, steer, details = pursuit_command(points, side, 1., max_speed=2.5)
            assert 0 < speed <= 2.5
            assert np.isfinite(steer) and abs(steer) <= .37
            assert details['target_xy_rear_m'][0] > .10
            if yaw == 0 and lateral:
                assert steer * lateral < 0  # 중심선 방향으로 복귀


def test_swept_footprint_and_age_sensitive_braking():
    assert swept_limit(np.array([[.50, 0.]]), 0.) < .40
    assert swept_limit(np.array([[.50, .3]]), 0.) == 3.
    assert braking_speed(.5, .2) < braking_speed(.5, 0.)
    assert braking_speed(0., 0.) == 0.


def test_post_run_road_audit_uses_rotated_collision_patches(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT/'scripts'))
    from audit_local_pursuit_road import road_polygon
    from shapely.geometry import Point
    source = tmp_path/'world.sdf'
    source.write_text('''<sdf><world><model name="it_arena_track_static"><link name="track_surface">
      <pose>1 2 0 0 0 1.5707963267948966</pose><collision><pose>1 0 0 0 0 0</pose>
      <geometry><box><size>2 .4 .02</size></box></geometry></collision></link></model></world></sdf>''')
    road = road_polygon(source)
    assert road.area == pytest.approx(.8)
    assert road.contains(Point(1, 3))
    assert not road.contains(Point(1.3, 3))


def test_b0183_model_and_temporary_bump_removal(tmp_path):
    spec = importlib.util.spec_from_file_location('launch_new_profile', ROOT/'src/arena_bringup/launch/simulation.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    profile = yaml.safe_load((ROOT/'src/arena_description/config/b0183_c1.yaml').read_text())
    xml = xacro.process_file(str(ROOT/'src/arena_description/models/arena_car/model.sdf.xacro'),
                             mappings={'d435i_depth_enabled': 'false', 'tof_enabled': 'false', 'body_mass': '1.697'}).toxml()
    model = ET.fromstring(launch._b0183_model(xml, profile)).find('model')
    assert model.find("link[@name='d435i_link']") is None
    assert model.find("link[@name='b0183_link']") is not None
    assert not model.findall(".//sensor[@type='depth_camera']")
    assert not [s for s in model.findall('.//sensor') if s.get('name', '').startswith('tof_')]
    assert model.findtext(".//sensor[@name='lsm6dsox_imu']/update_rate") == '208.0'
    assert model.findtext(".//sensor[@name='lsm6dsox_imu']/topic") == '/imu/data'
    assert sum(float(link.findtext('inertial/mass')) for link in model.findall('link')) == pytest.approx(2.)
    sdf_path = tmp_path/'vehicle.sdf'
    sdf_path.write_text(launch._b0183_model(xml, profile))
    checked = subprocess.run(['gz', 'sdf', '-k', str(sdf_path)], capture_output=True, text=True, timeout=20)
    assert checked.returncode == 0, checked.stdout + checked.stderr
    source = ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf'
    before = source.read_bytes()
    output = tmp_path/'world.sdf'
    launch._runtime_world_overrides(source, output, 'configured', True, False)
    assert not ET.parse(output).findall(".//link[@name='safety_bump_0']")
    assert source.read_bytes() == before


def test_visual_batching_preserves_collision_and_marker_xml(tmp_path):
    spec = importlib.util.spec_from_file_location('batch_launch', ROOT/'src/arena_bringup/launch/simulation.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    source = ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf'
    plain, batched = tmp_path/'plain.sdf', tmp_path/'batched.sdf'
    launch._runtime_world_overrides(source, plain, 'bullet', True, False, False)
    launch._runtime_world_overrides(source, batched, 'bullet', True, False, True)
    old, new = ET.parse(plain), ET.parse(batched)
    assert [ET.tostring(v) for v in old.findall('.//collision')] == [ET.tostring(v) for v in new.findall('.//collision')]
    for marker in (0, 20, 30, 45):
        xpath = f".//link[@name='aruco_{marker}']"
        assert ET.tostring(old.find(xpath)) == ET.tostring(new.find(xpath))
    box_count = 0
    for name in ('track_surface', 'grass', 'walls'):
        original = old.find(f".//link[@name='{name}']")
        changed = new.find(f".//link[@name='{name}']")
        count = len(original.findall('visual'))
        box_count += count
        assert len(changed.findall('visual')) == 1
        lines = Path(changed.findtext('visual/geometry/mesh/uri')).read_text().splitlines()
        vertices = np.array([[float(v) for v in line.split()[1:]] for line in lines if line.startswith('v ')])
        assert len(vertices) == count * 8
        assert sum(line.startswith('f ') for line in lines) == count * 12
        for i, visual in enumerate(original.findall('visual')):
            pose = np.array(list(map(float, visual.findtext('pose').split())))
            size = np.array(list(map(float, visual.findtext('geometry/box/size').split())))
            points = vertices[8*i:8*(i+1)] - pose[:3]
            c, s = np.cos(pose[5]), np.sin(pose[5])
            local = points @ np.array([[c,-s,0],[s,c,0],[0,0,1]])
            assert np.allclose(np.abs(local), size/2, atol=1e-9)
    assert box_count > 3000
