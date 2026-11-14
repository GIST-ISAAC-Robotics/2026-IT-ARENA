"""기본 장착과 명시 시험 장착의 분리 및 실제 SDF 카메라 방향."""
from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

from test_low_speed_arc_reacquire import launch_module

ROOT = Path(__file__).resolve().parents[1]
SHARE = ROOT/'src/arena_description'
CANDIDATE = ROOT/'artifacts/validation/2026-11-02/official_camera_mount_v1/b0183_up20.yaml'


def test_empty_override_keeps_original_purchase_profile():
    path = SHARE/'config/b0183_c1.yaml'
    before = path.read_bytes()
    assert launch_module()._local_sensor_profile(SHARE, '', True) == yaml.safe_load(before)
    assert path.read_bytes() == before


def test_selected_mount_changes_only_camera_pitch():
    launch = launch_module()
    original = launch._local_sensor_profile(SHARE, '', True)
    candidate = launch._local_sensor_profile(SHARE, str(CANDIDATE), True)
    expected = deepcopy(original)
    expected['camera']['rpy_rad'][1] = -math.radians(20.)
    assert candidate == expected


def test_override_cannot_be_silently_ignored_in_another_sensor_mode():
    launch = launch_module()
    assert launch._local_sensor_profile(SHARE, '', False) is None
    with pytest.raises(ValueError, match='local_pursuit'):
        launch._local_sensor_profile(SHARE, str(CANDIDATE), False)


def test_missing_profile_is_an_error(tmp_path):
    with pytest.raises(FileNotFoundError):
        launch_module()._local_sensor_profile(SHARE, str(tmp_path/'missing.yaml'), True)


@pytest.mark.parametrize('text', ['', 'camera: {}\nlidar: {}\n', 'camera: 3\nlidar: {}\nimu: {}\n'])
def test_incomplete_profile_is_not_a_default_fallback(tmp_path, text):
    path = tmp_path/'configuration with spaces.yaml'
    path.write_text(text, encoding='utf-8')
    with pytest.raises(ValueError, match='camera, lidar and imu'):
        launch_module()._local_sensor_profile(SHARE, str(path), True)


def test_upward_mount_is_positive_world_z_for_camera_forward_axis():
    launch = launch_module()
    profile = launch._local_sensor_profile(SHARE, str(CANDIDATE), True)
    camera = profile['camera']
    mappings = {'d435i_depth_enabled': 'false', 'tof_enabled': 'false',
                **{f'd435i_{axis}': str(value) for axis, value in zip(('x', 'y', 'z'), camera['xyz_m'])},
                **{f'd435i_{axis}': str(value) for axis, value in zip(('roll', 'pitch', 'yaw'), camera['rpy_rad'])}}
    xml = xacro.process_file(str(SHARE/'models/arena_car/model.sdf.xacro'), mappings=mappings).toxml()
    model = ET.fromstring(launch._b0183_model(xml, profile)).find('model')
    pose = list(map(float, model.findtext("link[@name='b0183_link']/pose").split()))
    assert pose[:3] == camera['xyz_m']
    assert pose[3:] == pytest.approx(camera['rpy_rad'])
    assert -math.sin(pose[4]) == pytest.approx(math.sin(math.radians(20.)))
    assert model.findtext("link[@name='lsm6dsox_link']/sensor/update_rate") == '208.0'
