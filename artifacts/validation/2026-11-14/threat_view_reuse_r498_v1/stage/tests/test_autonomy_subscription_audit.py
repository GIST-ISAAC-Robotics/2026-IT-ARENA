"""ROS graph 관측의 구독 경계. 새 입력 선택이 평가용 정답 허용으로 번지지 않는다."""
import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
tree = ast.parse((ROOT / 'scripts/validate_basic_autonomy.py').read_text(encoding='utf-8'))
function = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                and n.name == 'audit_sensor_subscriptions')
namespace = {}
exec(compile(ast.Module(body=[function], type_ignores=[]), '<subscription-audit>', 'exec'), namespace)
audit = namespace['audit_sensor_subscriptions']


def graph(native):
    scan = '/scan/native' if native else '/scan'
    return ({scan, '/imu/data', '/drive_motor/encoder', '/camera/color/image_raw', '/clock'},
            {scan, '/imu/data', '/drive_motor/encoder', '/drive', '/clock'})


def check(inputs, safety, native=True):
    return audit(inputs, safety, mode='local_pursuit', compensation='both',
                 feedback_topic='/drive_motor/encoder', native_sampling=native)


@pytest.mark.parametrize('native', [False, True])
def test_selected_input_and_independent_guard_are_verified(native):
    assert check(*graph(native), native) == (True, True)


@pytest.mark.parametrize('extra', ['/scan', '/scan/native', '/model/pose', '/map', '/drive/safe'])
@pytest.mark.parametrize('side', [0, 1])
def test_wrong_or_additional_source_does_not_pass(extra, side):
    native = extra != '/scan/native'
    inputs = list(graph(native))
    inputs[side].add(extra)
    result = check(*inputs, native)
    assert result[side] is False and result[1-side] is True


@pytest.mark.parametrize('missing', ['/scan/native', '/imu/data', '/drive_motor/encoder'])
def test_missing_required_sensor_fails_both(missing):
    a, b = graph(True)
    assert check(a - {missing}, b - {missing}) == (False, False)


def test_missing_request_to_guard_fails():
    a, b = graph(True)
    assert check(a, b - {'/drive'}) == (True, False)


def test_original_stereo_and_tof_contract_preserved():
    a = {'/camera/depth/image_rect_raw', '/camera/depth/camera_info', '/camera/color/image_raw'}
    b = {'/drive', '/drive_motor/encoder'} | {
        f'/tof/{name}/points' for name in ('front_left', 'rear_left', 'rear_right', 'front_right')}
    kwargs = dict(mode='stereo', compensation='none', feedback_topic='/drive_motor/encoder')
    assert audit(a, b, **kwargs) == (True, True)
    assert audit(a, b - {'/tof/rear_left/points'}, **kwargs) == (True, False)
    assert audit(a, set(), disable_tof_safety=True, **kwargs) == (True, None)


def test_original_lidar_camera_imu_contract_preserved():
    a = {'/scan', '/camera/color/image_raw', '/camera/imu', '/drive_motor/encoder'}
    assert audit(a, set(), mode='lidar', compensation='both',
                 feedback_topic='/drive_motor/encoder', disable_tof_safety=True) == (True, None)
    assert audit(a - {'/camera/imu'} | {'/imu/data'}, set(), mode='lidar', compensation='both',
                 feedback_topic='/drive_motor/encoder', disable_tof_safety=True) == (False, None)


def test_native_not_silently_accepted_in_unsupported_mode():
    with pytest.raises(ValueError):
        audit(set(), set(), mode='stereo', compensation='both',
              feedback_topic='/drive_motor/encoder', native_sampling=True)
