"""원시 자료를 바꾸거나 운영 허가를 내지 않는 형식 진단."""
import copy
import json
import math
import sys
from types import SimpleNamespace as NS

import pytest

import inspect_sensor_probe_inputs as tool
from inspect_sensor_probe_inputs import InputInspection
from arena_vehicle_interface.drive_feedback import MOTOR_TOPIC, MOTOR_JOINT
from test_sensor_probe_audit import fixture


def message(kind):
    header = NS(frame_id='declared_frame', stamp=NS(sec=1, nanosec=0))
    if kind == 'imu':
        return NS(header=header, angular_velocity=NS(x=.1, y=.2, z=.3),
                  linear_acceleration=NS(x=0., y=0., z=9.8),
                  angular_velocity_covariance=[0.]*9, linear_acceleration_covariance=[0.]*9)
    if kind == 'scan':
        return NS(header=header, angle_min=-1., angle_max=1., angle_increment=.5,
                  range_min=.05, range_max=12., time_increment=.001, scan_time=.1,
                  ranges=[1.]*5, intensities=[])
    if kind == 'motor':
        return NS(header=header, name=[MOTOR_JOINT], position=[-.5], velocity=[12.])
    if kind == 'info':
        return NS(header=header, width=4, height=2, distortion_model='', k=[0.]*9,
                  r=[0.]*9, p=[0.]*12, d=[])
    if kind == 'image':
        return NS(header=header, width=4, height=2, step=16, encoding='rgb8', data=bytes(32))
    raise ValueError(kind)


TOPIC = dict(imu='/imu/data', scan='/scan', motor=MOTOR_TOPIC,
             info='/camera/color/camera_info', image='/camera/color/image_raw')


def observe(kind, msg=None, expected=None):
    inspector = InputInspection(expected)
    msg = message(kind) if msg is None else msg
    before = copy.deepcopy(vars(msg))
    inspector.observe(dict(topic=TOPIC[kind], index=1, source_ns=1_000_000_000), msg)
    # NaN != itself; compare stable representation, not accidental ndarray equality.
    assert repr(vars(msg)) == repr(before)
    result = inspector.result()
    assert not any(v for k, v in result['interpretation'].items() if k != 'notes')
    json.dumps(result, allow_nan=False)
    return result['per_topic'][TOPIC[kind]]


@pytest.mark.parametrize('kind', list(TOPIC))
def test_all_five_inputs_read_only_and_bounded(kind):
    r = observe(kind)
    assert r['count'] == 1 and r['frames'] == {'declared_frame': 1}


@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8'])
def test_valid_row_padding(encoding):
    msg = message('image')
    msg.encoding = encoding
    r = observe('image', msg)
    assert not r['findings'] and r['values']['row_padding_bytes']['minimum'] == 4


@pytest.mark.parametrize('key,value', [('encoding','nv12'), ('encoding','rgba8'),
    ('height',0),('width',0),('step',11),('data',bytes(31)),('data',bytes(33))])
def test_image_rejections_match_existing_layout_boundary(key,value):
    msg = message('image')
    setattr(msg,key,value)
    assert observe('image',msg)['findings']['vision_layout_unsupported'] == 1


@pytest.mark.parametrize('change', ['two_wheels','wrong_name','no_velocity','nan_position','inf_velocity'])
def test_single_motor_uses_existing_parser(change):
    msg = message('motor')
    if change == 'two_wheels':
        msg.name = ['rear_left_wheel_joint','rear_right_wheel_joint']
        msg.position, msg.velocity = [0.,0.], [0.,0.]
    elif change == 'wrong_name':
        msg.name = ['wheel']
    elif change == 'no_velocity':
        msg.velocity = []
    elif change == 'nan_position':
        msg.position = [math.nan]
    else:
        msg.velocity = [math.inf]
    assert observe('motor', msg)['findings']['single_motor_format_or_nonfinite'] == 1


def test_motor_is_not_converted_to_body_or_wheel_velocity():
    r = observe('motor')
    assert r['values']['motor_velocity_rad_s'] == dict(minimum=12.,maximum=12.)
    assert 'wheel' not in json.dumps(r) and 'speed_mps' not in json.dumps(r)


def test_imu_unknown_covariance_is_not_fake_confidence():
    r = observe('imu')
    assert r['findings'] == {'gyro_covariance_unknown':1,'accel_covariance_unknown':1}
    assert r['values']['accel_z']['maximum'] == 9.8


def test_imu_unavailable_and_nonfinite_are_distinct():
    msg = message('imu')
    msg.angular_velocity_covariance[0] = -1
    msg.angular_velocity.z = math.nan
    msg.linear_acceleration_covariance[3] = math.inf
    r = observe('imu',msg)
    assert r['findings']['gyro_unavailable'] == 1 and r['findings']['gyro_nonfinite'] == 1
    assert 'gyro_z' not in r['values']
    assert r['findings']['accel_covariance_nonfinite'] == 1


def test_scan_missing_and_out_of_range_remain_unknown():
    msg = message('scan')
    msg.ranges = [math.nan,math.inf,-math.inf,0.,13.,2.]
    msg.angle_max = 1.5
    r = observe('scan',msg)
    assert r['scan_returns'] == dict(nan=1,positive_inf=1,negative_inf=1,
                                    below_minimum=1,above_maximum=1,finite_in_range=1)


def test_invalid_range_limits_do_not_produce_in_range_count():
    msg = message('scan')
    msg.range_min = math.nan
    r = observe('scan',msg)
    assert r['scan_returns'] == {'finite_bounds_invalid':5}
    assert r['findings']['scan_layout_invalid'] == 1


@pytest.mark.parametrize('key,value', [('angle_increment',0.),('angle_increment',-.5),
    ('range_min',-1.),('range_max',.01),('time_increment',-.1),('scan_time',-.1),('ranges',[])])
def test_scan_layout_issues(key,value):
    msg = message('scan')
    setattr(msg,key,value)
    assert observe('scan',msg)['findings']['scan_layout_invalid'] == 1


def test_scan_endpoints_and_quality_length_reported_without_repair():
    msg = message('scan')
    msg.angle_max, msg.intensities, msg.time_increment = 2., [3.], 0.
    findings = observe('scan',msg)['findings']
    assert findings == {'scan_endpoint_inconsistent':1,'scan_intensity_length_mismatch':1,'zero_time_increment':1}


def test_camera_info_uncalibrated_not_guessed_from_dimensions():
    r = observe('info')
    assert r['findings'] == {'camera_info_uncalibrated':1}


def test_camera_info_malformed_matrix_does_not_index_empty_array():
    msg = message('info')
    msg.k = []
    assert observe('info',msg)['findings']['camera_info_matrix_shape'] == 1


def test_frame_check_is_explicit_and_missing_streams_remain_visible():
    inspector = InputInspection({'/scan':'laser_frame','/imu/data':'imu_frame'})
    inspector.observe(dict(topic='/scan',index=1,source_ns=0),message('scan'))
    r = inspector.result()
    assert r['missing_declared_topics'] == ['/imu/data']
    assert r['per_topic']['/scan']['findings']['declared_frame_mismatch'] == 1
    assert r['per_topic']['/scan']['findings']['nonpositive_header_time'] == 1


@pytest.mark.parametrize('frames', [{'/drive':'x'},{'/scan':''},{'/scan':' '*3},{'/scan':123},{'/scan':'x'*129}])
def test_bad_declarations_rejected(frames):
    with pytest.raises(ValueError):
        InputInspection(frames)


def test_categories_and_examples_bounded_over_many_messages():
    inspector = InputInspection({'/scan':'expected'})
    msg = message('scan')
    for i in range(100):
        msg.header.frame_id = str(i)
        inspector.observe(dict(topic='/scan',index=i+1,source_ns=0),msg)
    r = inspector.result()['per_topic']['/scan']
    assert len(r['frames']) == 16 and r['frame_overflow_samples'] == 84
    assert r['examples']['declared_frame_mismatch'] == [1,2,3]


def configure_reader(monkeypatch):
    def deserialize(data, cls):
        msg = message('imu')
        msg.header.stamp.sec, msg.header.stamp.nanosec = 0, 0 if data == b'a' else 100
        return msg
    monkeypatch.setattr(sys.modules['rclpy.serialization'],'deserialize_message',deserialize)


def test_real_audit_observer_contract_before_return(fixture, monkeypatch):
    root, _, _, _, _ = fixture
    configure_reader(monkeypatch)
    r = tool.analyze(root)
    assert r['record_integrity']['record_integrity_verified']
    assert r['per_topic']['/imu/data']['count'] == 2
    assert r['unchecked_frame_topics'] == ['/imu/data']


def test_late_digest_failure_prevents_partial_output(fixture, monkeypatch):
    root, _, _, manifest, save = fixture
    configure_reader(monkeypatch)
    manifest['stats']['/imu/data']['cdr_sha256'] = '0'*64
    save()
    output = root/'inspection.json'
    with pytest.raises(ValueError,match='digest'):
        tool.main([str(root),'--output',str(output)])
    assert not output.exists()


def test_existing_output_is_not_overwritten_or_read(tmp_path,monkeypatch):
    output = tmp_path/'old.json'
    output.write_text('evidence')
    monkeypatch.setattr(tool,'analyze',lambda *args:pytest.fail('must not read'))
    with pytest.raises(FileExistsError):
        tool.main([str(tmp_path),'--output',str(output)])
    assert output.read_text() == 'evidence'
