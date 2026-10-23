"""센서 기록 시험기의 실물 아닌 RGB/CameraInfo/단일 축 입력 계약."""
import math

import pytest
from rclpy.serialization import serialize_message, deserialize_message
from validate_sensor_probe import fixture_messages
from arena_vehicle_interface.drive_feedback import MOTOR_TOPIC, MOTOR_JOINT, MOTOR_FRAME, MotorEncoderSpec


@pytest.mark.parametrize('index', range(4))
def test_existing_motion_fixture_preserved(index):
    m = fixture_messages(index)
    assert set(m) == {'/imu/data', '/scan'}
    assert all(x.header.stamp.sec == (0, 1, 2, 1)[index] for x in m.values())
    assert math.isinf(m['/scan'].ranges[1])
    assert m['/imu/data'].angular_velocity.z == index/1000.


@pytest.mark.parametrize('index', [0, 1, 2, 3, 19])
def test_full_input_shapes_bytes_and_single_motor(index):
    messages = fixture_messages(index, True)
    assert len(messages) == 5 and '/wheel_states' not in messages
    image = messages['/camera/color/image_raw']
    info = messages['/camera/color/camera_info']
    assert (image.width, image.height) == (1280, 720)
    assert image.encoding == ('rgb8' if index % 2 == 0 else 'bgr8')
    assert len(image.data) == image.step*image.height
    assert bytes(image.data[:3]) == bytes((index, 37, 191))
    if index % 2:
        assert bytes(image.data[3840:3856]) == b'\xee'*16
    assert info.k[0] == 0 and info.header == image.header
    motor = messages[MOTOR_TOPIC]
    assert motor.name == [MOTOR_JOINT] and motor.header.frame_id == MOTOR_FRAME
    spec = MotorEncoderSpec(28, 15., .0325)
    assert spec.counts_from_position(motor.position[0]) == index-10
    assert motor.velocity[0] == spec.motor_rad_s_from_counts((0, 1, -1)[index % 3], .01)
    for message in messages.values():
        payload = serialize_message(message)
        restored = deserialize_message(payload, type(message))
        assert serialize_message(restored) == payload
