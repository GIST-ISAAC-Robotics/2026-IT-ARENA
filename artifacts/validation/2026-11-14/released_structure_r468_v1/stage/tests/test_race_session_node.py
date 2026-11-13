"""B3 선택형 연결의 실제 `LocalPursuit` 노드 회귀. ROS가 없는 PC에서는 건너뛴다.

센서/보호층/MCU 프로세스를 실행하지 않는다. 노드 생성·서비스 콜백·명령 콜백만 확인하며
ROS 그래프 배송과 정지 동작은 별도 제한 ROS 시험에서 확인한다.
"""
import json
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from arena_autonomy.local_pursuit import LocalPursuit

BASE = dict(use_sim_time='true', lidar_compensation='both', motion_scan_timing_verified='true',
            motion_imu_topic='/imu/data', motion_feedback_mode='drive_motor_shaft', motion_gear_ratio='15.0',
            motion_wheel_radius_m='0.0325', motion_lidar_x_m='0.06', motion_rear_axle_x_m='-0.0675',
            wheelbase_m='0.135', max_steering_angle_rad='0.37', vision_execution='process',
            start_signal_mode='official_4red_candidate')


def make_node(**parameters):
    values = dict(BASE, **parameters)
    args = ['--ros-args'] + [item for key, value in values.items() for item in ('-p', f'{key}:={value}')]
    rclpy.init(args=args)
    try:
        return LocalPursuit()
    except Exception:
        rclpy.shutdown()
        raise


def close(node):
    try:
        node.destroy_node()
    finally:
        rclpy.shutdown()


def topics(node):
    return ({s.topic_name for s in node.subscriptions}, {p.topic_name for p in node.publishers})


def test_default_mode_keeps_previous_graph_and_endpoint_behavior():
    node = make_node()
    try:
        subscriptions, publishers = topics(node)
        assert node.race is None and node.race_control_mode == 'legacy'
        assert '/actuation/status' not in subscriptions and '/race/command' not in subscriptions
        assert '/race/command_result' not in publishers
        response = node.on_enable(SetBool.Request(data=False), SetBool.Response())
        assert response.success and not node.enabled
        response = node.on_enable(SetBool.Request(data=True), SetBool.Response())
        assert response.success and node.enabled
    finally:
        close(node)


def test_b3_requires_process_vision():
    with pytest.raises(ValueError, match='process vision'):
        make_node(race_control_mode='b3_candidate', vision_execution='inline', start_signal_mode='temporary_3light')
    with pytest.raises(ValueError, match='race_control_mode'):
        make_node(race_control_mode='automatic')


def test_b3_endpoints_cannot_bypass_latch_and_command_resets_both_generations():
    node = make_node(race_control_mode='b3_candidate')
    results = []
    try:
        subscriptions, publishers = topics(node)
        assert {'/actuation/status', '/race/command'} <= subscriptions and '/race/command_result' in publishers
        node.race_result_publisher = SimpleNamespace(publish=lambda m: results.append(json.loads(m.data)))
        race = node.race
        # 프로세스 시작은 래치로 시작한다(이전 ARMED 재사용 금지).
        assert race.permission.latch == 'process_start' and race.permission.intervention == 1
        assert not node.on_enable(SetBool.Request(data=True), SetBool.Response()).success
        assert node.on_enable(SetBool.Request(data=False), SetBool.Response()).success
        assert race.permission.latch == 'autonomy_enable_false' and race.permission.intervention == 2
        assert not node.on_enable(SetBool.Request(data=True), SetBool.Response()).success
        assert race.permission.latch == 'autonomy_enable_false'
        vision_epoch = node.vision_pipeline.epoch
        response = node.on_reset(Trigger.Request(), Trigger.Response())
        assert response.success and 'latched intervention 3' in response.message
        assert race.permission.race_epoch == 1 and race.permission.latch == 'legacy_reset'
        assert node.vision_pipeline.epoch == vision_epoch+1 == race.bound_vision_epoch
        assert node.signal is node.vision_pipeline.signal
        node.on_race_command(String(data=json.dumps(dict(v=1, request='n1', op='new_race',
            session=race.session, race_epoch=1))))
        assert results[-1]['accepted'] and race.permission.race_epoch == 2
        assert node.vision_pipeline.epoch == vision_epoch+2 == race.bound_vision_epoch
        assert race.permission.latch == 'new_race' and race.permission.intervention == 4  # 새 경기도 새 개입
        node.on_race_command(String(data='{not json'))
        assert not results[-1]['accepted'] and results[-1]['reason'] == 'invalid_payload'
        node.on_race_command(String(data=json.dumps(dict(op='stop'))))
        assert results[-1]['accepted'] and race.permission.latch == 'explicit_stop'
        node.on_mcu_status(String(data='nope'))
        assert race.mcu.counts['invalid'] == 1
        node.control()  # 센서 없음: 상태에 경기 감독 기록을 포함해 0 명령
        assert node.last_status == 'SENSOR_STOP' and node.last_speed == 0.
    finally:
        close(node)


@pytest.mark.parametrize('reason,state', [('wait_visual_start', 'WAIT_GREEN'),
    ('actuator_not_ready', 'ACTUATOR_WAIT'), ('explicit_stop', 'RACE_LATCHED'), ('race_permitted', 'unchanged')])
def test_motion_gate_maps_policy_reason_to_status(reason, state):
    fake = SimpleNamespace(race=SimpleNamespace(decide=lambda **kw: dict(
        allowed=reason == 'race_permitted', reason=reason, actuator_reason='x')))
    status = dict(state='unchanged', sim_time_s=1.)
    assert LocalPursuit.motion_gate(fake, status) is (reason == 'race_permitted')
    assert status['state'] == state
