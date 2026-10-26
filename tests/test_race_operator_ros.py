"""격리 ROS에서 실제 LocalPursuit와 도구를 연결. MCU/센서/시뮬레이터/차량 없음."""
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import race_operator as operator
from arena_autonomy.local_pursuit import LocalPursuit


@pytest.fixture
def server():
    # 벽시계 timer로 상태를 발행하되 센서/MCU 없음으로 끝까지 움직임 불허.
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=false', '-p', 'lidar_compensation:=both',
        '-p', 'race_control_mode:=b3_candidate', '-p', 'vision_execution:=process'])
    node = LocalPursuit()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield node
    finally:
        executor.shutdown(timeout_sec=5.)
        thread.join(timeout=5.)
        node.destroy_node()
        rclpy.shutdown()
        assert not thread.is_alive()


def test_real_node_status_ack_new_race_stop_and_stale_token(server):
    code, observed = operator.exchange('status')
    assert code == 0 and observed['race']['latch'] == 'process_start'
    race = observed['race']
    ack = operator.build_command('acknowledge', session=race['session'], race_epoch=race['race_epoch'],
        intervention=race['intervention'], hazards_cleared=True, operator_reviewed=True)
    code, result = operator.exchange('acknowledge', ack)
    assert code == 0 and result['result']['latch'] is None
    assert server.race.rearm_required and not server.race.permission.started
    assert server.last_speed == 0.

    new = operator.build_command('new_race', session=race['session'], race_epoch=race['race_epoch'])
    code, result = operator.exchange('new_race', new)
    assert code == 0 and result['result']['race_epoch'] == race['race_epoch']+1
    assert result['result']['latch'] == 'new_race'
    commands_before = server.race.counts['commands']
    code, result = operator.exchange('acknowledge', ack)
    assert code == 2 and result['outcome'] == 'not_sent'
    assert server.race.counts['commands'] == commands_before

    code, result = operator.exchange('stop', operator.build_command('stop'))
    assert code == 0 and result['result']['latch'] == 'explicit_stop'
    assert server.last_speed == 0. and server.race.rearm_required
    assert '/actuation/command' not in {p.topic_name for p in server.publishers}
    assert '/drive/safe' not in {p.topic_name for p in server.publishers}


def test_result_loss_is_unknown_and_never_retransmitted(server):
    original = server.race_result_publisher
    server.race_result_publisher = SimpleNamespace(publish=lambda message: None)
    before = server.race.counts['commands']
    # v2는 1.5초의 짧은 discovery 예산에서 전송조차 못했다. 도구 기본 5초로
    # 적용 여부를 확인하고 응답만 버린다. 운영 TTL/보호 한도를 늘리는 시험이 아니다.
    code, result = operator.exchange('stop', operator.build_command('stop'))
    server.race_result_publisher = original
    assert code == 4 and result['outcome'] == 'unknown_after_send', (result, server.race.counts['commands']-before)
    assert server.race.counts['commands'] == before+1, result
    assert server.race.permission.latch == 'explicit_stop'
    assert server.last_speed == 0.


def test_unavailable_server_does_not_send():
    code, result = operator.exchange('stop', operator.build_command('stop'), timeout_s=.2)
    assert code == 2 and result['outcome'] == 'not_sent', result
    assert result['error'].startswith('timeout;')  # ROS 초기화 예외를 미전송 대기 검사의 통과로 세지 않는다.
