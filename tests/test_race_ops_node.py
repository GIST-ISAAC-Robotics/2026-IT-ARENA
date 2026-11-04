"""운영 어댑터 노드의 HTTP 경계. ROS가 없는 PC에서는 건너뛴다. 제어기·MCU를 실행하지 않는다."""
import json
import urllib.error
import urllib.request

import pytest

rclpy = pytest.importorskip('rclpy')

from arena_autonomy.race_ops_node import RaceOps


@pytest.fixture
def ops(tmp_path):
    token = tmp_path/'token'
    token.write_text('operator-token', encoding='utf-8')
    view = tmp_path/'view'
    view.write_text('viewer-token', encoding='utf-8')
    rclpy.init(args=['--ros-args', '-p', 'vehicle_id:=car7', '-p', 'port:=0', '-p', f'token_path:={token}',
                     '-p', f'view_token_path:={view}', '-p', f'record_path:={tmp_path/"record.json"}',
                     '-p', f'log_path:={tmp_path/"log.jsonl"}'])
    node = RaceOps()
    node.port = node.server.server_address[1]
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def call(node, path, *, body=None, token='operator-token', vehicle='car7'):
    request = urllib.request.Request(f'http://127.0.0.1:{node.port}{path}',
        data=None if body is None else json.dumps(body).encode(),
        headers={'X-Arena-Token': token, 'X-Arena-Vehicle': vehicle})
    try:
        with urllib.request.urlopen(request, timeout=2.) as response:
            return response.status, dict(response.headers), response.read()
    except urllib.error.HTTPError as error:
        return error.code, dict(error.headers), error.read()


COMMAND = dict(op='stop', request='req-1', client='client-1')


def test_page_is_public_but_state_needs_token_and_vehicle(ops):
    code, _, page = call(ops, '/', token='', vehicle='')
    assert code == 200 and 'STOP'.encode() in page and b'operator-token' not in page
    assert call(ops, '/api/state', token='wrong')[0] == 401
    assert call(ops, '/api/state', vehicle='car8')[0] == 409
    code, headers, data = call(ops, '/api/state?client=abc')
    assert code == 200 and headers['X-Arena-Role'] == 'operator'
    assert json.loads(data)['vehicle_id'] == 'car7'


def test_view_token_cannot_command(ops):
    code, headers, _ = call(ops, '/api/state', token='viewer-token')
    assert code == 200 and headers['X-Arena-Role'] == 'view'
    assert call(ops, '/api/command', body=COMMAND, token='viewer-token')[0] == 403
    assert ops.core.next_action() is None


def test_command_shape_and_queue_only(ops):
    assert call(ops, '/api/command', body=dict(COMMAND, request='bad id!'))[0] == 400
    code, _, data = call(ops, '/api/command', body=COMMAND)
    assert code == 200 and json.loads(data)['stage'] == 'queued'
    assert ops.core.next_action()['request'] == 'req-1'
    assert call(ops, '/api/nothing')[0] == 404


def test_no_drive_publishers_exist(ops):
    topics = {name for name, _ in ops.get_topic_names_and_types()}
    published = {p.topic_name for p in ops.publishers}
    assert {'/race/command', '/race/operator_link'} <= published
    assert not published & {'/drive', '/drive/safe', '/actuation/tx', '/sim/cmd_vel'}
    assert isinstance(topics, set)
