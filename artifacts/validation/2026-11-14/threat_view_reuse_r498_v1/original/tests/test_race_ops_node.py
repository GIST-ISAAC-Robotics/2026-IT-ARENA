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


def link_fixture():
    """ROS 메시지 형식은 실제 것을 쓰되, 경과 시간과 송신 도착 순서는 직접 정한다."""
    from types import SimpleNamespace
    from unittest.mock import Mock
    view = dict(session='s', race_epoch=1, intervention=2, race_id='race')
    entry = dict(client='desk', op='resume_start', request='start-2', view=view)
    race = dict(view, resume=None, last_resume=None)
    job = dict(entry=entry, session='s', client='desk', status_deadline=13.)
    n = SimpleNamespace(start=job, rearm=None, last_link=0., race=lambda: race,
                        core=Mock(), link_publisher=Mock(), publish=Mock(), rearm_step=Mock())
    n.core.client_fresh.return_value = True
    return n, race


def test_operator_link_covers_ack_until_first_status_without_rearming():
    n, race = link_fixture()
    for wall in (10., 10.02, 10.21, 10.42, 10.63, 10.84, 11.05):
        RaceOps.tick_start(n, wall)
    beats = n.link_publisher.publish.call_args_list
    assert len(beats) == 6 and all(call.args[0].data == 'start-2' for call in beats)
    n.rearm_step.assert_not_called()
    race['resume'] = dict(request='start-2', phase='inputs')
    RaceOps.tick_start(n, 11.26)
    n.rearm_step.assert_not_called()
    race['resume']['phase'] = 'mcu'
    RaceOps.tick_start(n, 11.47)
    n.rearm_step.assert_called_once()


@pytest.mark.parametrize('case', ['old_epoch', 'changed_intervention', 'other_pending',
                                  'completed', 'status_deadline', 'client_gone'])
def test_accepted_start_cannot_keep_an_unrelated_or_expired_job_alive(case):
    n, race = link_fixture()
    if case == 'old_epoch': race['race_epoch'] += 1
    if case == 'changed_intervention': race['intervention'] += 1
    if case == 'other_pending': race['resume'] = dict(request='other', phase='mcu')
    if case == 'completed': race['last_resume'] = dict(request='start-2', outcome='permitted', reason='race_permitted')
    if case == 'client_gone': n.core.client_fresh.return_value = False
    RaceOps.tick_start(n, 13. if case == 'status_deadline' else 10.)
    n.link_publisher.publish.assert_not_called()
    n.rearm_step.assert_not_called()
    if case == 'client_gone':
        assert n.publish.call_args.args[0]['start_request'] == 'start-2' and n.start is None
    if case == 'status_deadline':
        assert n.start is None and n.core.mark.call_args.kwargs['stage'] == 'unknown_after_send'
    if case == 'completed':
        assert n.start is None and n.core.mark.call_args.kwargs['stage'] == 'permitted'


@pytest.mark.parametrize('change', [{}, {'op': 'stop'}, {'race_epoch': 0},
                                    {'intervention': 3}, {'session': 'old'}])
def test_first_operator_link_requires_bound_policy_ack(change):
    from std_msgs.msg import String
    n, race = link_fixture()
    entry = n.start['entry']
    n.core.result.return_value = entry
    n.result_timeout_s, n.start = 3., None
    ack = dict(op='resume_start', reason='preparing_not_motion_permission', accepted=True,
               request='start-2', session=race['session'], race_epoch=1, intervention=2)
    RaceOps.on_result(n, String(data=json.dumps({**ack, **change})))
    assert (n.start is not None) == (not change)
    n.rearm_step.assert_not_called()
