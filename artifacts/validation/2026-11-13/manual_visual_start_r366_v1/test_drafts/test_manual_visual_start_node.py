"""수동 출발의 실제 노드 메서드 연결만 확인한다. 노드/센서/MCU 프로세스는 만들지 않는다."""
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip('rclpy')
from std_msgs.msg import String
from arena_autonomy.local_pursuit import LocalPursuit
from arena_autonomy.race_ops_node import RaceOps
from test_manual_visual_start import waiting


def node_waiting():
    node = SimpleNamespace(race=waiting(), enabled=True, last_status='WAIT_GREEN', last_control_time=.3,
        now_s=lambda: .32, scan=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=300_000_000))),
        rgb_stamp=.3, scan_wall_time=1., image_wall_time=1.,
        settings=dict(scan_timeout_s=.45, image_timeout_s=1., sensor_wall_timeout_s=3.),
        motion=Mock(), race_result_publisher=Mock(), stop=Mock())
    node.manual_visual_start_waiting = lambda: LocalPursuit.manual_visual_start_waiting(node)
    return node


@pytest.mark.parametrize('case', ['ok', 'disabled', 'running', 'scan_stale', 'rgb_stale', 'wall_stale', 'clock_backwards', 'stop'])
def test_node_rechecks_existing_wait_state_without_projection_or_publish_drive(monkeypatch, case):
    monkeypatch.setattr('arena_autonomy.local_pursuit.time.monotonic', lambda: 1.)
    node = node_waiting()
    if case == 'disabled': node.enabled = False
    if case == 'running': node.last_status = 'RUNNING'
    if case == 'scan_stale': node.scan = None
    if case == 'rgb_stale': node.rgb_stamp = None
    if case == 'wall_stale': node.scan_wall_time = -3.
    if case == 'clock_backwards': node.last_control_time = .4
    if case == 'stop': node.race.stop(now_s=.31)
    payload = dict(v=1, request='manual', op='manual_visual_start', session='s', race_epoch=0,
                   intervention=node.race.permission.intervention, signal_confirmed=True)
    LocalPursuit.on_race_command(node, String(data=json.dumps(payload)))
    result = json.loads(node.race_result_publisher.publish.call_args.args[0].data)
    assert result['accepted'] is (case == 'ok')
    node.motion.project.assert_not_called()
    node.stop.assert_not_called()


def test_ops_only_publishes_bound_fact_and_never_calls_mcu(monkeypatch):
    monkeypatch.setattr('arena_autonomy.race_ops_node.time.monotonic', lambda: 1.)
    node = SimpleNamespace(publish=Mock(), call=Mock(), core=Mock())
    node.bound = lambda entry, op: RaceOps.bound(node, entry, op)
    entry = dict(op='manual_visual_start', request='manual',
                 view=dict(session='s', race_epoch=0, intervention=2, race_id='r'))
    RaceOps.execute(node, entry)
    assert node.publish.call_args.args[0] == dict(v=1, request='manual', op='manual_visual_start',
        session='s', race_epoch=0, intervention=2, signal_confirmed=True)
    node.call.assert_not_called()
