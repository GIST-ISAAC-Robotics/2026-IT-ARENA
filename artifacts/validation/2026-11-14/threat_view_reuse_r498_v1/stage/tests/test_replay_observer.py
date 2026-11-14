"""독립 프로세스 수집, 스냅샷 경계, 오류 전파/정상 종료 검증."""
import json
import os
import random
import sys
import time
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from replay_observer import ReplayObserver, sequence_coverage


def row(seq):
    return dict(node='test', callback='imu', sequence=seq)


def test_coverage_detects_holes_duplicates_and_ignores_later_callbacks():
    result = sequence_coverage([row(1), row(3), row(3), row(4)], {'test/imu': 3})['test/imu']
    assert result['first_missing'] == [2]
    assert result['duplicates'] == 1
    assert result['after_snapshot'] == 1
    assert not result['complete']
    assert sequence_coverage([row(1), row(2), row(3)], {'test/imu': 2})['test/imu']['complete']
    assert not sequence_coverage([row(0), row(1)], {'test/imu': 1})['test/imu']['complete']


@pytest.fixture
def ros_nodes(tmp_path, monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    domain = random.randrange(160, 190)
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    source = rclpy.create_node('test_source', context=context)
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1')
    observer = ReplayObserver(tmp_path/'observer', env=env)
    observer.start()
    yield source, observer
    observer.close()
    source.destroy_node()
    context.shutdown()


def publisher_connected(source, observer, message_type):
    publisher = source.create_publisher(message_type, '/replay/diagnostics/timing', 1000)
    deadline = time.monotonic()+5
    while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
        time.sleep(.02)
    assert publisher.get_subscription_count() == 1
    observer.wait_for_publishers(source, {'/replay/diagnostics/timing': 1})
    return publisher


def test_collects_burst_without_spinning_publisher_and_stops(ros_nodes):
    from std_msgs.msg import String
    source, observer = ros_nodes
    publisher = publisher_connected(source, observer, String)
    with pytest.raises(RuntimeError, match='while running'):
        observer.snapshot()
    for i in range(1, 501):
        publisher.publish(String(data=json.dumps(row(i))))
    observer.close()
    rows = observer.snapshot()['timing']
    coverage = sequence_coverage(rows, {'test/imu': 500})['test/imu']
    assert coverage['complete'], dict(coverage=coverage, rows=len(rows), path=str(observer.output))
    assert observer.process.returncode == 0
    assert not json.loads((observer.output/'result.json').read_text())['errors']


def test_callback_failure_is_not_hidden(ros_nodes):
    from std_msgs.msg import String
    source, observer = ros_nodes
    publisher = publisher_connected(source, observer, String)
    publisher.publish(String(data='not valid JSON'))
    observer.process.wait(timeout=5)
    with pytest.raises(RuntimeError, match='exited early'):
        observer.check()
    with pytest.raises(RuntimeError, match='observer failed'):
        observer.close()
    assert json.loads((observer.output/'result.json').read_text())['errors']
    # 실패 종료를 검증한 뒤 fixture의 중복 close 검사만 생략한다.
    observer.process = None
