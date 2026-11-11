"""실제 spawn 통신/판단 동일성·세대 폐기. ROS 출력 연결은 검사 범위가 아니다."""
import copy
from dataclasses import asdict
import math
import time

import pytest

from arena_autonomy.decision_process import DecisionProcess, make_controller
from arena_autonomy.follow_trace import snapshot_call, replay_call, freeze, comparable_decision, equal_output
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionHistory, MotionConfig
from arena_autonomy.measurement_scan import MeasurementScan


def fixture():
    history = MotionHistory(MotionConfig())
    for i in range(141):
        history.add_mean_wheel(i*.005, 0.)
        history.add_gyro(i*.005, 0.)
    config = FollowConfig()
    header = dict(schema=1, follow_config=asdict(config), motion_config=asdict(history.config))
    angles = tuple(math.pi-i*math.tau/508 for i in range(508))
    # 두 평행 벽, 앞뒤는 반환 없음. 관측 여부를 만들어 넣지 않는다.
    distances = tuple(.3/abs(math.sin(a)) if abs(math.sin(a)) > .03 else math.inf for a in angles)

    def sample(reference):
        scan = MeasurementScan(distances, angles,
            tuple(reference-.12+i*.1/508 for i in range(508)), time_basis='simulation_acquisition')
        return snapshot_call(scan, history, reference, 0, 0., 'left', 0., .1, True, 30.)
    return header, sample


def receive(worker, timeout=10.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        result = worker.pump()
        assert worker.fault is None, worker.fault
        if result is not None:
            return result
        time.sleep(.005)
    raise AssertionError('decision result not delivered')


def test_actual_process_stateful_commands_equal_and_input_is_owned():
    header, sample = fixture()
    worker = DecisionProcess(header)
    local, motion = make_controller(header)
    try:
        for reference in (.32, .42, .52):
            data = sample(reference)
            expected = comparable_decision(freeze(replay_call(local, data, motion)))
            worker.offer(data)
            data['speed_mps'] = 999.  # offer 뒤의 외부 변경이 공유 입력을 바꾸면 안 된다.
            result = receive(worker)
            assert result['reference_s'] == reference
            assert equal_output(comparable_decision(result['output']), expected)
            assert result['submitted_wall_s'] <= result['completed_wall_s'] <= result['received_wall_s']
    finally:
        cleanup = worker.close()
    assert cleanup == dict(forced=False, exitcode=0)
    assert not worker.process.is_alive()


def test_busy_latest_pending_only_and_reset_discards_old_result():
    header, sample = fixture()
    worker = DecisionProcess(header)
    try:
        worker.offer(sample(.32))
        assert worker.pump() is None
        worker.offer(sample(.42))
        worker.offer(sample(.52))
        assert worker.stats['replaced'] == 1
        worker.invalidate()  # 진행/대기 이전 세대 모두 출력으로 살아나면 안 된다.
        worker.offer(sample(.62))
        result = receive(worker)
        assert result['generation'] == 1 and result['reference_s'] == .62
        assert worker.stats['old_generation'] == 1
        assert worker.stats['submitted'] == 2
        local, motion = make_controller(header)
        expected = freeze(replay_call(local, sample(.62), motion))
        assert equal_output(comparable_decision(result['output']), comparable_decision(expected))
    finally:
        worker.close()


def test_child_exit_and_corrupt_payload_do_not_return_a_command():
    header, sample = fixture()
    worker = DecisionProcess(header)
    try:
        broken = copy.deepcopy(sample(.32))
        del broken['history']
        worker.offer(broken)
        end = time.monotonic()+5.
        while worker.fault is None and time.monotonic() < end:
            assert worker.pump() is None
            time.sleep(.005)
        assert worker.fault is not None
        with pytest.raises(RuntimeError):
            worker.offer(sample(.42))
    finally:
        worker.close()
    assert not worker.process.is_alive()


def test_work_deadline_fails_without_renewing_old_command():
    header, sample = fixture()
    worker = DecisionProcess(header, work_timeout=1e-9)
    try:
        worker.offer(sample(.32))
        assert worker.pump() is None
        assert worker.pump() is None
        assert 'watchdog' in worker.fault
    finally:
        worker.close()


def test_input_capacity_is_checked_before_submission():
    header, sample = fixture()
    worker = DecisionProcess(header, capacity=4096)
    try:
        with pytest.raises(ValueError, match='capacity'):
            worker.offer(sample(.32))
        assert worker.inflight is None and worker.pending is None
        assert worker.stats['submitted'] == 0
    finally:
        worker.close()
