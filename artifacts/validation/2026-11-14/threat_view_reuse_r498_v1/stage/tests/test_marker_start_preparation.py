"""늦은 녹색 출발의 실제 픽셀·영상 세대·B3 준비 연결. 차량 운동 증거는 아니다."""
import time
from types import SimpleNamespace

import pytest

from arena_autonomy.race_session import RaceSupervisor
from arena_autonomy.vision_process import VisionPipeline, VisionProcess
from test_marker_late_start import frame
from test_vision_process import WorkerDouble


MODE = 'official_marker_late_candidate'


def message(t, kind='green'):
    rgb = frame(kind)
    sec, nanosec = divmod(round(t * 1e9), 1_000_000_000)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        height=rgb.shape[0], width=rgb.shape[1], step=rgb.shape[1]*3,
        encoding='rgb8', data=rgb.tobytes())


def setup():
    worker = WorkerDouble()
    pipe = VisionPipeline(worker, signal_mode=MODE)
    race = RaceSupervisor(pipe, session='late-start-fixture', now_s=.1)
    return worker, pipe, race


def complete(worker, pipe, race, t, kind='green'):
    pipe.offer(message(t, kind), t+100.)
    # 수신은 촬영 뒤다. 정확히 같은 float를 쓰면 ns 변환의 1 ULP가 미래 프레임이 된다.
    pipe.pump(t+.001, t+100.001)
    worker.finish()
    accepted = pipe.pump(t+.01, t+100.01)
    assert accepted is not None and not pipe.fault
    race.observe_vision(accepted, pipe.epoch, t+.01)


def prepare(race, t, request='prepare'):
    return race.handle_command(dict(v=1, request=request, op='acknowledge', session=race.session,
        race_epoch=race.permission.race_epoch, intervention=race.permission.intervention,
        hazards_cleared=True, operator_reviewed=True), t)


def test_preparation_discards_probe_green_and_needs_current_green_again():
    worker, pipe, race = setup()
    for t in (1., 1.1, 1.2):
        complete(worker, pipe, race, t)
    assert pipe.signal.started and not race.permission.started
    assert not race.decide(sensors_ready=True, now_s=1.3, wall_s=101.3)['allowed']
    previous_epoch = pipe.epoch
    assert prepare(race, 1.3)['accepted']
    assert pipe.epoch == previous_epoch+1 and pipe.input_floor_s == 1.3
    for t in (1.4, 1.5, 1.6):
        complete(worker, pipe, race, t, 'off')
    assert not pipe.signal.started and not race.permission.started
    for t in (1.7, 1.8, 1.9):
        complete(worker, pipe, race, t)
    assert pipe.signal.started and race.permission.started
    # 영상 출발 확인은 별도 MCU ARM·센서·보호층을 대신하지 않는다.
    assert not race.decide(sensors_ready=True, now_s=2., wall_s=102.)['allowed']
    epoch = pipe.epoch
    duplicate = prepare(race, 2., request='prepare')
    assert pipe.epoch == epoch and not duplicate['accepted']
    assert duplicate['reason'] == 'duplicate_request'
    race.stop(now_s=2.1)
    complete(worker, pipe, race, 2.2)
    assert not race.decide(sensors_ready=True, now_s=2.21, wall_s=102.21)['allowed']


def test_inflight_and_delayed_capture_before_preparation_are_not_adopted():
    worker, pipe, race = setup()
    for t in (1., 1.1):
        complete(worker, pipe, race, t)
    pipe.offer(message(1.2), 101.2)
    pipe.pump(1.2, 101.2)
    worker.finish()
    assert worker.result['signal']['started']
    assert prepare(race, 1.3)['accepted']
    assert pipe.pump(1.31, 101.31) is None
    assert pipe.stats['rejected_epoch'] == 1 and not pipe.signal.started
    pipe.offer(message(1.25), 101.32)
    assert pipe.stats['before_boundary'] == 1 and pipe.pending is None
    for t in (1.4, 1.5, 1.6):
        complete(worker, pipe, race, t)
    assert race.permission.started


@pytest.mark.parametrize('t', [float('nan'), float('inf'), -1.])
def test_invalid_preparation_clock_does_not_release_latch(t):
    _, pipe, race = setup()
    assert not prepare(race, t)['accepted']
    assert race.permission.latch == 'process_start' and pipe.epoch == 0


def test_marker_detection_is_shared_with_route_ids(monkeypatch):
    import arena_autonomy.vision_process as module
    original, calls = module.marker_observations, []
    def detect(rgb, distance):
        calls.append(distance)
        return original(rgb, distance)
    monkeypatch.setattr(module, 'marker_observations', detect)
    worker, pipe, race = setup()
    complete(worker, pipe, race, 1.)
    assert calls == [.02] and pipe.ids == [0, 45]


def test_real_worker_serializes_marker_state_and_closes_cleanly():
    worker = VisionProcess()
    pipe = VisionPipeline(worker, work_timeout=3., signal_mode=MODE)
    try:
        for t in (1., 1.1, 1.2):
            pipe.offer(message(t))
            pipe.pump(t)
            deadline = time.monotonic()+3.
            accepted = None
            while time.monotonic() < deadline and accepted is None and not pipe.fault:
                time.sleep(.01)
                accepted = pipe.pump(t)
            assert accepted is not None and not pipe.fault
        assert pipe.signal.started and pipe.signal.observed == 'late_green_confirmed'
    finally:
        closed = worker.close()
    assert closed == {'forced': False, 'exitcode': 0}
