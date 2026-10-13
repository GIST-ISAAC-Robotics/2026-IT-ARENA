import time
from types import SimpleNamespace

import pytest

from arena_autonomy.vision_process import VisionPipeline, VisionProcess
from test_vision_process import WorkerDouble
from audit_start_signal_layout import light_image


def message(t, red=4, green=False):
    rgb = light_image(red, green, diameter=12, paired=True)
    sec, nanosec = divmod(round(t*1e9), 1_000_000_000)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
                           height=480, width=848, step=848*3, encoding='rgb8', data=rgb.tobytes())


def complete(pipe, worker, t, red=4, green=False):
    pipe.offer(message(t, red, green), wall_now=t+10)
    pipe.pump(t, wall_now=t+10)
    worker.finish()
    return pipe.pump(t+.01, wall_now=t+10.01)


def test_pipeline_uses_only_accepted_candidate_state():
    worker = WorkerDouble()
    pipe = VisionPipeline(worker, signal_mode='official_4red_candidate')
    for t in (1., 1.1):
        assert complete(pipe, worker, t)
    for t in (1.2, 1.3):
        assert complete(pipe, worker, t, 0, True)
        assert not pipe.signal.started
    assert complete(pipe, worker, 1.4, 0, True)
    assert pipe.signal.started
    assert pipe.snapshot()['signal_mode'] == 'official_4red_candidate'


def test_reset_rejects_old_candidate_green_and_keeps_mode():
    worker = WorkerDouble()
    pipe = VisionPipeline(worker, signal_mode='official_4red_candidate')
    for t in (1., 1.1):
        complete(pipe, worker, t)
    for t in (1.2, 1.3):
        complete(pipe, worker, t, 0, True)
    pipe.offer(message(1.4, 0, True), wall_now=11.4)
    pipe.pump(1.4, 11.4)
    worker.finish()
    assert worker.result['signal']['started']
    pipe.reset()
    assert pipe.pump(1.41, 11.41) is None
    assert not pipe.signal.started and pipe.signal.bank is None
    assert pipe.stats['rejected_epoch'] == 1
    for t in (1.5, 1.6, 1.7):
        complete(pipe, worker, t, 0, True)
    assert not pipe.signal.started


def test_mixed_signal_modes_latch_fault():
    worker = WorkerDouble()
    pipe = VisionPipeline(worker, signal_mode='official_4red_candidate')
    pipe.offer(message(1.), wall_now=11.)
    pipe.pump(1., 11.)
    worker.finish()
    worker.result['signal_mode'] = 'temporary_3light'
    assert pipe.pump(1.01, 11.01) is None
    assert 'mode mismatch' in pipe.fault


def test_invalid_mode_not_silently_replaced():
    with pytest.raises(ValueError, match='start mode'):
        VisionPipeline(WorkerDouble(), signal_mode='green_anywhere')


def test_real_spawn_worker_roundtrip_and_clean_close():
    worker = VisionProcess()
    pipe = VisionPipeline(worker, work_timeout=3., signal_mode='official_4red_candidate')
    try:
        for i in range(5):
            t = 1.+i*.1
            pipe.offer(message(t, 4 if i < 2 else 0, i >= 2))
            pipe.pump(t)
            deadline = time.monotonic()+3.
            accepted = None
            while time.monotonic() < deadline and accepted is None and not pipe.fault:
                time.sleep(.01)
                accepted = pipe.pump(t)
            assert accepted is not None and not pipe.fault
        assert pipe.signal.started
    finally:
        closed = worker.close()
    assert closed == {'forced': False, 'exitcode': 0}
