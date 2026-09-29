"""영상 비동기 경계: 상태/시각/세대/공유 버퍼/고장·종료. ROS 발행 없음."""
import copy
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/arena_autonomy'))
from arena_autonomy.core import StartSignal, image_rgb, marker_ids
from arena_autonomy.vision_process import VisionPipeline, VisionProcess, recognize


def message(stamp=1., state='red'):
    rgb = np.full((480, 848, 3), 100, np.uint8)
    cv2.rectangle(rgb, (250, 130), (390, 180), (3, 3, 3), -1)
    for name, x, color in [('red', 280, (255, 4, 1)), ('yellow', 320, (255, 180, 1)),
                            ('green', 360, (1, 255, 10))]:
        cv2.circle(rgb, (x, 155), 10, color if state == name else (0, 0, 0), -1)
    ns = round(stamp*1e9)
    sec, nanosec = divmod(ns, 1_000_000_000)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        data=rgb.tobytes(), height=480, width=848, step=848*3, encoding='rgb8')


class WorkerDouble:
    def __init__(self):
        self.sent = []
        self.result = None
        self.error = None

    def submit(self, request, data):
        self.sent.append((copy.deepcopy(request), data))

    def poll(self):
        if self.error:
            raise RuntimeError(self.error)
        result, self.result = self.result, None
        return result

    def finish(self):
        req, data = self.sent[-1]
        self.result = recognize(req, data)


def begin(pipeline, stamp=1., wall=10., state='red'):
    pipeline.offer(message(stamp, state), wall_now=wall)
    return pipeline.pump(stamp, wall_now=wall)


def test_single_inflight_latest_pending_and_throttle():
    worker = WorkerDouble()
    pipe = VisionPipeline(worker)
    assert begin(pipe) is None
    pipe.offer(message(1.01), 10.01)
    pipe.offer(message(1.10), 10.10)
    pipe.offer(message(1.20), 10.20)
    assert len(worker.sent) == 1
    assert pipe.stats['throttled'] == 1 and pipe.stats['replaced'] == 1
    worker.finish()
    accepted = pipe.pump(1.22, 10.22)
    assert accepted['stamp'] == 1. and accepted['received'] == 10.
    assert len(worker.sent) == 2 and worker.sent[-1][0]['stamp'] == 1.20
    assert worker.sent[-1][0]['signal']['red_frames'] == 1


def test_reset_rejects_old_green_without_overwriting_inflight_buffer():
    worker = WorkerDouble()
    pipe = VisionPipeline(worker)
    begin(pipe)
    worker.finish()
    worker.result['signal']['started'] = True
    pipe.reset()
    pipe.offer(message(.5), 10.1)
    assert len(worker.sent) == 1
    assert pipe.pump(.5, 10.2) is None
    assert not pipe.signal.started and pipe.signal.red_frames == 0
    assert pipe.stats['rejected_epoch'] == 1
    assert worker.sent[-1][0]['epoch'] == 1
    assert worker.sent[-1][0]['signal']['red_frames'] == 0


@pytest.mark.parametrize('now,wall', [(2., 10.2), (.99, 10.2), (1.2, 13.), (1.2, 11.)])
def test_stale_future_or_late_result_cannot_latch_start(now, wall):
    worker = WorkerDouble()
    pipe = VisionPipeline(worker)
    begin(pipe)
    worker.finish()
    worker.result['signal']['started'] = True
    assert pipe.pump(now, wall) is None
    assert pipe.stats['rejected_stale'] == 1
    assert not pipe.signal.started and pipe.last_result is None


def test_stale_pending_not_sent_and_out_of_order_ignored():
    worker = WorkerDouble()
    pipe = VisionPipeline(worker)
    pipe.offer(message(1.), 10.)
    pipe.offer(message(.9), 10.1)
    pipe.offer(message(1.), 10.2)
    pipe.pump(2., 10.3)
    assert not worker.sent and pipe.stats['out_of_order'] == 2
    assert pipe.stats['rejected_stale'] == 1


@pytest.mark.parametrize('kind', ['death', 'timeout', 'identity', 'image'])
def test_fault_is_latched_and_does_not_auto_resume_on_reset(kind):
    worker = WorkerDouble()
    pipe = VisionPipeline(worker)
    if kind == 'image':
        msg = message()
        msg.step = 1
        pipe.offer(msg, 10.)
        pipe.pump(1., 10.)
    else:
        begin(pipe)
        if kind == 'death':
            worker.error = 'worker exited'
        elif kind == 'identity':
            worker.finish()
            worker.result['sequence'] += 1
        pipe.pump(1.1, 11. if kind == 'timeout' else 10.1)
    assert pipe.fault
    sent = len(worker.sent)
    pipe.reset()
    pipe.offer(message(2.), 12.)
    assert pipe.pump(2., 12.) is None
    assert pipe.fault and len(worker.sent) == sent


def test_recognize_preserves_input_state_and_exact_baseline_decisions():
    state = StartSignal()
    direct = StartSignal()
    for n, color in enumerate(('green', 'red', 'red', 'yellow', 'green', 'green', 'green')):
        msg = message(1+n*.1, color)
        before = copy.deepcopy(vars(state))
        req = dict(epoch=0, sequence=n, stamp=1+n*.1, signal=before,
            image={k: getattr(msg, k) for k in ('height', 'width', 'step', 'encoding')})
        result = recognize(req, msg.data)
        assert before == vars(state)
        rgb = image_rgb(msg)
        direct.update(rgb)
        assert result['signal'] == vars(direct)
        assert result['ids'] == sorted(set(marker_ids(rgb, .02)))
        state.__dict__.update(result['signal'])
    assert state.started


def test_real_spawn_shared_buffer_result_and_normal_cleanup():
    worker = VisionProcess()
    try:
        msg = message()
        request = dict(epoch=2, sequence=3, stamp=1., signal=vars(StartSignal()),
            image={k: getattr(msg, k) for k in ('height', 'width', 'step', 'encoding')})
        worker.submit(request, msg.data)
        with pytest.raises(RuntimeError, match='unavailable'):
            worker.submit(request, msg.data)
        deadline = time.monotonic()+5
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll()
            time.sleep(.005)
        assert result['epoch'] == 2 and result['sequence'] == 3
        assert result['signal']['red_frames'] == 1
        assert result['ids'] == []
    finally:
        cleanup = worker.close()
    assert cleanup == {'forced': False, 'exitcode': 0}
    assert not worker.process.is_alive()


def test_real_process_crash_is_reported_and_reaped():
    worker = VisionProcess()
    worker.process.terminate()
    worker.process.join(3)
    with pytest.raises(RuntimeError, match='exited'):
        worker.poll()
    worker.close()
    assert not worker.process.is_alive()
