"""ROS를 소유하지 않는 영상 작업 프로세스와 단일 요청 공유 버퍼."""
import copy
import multiprocessing as mp
import signal
import time
from types import SimpleNamespace

import numpy as np

from arena_autonomy.core import StartSignal, image_rgb, marker_ids


def recognize(request, data):
    """상태 변경은 후보로만 반환한다. 부모가 채택하지 않은 결과는 누적되지 않는다."""
    began = time.perf_counter_ns()
    rgb = image_rgb(SimpleNamespace(data=data, **request['image']))
    decoded = time.perf_counter_ns()
    start = StartSignal()
    start.__dict__.update(copy.deepcopy(request['signal']))
    start.update(rgb)
    signaled = time.perf_counter_ns()
    ids = sorted(set(marker_ids(rgb, .02)))
    ended = time.perf_counter_ns()
    return dict(epoch=request['epoch'], sequence=request['sequence'], stamp=request['stamp'],
        signal=vars(start), ids=ids, stages_ms=dict(decode=(decoded-began)/1e6,
        signal=(signaled-decoded)/1e6, markers=(ended-signaled)/1e6),
        duration_ms=(ended-began)/1e6)


def worker_main(buffer, connection):
    # 그룹 SIGINT로 결과 작성 중 끊기지 않는다. 부모의 close/terminate가 수명 관리.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        connection.send({'ready': True})
        while True:
            request = connection.recv()
            if request is None:
                break
            try:
                data = np.frombuffer(buffer, dtype=np.uint8, count=request['size'])
                result = recognize(request, data)
            except Exception as error:
                connection.send({'error': type(error).__name__+': '+str(error)})
                break
            connection.send(result)
    except (EOFError, BrokenPipeError):
        pass
    finally:
        connection.close()


class VisionProcess:
    """spawn을 사용해 ROS/DDS 상태를 fork하지 않는다. 큰 영상은 pipe에 넣지 않는다."""
    def __init__(self, capacity=1920*1080*3, startup_timeout=10.):
        context = mp.get_context('spawn')
        self.buffer = context.RawArray('B', capacity)
        self.capacity = capacity
        self.connection, child = context.Pipe()
        self.process = context.Process(target=worker_main, args=(self.buffer, child),
                                       name='arena-vision', daemon=True)
        self.busy = False
        self.closed = False
        self.process.start()
        child.close()
        if not self.connection.poll(startup_timeout) or self.connection.recv() != {'ready': True}:
            self.close()
            raise RuntimeError('Vision process startup failed')

    def submit(self, request, data):
        if self.busy or self.closed or not self.process.is_alive():
            raise RuntimeError('Vision process unavailable')
        size = len(data)
        if size > self.capacity:
            raise ValueError('Vision image exceeds shared buffer capacity')
        np.frombuffer(self.buffer, np.uint8, count=size)[:] = np.frombuffer(data, np.uint8)
        # 1 요청 진행 중에는 공유 버퍼를 덮어쓰지 않는다. pipe에는 작은 메타데이터만 보낸다.
        self.connection.send(dict(request, size=size))
        self.busy = True

    def poll(self):
        if self.closed or not self.process.is_alive():
            raise RuntimeError('Vision process exited')
        if not self.connection.poll():
            return None
        result = self.connection.recv()
        self.busy = False
        if 'error' in result:
            raise RuntimeError('Vision processing failed: '+result['error'])
        return result

    def close(self):
        if self.closed:
            return
        self.closed = True
        forced = False
        if self.process.is_alive():
            try:
                # 최대 하나의 결과 메시지만 생성하므로 종료 요청을 무제한 큐에 넣지 않는다.
                self.connection.send(None)
            except (BrokenPipeError, EOFError):
                pass
            self.process.join(timeout=1.)
            if self.process.is_alive():
                forced = True
                self.process.terminate()
                self.process.join(timeout=2.)
        self.connection.close()
        return dict(forced=forced, exitcode=self.process.exitcode)


class VisionPipeline:
    """부모 단일 executor 전용. 결과의 세대·촬영 시각·수신 후 나이를 함께 검사한다."""
    def __init__(self, worker, image_timeout=1., wall_timeout=3., work_timeout=1.):
        if min(image_timeout, wall_timeout, work_timeout) <= 0:
            raise ValueError('Vision timeouts must be positive')
        self.worker = worker
        self.image_timeout, self.wall_timeout, self.work_timeout = image_timeout, wall_timeout, work_timeout
        self.epoch = 0
        self.sequence = 0
        self.inflight = None
        self.pending = None
        self.signal = StartSignal()
        self.ids = []
        self.last_submitted = float('-inf')
        self.last_received = float('-inf')
        self.last_accepted = float('-inf')
        self.fault = None
        self.stats = {key: 0 for key in ('received', 'throttled', 'out_of_order', 'replaced',
                     'submitted', 'completed', 'accepted', 'rejected_epoch', 'rejected_stale', 'resets')}
        self.last_result = None
        self.processing = dict(count=0, total_ms=0., maximum_ms=0., over_20ms=0)

    def reset(self):
        self.epoch += 1
        self.pending = None
        self.signal = StartSignal()
        self.ids = []
        self.last_submitted = self.last_received = self.last_accepted = float('-inf')
        self.last_result = None
        self.stats['resets'] += 1
        # 진행 중인 공유 버퍼를 재사용하지 않으며 고장을 자동으로 해제하지 않는다.

    def offer(self, message, wall_now=None):
        wall_now = time.monotonic() if wall_now is None else wall_now
        self.stats['received'] += 1
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if stamp <= self.last_received:
            self.stats['out_of_order'] += 1
            return
        self.last_received = stamp
        if self.fault or stamp-self.last_submitted < .095:
            self.stats['throttled'] += 1
            return
        if self.pending is not None:
            self.stats['replaced'] += 1
        self.pending = (message, stamp, wall_now)

    def fresh(self, stamp, received, now, wall_now):
        return 0 <= now-stamp < self.image_timeout and 0 <= wall_now-received < self.wall_timeout

    def pump(self, now, wall_now=None):
        wall_now = time.monotonic() if wall_now is None else wall_now
        if self.fault:
            return None
        accepted = None
        try:
            result = self.worker.poll()
            if result is not None:
                request = self.inflight
                self.inflight = None
                self.stats['completed'] += 1
                self.processing['count'] += 1
                self.processing['total_ms'] += result['duration_ms']
                self.processing['maximum_ms'] = max(self.processing['maximum_ms'], result['duration_ms'])
                self.processing['over_20ms'] += result['duration_ms'] > 20.
                if request is None or any(result.get(k) != request[k] for k in ('epoch', 'sequence', 'stamp')):
                    raise RuntimeError('Vision result identity mismatch')
                if request['epoch'] != self.epoch:
                    self.stats['rejected_epoch'] += 1
                elif (not self.fresh(request['stamp'], request['received'], now, wall_now)
                      or wall_now-request['submitted'] >= self.work_timeout
                      or request['stamp'] <= self.last_accepted):
                    self.stats['rejected_stale'] += 1
                else:
                    self.signal.__dict__.update(result['signal'])
                    self.ids = result['ids']
                    self.last_accepted = request['stamp']
                    self.last_result = dict(stamp=request['stamp'], received=request['received'],
                        processing_ms=result['duration_ms'], stages_ms=result['stages_ms'])
                    self.stats['accepted'] += 1
                    accepted = self.last_result
            if self.inflight is not None and wall_now-self.inflight['submitted'] >= self.work_timeout:
                raise RuntimeError('Vision processing watchdog expired')
            if self.inflight is None and self.pending is not None:
                message, stamp, received = self.pending
                self.pending = None
                if not self.fresh(stamp, received, now, wall_now):
                    self.stats['rejected_stale'] += 1
                else:
                    image = dict(height=message.height, width=message.width, step=message.step,
                                 encoding=message.encoding)
                    if (message.encoding not in ('rgb8', 'bgr8') or message.height <= 0
                        or message.width <= 0 or message.step < message.width*3
                        or len(message.data) != message.height*message.step):
                        raise ValueError('Invalid vision image layout')
                    self.sequence += 1
                    request = dict(epoch=self.epoch, sequence=self.sequence, stamp=stamp,
                        received=received, submitted=wall_now, image=image,
                        signal=copy.deepcopy(vars(self.signal)))
                    self.worker.submit(request, message.data)
                    self.inflight = request
                    self.last_submitted = stamp
                    self.stats['submitted'] += 1
        except Exception as error:
            self.fault = type(error).__name__+': '+str(error)
            self.pending = None
            return None
        return accepted

    def snapshot(self):
        return dict(mode='process', epoch=self.epoch, fault=self.fault, stats=dict(self.stats),
                    inflight=self.inflight is not None, pending=self.pending is not None,
                    last_result=self.last_result, processing=dict(self.processing),
                    worker_pid=getattr(getattr(self.worker, 'process', None), 'pid', None))

    def close(self):
        return self.worker.close()
