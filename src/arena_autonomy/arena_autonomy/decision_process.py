"""실행 미연결 판단 프로세스. ROS·출력 허가·명령 재발행은 소유하지 않는다.

검증된 snapshot_call 입력을 그대로 재생한다. 부모의 수신 콜백과 다른
프로세스에서 상태 있는 판단을 수행하며 세대가 바뀌면 그 상태를 버린다.
"""
import json
import math
import multiprocessing as mp
import signal
import time

import numpy as np


def make_controller(header):
    from arena_autonomy.follow_trace import FollowConfig, LocalFollow, MotionConfig
    mode = header.get('controller_mode', 'g1_follow')
    if mode == 'g1_follow' and header.get('schema') == 1:
        follow = LocalFollow(FollowConfig(**header['follow_config']))
    elif mode == 'g1_g2_handoff_trial' and header.get('schema') == 5:
        from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
        follow = BypassHandoffFollow(FollowConfig(**header['follow_config']),
            HandoffExecutionConfig.from_mapping(header['execution_config']), allow_simulation_trial=True)
    else:
        raise ValueError('unregistered decision process controller')
    return follow, MotionConfig(**header['motion_config'])


def _encode(value, capacity):
    encoded = json.dumps(value, allow_nan=False, separators=(',', ':')).encode('utf8')
    if len(encoded) > capacity:
        raise ValueError('decision message exceeds shared capacity')
    return encoded


def _worker(input_buffer, output_buffer, connection, header, capacity):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        from arena_autonomy.follow_trace import freeze, replay_call
        follow, motion = make_controller(header)
        generation = None
        retained_reference = False
        connection.send({'ready': True})
        while True:
            request = connection.recv()
            if request is None:
                break
            if generation != request['generation']:
                retained_reference = False
                if (follow.config.expiry_route_reference_trial
                        and request.get('reset_reason') == 'decision_source_expired'):
                    from arena_autonomy.expiry_reference import reset_after_source_expiry
                    retained_reference = reset_after_source_expiry(follow)
                else:
                    follow.reset()
                generation = request['generation']
            began = time.monotonic()
            data = json.loads(bytes(np.frombuffer(input_buffer, np.uint8, count=request['size'])))
            output = freeze(replay_call(follow, data, motion))
            payload = _encode(output, capacity)
            np.frombuffer(output_buffer, np.uint8, count=len(payload))[:] = np.frombuffer(payload, np.uint8)
            connection.send(dict(generation=generation, sequence=request['sequence'], size=len(payload),
                                 began_wall_s=began, completed_wall_s=time.monotonic(),
                                 **({'expiry_reference_retained': retained_reference}
                                    if follow.config.expiry_route_reference_trial else {})))
    except (EOFError, BrokenPipeError):
        pass
    except Exception as error:
        try:
            connection.send({'error': type(error).__name__+': '+str(error)})
        except (EOFError, BrokenPipeError):
            pass
    finally:
        connection.close()


class DecisionProcess:
    """부모 단일 콜백용. 진행1개+최신 대기1개이며 판단 결과는 허가가 아니다.

    invalidate는 외부 STOP/RESET/시계 경계와 같은 콜백에서 호출한다. 외부가
    계산/적용 시점 차이와 입력 freshness를 확인한 뒤에만 결과를 사용할 수 있다.
    """
    def __init__(self, header, capacity=1024*1024, startup_timeout=15., work_timeout=2.):
        if type(capacity) is not int or not 4096 <= capacity <= 16*1024*1024:
            raise ValueError('bounded integer shared capacity required')
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0
               for v in (startup_timeout, work_timeout)):
            raise ValueError('positive process deadlines required')
        context = mp.get_context('spawn')
        self.capacity, self.work_timeout = capacity, work_timeout
        self.input_buffer = context.RawArray('B', capacity)
        self.output_buffer = context.RawArray('B', capacity)
        self.connection, child = context.Pipe()
        self.generation = self.sequence = 0
        self.expiry_reference_trial = header.get('follow_config', {}).get('expiry_route_reference_trial') is True
        self.expiry_generation = None
        self.inflight = self.pending = self.fault = None
        self.closed = False
        self.stats = {key: 0 for key in ('offered', 'replaced', 'submitted', 'completed', 'old_generation', 'invalidated')}
        self.process = context.Process(target=_worker,
            args=(self.input_buffer, self.output_buffer, child, header, capacity),
            name='arena-decision', daemon=True)
        self.process.start()
        child.close()
        if not self.connection.poll(startup_timeout) or self.connection.recv() != {'ready': True}:
            self.close()
            raise RuntimeError('Decision process startup failed')

    def offer(self, snapshot):
        if self.closed or self.fault:
            raise RuntimeError('Decision process unavailable')
        # 호출자가 나중에 바꿔도 영향받지 않는 JSON 바이트를 소유한다.
        payload = _encode(snapshot, self.capacity)
        self.stats['offered'] += 1
        self.stats['replaced'] += self.pending is not None
        self.pending = payload

    def invalidate(self):
        self.expiry_generation = None
        self.generation += 1
        self.pending = None
        self.stats['invalidated'] += 1
        # 진행 중 버퍼를 재사용하지 않는다. 완료는 pump에서 세대 불일치로 폐기한다.

    def mark_source_expiry(self):
        """폐기한 세대의 다음 첫 요청에만 원인을 전달한다. 이후 STOP은 이를 지운다."""
        if self.expiry_reference_trial:
            self.expiry_generation = self.generation

    def pump(self):
        if self.closed or self.fault:
            return None
        accepted = None
        try:
            if self.connection.poll():
                result = self.connection.recv()
                if 'error' in result:
                    raise RuntimeError(result['error'])
                request = self.inflight
                if request is None or any(result.get(k) != request[k] for k in ('generation', 'sequence')):
                    raise RuntimeError('Decision result identity mismatch')
                now = time.monotonic()
                began, completed = result['began_wall_s'], result['completed_wall_s']
                if not request['submitted_wall_s'] <= began <= completed <= now:
                    raise RuntimeError('Decision completion time invalid')
                if completed-request['submitted_wall_s'] >= self.work_timeout:
                    raise RuntimeError('Decision processing watchdog expired')
                size = result['size']
                if type(size) is not int or not 0 < size <= self.capacity:
                    raise RuntimeError('Decision output size invalid')
                self.inflight = None
                self.stats['completed'] += 1
                if result['generation'] != self.generation:
                    self.stats['old_generation'] += 1
                else:
                    output = json.loads(bytes(np.frombuffer(self.output_buffer, np.uint8, count=size)))
                    accepted = dict(result, output=output, received_wall_s=now,
                                    reference_s=request['reference_s'],
                                    submitted_wall_s=request['submitted_wall_s'])
            if self.inflight is not None and time.monotonic()-self.inflight['submitted_wall_s'] >= self.work_timeout:
                raise RuntimeError('Decision processing watchdog expired')
            if not self.process.is_alive():
                raise RuntimeError('Decision process exited')
            if self.inflight is None and self.pending is not None:
                payload = self.pending
                data = json.loads(payload)
                reference = data['reference_s']
                if type(reference) not in (int, float) or not math.isfinite(reference) or reference < 0:
                    raise ValueError('finite nonnegative reference required')
                self.pending = None
                np.frombuffer(self.input_buffer, np.uint8, count=len(payload))[:] = np.frombuffer(payload, np.uint8)
                self.sequence += 1
                request = dict(sequence=self.sequence, generation=self.generation, size=len(payload),
                               reference_s=reference, submitted_wall_s=time.monotonic())
                if self.expiry_generation == self.generation:
                    request['reset_reason'] = 'decision_source_expired'
                    self.expiry_generation = None
                self.connection.send(request)
                self.inflight = request
                self.stats['submitted'] += 1
        except Exception as error:
            self.fault = type(error).__name__+': '+str(error)
            self.pending = None
            return None
        return accepted

    def close(self):
        if self.closed:
            return None
        self.closed = True
        self.pending = None
        forced = False
        if self.process.is_alive():
            try:
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
