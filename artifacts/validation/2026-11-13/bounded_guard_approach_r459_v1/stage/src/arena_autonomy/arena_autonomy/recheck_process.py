"""명시 선택형 두 단계 계산 후보. 기존 계획 자식에 순차 현재 재검사 작업을 추가한다.

발행/허가/TTL 갱신을 하지 않는다. 단계마다 소유한 바이트를 전송하며 완료
시각은 비용 계측용이고 출력 reference를 대체하지 않는다.
"""
import json
import math
import multiprocessing as mp
import signal
import time

import numpy as np

from arena_autonomy.decision_process import make_controller, _encode


def evaluate_recheck(output, submitted, current, motion, config):
    from arena_autonomy.follow_trace import restore_call, freeze
    from arena_autonomy.scene_motion import motion_scene
    from arena_autonomy.lidar_motion import MotionUnavailable
    from arena_autonomy import guard_observation_policy as policy
    info, scan, history = restore_call(current, motion)
    try:
        scene = motion_scene(scan, history, info['reference_s'], timing_verified=info['timing_verified'],
                             epoch=info['epoch'], rear_half_angle_deg=info['rear_half_angle_deg'],
                             timing_policy=info.get('timing_policy','verified_only'))
        answer = policy.rebase(output, submitted, current, scene, history, config)
    except (MotionUnavailable, ValueError) as error:
        answer = dict(speed_mps=0., steering_rad=info['previous_steering_rad'], reference_s=info['reference_s'],
                      reason='rebase_input_stop', input_error=str(error),
                      independent_guard_required=True, runtime_authorized=False)
    if answer is None:
        raise ValueError('selected current recheck returned no result')
    return freeze(answer)


def _worker(input_buffer, output_buffer, connection, header, capacity):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        from arena_autonomy.follow_trace import freeze, replay_call
        from arena_autonomy.numeric_kernels import configure
        configure(header.get('numeric_execution','python'))
        follow, motion = make_controller(header)
        generation, planned = None, None
        retained_reference = False
        connection.send({'ready': True})
        while True:
            request = connection.recv()
            if request is None:
                break
            if generation != request['generation']:
                planned = None
                retained_reference = False
                if follow.config.expiry_route_reference_trial and request.get('reset_reason') == 'decision_source_expired':
                    from arena_autonomy.expiry_reference import reset_after_source_expiry
                    retained_reference = reset_after_source_expiry(follow)
                else:
                    follow.reset()
                generation = request['generation']
            began = time.monotonic()
            data = json.loads(bytes(np.frombuffer(input_buffer, np.uint8, count=request['size'])))
            stage = request['stage']
            if stage == 'plan':
                output = freeze(replay_call(follow, data, motion))
                planned = (request['sequence'], data, output)
            elif stage == 'recheck':
                if planned is None or planned[0] != request['plan_sequence']:
                    raise ValueError('recheck without matching completed plan')
                _, submitted, output = planned
                planned = None  # 같은 계획의 재검사는 한 번만 허용한다.
                output = evaluate_recheck(output, submitted, data, motion, follow.config)
            else:
                raise ValueError('unknown work stage')
            payload = _encode(output, capacity)
            np.frombuffer(output_buffer, np.uint8, count=len(payload))[:] = np.frombuffer(payload, np.uint8)
            connection.send(dict(generation=generation, sequence=request['sequence'], stage=stage, size=len(payload),
                                 began_wall_s=began, completed_wall_s=time.monotonic(),
                                 **({'expiry_reference_retained': retained_reference}
                                    if stage == 'plan' and follow.config.expiry_route_reference_trial else {})))
    except (EOFError, BrokenPipeError):
        pass
    except Exception as error:
        try:
            connection.send({'error': type(error).__name__+': '+str(error)})
        except (EOFError, BrokenPipeError):
            pass
    finally:
        connection.close()


class TwoStageProcess:
    """비교용 직렬 소유자. 재검사 중 다음 계획·대기열 적재를 허용하지 않는다."""
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
        self.inflight = self.pending = self.fault = self.plan_token = None
        self.closed = False
        self.stats = {key: 0 for key in ('offered', 'submitted', 'completed', 'old_generation', 'invalidated', 'rechecks')}
        self.process = context.Process(target=_worker,
            args=(self.input_buffer, self.output_buffer, child, header, capacity),
            name='arena-two-stage', daemon=True)
        self.process.start()
        child.close()
        if not self.connection.poll(startup_timeout) or self.connection.recv() != {'ready': True}:
            self.close()
            raise RuntimeError('Two-stage process startup failed')

    def _offer(self, snapshot, stage, plan_sequence=None):
        if self.closed or self.fault:
            raise RuntimeError('Two-stage process unavailable')
        if self.inflight is not None or self.pending is not None:
            raise RuntimeError('serial stage already active')
        payload = _encode(snapshot, self.capacity)
        reference = snapshot['reference_s']
        if type(reference) not in (int, float) or not math.isfinite(reference) or reference < 0:
            raise ValueError('finite nonnegative reference required')
        self.pending = dict(payload=payload, stage=stage, reference_s=reference, plan_sequence=plan_sequence)
        self.stats['offered'] += 1

    def offer(self, snapshot):
        self._offer(snapshot, 'plan')
        self.plan_token = None

    def offer_recheck(self, snapshot, plan_result):
        identity = (plan_result.get('generation'), plan_result.get('sequence'), plan_result.get('stage'))
        if self.plan_token is None or identity != self.plan_token or identity[0] != self.generation:
            raise ValueError('current recheck requires unused current-generation plan result')
        self._offer(snapshot, 'recheck', identity[1])
        self.plan_token = None
        self.stats['rechecks'] += 1

    def invalidate(self):
        self.expiry_generation = None
        self.generation += 1
        self.pending = self.plan_token = None
        self.stats['invalidated'] += 1

    def mark_source_expiry(self):
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
                if request is None or any(result.get(k) != request[k] for k in ('generation', 'sequence', 'stage')):
                    raise RuntimeError('Two-stage result identity mismatch')
                now = time.monotonic()
                began, completed = result['began_wall_s'], result['completed_wall_s']
                if not request['submitted_wall_s'] <= began <= completed <= now:
                    raise RuntimeError('Two-stage completion time invalid')
                if completed-request['submitted_wall_s'] >= self.work_timeout:
                    raise RuntimeError('Two-stage processing watchdog expired')
                size = result['size']
                if type(size) is not int or not 0 < size <= self.capacity:
                    raise RuntimeError('Two-stage output size invalid')
                self.inflight = None
                self.stats['completed'] += 1
                if result['generation'] != self.generation:
                    self.stats['old_generation'] += 1
                else:
                    output = json.loads(bytes(np.frombuffer(self.output_buffer, np.uint8, count=size)))
                    accepted = dict(result, output=output, received_wall_s=now,
                                    reference_s=request['reference_s'], submitted_wall_s=request['submitted_wall_s'])
                    if result['stage'] == 'plan':
                        self.plan_token = (result['generation'], result['sequence'], result['stage'])
            if self.inflight is not None and time.monotonic()-self.inflight['submitted_wall_s'] >= self.work_timeout:
                raise RuntimeError('Two-stage processing watchdog expired')
            if not self.process.is_alive():
                raise RuntimeError('Two-stage process exited')
            if self.inflight is None and self.pending is not None:
                pending = self.pending
                payload = pending['payload']
                self.pending = None
                np.frombuffer(self.input_buffer, np.uint8, count=len(payload))[:] = np.frombuffer(payload, np.uint8)
                self.sequence += 1
                request = dict(sequence=self.sequence, generation=self.generation, size=len(payload),
                               stage=pending['stage'], plan_sequence=pending['plan_sequence'],
                               reference_s=pending['reference_s'], submitted_wall_s=time.monotonic())
                if self.expiry_generation == self.generation:
                    request['reset_reason'] = 'decision_source_expired'
                    self.expiry_generation = None
                self.connection.send(request)
                self.inflight = request
                self.stats['submitted'] += 1
        except Exception as error:
            self.fault = type(error).__name__+': '+str(error)
        return accepted

    def close(self):
        if self.closed:
            return None
        self.closed = True
        forced = False
        if self.process.is_alive():
            try:
                self.connection.send(None)
            except (EOFError, BrokenPipeError):
                pass
            self.process.join(timeout=1.)
            if self.process.is_alive():
                forced = True
                self.process.terminate()
                self.process.join(timeout=2.)
        self.connection.close()
        return dict(forced=forced, exitcode=self.process.exitcode)
