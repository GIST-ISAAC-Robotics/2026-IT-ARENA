"""명시 선택형 두 계산 슬롯. 정책 factory를 받아 원 바이트·세대·완료를 관리한다.

알고리즘, ROS 시계, 발행, 실물 허가를 선택하지 않는다. factory(stage, header)는
spawn 가능한 최상위 callable이며 reset(reason), __call__(owned_data)를 제공한다.
시험용 대기·artifact 경로·단일 process 속성은 제공하지 않는다.
"""
import copy
import json
import math
import multiprocessing as mp
from multiprocessing.connection import wait
import signal
import time
import numpy as np
from arena_autonomy.decision_process import _encode


class BothConnections:
    def __init__(self, *connections):
        self.connections = connections

    def poll(self, timeout=0.):
        return bool(wait(self.connections, timeout))


def plan_reference(snapshot):
    """계획을 실제 계산한 부모 소유 입력에서만 만든 내부 기하 참조다.

    이전 광선/운동 이력은 다시 쓰지 않는다. 현재 광선/이력은 원문 그대로
    전달하며 부모의 만료·epoch 검사와 기록에는 원 전체 입력을 보존한다.
    """
    return dict(reference_s=snapshot['reference_s'],epoch=snapshot['epoch'],
                side=snapshot['side'],previous_steering_rad=snapshot['previous_steering_rad'],
                scan=dict(representation=snapshot['scan']['representation'],
                          measurements=dict(source_epoch=snapshot['scan']['measurements']['source_epoch'])))


def _worker(stage, input_buffer, output_buffer, connection, header, capacity, evaluator_factory):
    # factory와 그 결과는 명시 선택된 알고리즘 경계다. 이 모듈은 정책을 고르지 않는다.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    evaluator = None
    try:
        evaluator = evaluator_factory(stage, header)
        generation = None
        connection.send({'ready': True})
        while True:
            request = connection.recv()
            if request is None:
                break
            if request['generation'] != generation:
                evaluator.reset(request.get('reset_reason'))
                generation = request['generation']
            began = time.monotonic()
            data = json.loads(bytes(np.frombuffer(input_buffer, np.uint8, count=request['size'])))
            result = evaluator(data)
            payload = _encode(result, capacity)
            np.frombuffer(output_buffer, np.uint8, count=len(payload))[:] = np.frombuffer(payload, np.uint8)
            connection.send(dict(generation=generation, sequence=request['sequence'], stage=stage,
                                 size=len(payload), began_wall_s=began, completed_wall_s=time.monotonic()))
    except (EOFError, BrokenPipeError):
        pass
    except Exception as error:
        try:
            connection.send({'error': type(error).__name__+': '+str(error)})
        except (EOFError, BrokenPipeError):
            pass
    finally:
        try:
            close = getattr(evaluator, 'close', None)
            if callable(close):
                close()
        finally:
            connection.close()


class Lane:
    def __init__(self, stage, header, *, capacity, timeout, evaluator_factory):
        ctx = mp.get_context('spawn')
        self.stage, self.capacity, self.timeout = stage, capacity, timeout
        self.input = ctx.RawArray('B', capacity)
        self.output = ctx.RawArray('B', capacity)
        self.connection, child = ctx.Pipe()
        self.process = ctx.Process(target=_worker,
            args=(stage,self.input,self.output,child,header,capacity,evaluator_factory),
            name='arena-runtime-dual-'+stage, daemon=True)
        self.inflight = None
        self.closed = False
        self.cleanup = None
        self.process.start()
        child.close()
        try:
            if not self.connection.poll(15.) or self.connection.recv() != {'ready': True}:
                raise RuntimeError(stage+' startup failed')
        except BaseException:
            # EOF/수신 예외도 실패한 Lane 소유 자식과 pipe를 먼저 회수한다.
            # 상위 생성자는 이 __init__이 끝나지 않은 current를 아직 소유하지 않는다.
            self.close()
            raise

    def offer(self, data, generation, sequence, reset_reason=None):
        if self.closed or self.inflight is not None:
            raise RuntimeError('lane unavailable or occupied')
        payload = _encode(data, self.capacity)
        np.frombuffer(self.input,np.uint8,count=len(payload))[:] = np.frombuffer(payload,np.uint8)
        self.inflight = dict(generation=generation,sequence=sequence,stage=self.stage,
                             submitted_wall_s=time.monotonic(),size=len(payload),reset_reason=reset_reason)
        self.connection.send(self.inflight)

    def collect(self):
        if self.closed:
            return None
        if self.connection.poll():
            result = self.connection.recv()
            if 'error' in result:
                raise RuntimeError(self.stage+': '+result['error'])
            request = self.inflight
            if request is None or any(result.get(k)!=request[k] for k in ('generation','sequence','stage')):
                raise RuntimeError('completion identity mismatch')
            now=time.monotonic()
            if not request['submitted_wall_s']<=result['began_wall_s']<=result['completed_wall_s']<=now:
                raise RuntimeError('completion clock invalid')
            if result['completed_wall_s']-request['submitted_wall_s']>=self.timeout:
                raise RuntimeError('processing deadline expired')
            size=result['size']
            if type(size) is not int or not 0<size<=self.capacity:
                raise RuntimeError('output size invalid')
            output=json.loads(bytes(np.frombuffer(self.output,np.uint8,count=size)))
            self.inflight=None
            return dict(result,output=output,submitted_wall_s=request['submitted_wall_s'],received_wall_s=now)
        if self.inflight and time.monotonic()-self.inflight['submitted_wall_s']>=self.timeout:
            raise RuntimeError('processing deadline expired')
        if not self.process.is_alive():
            raise RuntimeError('child process exited')
        return None

    def close(self):
        if self.closed:
            return self.cleanup
        self.closed=True
        forced=False
        if self.process.is_alive():
            try:self.connection.send(None)
            except (EOFError,BrokenPipeError):pass
            self.process.join(1.)
            if self.process.is_alive():
                forced=True
                self.process.terminate()
                self.process.join(2.)
        self.connection.close()
        self.cleanup = dict(stage=self.stage, forced=forced, exitcode=self.process.exitcode,
                            alive=self.process.is_alive())
        return self.cleanup


class DualSlots:
    def __init__(self, header, *, evaluator_factory, capacity=1024*1024, work_timeout=3.,
                 lane_factory=Lane):
        if type(capacity) is not int or not 4096 <= capacity <= 16*1024*1024:
            raise ValueError('bounded capacity required')
        if type(work_timeout) not in (int, float) or not math.isfinite(work_timeout) or work_timeout <= 0:
            raise ValueError('positive processing deadline required')
        if not callable(evaluator_factory):
            raise ValueError('explicit spawn-compatible evaluator factory required')
        self.generation = self.sequence = 0
        self.saved_plan = self.plan_input = self.current_input = None
        self.fault = None
        self.closed = False
        self.cleanup = None
        self.old_generation = 0
        self.reset_reason = None
        self.plan = lane_factory('plan', header, capacity=capacity, timeout=work_timeout,
                                 evaluator_factory=evaluator_factory)
        try:
            self.current = lane_factory('current', header, capacity=capacity, timeout=work_timeout,
                                        evaluator_factory=evaluator_factory)
        except BaseException:
            self.plan.close()
            raise
        self.connection = BothConnections(self.plan.connection, self.current.connection)

    def offer_plan(self, snapshot):
        if self.closed or self.fault:
            raise RuntimeError('overlap process unavailable')
        if self.plan.inflight is not None or self.saved_plan is not None:
            return False
        owned=copy.deepcopy(snapshot)
        self.sequence+=1
        self.plan.offer(owned,self.generation,self.sequence,self.reset_reason)
        self.reset_reason=None
        self.plan_input=owned
        return True

    def discard_plan(self, token):
        if self.saved_plan is None or token!=(self.saved_plan['generation'],self.saved_plan['sequence']):
            raise ValueError('current saved plan token required')
        self.saved_plan=None

    def invalidate(self, reason=None):
        self.generation+=1
        self.saved_plan=None
        self.reset_reason='decision_source_expired' if reason=='decision_source_expired' else None

    def collect(self):
        if self.closed or self.fault:
            return []
        results=[]
        try:
            for lane in (self.current,self.plan):
                result=lane.collect()
                if result is None:continue
                owned=self.plan_input if lane is self.plan else self.current_input
                if lane is self.plan:self.plan_input=None
                else:self.current_input=None
                if result['generation']!=self.generation:
                    self.old_generation+=1
                    continue
                if owned is None:
                    raise RuntimeError('completion without owned input')
                if lane is self.plan:
                    if self.saved_plan is not None:raise RuntimeError('saved plan overflow')
                    result=dict(result,input=owned)
                    self.saved_plan=copy.deepcopy(result)
                else:
                    result=dict(result,plan=owned['plan'],input=owned['current'])
                results.append(result)
        except Exception as error:
            self.fault=type(error).__name__+': '+str(error)
        return results

    def close(self):
        """소유한 두 자식의 결과를 반환한다. 한 자식 정리 실패도 다른 자식 정리를 막지 않는다."""
        if self.closed:
            return self.cleanup
        self.closed = True
        rows = []
        for stage, lane in (('current', self.current), ('plan', self.plan)):
            try:
                rows.append(lane.close())
            except Exception as error:
                rows.append(dict(stage=stage, forced=True, exitcode=None, alive=True,
                                 error=type(error).__name__+': '+str(error)))
        self.cleanup = dict(lanes=rows, forced=any(r['forced'] for r in rows),
                            alive=any(r.get('alive', False) for r in rows),
                            exitcode=next((r['exitcode'] for r in rows if r['exitcode'] != 0), 0))
        return self.cleanup
