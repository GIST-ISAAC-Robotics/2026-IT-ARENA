"""계획/현재 확인의 두 단일 슬롯 후보. ROS·발행 권한은 소유하지 않는다."""
import copy
import json
import math
import multiprocessing as mp
import signal
import time

import numpy as np

from arena_autonomy.decision_process import make_controller, _encode
from arena_autonomy.decision_delivery import source_identity


def plan_reference(snapshot):
    """계획을 실제 계산한 부모 소유 입력에서만 만든 내부 기하 참조다.

    이전 광선/운동 이력은 다시 쓰지 않는다. 현재 광선/이력은 원문 그대로
    전달하며 부모의 만료·epoch 검사와 기록에는 원 전체 입력을 보존한다.
    """
    return dict(reference_s=snapshot['reference_s'],epoch=snapshot['epoch'],
                side=snapshot['side'],previous_steering_rad=snapshot['previous_steering_rad'],
                scan=dict(representation=snapshot['scan']['representation'],
                          measurements=dict(source_epoch=snapshot['scan']['measurements']['source_epoch'])))


def evaluate_current(output, submitted, current, motion, config, *, retained=False):
    from arena_autonomy.follow_trace import restore_call, freeze
    from arena_autonomy.scene_motion import motion_scene
    from arena_autonomy.lidar_motion import MotionUnavailable
    import route_proof
    info, scan, history = restore_call(current, motion)
    try:
        scene = motion_scene(scan, history, info['reference_s'], timing_verified=info['timing_verified'],
                             epoch=info['epoch'], rear_half_angle_deg=info['rear_half_angle_deg'])
        result = route_proof.rebase(output, submitted, current, scene, history, config, require_current_region=True)
    except (MotionUnavailable, ValueError) as error:
        result = dict(speed_mps=0., steering_rad=info['previous_steering_rad'], reference_s=info['reference_s'],
                      reason='rebase_input_stop', input_error=str(error),
                      independent_guard_required=True, runtime_authorized=False)
    if result is None:
        raise ValueError('unsupported current recheck')
    return freeze(result)


def _worker(stage, input_buffer, output_buffer, connection, header, capacity, test_floor_s):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        from arena_autonomy.follow_trace import freeze, replay_call, replay_reset
        from arena_autonomy.numeric_kernels import configure
        configure(header.get('numeric_execution', 'python'))
        follow, motion = make_controller(header)
        generation = None
        connection.send({'ready': True})
        while True:
            request = connection.recv()
            if request is None:
                break
            if request['generation'] != generation:
                replay_reset(follow, {'reason': request.get('reset_reason')})
                generation = request['generation']
            began = time.monotonic()
            data = json.loads(bytes(np.frombuffer(input_buffer, np.uint8, count=request['size'])))
            if stage == 'plan':
                result = freeze(replay_call(follow, data, motion))
            else:
                result = evaluate_current(data['output'], data['submitted'], data['current'], motion, follow.config, retained=data.get('retained', False))
            if test_floor_s:
                time.sleep(max(0., test_floor_s-(time.monotonic()-began)))
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
        connection.close()


class Lane:
    def __init__(self, stage, header, *, capacity, timeout, test_floor_s):
        ctx = mp.get_context('spawn')
        self.stage, self.capacity, self.timeout = stage, capacity, timeout
        self.input = ctx.RawArray('B', capacity)
        self.output = ctx.RawArray('B', capacity)
        self.connection, child = ctx.Pipe()
        self.process = ctx.Process(target=_worker,
            args=(stage,self.input,self.output,child,header,capacity,test_floor_s),
            name='arena-overlap-'+stage, daemon=True)
        self.inflight = None
        self.closed = False
        self.process.start()
        child.close()
        if not self.connection.poll(15.) or self.connection.recv() != {'ready': True}:
            self.close()
            raise RuntimeError(stage+' startup failed')

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
            return None
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
        return dict(stage=self.stage,forced=forced,exitcode=self.process.exitcode)


class OverlapProcess:
    """각 단계 진행1개, 회수한 계획1개. 부모만 세대·발행 상태를 결정한다."""
    def __init__(self, header, *, capacity=1024*1024, work_timeout=2., test_floor_s=0., current_test_floor_s=None):
        if type(capacity) is not int or not 4096<=capacity<=16*1024*1024:
            raise ValueError('bounded capacity required')
        if (type(work_timeout) not in (int,float) or not math.isfinite(work_timeout) or work_timeout<=0
            or type(test_floor_s) not in (int,float) or not math.isfinite(test_floor_s) or not 0<=test_floor_s<work_timeout):
            raise ValueError('valid processing/test deadlines required')
        if current_test_floor_s is None: current_test_floor_s=test_floor_s
        if not math.isfinite(current_test_floor_s) or not 0<=current_test_floor_s<work_timeout:
            raise ValueError("valid current test deadline required")
        self.generation=self.sequence=0
        self.saved_plan=self.plan_input=self.current_input=None
        self.fault=None
        self.closed=False
        self.old_generation=0
        self.reset_reason=None
        self.plan=Lane('plan',header,capacity=capacity,timeout=work_timeout,test_floor_s=test_floor_s)
        try:
            self.current=Lane('current',header,capacity=capacity,timeout=work_timeout,test_floor_s=current_test_floor_s)
        except Exception:
            self.plan.close()
            raise

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

    def offer_current(self, token, snapshot):
        if self.closed or self.fault:
            raise RuntimeError('overlap process unavailable')
        if self.current.inflight is not None:
            return False
        plan=self.saved_plan
        if plan is None or token!=(plan['generation'],plan['sequence']) or token[0]!=self.generation:
            raise ValueError('unused current-generation plan token required')
        if snapshot['reference_s']<plan['input']['reference_s']:
            raise ValueError('current snapshot precedes plan')
        import candidate_policy
        if not candidate_policy.supported(plan['output']):
            raise ValueError('plan outside current recheck scope')
        owned=copy.deepcopy(snapshot)
        data=dict(output=plan['output'],submitted=plan_reference(plan['input']),current=owned)
        self.sequence+=1
        self.current.offer(data,self.generation,self.sequence)
        self.current_input=dict(plan=plan,current=owned)
        self.saved_plan=None
        return True

    def discard_plan(self, token):
        if self.saved_plan is None or token!=(self.saved_plan['generation'],self.saved_plan['sequence']):
            raise ValueError('current saved plan token required')
        self.saved_plan=None

    def invalidate(self, reason=None):
        self.generation+=1
        self.saved_plan=None
        self.reset_reason='decision_source_expired' if reason=='decision_source_expired' else None
        # 진행 중의 바이트는 결과 회수 전까지 그대로 둔다.

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
        if self.closed:return None
        self.closed=True
        return [self.current.close(),self.plan.close()]


def application_reason(result, *, generation, now_s, previous_steering_rad, side,
                       motion_epoch, sensor_epoch):
    """현재 검사 이후의 변경을 판정한다. None이어도 독립 보호와 발행 간격은 필요하다."""
    if result.get('stage')!='current' or result.get('generation')!=generation:
        return 'current_generation_changed'
    old=result['plan']['input'];current=result['input'];output=result['output']
    if output.get('reference_s')!=current['reference_s']:
        return 'current_reference_changed'
    for data in (old,current):
        epoch,first=source_identity(data)
        if not math.isfinite(now_s) or now_s<data['reference_s']:
            return 'current_clock_boundary'
        if data['epoch']!=motion_epoch or epoch!=sensor_epoch or data['side']!=side:
            return 'current_input_epoch_or_side_changed'
        if now_s-first>=.3:return 'current_source_expired'
        if now_s-data['reference_s']>=.2:return 'current_command_expired'
    # 원 계획의 이전 조향은 최신 재검사에서 대체했다. 검사 뒤 변경은 허용하지 않는다.
    if previous_steering_rad!=current['previous_steering_rad']:
        return 'current_previous_request_changed'
    return None
