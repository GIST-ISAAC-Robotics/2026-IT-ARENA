"""기존 시험 계약의 선택 메서드만 추출한다. 정책·지연 주입·노드 선택은 제외한다."""
import ast
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
OUT = ROOT/'src/arena_autonomy/arena_autonomy'
DAY = A.parent
OLD = DAY.parent/'2026-11-12'


def source(path):
    return path.read_text(encoding='utf-8')


def item(path, name, method=None):
    text = source(path)
    tree = ast.parse(text)
    node = next(n for n in tree.body if getattr(n, 'name', None) == name)
    if method:
        node = next(n for n in node.body if getattr(n, 'name', None) == method)
    return '\n'.join(text.splitlines()[node.lineno-1:node.end_lineno])+'\n'


overlap = OLD/'retained_route_execution_r266_v1/overlap.py'
route = OLD/'retained_route_execution_r266_v1/route_delivery.py'
priority = OLD/'current_priority_r286_v1/priority.py'
deceleration = DAY/'current_deceleration_r334_v1/deceleration.py'
delivery = OLD/'current_region_runtime_r282_v1/delivery.py'
ros_delivery = DAY/'current_ros_bridge_r336_v1/ros_delivery.py'

worker = '''def _worker(stage, input_buffer, output_buffer, connection, header, capacity, evaluator_factory):
    # factory와 그 결과는 명시 선택된 알고리즘 경계다. 이 모듈은 정책을 고르지 않는다.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
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
        connection.close()
'''
lane = item(overlap, 'Lane').replace('timeout, test_floor_s', 'timeout, evaluator_factory')
lane = lane.replace('header,capacity,test_floor_s)', 'header,capacity,evaluator_factory)')
lane = lane.replace("name='arena-overlap-'+stage", "name='arena-runtime-dual-'+stage")
lane = lane.replace('self.closed = False', 'self.closed = False\n        self.cleanup = None')
lane = lane.replace('if self.closed:\n            return None\n        self.closed=True',
                    'if self.closed:\n            return self.cleanup\n        self.closed=True')
lane = lane.replace('return dict(stage=self.stage,forced=forced,exitcode=self.process.exitcode)',
                    'self.cleanup = dict(stage=self.stage, forced=forced, exitcode=self.process.exitcode,\n'
                    '                            alive=self.process.is_alive())\n        return self.cleanup')
slots_init = '''    def __init__(self, header, *, evaluator_factory, capacity=1024*1024, work_timeout=3.,
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
'''
slots_close = '''    def close(self):
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
'''
transport = '''"""명시 선택형 두 계산 슬롯. 정책 factory를 받아 원 바이트·세대·완료를 관리한다.

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


'''+item(overlap, 'plan_reference')+'\n\n'+worker+'\n\n'+lane+'\n\nclass DualSlots:\n'+slots_init
for name in ('offer_plan', 'discard_plan', 'invalidate', 'collect'):
    transport += '\n'+item(overlap, 'OverlapProcess', name)
transport += '\n'+slots_close

state = '''"""r334의 두 슬롯 경로 인계 계약. 현재 검사 정책은 외부에서 명시한다.

원천300ms·명령200ms·경로500ms·양수발행100ms를 유지한다. 새로운 시각,
점유 허가, 독립 보호 통과를 만들지 않는다. 노드/기본값에는 연결하지 않는다.
"""
import copy
import math
from arena_autonomy.decision_delivery import source_identity
from arena_autonomy.runtime_dual_process import DualSlots, plan_reference


'''+item(priority, 'may_finish_current')+'\n\n'+item(deceleration, 'same_positive_intent')
state += '''

class RuntimeDualProcess(DualSlots):
    current_schedule_budget_s = .100
    retained_schedule_budget_s = .100

    def __init__(self, header, *, supported, **kwargs):
        if not callable(supported):
            raise ValueError('explicit current-check support predicate required')
        self.supported = supported
        super().__init__(header, **kwargs)
        self.retained = self.ready = self.last_checked_source = None
        self.revision = 0
        self.last_application = -math.inf
        self.geometry_age_limit = .5
        self.pending_speed_cap = None
'''
state += '\n'+item(route, 'RouteProcess', 'invalidate').replace(
    'self.revision += 1', 'self.revision += 1\n        self.pending_speed_cap = None')
state += '\n'+item(route, 'RouteProcess', '_usable').replace('candidate_policy.supported', 'self.supported')
state += '\n'+item(deceleration, 'CurrentDecelerationProcess', '_cap_ready')
state += '\n'+item(deceleration, 'CurrentDecelerationProcess', 'collect').replace('OverlapProcess.collect', 'DualSlots.collect')
offer = item(route, 'RouteProcess', 'offer_next_current')
offer = offer.replace('def offer_next_current(self, snapshot):', 'def offer_next_current(self, snapshot, *, now_s):')
guards = '''        ref = snapshot['reference_s']
        first = source_identity(snapshot)[1]
        if not all(math.isfinite(v) for v in (now_s, ref, first)) or now_s < ref:
            return False
        if now_s-ref+self.current_schedule_budget_s >= .2:
            return False
        if now_s-first+self.current_schedule_budget_s >= .3:
            return False
        if ref <= self.last_application:
            return False
        if self.saved_plan is None and self.retained is not None:
            source = self.retained['output'][2]['normal'].get('path_source_s')
            if not isinstance(source, (int, float)) or not math.isfinite(source):
                return False
            if ref-source+self.retained_schedule_budget_s >= self.geometry_age_limit:
                return False
'''
offer = offer.replace('        if self.closed or self.fault:', guards+'        if self.closed or self.fault:')
offer = offer.replace('        return True', '        self.pending_speed_cap = None\n        return True')
state += '\n'+offer
for name in ('offer_current', 'application_reason', 'prepared', 'commit_published', 'discard_ready'):
    state += '\n'+item(route, 'RouteProcess', name)

delivery_text = '''"""r336 실제 발행 확인 계약. 반환 후보만으로 경로를 보관하지 않는다.

호출자는 같은 콜백에서 poll → 기존 발행 경계 → acknowledge_publication을 수행한다.
STOP/RESET/시계 역행 때 invalidate한다. 실제 보호 통과나 MCU 적용을 뜻하지 않는다.
"""
import copy
import math
from arena_autonomy.decision_delivery import source_identity


'''+item(OLD/'latest_scan_submission_r273_v1/submission.py', 'eligible')+'\n\nclass RuntimeDualDelivery:\n'
delivery_text += '''    scan_timeout = .3
    command_timeout = .2
    dt = .1
'''
delivery_text += '\n'+item(delivery, 'Delivery', '__init__').replace(
    "self.counts=dict(submitted=0,applied=0,discarded=0,invalidated=0)",
    "self.counts=dict(submitted=0,applied=0,discarded=0,invalidated=0)\n        self.pending_publication = None")
delivery_text += '\n'+item(delivery, 'Delivery', 'invalidate').replace(
    "self.counts['invalidated']+=1", "self.counts['invalidated']+=1\n        self.pending_publication = None")
for name in ('offer', 'collect', '_item'):
    delivery_text += '\n'+item(delivery, 'Delivery', name)
delivery_text += '\n'+item(ROOT/'src/arena_autonomy/arena_autonomy/recheck_delivery.py', 'RecheckDelivery', '_reason')
for name in ('offer_recheck', 'poll', 'acknowledge_publication'):
    method = item(ros_delivery, 'Delivery', name).replace('base.RecheckDelivery._reason(self,', 'self._reason(')
    delivery_text += '\n'+method

for name, text in [('process', transport), ('state', state), ('delivery', delivery_text)]:
    ast.parse(text)
    (OUT/f'runtime_dual_{name}.py').write_text(text, encoding='utf-8', newline='\n')
print('Extracted 3 runtime modules; no algorithm policy or node activated.')

# 기존 경계 반례를 새 import 경계로 그대로 옮긴다. 실장/ROS 실행은 만들지 않는다.
tests = source(DAY/'current_deceleration_r334_v1/test_deceleration.py')
tests = tests.replace('from deceleration import CurrentDecelerationProcess, same_positive_intent, OverlapProcess',
    'from arena_autonomy.runtime_dual_state import RuntimeDualProcess as CurrentDecelerationProcess, same_positive_intent\n'
    'from arena_autonomy.runtime_dual_process import DualSlots as OverlapProcess\n'
    'from arena_autonomy.runtime_dual_delivery import RuntimeDualDelivery as Delivery\n'
    'from types import SimpleNamespace')
tests = tests.replace('w.revision, w.generation = 8, 0',
    'w.revision, w.generation = 8, 0\n    w.supported = lambda output: output[0] > 0')
tests = tests[:tests.index('def test_generation_invalidation_clears_deceleration_cap():')]
tests += '''def test_generation_invalidation_clears_deceleration_cap():
    old, new, current = records(); w = worker(old)
    collect(w, [new, current])
    w.invalidate('explicit_stop')
    assert w.generation == 1 and w.revision == 9
    assert w.pending_speed_cap is w.ready is w.retained is w.saved_plan is None
    assert w.reset_reason is None


'''
ack_tests = source(DAY/'current_ros_bridge_r336_v1/test_delivery.py')
ack_tests = ack_tests[ack_tests.index('def setup_case():'):]
ack_tests = ack_tests.replace('fixture.records()', 'records()').replace('fixture.worker(old)', 'worker(old)')
tests += ack_tests
ast.parse(tests)
(ROOT/'tests/test_runtime_dual_contract.py').write_text(tests, encoding='utf-8', newline='\n')
