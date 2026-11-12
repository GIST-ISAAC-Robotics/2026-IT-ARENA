"""저장 입력용 현재 검사 우선 후보. ROS/공통/실물에는 연결하지 않는다."""
import copy
import math
from pathlib import Path
import sys

A=Path(__file__).resolve().parent
sys.path.insert(0,str(A.parent/'retained_route_execution_r266_v1'))
from route_delivery import RouteProcess
from overlap import OverlapProcess
from arena_autonomy.decision_delivery import source_identity


def may_finish_current(old,new):
    """같은 FOLLOW 의도·비감속 새 양수 계획만 현재 검사 종료까지 보관한다.

    현재 경로의 점유 허가는 재사용하지 않는다. 실제 적용에는 원 RouteProcess의
    현재 관측/시각/기하/직전 조향 검사가 여전히 필요하다.
    """
    if old is None or new is None or old['generation']!=new['generation']:
        return False
    a,b=old['input'],new['input']
    if (a['epoch']!=b['epoch'] or source_identity(a)[0]!=source_identity(b)[0]
            or a['side']!=b['side'] or b['reference_s']<a['reference_s']):
        return False
    x,y=old['output'],new['output']
    if not all(math.isfinite(v) for v in (x[0],y[0])) or not 0<x[0]<=y[0]:
        return False
    for d in (x[2],y[2]):
        if d.get('handoff_state')!='FOLLOW' or d.get('reason')!='handoff_follow':
            return False
    fields=('mode','preferred_wall','path_wall','selector_mode','selector_choice')
    nx,ny=x[2].get('normal',{}),y[2].get('normal',{})
    if any(not isinstance(nx.get(k),str) or nx[k]!=ny.get(k) for k in fields):
        return False
    return nx['selector_mode']=='SOLO' and nx['selector_choice'] in ('baseline','parallel')


class CurrentPriorityProcess(RouteProcess):
    def collect(self):
        active=(self.current_input or {}).get('plan')
        if active is None and self.ready is not None:
            active=self.ready['plan']
        rows=OverlapProcess.collect(self)
        for row in rows:
            if row['stage']!='plan':
                continue
            if self._usable(row) and may_finish_current(active,row):
                # 새 계획 슬롯은 보존한다. 다음 현재 검사는 이 새 계획을 먼저 쓴다.
                # 단일 진행 검사/ready 이후까지 별도 양수 계획 큐를 늘리지 않는다.
                row['deferred_positive_transition']=True
            else:
                self.revision+=1
                self.ready=None
                if not self._usable(row):
                    self.retained=None
        for row in rows:
            if row['stage']!='current':
                continue
            metadata=row['plan'].get('route_check') or {}
            row['route_check']=copy.deepcopy(metadata)
            if metadata.get('revision')!=self.revision:
                row['discard_reason']='new_plan_superseded_current_check'
                continue
            row['generation']=self.generation
            if metadata.get('retained') and row['output']['speed_mps']<=0:
                row['discard_reason']='retained_route_not_proven'
                continue
            if row['output']['speed_mps']<=0:
                self.retained=None
            self.ready=copy.deepcopy(row)
        if self.fault:
            self.retained=self.ready=None
        return rows
