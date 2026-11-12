"""r273 신규 계획 전용 ROS 인계 후보. 기본 런타임/실물에는 연결하지 않는다."""
import copy
import math
from multiprocessing.connection import wait
from pathlib import Path
import sys

A = Path(__file__).resolve().parent
sys.path[:0] = [str(A.parent/'retained_route_execution_r266_v1'),
                str(A.parent/'latest_scan_submission_r273_v1')]
from route_delivery import RouteProcess
from submission import eligible
from arena_autonomy.decision_delivery import source_identity
from arena_autonomy.recheck_delivery import RecheckDelivery


class BothConnections:
    def __init__(self, *connections):self.connections=connections
    def poll(self, timeout=0.):return bool(wait(self.connections, timeout))


class DelayedLane:
    """실제 계산 이후에도 등록한 sim-clock 최소 결과 나이까지 단일 슬롯을 유지한다."""
    def __init__(self, lane, clock, delay_s, wake):
        self.lane, self.clock, self.delay_s, self.wake = lane, clock, delay_s, wake
        self.held=None
        self.request=None
        self.due=None

    @property
    def inflight(self):return self.request
    def __getattr__(self, name):return getattr(self.lane,name)

    def offer(self, *args, **kwargs):
        if self.request is not None:raise RuntimeError('delayed lane occupied')
        self.due=self.clock()+self.delay_s
        self.lane.offer(*args,**kwargs)
        self.request=copy.deepcopy(self.lane.inflight)

    def collect(self):
        if self.held is None:
            self.held=self.lane.collect()
        if self.held is None:return None
        if self.clock()+1e-9<self.due:
            self.wake(self.due)
            return None
        row=self.held
        row['submitted_sim_s']=self.due-self.delay_s
        row['result_available_sim_s']=self.clock()
        row['minimum_result_age_s']=self.delay_s
        self.held=self.request=None
        self.due=None
        return row

    def close(self):
        self.held=self.request=None
        return self.lane.close()


class Worker(RouteProcess):
    def __init__(self, header, *, clock, delays, wake):
        super().__init__(header,work_timeout=3.)
        self.connection=BothConnections(self.plan.connection,self.current.connection)
        self.plan=DelayedLane(self.plan,clock,delays[0],wake)
        self.current=DelayedLane(self.current,clock,delays[1],wake)

    def close(self):
        rows=super().close()
        if rows is None:return None
        return dict(forced=any(r['forced'] for r in rows),
                    exitcode=next((r['exitcode'] for r in rows if r['exitcode']!=0),0),lanes=rows)


class Delivery:
    scan_timeout=.3
    command_timeout=.2
    dt=.1
    def __init__(self, worker, clock, emit=lambda *a:None):
        self.worker,self.clock,self.emit=worker,clock,emit
        self.last_source=self.boundary_source=None
        self.last_submission=self.last_application=-math.inf
        self.counts=dict(submitted=0,applied=0,discarded=0,invalidated=0)

    def invalidate(self):
        self.worker.invalidate()
        self.last_source=self.boundary_source=None
        self.last_submission=-math.inf
        self.counts['invalidated']+=1

    def offer(self,snapshot):
        source=source_identity(snapshot)
        if self.last_source is None:
            if self.boundary_source is None:self.boundary_source=source
            if source==self.boundary_source:return False
        if snapshot['dt_s']!=.1:raise ValueError('100ms plan step required')
        if not eligible(snapshot,now_s=self.clock(),last_reference_s=self.last_submission,
                        last_source=self.last_source,policy='latest'):
            return False
        if not self.worker.offer_plan(snapshot):return False
        self.last_source,self.last_submission=source,snapshot['reference_s']
        self.counts['submitted']+=1
        return True

    def collect(self):
        for row in self.worker.collect():
            if row.get('discard_reason'):
                self.counts['discarded']+=1
                self.emit('parallel_discard',dict(reason=row['discard_reason'],
                    stage=row['stage'],sequence=row['sequence'],generation=row['generation']))

    def offer_recheck(self,snapshot):
        if self.worker.saved_plan is None:
            raise RuntimeError('new plan required; retained fallback disabled')
        return self.worker.offer_next_current(snapshot)

    def _item(self,plan,now,*,checked=None,reason=None):
        old=plan['input']
        item=dict(kind='discarded' if reason else 'candidate',input=old,result=plan,
                  reference_s=old['reference_s'],source_age_s=now-source_identity(old)[1])
        if reason:item['reason']=reason
        else:item['output']=plan['output']
        if checked is not None:
            result=checked['output']
            item.update(recheck_input=checked['input'],recheck_result=checked,
                        command_reference_s=result['reference_s'])
            if not reason:
                detail={**plan['output'][2],'current_rebase':{k:v for k,v in result.items() if k!='path_points'}}
                if result['speed_mps']<=0:detail['reason']=result['reason']
                item['output']=(result['speed_mps'],result['steering_rad'],detail)
        self.counts['discarded' if reason else 'applied']+=1
        return item

    def poll(self,now_s,*,previous_steering_rad,side,motion_epoch,sensor_epoch):
        self.collect()
        w=self.worker
        if w.fault:return dict(kind='fault',reason=w.fault)
        # 새 계획의0/상태 전이를 먼저 처리한다. 기존 기본의 원 입력 시한을 유지한다.
        plan=w.saved_plan
        if plan is not None and not w._usable(plan):
            reason=RecheckDelivery._reason(self,plan['input'],now_s,previous_steering_rad,
                                           side,motion_epoch,sensor_epoch)
            if reason is None and plan['output'][0]>0 and now_s-self.last_application<.1-1e-9:
                return None
            w.discard_plan((plan['generation'],plan['sequence']))
            return self._item(plan,now_s,reason=reason)
        if w.ready is not None:
            row=w.ready
            w.last_application=self.last_application
            state=dict(now_s=now_s,previous_steering_rad=previous_steering_rad,
                       side=side,motion_epoch=motion_epoch,sensor_epoch=sensor_epoch)
            prepared=w.prepared(**state)
            if prepared['kind']=='candidate':
                w.discard_ready()  # 소비만 한다. 발행 경로 보관/재사용은 하지 않는다.
                return self._item(row['plan'],now_s,checked=row)
            if prepared['reason']!='positive_application_interval':
                return self._item(row['plan'],now_s,checked=row,reason=prepared['reason'])
        if w.saved_plan is not None and w.current.inflight is None and w.ready is None:
            return dict(kind='needs_recheck')
        return None
