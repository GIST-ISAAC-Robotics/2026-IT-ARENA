"""r336 실제 발행 확인 계약. 반환 후보만으로 경로를 보관하지 않는다.

호출자는 같은 콜백에서 poll → 기존 발행 경계 → acknowledge_publication을 수행한다.
STOP/RESET/시계 역행 때 invalidate한다. 실제 보호 통과나 MCU 적용을 뜻하지 않는다.
"""
import copy
import math
from arena_autonomy.decision_delivery import source_identity


def eligible(snapshot, *, now_s, last_reference_s, last_source, policy):
    if policy not in ('fresh', 'latest'):
        raise ValueError('explicit submission policy required')
    ref = snapshot['reference_s']
    if (not math.isfinite(now_s) or not math.isfinite(ref) or now_s < ref
            or math.isnan(last_reference_s) or ref-last_reference_s < .1-1e-9):
        return False
    raw = snapshot['scan']
    if raw.get('representation') != 'native_measurements_v1':
        raise ValueError('native scan required')
    stamps = raw['measurements']['acquired_at_s']
    if (not stamps or any(type(x) not in (int, float) or not math.isfinite(x) for x in stamps)
            or any(b <= a for a,b in zip(stamps,stamps[1:]))):
        raise ValueError('ordered finite original ray timestamps required')
    if stamps[-1] > ref or source_identity(snapshot) == last_source:
        return False
    if now_s-stamps[0] >= .3 or now_s-ref >= .2:
        return False
    return policy == 'latest' or now_s-stamps[-1] <= .025+1e-9


class RuntimeDualDelivery:
    scan_timeout = .3
    command_timeout = .2
    dt = .1

    def __init__(self, worker, clock, emit=lambda *a:None, *, complete_plan_trace=False):
        self.worker,self.clock,self.emit=worker,clock,emit
        self.complete_plan_trace = complete_plan_trace
        self.last_source=self.boundary_source=None
        self.last_submission=self.last_application=-math.inf
        self.counts=dict(submitted=0,applied=0,discarded=0,invalidated=0)
        self.pending_publication = None

    def invalidate(self):
        self.worker.invalidate()
        self.last_source=self.boundary_source=None
        self.last_submission=-math.inf
        self.counts['invalidated']+=1
        self.pending_publication = None

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
        if self.complete_plan_trace:
            # offer가 소유한 사본과 실제 lane 식별자다. 계산 완료/발행 증명은 아니다.
            request = self.worker.plan.inflight
            self.emit('runtime_plan_submitted', dict(
                **{k: request[k] for k in ('stage','generation','sequence','reset_reason','submitted_wall_s')},
                input=self.worker.plan_input))
        self.last_source,self.last_submission=source,snapshot['reference_s']
        self.counts['submitted']+=1
        return True

    def collect(self):
        for row in self.worker.collect():
            if self.complete_plan_trace and row['stage'] == 'plan':
                # 입력은 submitted에 한 번만 남긴다. 부모의 적용/폐기 여부와 별도다.
                self.emit('runtime_plan_completed', {k: row[k] for k in (
                    'stage','generation','sequence','submitted_wall_s','began_wall_s',
                    'completed_wall_s','received_wall_s','output')})
            if row.get('discard_reason'):
                self.counts['discarded']+=1
                self.emit('parallel_discard',dict(reason=row['discard_reason'],
                    stage=row['stage'],sequence=row['sequence'],generation=row['generation']))

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

    def _reason(self, data, now, steering, side, motion_epoch, sensor_epoch):
        reference = data['reference_s']; epoch, first = source_identity(data)
        if not math.isfinite(now) or now < reference:
            return 'decision_clock_boundary'
        if motion_epoch != data['epoch'] or sensor_epoch != epoch:
            return 'decision_epoch_changed'
        if steering != data['previous_steering_rad']:
            return 'decision_previous_request_changed'
        if side != data['side']:
            return 'decision_wall_side_changed'
        if now-first >= self.scan_timeout:
            return 'decision_source_expired'
        if now-reference >= self.command_timeout:
            return 'decision_command_expired'
        return None

    def offer_recheck(self, snapshot):
        return self.worker.offer_next_current(snapshot, now_s=self.clock())

    def poll(self, now_s, *, previous_steering_rad, side, motion_epoch, sensor_epoch,
             output_dt_s=None):
        if self.pending_publication is not None:
            raise RuntimeError('previous candidate requires publication acknowledgement')
        self.collect()
        w = self.worker
        if w.fault:
            return dict(kind='fault', reason=w.fault)
        plan = w.saved_plan
        if plan is not None and not w._usable(plan):
            reason = self._reason( plan['input'], now_s, previous_steering_rad,
                                                  side, motion_epoch, sensor_epoch)
            if reason is None and plan['output'][0] > 0 and now_s-self.last_application < .1-1e-9:
                return None
            w.discard_plan((plan['generation'], plan['sequence']))
            return self._item(plan, now_s, reason=reason)
        if w.ready is not None:
            row = w.ready
            w.last_application = self.last_application
            state = dict(now_s=now_s, previous_steering_rad=previous_steering_rad, side=side,
                         motion_epoch=motion_epoch, sensor_epoch=sensor_epoch)
            if output_dt_s is not None:
                # 같은 control 시작 dt를 pending/ack/commit까지 보존한다.
                state['output_dt_s'] = output_dt_s
            prepared = w.prepared(**state)
            if prepared['kind'] == 'candidate':
                row = prepared['result']
                self.pending_publication = dict(row=copy.deepcopy(row), state=state)
                return self._item(row['plan'], now_s, checked=row)
            if prepared['reason'] != 'positive_application_interval':
                return self._item(row['plan'], now_s, checked=row, reason=prepared['reason'])
        if (w.saved_plan is not None or w.retained is not None) and w.current.inflight is None and w.ready is None:
            return dict(kind='needs_recheck')
        return None

    def acknowledge_publication(self, *, published, reference_s, speed_mps, steering_rad, now_s):
        pending, self.pending_publication = self.pending_publication, None
        if pending is None:
            return dict(retained=False, reason='no_current_candidate')
        w, row = self.worker, pending['row']
        expected = row['output']
        finite = all(type(v) in (float, int) and math.isfinite(v)
                     for v in (reference_s, speed_mps, steering_rad, now_s))
        exact = (published and finite and reference_s == expected['reference_s']
                 and 0 < speed_mps <= expected['speed_mps']
                 and steering_rad == expected['steering_rad'])
        if not exact:
            w.discard_ready()
            if published:
                w.retained = None
            return dict(retained=False, reason='not_published_as_checked')
        # 마지막 요청각은 발행 직전 값으로 검사한다. 발행 결과의 새 각도로 바꿔 넣지 않는다.
        state = {**pending['state'], 'now_s': now_s}
        reason = w.application_reason(row, **state)
        if reason:
            w.discard_ready()
            return dict(retained=False, reason='post_publication_'+reason)
        w.commit_published((row['generation'], row['sequence']), **state)
        self.last_application = now_s
        return dict(retained=True, reason='published_checked_route',
                    actual_speed_mps=speed_mps, checked_speed_mps=expected['speed_mps'])
