"""계획 뒤 현재 재검사를 같은 자식에 맡기는 명시 선택 경계. 발행/보호는 ROS 소유자에 남긴다."""
import copy
import math

from arena_autonomy.decision_delivery import source_identity
from arena_autonomy.fresh_submission import FreshStartDelivery
from arena_autonomy import guard_observation_policy


class RecheckDelivery(FreshStartDelivery):
    def __init__(self, worker, **kwargs):
        super().__init__(worker, **kwargs)
        self.current = self.rechecked = None

    def invalidate(self):
        super().invalidate()
        self.current = self.rechecked = None

    def collect(self):
        """완료만 회수. 센서 불량/대기 중에도 pipe를 비워 알림의 반복 깨움을 막는다."""
        result = self.worker.pump()
        if result is None:
            return
        if self.active is None:
            raise RuntimeError('result without active plan')
        if result['stage'] == 'plan':
            if self.ready is not None or self.current is not None:
                raise RuntimeError('duplicate or misplaced plan result')
            self.ready = result
        elif result['stage'] == 'recheck':
            if self.ready is None or self.current is None or self.rechecked is not None:
                raise RuntimeError('recheck without active current snapshot')
            self.rechecked = result
        else:
            raise RuntimeError('unregistered completion stage')

    def offer_recheck(self, snapshot):
        if self.active is None or self.ready is None or self.current is not None:
            raise RuntimeError('no plan awaiting current snapshot')
        if snapshot['reference_s'] < self.active['reference_s']:
            raise ValueError('current snapshot precedes plan')
        owned = copy.deepcopy(snapshot)
        self.worker.offer_recheck(owned, self.ready)
        self.current = owned
        self.worker.pump()

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

    def _discard(self, reason, now):
        data, plan = self.active, self.ready
        current, checked = self.current, self.rechecked
        self.counts['discarded'] += 1
        self.invalidate()
        if self.retain_expired_reference and reason == 'decision_source_expired':
            self.worker.mark_source_expiry()
        return dict(kind='discarded', reason=reason, input=data, result=plan,
                    reference_s=data['reference_s'], source_age_s=now-source_identity(data)[1],
                    recheck_input=current, recheck_result=checked)

    def poll(self, now_s, *, previous_steering_rad, side, motion_epoch, sensor_epoch):
        self.collect()
        if self.worker.fault:
            return dict(kind='fault', reason=self.worker.fault)
        if self.ready is None:
            return None
        reason = self._reason(self.active, now_s, previous_steering_rad, side, motion_epoch, sensor_epoch)
        if reason is None and self.current is not None:
            reason = self._reason(self.current, now_s, previous_steering_rad, side, motion_epoch, sensor_epoch)
        if reason is not None:
            return self._discard(reason, now_s)
        output = self.ready['output']
        if self.current is None and guard_observation_policy.supported(output):
            return dict(kind='needs_recheck')
        if self.current is not None and self.rechecked is None:
            return None
        reference = self.active['reference_s']
        if self.rechecked is not None:
            current = self.rechecked['output']
            if current['reference_s'] != self.current['reference_s']:
                raise RuntimeError('recheck changed source reference')
            detail = {**output[2], 'current_rebase': {k:v for k,v in current.items() if k != 'path_points'}}
            if current['speed_mps'] <= 0:
                detail['reason'] = current['reason']
            output = (current['speed_mps'], current['steering_rad'], detail)
            reference = current['reference_s']
        if output[0] > 0 and now_s-self.last_application < self.dt-1e-9:
            return None
        result = dict(kind='candidate', input=self.active, result=self.ready,
                      reference_s=self.active['reference_s'], command_reference_s=reference,
                      source_age_s=now_s-source_identity(self.active)[1], output=output,
                      recheck_input=self.current, recheck_result=self.rechecked)
        self.active = self.ready = self.current = self.rechecked = None
        self.counts['applied'] += 1
        # last_application은 여기서 바꾸지 않는다. 실제 /drive 발행 뒤 노드가 기록한다.
        return result
