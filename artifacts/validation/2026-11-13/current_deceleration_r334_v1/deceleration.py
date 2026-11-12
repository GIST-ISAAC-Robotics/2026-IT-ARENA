"""같은 양수 주행 의도의 감속 상한만 적용한다. 저장 재생 전용, ROS 미연결.

기존 현재 검사가 더 큰 목표속도로 확인한 같은 조향을 유지한다. 새 계획의
작은 목표속도만 상한으로 적용하고 새 기하/점유 허가로 바꾸지 않는다.
0명령·상태/벽/epoch 전이는 기존대로 즉시 무효화한다.
"""
import copy
import math
from pathlib import Path
import sys

A = Path(__file__).resolve().parent
sys.path.insert(0, str(A.parent/'current_deadline_r325_v1'))
from deadline import CurrentDeadlineProcess
from priority import may_finish_current, OverlapProcess


def same_positive_intent(old, new):
    if old is None or new is None:
        return False
    x, y = old['output'][0], new['output'][0]
    if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or v <= 0
           for v in (x, y)):
        return False
    # 속도 비교만 분리한다. 원 입력/경로를 바꾸거나 이 사본을 실행하지 않는다.
    probe = {**old, 'output': [min(x, y), *old['output'][1:]]}
    return may_finish_current(probe, new)


class CurrentDecelerationProcess(CurrentDeadlineProcess):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.pending_speed_cap = None

    def invalidate(self, reason=None):
        super().invalidate(reason)
        self.pending_speed_cap = None

    def offer_next_current(self, *args, **kwargs):
        accepted = super().offer_next_current(*args, **kwargs)
        if accepted:
            self.pending_speed_cap = None
        return accepted

    def _cap_ready(self):
        cap = self.pending_speed_cap
        if cap is None or self.ready is None or self.ready['output']['speed_mps'] <= 0:
            return
        out = self.ready['output']
        checked_speed = (out.get('new_plan_deceleration_cap') or {}).get('checked_target_mps', out['speed_mps'])
        out['speed_mps'] = min(out['speed_mps'], cap['speed_mps'])
        out['new_plan_deceleration_cap'] = dict(
            **cap, checked_target_mps=checked_speed,
            applied=out['speed_mps'] < checked_speed,
            steering_changed=False, current_check_reused_without_timestamp_renewal=True)

    def collect(self):
        active = (self.current_input or {}).get('plan')
        if active is None and self.ready is not None:
            active = self.ready['plan']
        rows = OverlapProcess.collect(self)
        for row in rows:
            if row['stage'] != 'plan':
                continue
            if self._usable(row) and same_positive_intent(active, row):
                row['deferred_positive_transition'] = True
                speed = row['output'][0]
                old_cap = self.pending_speed_cap
                if speed < active['output'][0] and (old_cap is None or speed < old_cap['speed_mps']):
                    self.pending_speed_cap = dict(speed_mps=speed, generation=row['generation'],
                        sequence=row['sequence'], reference_s=row['input']['reference_s'])
            else:
                self.revision += 1
                self.ready = None
                self.pending_speed_cap = None
                if not self._usable(row):
                    self.retained = None
        for row in rows:
            if row['stage'] != 'current':
                continue
            metadata = row['plan'].get('route_check') or {}
            row['route_check'] = copy.deepcopy(metadata)
            if metadata.get('revision') != self.revision:
                row['discard_reason'] = 'new_plan_superseded_current_check'
                continue
            row['generation'] = self.generation
            if metadata.get('retained') and row['output']['speed_mps'] <= 0:
                row['discard_reason'] = 'retained_route_not_proven'
                continue
            if row['output']['speed_mps'] <= 0:
                self.retained = None
            self.ready = copy.deepcopy(row)
        self._cap_ready()
        if self.fault:
            self.retained = self.ready = self.pending_speed_cap = None
        return rows
