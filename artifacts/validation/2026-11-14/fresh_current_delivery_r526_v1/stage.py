"""r526: 현재 재검사만 실제 출력 조향 예산으로 조기 전달하는 stage.

원 명령/점유/proof/source 시각은 불변이다. ROS 연결은 없다.
output_dt_s는 control 시작 때 계산한 기존 decision_output_dt_s다.
선택형 runtime의 기존 50 Hz control 주기(.02 s)보다 일찍 반복 발행하지 않는다.
"""
import copy
import math
from arena_autonomy.runtime_dual_state import RuntimeDualProcess
from arena_autonomy.runtime_dual_delivery import RuntimeDualDelivery

MIN_APPLICATION_INTERVAL_S = .02


def checked_angle_fits_actual_interval(output, previous, output_dt_s, elapsed_s):
    """후속 WallFollow.control의 같은 조향 clamp가 원 출력을 바꾸지 않는가."""
    values = (previous, output_dt_s, elapsed_s, output.get('steering_rad'))
    if not all(type(v) in (float, int) and math.isfinite(v) for v in values):
        return False
    if elapsed_s < MIN_APPLICATION_INTERVAL_S-1e-9 or not 0 < output_dt_s <= min(.1, elapsed_s):
        return False
    return previous-3*output_dt_s <= output['steering_rad'] <= previous+3*output_dt_s


class FreshCurrentProcess(RuntimeDualProcess):
    def application_reason(self, row, *, output_dt_s=None, **state):
        reason = super().application_reason(row, **state)
        if reason != 'positive_application_interval':
            return reason
        # 내부 소유 ready의 원시 입력·epoch·revision·TTL·경로/proof가 모두
        # 승인된 뒤 마지막 간격 거절만 대체한다. 외부 사본을 허가로 쓰지 않는다.
        owned = self.ready
        if owned['input']['reference_s'] <= self.last_application:
            return reason
        if checked_angle_fits_actual_interval(owned['output'], state['previous_steering_rad'],
                output_dt_s, state['now_s']-self.last_application):
            return None
        return reason


class FreshCurrentDelivery(RuntimeDualDelivery):
    def poll(self, now_s, *, previous_steering_rad, side, motion_epoch, sensor_epoch,
             output_dt_s=None):
        # 원 poll의 current state에 output_dt_s만 추가한다. nonretained는 불변.
        if self.pending_publication is not None:
            raise RuntimeError('previous candidate requires publication acknowledgement')
        self.collect()
        w = self.worker
        if w.fault:
            return dict(kind='fault', reason=w.fault)
        plan = w.saved_plan
        if plan is not None and not w._usable(plan):
            reason = self._reason(plan['input'], now_s, previous_steering_rad,
                                  side, motion_epoch, sensor_epoch)
            if reason is None and plan['output'][0] > 0 and now_s-self.last_application < .1-1e-9:
                return None
            w.discard_plan((plan['generation'], plan['sequence']))
            return self._item(plan, now_s, reason=reason)
        if w.ready is not None:
            row = w.ready
            w.last_application = self.last_application
            state = dict(now_s=now_s, previous_steering_rad=previous_steering_rad,
                side=side, motion_epoch=motion_epoch, sensor_epoch=sensor_epoch,
                output_dt_s=output_dt_s)
            prepared = w.prepared(**state)
            if prepared['kind'] == 'candidate':
                row = prepared['result']
                # 같은 control 시작 시각의 dt가 ack/commit까지 전달된다.
                self.pending_publication = dict(row=copy.deepcopy(row), state=state)
                return self._item(row['plan'], now_s, checked=row)
            if prepared['reason'] != 'positive_application_interval':
                return self._item(row['plan'], now_s, checked=row, reason=prepared['reason'])
        if (w.saved_plan is not None or w.retained is not None) and w.current.inflight is None and w.ready is None:
            return dict(kind='needs_recheck')
        return None
