"""r334의 ROS 부모 연결. 실제 발행 인계 전에는 양수 경로를 보관하지 않는다."""
import copy
import math
from pathlib import Path

A = Path(__file__).resolve().parent
BASE = A.parents[1]/'2026-11-12/current_region_runtime_r282_v1'
import importlib.util
spec = importlib.util.spec_from_file_location('r282_delivery_base', BASE/'delivery.py')
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)


class Delivery(base.Delivery):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.pending_publication = None

    def invalidate(self):
        super().invalidate()
        self.pending_publication = None

    def offer_recheck(self, snapshot):
        return self.worker.offer_next_current(snapshot, now_s=self.clock())

    def poll(self, now_s, *, previous_steering_rad, side, motion_epoch, sensor_epoch):
        if self.pending_publication is not None:
            raise RuntimeError('previous candidate requires publication acknowledgement')
        self.collect()
        w = self.worker
        if w.fault:
            return dict(kind='fault', reason=w.fault)
        plan = w.saved_plan
        if plan is not None and not w._usable(plan):
            reason = base.RecheckDelivery._reason(self, plan['input'], now_s, previous_steering_rad,
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
