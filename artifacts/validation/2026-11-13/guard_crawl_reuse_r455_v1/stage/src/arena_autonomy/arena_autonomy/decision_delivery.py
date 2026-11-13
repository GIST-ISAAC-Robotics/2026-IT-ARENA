"""선택형 비동기 판단의 제출/회수 경계. 명령 발행과 독립 보호는 호출자가 맡는다."""
import copy
import math


def source_identity(snapshot):
    raw = snapshot['scan']
    if raw.get('representation') == 'native_measurements_v1':
        scan = raw['measurements']
        return (scan['source_epoch'], scan['acquired_at_s'][0])
    return (0, raw['sec']+raw['nanosec']*1e-9)


class DecisionDelivery:
    """진행1건·스캔당 최대1회. 낡은 이전 조향을 대기 슬롯에 예약하지 않는다.

    반환은 적용 후보이며 같은 콜백에서 기존 clamp와 독립 보호를 거쳐야 한다.
    이전 조향은 바퀴 실측각이 아니라 /drive의 마지막 발행 요청각이다.
    """
    def __init__(self, worker, *, scan_timeout_s=.3, command_timeout_s=.2, decision_dt_s=.1,
                 align_first_source=False):
        values = (scan_timeout_s, command_timeout_s, decision_dt_s)
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError('finite positive delivery periods required')
        if scan_timeout_s > .3 or command_timeout_s > .2 or decision_dt_s != .1:
            raise ValueError('delivery trial preserves existing ages and 100ms planning step')
        if type(align_first_source) is not bool:
            raise ValueError('explicit first-source alignment required')
        self.worker = worker
        self.align_first_source = align_first_source
        self.boundary_source = None
        self.scan_timeout, self.command_timeout, self.dt = values
        self.active = self.ready = self.last_source = None
        self.last_submission = self.last_application = -math.inf
        self.counts = dict(submitted=0, applied=0, discarded=0, invalidated=0)

    def invalidate(self):
        self.worker.invalidate()
        self.active = self.ready = self.last_source = None
        self.boundary_source = None
        self.last_submission = -math.inf
        self.counts['invalidated'] += 1

    def offer(self, snapshot):
        if self.active is not None or self.worker.inflight is not None or self.worker.pending is not None:
            return False
        identity = source_identity(snapshot)
        reference = snapshot['reference_s']
        if self.align_first_source and self.last_source is None:
            # 신호/STOP 경계 직후 남아 있던 스캔으로 100ms 제출 위상을 고정하지
            # 않는다. 그 뒤 처음 완성된 스캔에 맞추며 진행 중 결과/출력 간격은 유지한다.
            if self.boundary_source is None:
                self.boundary_source = identity
            if identity == self.boundary_source:
                return False
        if identity == self.last_source or reference-self.last_submission < self.dt-1e-9:
            return False
        if snapshot['dt_s'] != self.dt:
            raise ValueError('delivery snapshot dt mismatch')
        owned = copy.deepcopy(snapshot)
        self.worker.offer(owned)
        self.active = owned
        self.last_source, self.last_submission = identity, reference
        self.counts['submitted'] += 1
        self.worker.pump()
        return True

    def poll(self, now_s, *, previous_steering_rad, side, motion_epoch, sensor_epoch):
        result = self.worker.pump()
        if self.worker.fault:
            return dict(kind='fault', reason=self.worker.fault)
        if result is not None:
            if self.active is None or self.ready is not None:
                raise RuntimeError('delivery result without matching active request')
            self.ready = result
        if self.ready is None:
            return None
        data, result = self.active, self.ready
        reference = data['reference_s']; source_epoch, first = source_identity(data)
        reason = None
        if not math.isfinite(now_s) or now_s < reference:
            reason = 'decision_clock_boundary'
        elif motion_epoch != data['epoch'] or sensor_epoch != source_epoch:
            reason = 'decision_epoch_changed'
        elif previous_steering_rad != data['previous_steering_rad']:
            reason = 'decision_previous_request_changed'
        elif side != data['side']:
            reason = 'decision_wall_side_changed'
        elif now_s-first >= self.scan_timeout:
            reason = 'decision_source_expired'
        elif now_s-reference >= self.command_timeout:
            reason = 'decision_command_expired'
        if reason is not None:
            self.counts['discarded'] += 1
            self.invalidate()
            return dict(kind='discarded', reason=reason, input=data, result=result,
                        reference_s=reference, source_age_s=now_s-first)
        # 100ms 조향/가속 예산을 더 짧은 간격으로 반복 적용하지 않는다.
        # 0속도 후보는 감속을 늦추지 않고 호출자가 실제 경과시간으로 clamp한다.
        if result['output'][0] > 0 and now_s-self.last_application < self.dt-1e-9:
            return None
        self.active = self.ready = None
        self.counts['applied'] += 1
        self.last_application = now_s
        return dict(kind='candidate', input=data, result=result, reference_s=reference,
                    source_age_s=now_s-first, output=result['output'])
