"""ROS를 import하지 않는 bounded scalar 관측기. 원 구현을 한 번씩 호출한다."""
import json
import math
import time
from pathlib import Path


def stamp(message):
    header = getattr(message, 'header', None)
    value = getattr(header, 'stamp', None)
    return None if value is None else value.sec+value.nanosec*1e-9


def stamp_ns(message):
    header = getattr(message, 'header', None)
    value = getattr(header, 'stamp', None)
    return None if value is None else value.sec*1000000000+value.nanosec


def token(row):
    if not isinstance(row, dict):
        return None
    return ':'.join(str(row.get(k)) for k in ('stage', 'generation', 'sequence'))


class ScalarProbe:
    def __init__(self, role, capacity=30000):
        self.role, self.capacity = role, capacity
        self.rows, self.dropped, self.errors = [], 0, 0

    def record(self, owner, event, **fields):
        try:
            if len(self.rows) >= self.capacity:
                self.dropped += 1
                return
            row = dict(event=event, monotonic_ns=time.monotonic_ns(),
                       ros_s=owner.get_clock().now().nanoseconds*1e-9,
                       control_id=getattr(owner, '_receipt_control_id', 0))
            for key, value in fields.items():
                if value is None or type(value) in (str, bool, int):
                    row[key] = value
                elif isinstance(value, float):
                    row[key] = value if math.isfinite(value) else str(value)
                else:
                    raise TypeError('scalar receipt required: '+key)
            self.rows.append(row)
        except Exception:
            self.errors += 1  # 관측 실패를 제어 실패/추가 호출로 바꾸지 않는다.

    def write(self, root):
        path = Path(root)/('scalar_'+self.role+'.jsonl')
        with path.open('x', encoding='utf8') as stream:
            for row in self.rows:
                stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
        return dict(rows=len(self.rows), dropped=self.dropped, errors=self.errors)


class ControlReceipt:
    def control(self):
        self._receipt_control_id = getattr(self, '_receipt_control_id', 0)+1
        probe = self._receipt
        probe.record(self, 'control_begin')
        try:
            # 실제 G1/guard만 drain한다. 이 wrapper는 입력을 소비하지 않는다.
            return super().control()
        finally:
            drain = getattr(self.motion, 'last_drain', {})
            probe.record(self, 'control_end', state=getattr(self, 'last_status', None),
                reference_s=getattr(self, 'last_control_time', getattr(self, 'last_clock', None)),
                drain_count=drain.get('total'), drain_ms=drain.get('duration_ms'),
                drain_bounded=drain.get('bounded_stop'),
                command_source_s=stamp(getattr(self, 'command', None)),
                last_application_s=getattr(getattr(self, 'decision_delivery', None), 'last_application', None))


class GuardReceipt:
    def on_command(self, message):
        source = stamp(message)
        self._receipt.record(self, 'guard_receive_begin', original_command_source_s=source,
                             original_command_source_ns=stamp_ns(message))
        result = super().on_command(message)
        self._receipt.record(self, 'guard_receive_end', original_command_source_s=source,
                             original_command_source_ns=stamp_ns(message),
                             stored_command_source_s=stamp(self.command),
                             stored_command_source_ns=stamp_ns(self.command))
        return result


class ObservedPublisher:
    def __init__(self, publisher, owner, role):
        self.publisher, self.owner, self.role = publisher, owner, role

    def __getattr__(self, key):
        return getattr(self.publisher, key)

    def publish(self, message):
        owner = self.owner
        pending = getattr(getattr(owner, 'decision_delivery', None), 'pending_publication', None)
        fields = dict(token=token((pending or {}).get('row')), header_source_s=stamp(message),
                      header_source_ns=stamp_ns(message),
                      speed_mps=float(message.drive.speed), steering_rad=float(message.drive.steering_angle))
        if self.role == 'guard':
            fields.update(original_command_source_s=stamp(owner.command),
                          guard_reference_s=owner.last_clock)
        owner._receipt.record(owner, self.role+'_publish_begin', **fields)
        try:
            result = self.publisher.publish(message)
        except BaseException:
            owner._receipt.record(owner, self.role+'_publish_end', success=False, **fields)
            raise
        owner._receipt.record(owner, self.role+'_publish_end', success=True, **fields)
        return result


def hook_controller(owner):
    delivery, probe = owner.decision_delivery, owner._receipt
    worker = delivery.worker
    collect, prepared, ack = worker.collect, worker.prepared, delivery.acknowledge_publication

    def observed_collect():
        previous = token(worker.ready)
        rows = collect()
        for row in rows:
            current = row.get('input') or {}
            output = row.get('output') or {}
            fields = {k:row.get(k) for k in ('stage','generation','sequence','submitted_wall_s',
                       'began_wall_s','completed_wall_s','received_wall_s','discard_reason')}
            fields.update(token=token(row), reference_s=current.get('reference_s'))
            if isinstance(output, dict):
                fields.update(checked_reference_s=output.get('reference_s'),
                              checked_speed_mps=output.get('speed_mps'),
                              checked_steering_rad=output.get('steering_rad'))
            probe.record(owner, 'complete', **fields)
        if token(worker.ready) != previous:
            probe.record(owner, 'ready_transition', previous_token=previous,
                         ready_token=token(worker.ready), revision=worker.revision)
        return rows

    def observed_prepared(**state):
        candidate = token(worker.ready)
        result = prepared(**state)
        probe.record(owner, 'permit', token=candidate,
            kind=None if result is None else result.get('kind'),
            reason=None if result is None else result.get('reason'),
            evaluation_s=state.get('now_s'), output_dt_s=state.get('output_dt_s'),
            previous_steering_rad=state.get('previous_steering_rad'),
            last_application_s=worker.last_application, ready_after=token(worker.ready))
        return result

    def observed_ack(**state):
        candidate = token((delivery.pending_publication or {}).get('row'))
        result = ack(**state)
        probe.record(owner, 'ack', token=candidate, retained=result.get('retained'),
                     reason=result.get('reason'), reference_s=state.get('reference_s'),
                     ready_after=token(worker.ready), last_application_s=delivery.last_application)
        return result

    worker.collect, worker.prepared, delivery.acknowledge_publication = observed_collect, observed_prepared, observed_ack

