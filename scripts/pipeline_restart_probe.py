"""ROS 고장 시험 자식에서만 쓰는 명령 경로 진단. 운영 launch 미연결.

호스트의 `/drive/safe` 전달 여부와 가상 MCU의 명령 수락·고장 전환 문맥을 기록한다.
원래 메서드를 정확히 한 번 호출하고 반환/예외를 유지한다. 호스트 분류는 호출 직전
상태로 계산한 설명값이며 실제 전달 여부는 producer 순번 증가로 따로 기록한다.
수신 계약·TTL·임대·고장 고정을 바꾸지 않는다. OS/DDS/스케줄링 중 어느 단계가
지연을 만들었는지는 이 기록만으로 확정하지 않는다.
"""
from collections import deque
import json
import time

PROBE_FLAG = '_pipeline_restart_probe'
LATCHED_STATES = ('ESTOP_LATCHED', 'FAULT_LATCHED')


def default_emit(prefix):
    def emit(row):
        print(prefix + json.dumps(row, allow_nan=False, default=repr), flush=True)
    return emit


def _stamp_us(stamp):
    return stamp.sec * 1_000_000 + stamp.nanosec // 1000


def classify_host_drive(host, now_us, source_us, wall_s):
    """`ActuationHost.on_drive`/`CommandProducer.drive`의 거절 조건을 같은 순서로 이름 붙인다."""
    offer = host.offer
    if offer is None:
        return 'offer_missing'
    if wall_s - host.offer_wall >= host.wall_timeout:
        return 'offer_stale_wall'
    issued = offer.get('lease_issued_us', -1)
    if now_us - issued < 0:
        return 'offer_from_future'
    if now_us - issued >= 100_000:
        return 'offer_stale_sim'
    if not offer.get('output_ready'):
        return 'output_not_ready'
    if host.arm_barrier is None:
        return 'arm_barrier_missing'
    if source_us < host.arm_barrier:
        return 'source_before_arm_barrier'
    producer = host.producer
    if not 0 <= now_us - source_us < producer.ttl_us:
        return 'source_age_outside_ttl'
    if producer.last_source_us is not None and source_us <= producer.last_source_us:
        return 'source_not_increasing'
    if offer.get('owner') != producer.client or offer.get('state') not in ('ARMED', 'ACTIVE'):
        return 'offer_not_owner_or_armed'
    return 'forward_expected'


def install_test_host_drive_probe(host_type, emit=None, limit=4000):
    """전달 판단이 바뀔 때만 한 줄을 남긴다. 같은 판단 구간의 최대 호출 간격을 함께 남긴다."""
    original_drive, original_rx = host_type.on_drive, host_type.on_rx
    if getattr(original_drive, PROBE_FLAG, False) or getattr(original_rx, PROBE_FLAG, False):
        raise RuntimeError('test host drive probe already installed')
    emit = emit or default_emit('HOST_DRIVE_DECISION=')
    shared = dict(lines=0, truncated=False)

    def state(host):
        value = host.__dict__.get(PROBE_FLAG)
        if value is None:
            value = host.__dict__[PROBE_FLAG] = dict(run=None, offer_receipt=None, offers=0)
        return value

    def write(row):
        if shared['truncated']:
            return
        if shared['lines'] >= limit:
            shared['truncated'] = True
            row = dict(type='test_host_drive_decision_truncated', limit=limit)
        shared['lines'] += 1
        emit(row)

    def traced_rx(host, message):
        before = host.offer
        result = original_rx(host, message)
        try:
            if host.offer is not before:
                probe = state(host)
                probe['offers'] += 1
                probe['offer_receipt'] = dict(host_now_us=host.now_us(), wall_s=time.monotonic(),
                    lease=host.offer.get('lease'), lease_issued_us=host.offer.get('lease_issued_us'),
                    generation=host.offer.get('generation'), state=host.offer.get('state'),
                    output_ready=host.offer.get('output_ready'))
        except Exception:
            pass
        return result

    def traced_drive(host, message):
        context = None
        try:
            wall = time.monotonic()
            now, source = host.now_us(), _stamp_us(message.header.stamp)
            producer = host.producer
            # producer는 원래 호출에서 바뀌므로 호출 전 값은 스칼라로 복사한다.
            context = dict(now_us=now, wall_s=wall, source_us=source, producer=producer,
                           sequence=producer.sequence, last_source_us_before=producer.last_source_us,
                           label=classify_host_drive(host, now, source, wall))
        except Exception:
            context = None
        result = original_drive(host, message)
        if context is None:
            return result
        try:
            forwarded = host.producer is context['producer'] and host.producer.sequence == context['sequence'] + 1
            key = (context['label'], forwarded)
            probe = state(host)
            run = probe['run']
            if run is not None and run['key'] == key:
                run['count'] += 1
                run['max_call_gap_us'] = max(run['max_call_gap_us'], context['now_us'] - run['last_now_us'])
                run['max_call_gap_wall_s'] = max(run['max_call_gap_wall_s'], context['wall_s'] - run['last_wall_s'])
                run['last_now_us'], run['last_wall_s'] = context['now_us'], context['wall_s']
                return result
            # offer·arm_barrier는 on_drive가 바꾸지 않으므로 호출 뒤 읽어도 호출 전 값과 같다.
            offer = host.offer or {}
            write(dict(type='test_host_drive_decision', scope='validator_child_only',
                label=context['label'], forwarded=forwarded, now_us=context['now_us'], wall_s=context['wall_s'],
                source_us=context['source_us'], speed_mps=float(message.drive.speed),
                offer=dict(lease=offer.get('lease'), lease_issued_us=offer.get('lease_issued_us'),
                           generation=offer.get('generation'), state=offer.get('state'),
                           output_ready=offer.get('output_ready')),
                offer_receipt=probe['offer_receipt'], offers_received=probe['offers'],
                arm_barrier_us=host.arm_barrier, last_source_us_before=context['last_source_us_before'],
                last_source_us_after=getattr(context['producer'], 'last_source_us', None),
                previous_run=None if run is None else {k: v for k, v in run.items() if k != 'key'},
                interpretation='label is pre-call explanation; forwarded is producer sequence change'))
            probe['run'] = dict(key=key, label=context['label'], forwarded=forwarded, count=1,
                first_now_us=context['now_us'], first_wall_s=context['wall_s'],
                last_now_us=context['now_us'], last_wall_s=context['wall_s'],
                max_call_gap_us=0, max_call_gap_wall_s=0.)
        except Exception:
            pass
        return result

    for function in (traced_rx, traced_drive):
        setattr(function, PROBE_FLAG, True)
    host_type.on_rx, host_type.on_drive = traced_rx, traced_drive

    def restore():
        if host_type.on_rx is not traced_rx or host_type.on_drive is not traced_drive:
            raise RuntimeError('host implementation changed after probe installation')
        host_type.on_rx, host_type.on_drive = original_rx, original_drive
    return restore


def install_test_publish_timing_probe(publisher_type, emit=None, threshold_s=.015, limit=400):
    """발행 호출의 벽시계 소요가 한도 이상일 때만 남긴다. 발행 자체·예외는 그대로 둔다."""
    original = publisher_type.publish
    if getattr(original, PROBE_FLAG, False):
        raise RuntimeError('test publish timing probe already installed')
    emit = emit or default_emit('SLOW_PUBLISH=')
    shared = dict(lines=0, truncated=False)

    def traced(publisher, msg):
        began = time.monotonic()
        try:
            return original(publisher, msg)
        finally:
            elapsed = time.monotonic() - began
            if elapsed >= threshold_s and not shared['truncated']:
                try:
                    if shared['lines'] >= limit:
                        shared['truncated'] = True
                        emit(dict(type='test_slow_publish_truncated', limit=limit))
                    else:
                        shared['lines'] += 1
                        try:
                            topic = publisher.topic_name
                        except Exception:
                            topic = None
                        emit(dict(type='test_slow_publish', scope='validator_child_only', topic=topic,
                                  start_wall_s=began, duration_s=elapsed, threshold_s=threshold_s))
                except Exception:
                    pass

    setattr(traced, PROBE_FLAG, True)
    publisher_type.publish = traced

    def restore():
        if publisher_type.publish is not traced:
            raise RuntimeError('publisher implementation changed after probe installation')
        publisher_type.publish = original
    return restore


def install_test_mcu_command_probe(receiver_type, emit=None, history=24):
    """새 FAULT_LATCHED 전환마다 직전 수락 명령·수신 결과·tick을 한 번 남긴다."""
    originals = dict(receive=receiver_type.receive, tick=receiver_type.tick, _fault=receiver_type._fault)
    if any(getattr(function, PROBE_FLAG, False) for function in originals.values()):
        raise RuntimeError('test MCU command probe already installed')
    emit = emit or default_emit('MCU_FAULT_CONTEXT=')

    def state(receiver):
        value = receiver.__dict__.get(PROBE_FLAG)
        if value is None:
            value = receiver.__dict__[PROBE_FLAG] = dict(receives=deque(maxlen=history),
                ticks=deque(maxlen=history), last_accepted_drive=None, receive_count=0)
        return value

    def traced_receive(receiver, message, now_us):
        context = None
        try:
            fields = message if type(message) is dict else {}
            lease = fields.get('lease')
            context = dict(wall_s=time.monotonic(), now_us=now_us, kind=fields.get('kind'), seq=fields.get('seq'),
                lease=lease, message_generation=fields.get('generation'),
                lease_window_before=receiver._leases.get(lease) if type(lease) is int else None,
                speed_mps=fields.get('speed_mps'), source_age_us=fields.get('source_age_us'),
                ttl_us=fields.get('ttl_us'), state_before=receiver.state.value,
                generation_before=receiver.generation, deadline_before_us=receiver.deadline_us)
        except Exception:
            context = None
        result = originals['receive'](receiver, message, now_us)
        if context is not None:
            try:
                context.update(accepted=result[0], result_reason=result[1], state_after=receiver.state.value,
                               generation_after=receiver.generation, deadline_after_us=receiver.deadline_us)
                probe = state(receiver)
                probe['receive_count'] += 1
                probe['receives'].append(context)
                if context['kind'] == 'DRIVE' and result[0] is True:
                    probe['last_accepted_drive'] = context
            except Exception:
                pass
        return result

    def traced_tick(receiver, now_us):
        wall = time.monotonic()
        result = originals['tick'](receiver, now_us)
        try:
            feedback = receiver.feedback
            state(receiver)['ticks'].append(dict(wall_s=wall, now_us=now_us, state=receiver.state.value,
                deadline_us=receiver.deadline_us, last_control_us=receiver.last_control_us,
                feedback_seq=None if feedback is None else feedback[0],
                feedback_capture_us=None if feedback is None else feedback[1],
                feedback_valid=receiver.feedback_valid))
        except Exception:
            pass
        return result

    def traced_fault(receiver, reason):
        try:
            before = dict(state=receiver.state.value, reason=receiver.reason, generation=receiver.generation,
                deadline_us=receiver.deadline_us, last_control_us=receiver.last_control_us,
                now_us=receiver.now_us, feedback=None if receiver.feedback is None else list(receiver.feedback),
                feedback_valid=receiver.feedback_valid, owner_present=receiver.owner is not None,
                accepted_seq=receiver.sequence, leases=len(receiver._leases))
        except Exception:
            before = None
        result = originals['_fault'](receiver, reason)
        try:
            if (before is not None and before['state'] not in LATCHED_STATES
                    and receiver.state.value == 'FAULT_LATCHED'):
                probe = state(receiver)
                emit(dict(type='test_mcu_fault_context', scope='validator_child_only', reason=reason,
                    wall_s=time.monotonic(), before=before, generation_after=receiver.generation,
                    last_accepted_drive=probe['last_accepted_drive'], receive_count=probe['receive_count'],
                    recent_receives=list(probe['receives']), recent_ticks=list(probe['ticks']),
                    interpretation='actual first transition context; delivery/scheduling cause not measured'))
        except Exception:
            pass
        return result

    traced = dict(receive=traced_receive, tick=traced_tick, _fault=traced_fault)
    for name, function in traced.items():
        setattr(function, PROBE_FLAG, True)
        setattr(receiver_type, name, function)

    def restore():
        if any(getattr(receiver_type, name) is not function for name, function in traced.items()):
            raise RuntimeError('receiver implementation changed after probe installation')
        for name, function in originals.items():
            setattr(receiver_type, name, function)
    return restore
