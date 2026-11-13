"""r407 하네스 전용: 수락 응답과 표시 binding을 연결하며 권한을 만들지 않는다."""
from copy import deepcopy


def policy_accepted(entry):
    policy = entry.get('policy') or {}
    if entry.get('stage') in ('rejected_by_adapter', 'policy_rejected') or policy.get('accepted') is False:
        reason = entry.get('adapter_reason') or policy.get('reason') or 'unspecified'
        raise RuntimeError(f"operator {entry.get('request')} rejected: {reason}")
    return policy.get('accepted') is True


def accepted_reply(rows, request, op):
    reply = next((row for row in rows if row.get('request') == request), None)
    if reply is None:
        return None
    if reply.get('op') != op:
        raise RuntimeError(f'operator {request} reply operation mismatch')
    if reply.get('accepted') is not True:
        raise RuntimeError(f"operator {request} rejected: {reply.get('reason')}")
    return reply


def stopped_binding(snapshot, stop_reply, race_id):
    """새 STOP의 정확한 표시 상태를 기다린 뒤 그 view를 복사한다. 재전송 없음."""
    if stop_reply is None:
        return None
    if (stop_reply.get('op') != 'stop' or stop_reply.get('accepted') is not True
            or stop_reply.get('latch') != 'explicit_stop'):
        raise RuntimeError('accepted explicit STOP receipt required')
    if snapshot.get('connection', {}).get('status_stale') is not False:
        return None
    binding = snapshot.get('binding') or {}
    if not race_id or binding.get('race_id') != race_id:
        return None
    for key in ('session', 'race_epoch', 'intervention'):
        if stop_reply.get(key) is None or binding.get(key) != stop_reply[key]:
            return None
    race = snapshot.get('race') or {}
    if (race.get('latch') != 'explicit_stop' or race.get('resume_start_ready') is not True
            or race.get('resume') is not None):
        return None
    return deepcopy(binding)
