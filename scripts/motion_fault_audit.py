"""시험 중계기의 실제 발행/누락 기록을 설정·시간·수치와 대조한다."""
import json
import math
from pathlib import Path

from arena_vehicle_interface.motion_impairment import MotionFaultConfig


def audit(path, requested):
    config = MotionFaultConfig.from_dict(requested)
    if not Path(path).exists():
        return {'verified': False, 'error': 'missing relay log'}
    rows = [json.loads(line) for line in Path(path).read_text().splitlines()]
    checks = [bool(rows) and rows[0].get('config') == config.as_dict(),
              bool(rows) and rows[-1].get('event') == 'close',
              not any(r['event'] == 'clock_reset' for r in rows)]
    result = {}
    for kind in ('imu', 'wheels'):
        fault = getattr(config, kind)
        events = [r for r in rows if r.get('stream') == kind]
        published = [r for r in events if r['event'] == 'publish']
        dropped = [r for r in events if r['event'] == 'drop']
        checks.append(len(published) >= 30)
        for r in events:
            active = fault.active(r['source_stamp_s'])
            checks.append(r['fault_active'] == active)
            if r['event'] == 'drop':
                checks.append(fault.drop and active)
                continue
            checks.extend([not (fault.drop and active),
                           math.isclose(r['claimed_stamp_s'], fault.claimed_stamp(r['source_stamp_s']), abs_tol=1e-8),
                           r['released_s'] + 1e-8 >= r['due_s'],
                           math.isclose(r['due_s'], r['received_s'] + r['delay_s'], abs_tol=1e-8)])
            low, high = ((fault.delay_s-fault.jitter_s, fault.delay_s+fault.jitter_s) if active else (0., 0.))
            checks.append(low-1e-8 <= r['delay_s'] <= high+1e-8)
            checks.append(len(r['before']) == len(r['after']) == (1 if kind == 'imu' else 2))
            checks.extend(a is not None and b is not None and math.isclose(b, fault.transform(a, r['source_stamp_s']), abs_tol=1e-8)
                          for a, b in zip(r['before'], r['after']))
        checks.append(all(b['source_stamp_s'] > a['source_stamp_s'] for a, b in zip(published, published[1:])))
        result[kind] = {'published': len(published), 'dropped': len(dropped),
                        'fault_active_events': sum(r['fault_active'] for r in events),
                        'max_relay_wait_s': max((r['released_s']-r['received_s'] for r in published), default=None),
                        'max_source_age_s': max((r['released_s']-r['source_stamp_s'] for r in published), default=None),
                        'first_drop_s': dropped[0]['source_stamp_s'] if dropped else None,
                        'last_drop_s': dropped[-1]['source_stamp_s'] if dropped else None}
        nontrivial = fault.drop or fault.scale != 1 or fault.bias != 0 or fault.delay_s != 0 or fault.stamp_offset_s != 0
        if nontrivial:
            checks.append(result[kind]['fault_active_events'] > 0)
    return {'verified': all(checks), 'streams': result, 'checks': len(checks),
            'failed_checks': sum(not c for c in checks),
            'scope': 'injected stream values and release times; not measured hardware errors'}
