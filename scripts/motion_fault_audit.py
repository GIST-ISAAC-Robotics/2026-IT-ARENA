"""시험 중계기의 실제 발행/누락 기록을 설정·시간·수치와 대조한다."""
import json
import math
from pathlib import Path

from arena_vehicle_interface.motion_impairment import MotionFaultConfig
from arena_vehicle_interface.drive_feedback import MODE_LEGACY, MODE_MOTOR, validate_mode
from arena_vehicle_interface.drive_feedback import MotorEncoderSpec
from arena_vehicle_interface.motor_count_impairment import MODEL


def audit(path, requested, *, feedback_mode=None, counts_per_motor_revolution=None):
    config = MotionFaultConfig.from_dict(requested)
    if not Path(path).exists():
        return {'verified': False, 'error': 'missing relay log'}
    rows = [json.loads(line) for line in Path(path).read_text().splitlines()]
    recorded_mode = rows[0].get('feedback_mode', MODE_LEGACY) if rows else MODE_LEGACY
    try:
        recorded_mode = validate_mode(recorded_mode)
        expected_mode = validate_mode(feedback_mode) if feedback_mode is not None else recorded_mode
    except ValueError as error:
        return {'verified': False, 'error': str(error)}
    checks = [bool(rows) and rows[0].get('config') == config.as_dict(),
              recorded_mode == expected_mode,
              bool(rows) and rows[-1].get('event') == 'close',
              not any(r['event'] == 'clock_reset' for r in rows)]
    count_model = rows[0].get('motor_count_model') if rows else None
    spec = None
    if count_model is not None:
        if count_model != MODEL or recorded_mode != MODE_MOTOR:
            return {'verified': False, 'error': 'unknown or incompatible count model'}
        try:
            spec = MotorEncoderSpec(rows[0]['counts_per_motor_revolution'], 1., 1.)
        except (KeyError, ValueError, TypeError) as error:
            return {'verified': False, 'error': str(error)}
    if counts_per_motor_revolution is not None:
        checks.append(spec is not None and spec.counts_per_motor_revolution == counts_per_motor_revolution)
    result = {}
    for kind in ('imu', 'wheels'):
        fault = getattr(config, kind)
        events = [r for r in rows if r.get('stream') == kind]
        published = [r for r in events if r['event'] == 'publish']
        dropped = [r for r in events if r['event'] == 'drop']
        checks.append(len(published) >= 30)
        previous_count = None
        accumulated_count = 0.
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
            expected_count = 1 if kind == 'imu' or expected_mode == MODE_MOTOR else 2
            checks.append(len(r['before']) == len(r['after']) == expected_count)
            if kind == 'wheels' and spec is not None:
                # 독립 누적식으로 정수 위치와 차분 속도를 대조한다. 첫 속도만 정의되지 않는다.
                try:
                    checks.append(len(r['before_position']) == len(r['after_position']) == 1)
                    raw = spec.counts_from_position(r['before_position'][0])
                    actual = spec.counts_from_position(r['after_position'][0])
                    t = r['source_stamp_s']
                    if previous_count is None:
                        accumulated_count = float(raw)
                        checks.extend([r['after'] == [None], r['after_velocity_kind'] == 'nan',
                                       r['before_velocity_kind'] in ('finite', 'nan')])
                    else:
                        old_t, old_raw, old_actual = previous_count
                        dt = t-old_t
                        if dt <= 0:
                            raise ValueError('nonincreasing motor time')
                        fraction = max(0., min(t, fault.end_s if fault.end_s is not None else t)
                                       - max(old_t, fault.start_s))/dt
                        accumulated_count += (raw-old_raw)*(1.+(fault.scale-1.)*fraction)
                        checks.extend([r['before_velocity_kind'] == r['after_velocity_kind'] == 'finite',
                                       r['before'][0] is not None,
                                       r['after'][0] is not None and math.isclose(r['after'][0],
                                           (actual-old_actual)*spec.motor_rad_per_count/dt, abs_tol=1e-8)])
                    checks.append(actual == round(accumulated_count))
                    previous_count = (t, raw, actual)
                except (KeyError, ValueError, TypeError, IndexError):
                    checks.append(False)
            else:
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
            'feedback_mode': recorded_mode, 'expected_feedback_mode': expected_mode,
            'motor_count_model': count_model,
            'failed_checks': sum(not c for c in checks),
            'scope': 'injected stream values and release times; not measured hardware errors'}
