#!/usr/bin/env python3
"""장착 서보 수동 3점 측정과 한 번에 최대 12 counts 미동. 격리 probe 전용."""
import argparse
import json
import math
from pathlib import Path
import statistics
import struct
import sys

from probe_bus_servo import Probe

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/arena_vehicle_interface'))
from arena_vehicle_interface.steering_calibration import read_profile, record_zero, save_new_profile

STEP = 12                        # about 1.05 degrees at the servo shaft
SPAN = 56                        # initial check only, about 4.92 degrees from straight
ERROR = 5                        # feedback counts, not wheel-angle accuracy
COUNTS_PER_TURN = 4096


class UnstablePosition(RuntimeError):
    pass


def read_bytes(probe, address, count):
    return probe.read(address, count)[0]['data']


def position(rows, *, torque_off=False):
    counts = [row['position_counts'] for row in rows]
    if (len(counts) < 3 or min(counts) < 0 or max(counts) > 4095 or
            max(counts) - min(counts) > 2 or (not torque_off and any(row['moving'] for row in rows))):
        raise UnstablePosition('shaft position is not stable; hold the pose steady and recheck')
    return int(statistics.median(counts))


def bounds(config):
    if (config[2] != 0 or config[5] != 1 or config[6] != 0 or config[8] != 1 or
            config[30] != 1 or config[33] != 0 or config[18] & 0x10):
        raise RuntimeError('expected ID 1 / 1 Mbps / write replies / single-turn position mode')
    low, high = struct.unpack('<HH', config[9:13])
    if not 0 <= low < high <= 4095:
        raise RuntimeError('unexpected servo count bounds')
    return low, high


def checked_feedback(probe, seconds, label, low, high, target=None):
    rows = probe.capture(seconds, label, target)
    if not rows or any(not low <= row['position_counts'] <= high or row['servo_status'] or
                       not 9 <= row['voltage_v'] <= 12.6 or row['temperature_c'] >= 50 or
                       abs(row['current_signed']) >= 150 for row in rows):
        raise RuntimeError('position, status, voltage, temperature or current outside probe bounds')
    return rows


def require_torque_off(probe):
    if read_bytes(probe, 40, 1) != b'\0':
        raise RuntimeError('torque is not off; use release before moving wheels by hand')


def wheel_angles(side):
    """For each wheel, travel relative to its own straight orientation."""
    while True:
        raw = input('각도계로 잰 왼쪽 바퀴각 오른쪽 바퀴각 (도, 크기 2개 / 생략: Enter): ').strip()
        if not raw:
            return None
        try:
            left, right = map(float, raw.split())
            if not all(math.isfinite(value) and 0 < value <= 90 for value in (left, right)):
                raise ValueError()
        except ValueError:
            print('두 바퀴의 0° 초과, 90° 이하 값을 공백으로 구분해 입력하세요. 예: 28.5 22')
            continue
        sign = 1 if side == 'left_max' else -1
        return dict(left_wheel=sign*left, right_wheel=sign*right)


def summarize_manual(points, low, high):
    zero, left, right = (points[key]['shaft_count'] for key in ('straight', 'left_max', 'right_max'))
    half_turn = COUNTS_PER_TURN // 2
    raw_deltas = (left-zero, right-zero)
    deltas = tuple((delta + half_turn) % COUNTS_PER_TURN - half_turn for delta in raw_deltas)
    issues = []
    wrap = deltas != raw_deltas
    if wrap:
        issues.append('count_wrap_between_neutral_and_endpoint')
    ambiguous = any(abs(delta) == half_turn for delta in raw_deltas)
    consistent = (not ambiguous and deltas[0]*deltas[1] < 0 and
                  min(map(abs, deltas)) > ERROR)
    if not consistent:
        issues.append('endpoints_do_not_show_distinct_opposite_directions')
    if not all(low <= x <= high for x in (zero, left, right)):
        issues.append('measurement_outside_servo_command_bounds')
    for side, delta in zip(('left_max', 'right_max'), deltas):
        points[side]['shaft_delta_counts_from_straight'] = delta if consistent else None
        points[side]['shaft_delta_deg_from_straight'] = round(delta * 360 / COUNTS_PER_TURN, 3) if consistent else None
    return dict(count_direction_per_cad_positive=(1 if deltas[1] > 0 else -1) if not issues else None,
                measured_count_interval=[min(left, right), max(left, right)] if not issues else None,
                crosses_count_wrap=wrap, issues=issues)


def measure(probe, args, low, high, report):
    require_torque_off(probe)
    points = report['manual_points'] = {}
    report.update(torque_after=0, wheel_angle_reference='relative_to_each_wheel_at_straight_left_positive',
                  servo_command_bounds=[low, high], operating_limits_confirmed=False,
                  assembly_id=args.assembly_id, evidence=args.evidence)
    for key, label in (('straight', '앞바퀴 직진'), ('left_max', '차량 왼쪽 최대'), ('right_max', '차량 오른쪽 최대')):
        while True:
            report['torque_after'] = None
            require_torque_off(probe)
            input(f'{label}: 손으로 위치를 맞춰 유지한 다음 Enter를 누르세요. ')
            # The shaft is moved by hand. Only the READ path is used in this mode.
            require_torque_off(probe)
            samples = checked_feedback(probe, .30, f'manual_{key}', 0, 4095)
            require_torque_off(probe)
            report['torque_after'] = 0
            try:
                count = position(samples, torque_off=True)
            except UnstablePosition:
                print('위치가 흔들렸습니다. 같은 자세를 가볍게 유지한 뒤 다시 기록하세요.')
                continue
            break
        points[key] = dict(shaft_count=count, wheel_angle_deg=None)
        print(f'{label}: 서보 {count} counts 기록. 이제 손을 떼셔도 됩니다.')
        if key == 'straight':
            points[key].update(shaft_delta_counts_from_straight=0, shaft_delta_deg_from_straight=0.0)
            profile = record_zero(count, assembly_id=args.assembly_id,
                                  straight_confirmed=True, evidence_note=args.evidence)
            path = args.log / 'straight.json'
            save_new_profile(path, profile)
            report['zero_profile'] = str(path)
        else:
            points[key]['wheel_angle_deg'] = wheel_angles(key)
    report.update(summarize_manual(points, low, high))
    if report['issues']:
        print('세 위치의 원값은 저장했습니다. issues를 확인하고 이 결과를 운영 범위로 쓰지 마세요.')


def target_count(current, neutral, delta, low, high):
    if delta is None:  # take only one step toward zero, without passing it
        delta = max(-STEP, min(STEP, neutral - current))
    if abs(delta) > STEP:
        raise RuntimeError('one jog is at most 12 counts')
    target = current + delta
    # Leave space for feedback error near the numeric endpoints. Never wrap counts.
    if (not low + ERROR <= neutral <= high - ERROR or
            not low + ERROR <= min(current, target) <= max(current, target) <= high - ERROR or
            max(abs(current - neutral), abs(target - neutral)) > SPAN):
        raise RuntimeError('outside initial jog window or too close to a count endpoint; check horn indexing')
    return target


def release(probe):
    probe.command('X', 'disarmed')
    probe.active = False
    # X sends torque OFF only if the relay was armed. Read the servo itself.
    if read_bytes(probe, 40, 1) != b'\0':
        probe.write(40, b'\0')
        if read_bytes(probe, 40, 1) != b'\0':
            raise RuntimeError('torque OFF could not be verified')
    return 0


def jog(probe, config, current, target, low, high, report):
    if target == current:
        return
    speed = 100 if config[18] & 4 else 2
    limit_before = struct.unpack('<H', read_bytes(probe, 48, 2))[0]
    if not 1 <= limit_before <= 1000:
        raise RuntimeError('unexpected present torque limit')
    limit = struct.pack('<H', min(limit_before, 300))
    observed_low = max(low, min(current, target) - ERROR)
    observed_high = min(high, max(current, target) + ERROR)
    try:
        # Cleanup also runs if ARM was accepted but its host reply was lost.
        probe.command(f'ARM 1 {current} {speed}', 'armed')
        probe.active = True
        probe.write(48, limit)
        if read_bytes(probe, 48, 2) != limit:
            raise RuntimeError('reduced torque limit not verified')
        report['torque_limit_sram'] = int.from_bytes(limit, 'little')

        def goal(value):
            probe.write(41, struct.pack('<BHHH', 5, value, 0, speed))
            registers = read_bytes(probe, 42, 6)
            if registers[:2] != struct.pack('<H', value) or registers[4:] != struct.pack('<H', speed):
                raise RuntimeError('goal or speed readback differs')

        goal(current)
        if abs(position(checked_feedback(probe, .20, 'before_torque', low, high)) - current) > 2:
            raise RuntimeError('shaft moved since ARM; keeping torque off')
        if read_bytes(probe, 40, 1) != b'\0':
            raise RuntimeError('torque already enabled before hold')
        probe.write(40, b'\1')
        if read_bytes(probe, 40, 1) != b'\1':
            raise RuntimeError('hold torque not confirmed')
        checked_feedback(probe, .25, 'hold', max(low, current-ERROR), min(high, current+ERROR), current)
        goal(target)
        samples = checked_feedback(probe, 1.0, 'jog', observed_low, observed_high, target)
        settled = position(samples[-3:])
        report['position_before_release'] = settled
        report['observed_delta_counts'] = settled - current
        if abs(settled - target) > ERROR or (settled - current) * (target - current) <= 0:
            raise RuntimeError('requested displacement was not confirmed; inspect link and backlash')
    finally:
        report['torque_after'] = release(probe)


def run(args):
    profile = read_profile(args.profile) if args.command == 'jog' else None
    probe = Probe(args.port, args.log)
    report = {'command': args.command, 'completed': False, 'scope': 'mounted_steering_setup', 'torque_after': None}
    try:
        info = probe.connect()
        if (info.get('tx_gpio'), info.get('rx_gpio')) != (17, 18):
            raise RuntimeError('unexpected servo relay pins')
        if info['armed']:
            if args.command != 'release':
                raise RuntimeError('relay already armed; check the other session or use release')
            probe.command('X', 'disarmed')
        probe.command('B 1000000', 'baud')
        probe.servo_id = 1
        if args.command == 'release':
            report['torque_after'] = release(probe)
        else:
            config = read_bytes(probe, 0, 40)
            low, high = bounds(config)
            if args.command == 'measure':
                measure(probe, args, low, high, report)
                report['completed'] = True
                return
            if profile and profile['status'] == 'mount_checks_recorded':
                low = max(low, profile['safe_min_count'])
                high = min(high, profile['safe_max_count'])
            torque = read_bytes(probe, 40, 1)[0]
            report['torque_before'] = torque
            if torque != 0:
                raise RuntimeError('torque is not off; use release before moving wheels by hand')
            current = position(checked_feedback(probe, .30, 'stationary', low, high))
            report.update(position_counts=current, position_bounds=[low, high], torque_after=0)
            if args.command == 'zero':
                profile = record_zero(current, assembly_id=args.assembly_id,
                                      straight_confirmed=args.wheels_straight, evidence_note=args.evidence)
                save_new_profile(args.output, profile)
                report.update(zero_profile=str(args.output), mount_limits_verified=False)
            elif args.command == 'jog':
                neutral = profile['zero_count']
                target = target_count(current, neutral, args.counts, low, high)
                report.update(neutral_count=neutral, target_count=target,
                              requested_shaft_deg=(target-current)*360/4096)
                if target != current:
                    report['torque_after'] = None
                    jog(probe, config, current, target, low, high, report)
                    if read_bytes(probe, 0, 40) != config:
                        raise RuntimeError('servo configuration changed during session')
                    after = checked_feedback(probe, .20, 'released', low, high)
                    report['position_after_release'] = after[-1]['position_counts']
        report['completed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc) or type(exc).__name__
        raise
    finally:
        try:
            probe.close()
        finally:
            result = json.dumps(report, ensure_ascii=False, indent=2)
            (args.log / 'summary.json').write_text(result + '\n', encoding='utf-8')
            print(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='explicit USB-UART for ESP32 servo_probe_v1')
    parser.add_argument('--log', required=True, type=Path, help='new directory for raw replies and summary')
    cmds = parser.add_subparsers(dest='command', required=True)
    cmds.add_parser('read', help='read the stationary shaft position with torque off')
    cmds.add_parser('release', help='disable torque and read back register 40')
    manual = cmds.add_parser('measure', help='manually capture straight, left maximum and right maximum without driving the servo')
    manual.add_argument('--assembly-id', required=True)
    manual.add_argument('--evidence', default='Manual straight and linkage endpoints, torque off')
    manual.add_argument('--motor-power-disconnected', required=True, action='store_true')
    manual.add_argument('--wheels-raised', required=True, action='store_true')
    zero = cmds.add_parser('zero', help='save the present count as mechanically confirmed straight')
    zero.add_argument('--wheels-straight', required=True, action='store_true')
    zero.add_argument('--assembly-id', required=True)
    zero.add_argument('--evidence', required=True)
    zero.add_argument('--output', required=True, type=Path)
    move = cmds.add_parser('jog', help='one relative step followed by torque OFF')
    move.add_argument('--profile', required=True, type=Path)
    move.add_argument('--motor-power-disconnected', required=True, action='store_true')
    move.add_argument('--wheels-raised', required=True, action='store_true')
    delta = move.add_mutually_exclusive_group(required=True)
    delta.add_argument('--counts', type=int, choices=tuple(range(-STEP, 0)) + tuple(range(1, STEP+1)))
    delta.add_argument('--toward-zero', action='store_true')
    args = parser.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        print('Interrupted; check summary for torque-off readback.', file=sys.stderr)
        return 130
    except Exception as exc:
        print(f'Servo setup stopped: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
