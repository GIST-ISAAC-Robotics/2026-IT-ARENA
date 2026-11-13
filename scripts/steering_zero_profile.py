#!/usr/bin/env python3
"""장치 없는 수동 중립 저장/검사. 연결/읽기/구동/EEPROM 기능 없음."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/arena_vehicle_interface'))
from arena_vehicle_interface.steering_calibration import (
    check_profile, covers, read_profile, record_zero, save_new_profile, complete_mount_profile, cad_servo_goal,
    cpp_zero_profile,
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    record = commands.add_parser('record')
    record.add_argument('--neutral-count', type=int, required=True)
    record.add_argument('--assembly-id', required=True)
    record.add_argument('--confirm-wheels-straight', action='store_true', required=True)
    record.add_argument('--evidence-note', required=True)
    record.add_argument('--output', type=Path, required=True)
    check = commands.add_parser('check')
    check.add_argument('profile', type=Path)
    check.add_argument('--assembly-id', required=True)
    check.add_argument('--required-servo-min-rad', type=float, required=True)
    check.add_argument('--required-servo-max-rad', type=float, required=True)
    mount = commands.add_parser('confirm-mount')
    mount.add_argument('profile', type=Path)
    mount.add_argument('--direction', type=int, choices=(-1, 1), required=True,
                       help='CAD 양수 상대각 방향으로 움직일 때 count가 증가하면 1, 감소하면 -1')
    mount.add_argument('--safe-min-count', type=int, required=True)
    mount.add_argument('--safe-max-count', type=int, required=True)
    mount.add_argument('--confirm-position-mode-and-mounted-limits', action='store_true', required=True)
    mount.add_argument('--output', type=Path, required=True)
    goal = commands.add_parser('goal')
    goal.add_argument('profile', type=Path)
    goal.add_argument('--steering-deg', type=float, required=True,
                      help='좌회전 양수인 차량 중앙 등가각. 서보 축 각도가 아닙니다.')
    export = commands.add_parser('export-cpp', help='실장 확인 값을 MCU 빌드용 헤더로 내보내기. 업로드/출력 없음.')
    export.add_argument('profile', type=Path)
    export.add_argument('--assembly-id', required=True)
    export.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == 'record':
            data = record_zero(args.neutral_count, assembly_id=args.assembly_id,
                               straight_confirmed=args.confirm_wheels_straight,
                               evidence_note=args.evidence_note)
            save_new_profile(args.output, data)
            print(json.dumps(dict(saved=str(args.output), calibration_ready=False,
                                  output_enabled=False, status=data['status'])))
            return 0
        data = read_profile(args.profile)
        if args.command == 'export-cpp':
            header, digest = cpp_zero_profile(data, assembly_id=args.assembly_id)
            with args.output.open('x', encoding='ascii', newline='\n') as stream:
                stream.write(header)
            print(json.dumps(dict(saved=str(args.output), canonical_profile_sha256=digest,
                                  output_enabled=False, firmware_uploaded=False)))
            return 0
        if args.command == 'confirm-mount':
            data = complete_mount_profile(data, direction=args.direction, minimum=args.safe_min_count,
                maximum=args.safe_max_count, confirmed=args.confirm_position_mode_and_mounted_limits)
            save_new_profile(args.output, data)
            print(json.dumps(dict(saved=str(args.output), output_enabled=False, status=data['status'])))
            return 0
        if args.command == 'goal':
            import math
            print(json.dumps(cad_servo_goal(data, math.radians(args.steering_deg)), allow_nan=False))
            return 0
        check_profile(data, ready=True, assembly_id=args.assembly_id)
        complete = covers(data, args.required_servo_min_rad, args.required_servo_max_rad)
        print(json.dumps(dict(profile_valid=True, required_range_covered=complete,
                              output_enabled=False, hardware_angle_measured=False,
                              reason='profile_checked' if complete else 'required_range_crosses_mounted_or_count_limit')))
        return 0 if complete else 2
    except (ValueError, OSError) as exc:
        print(json.dumps(dict(profile_valid=False, output_enabled=False, reason=str(exc))))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
