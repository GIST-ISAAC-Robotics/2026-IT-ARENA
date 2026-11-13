#!/usr/bin/env python3
"""장치 없는 수동 중립 저장/검사. 연결/읽기/구동/EEPROM 기능 없음."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/arena_vehicle_interface'))
from arena_vehicle_interface.steering_calibration import (
    check_profile, covers, read_profile, record_zero, save_new_profile,
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
