#!/usr/bin/env python3
"""C++ 수신 상태기계 대조의 검출력 확인: 의도적 결함 변이체가 모두 불일치로 잡히는지 본다. 원본은 수정하지 않는다."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

from actuation_conformance import ROOT, digest
import validate_actuation_receiver_cpp as bench

PORTABLE = ROOT/'src/arena_vehicle_interface/portable'
HEADER = PORTABLE/'actuation_receiver.hpp'
FLAGS = ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O1', '-g',
         '-fsanitize=address,undefined', '-fno-omit-frame-pointer']
OWNER = ('    if (!armed() || !has_owner_ || !c.client.equals(owner_.code_points.data(), owner_.length))\n'
         '      return {false, ReceiveReason::kNotOwnerOrArmed};\n')
MUTANTS = [
    ('invalid_drive_checked_before_ownership',
     [(OWNER, '    if (cls == CommandClass::kDrive && !drive_values_ok(c, DriveLimits{config_.max_speed_mps, '
              'config_.max_steering_rad, config_.lease_us})) { fault("invalid_drive"); '
              'return {false, ReceiveReason::kInvalidDrive}; }\n' + OWNER)]),
    ('invalid_drive_ignored_without_fault', [('      fault("invalid_drive");\n', '')]),
    ('receive_feeds_control_watchdog', [('    const bool clock_ok = check(now);\n',
                                         '    const bool clock_ok = check(now);\n    if (clock_ok) { last_control_ = now; has_last_control_ = true; }\n')]),
    ('transition_centers_steering', [('    // 정지 중 갑작스러운 중앙 조향은 하지 않는다(steering 유지).\n', '    steering_ = 0.;\n')]),
    ('lease_evicts_newest', [('erase_lease(0);', 'erase_lease(lease_count_ - 1);')]),
    ('physical_press_ignored_when_latched',
     [('    if (pressed && initialized_) transition(ReceiverState::kEstop, "physical_stop");',
       '    if (pressed && initialized_ && state_ != ReceiverState::kFault) transition(ReceiverState::kEstop, "physical_stop");')]),
    ('clear_ignores_estop_held', [('if (!stationary(now) || estop_held_) return', 'if (!stationary(now)) return')]),
    ('no_progress_watch_removed', [('        fault("motor_feedback_no_progress");', '        (void)0;')]),
    ('expired_leases_not_pruned', [('      if (now < leases_[i].expires_us) leases_[kept++] = leases_[i];',
                                    '      leases_[kept++] = leases_[i];')]),
    ('count_delta_wraps_int64', [('    if (now_count >= old) return static_cast<double>(a - b);',
                                  '    (void)a; (void)b; return static_cast<double>(now_count - old);')]),
    ('duplicate_feedback_refreshes', [('if (sequence.value <= feedback_seq_ || capture.value <= feedback_capture_) return false;',
                                       'if (capture.value < feedback_capture_) return false;')]),
    ('source_age_ignored', [('const std::int64_t deadline = lease->expires_us < source_deadline ? lease->expires_us : source_deadline;',
                             'const std::int64_t deadline = lease->expires_us; (void)source_deadline;')]),
    ('feedback_freshness_inclusive', [('now - feedback_capture_ < config_.feedback_timeout_us;',
                                       'now - feedback_capture_ <= config_.feedback_timeout_us;')]),
]


def mutated_source(original, edits):
    """변이 주입 위치가 없거나 모호하면 코드 결함 검출과 구별해 실패시킨다."""
    text = original
    for old, new in edits:
        count = text.count(old)
        if count != 1:
            raise ValueError(f'mutation anchor must occur once; found {count}')
        text = text.replace(old, new, 1)
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--compiler', default='g++')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    original = HEADER.read_text(encoding='utf-8')
    before = digest(HEADER)
    plan, receive = bench.build_plan(), bench.receive_plan()
    results = []
    for name, edits in MUTANTS:
        try:
            text = mutated_source(original, edits)
        except ValueError as error:
            results.append(dict(name=name, error=str(error), detected=False))
            continue
        folder = args.output/name
        folder.mkdir()
        for other in ('actuation_wire.hpp', 'actuation_link.hpp', 'actuation_command.hpp'):
            shutil.copy(PORTABLE/other, folder/other)
        (folder/'actuation_receiver.hpp').write_text(text, encoding='utf-8', newline='\n')
        binary = folder/'receiver_driver'
        build = subprocess.run([args.compiler, *FLAGS, f'-I{folder}', str(ROOT/'scripts/actuation_receiver_cpp_driver.cpp'),
                                '-o', str(binary)], capture_output=True, text=True)
        (folder/'compiler.log').write_text(build.stdout + build.stderr, encoding='utf-8', newline='\n')
        if build.returncode:
            results.append(dict(name=name, error='compile failed', detected=False))
            continue
        report, _, _, stderr = bench.evaluate(binary, plan, receive)
        binary.unlink()
        results.append(dict(name=name, detected=not report['passed'], mismatches=report['mismatch_count'],
                            boundary_failures=sum(not r['passed'] for r in report['boundary_checks']),
                            sanitizer_output=bool(stderr), binary_exit_code=report['binary_exit_code']))
    summary = dict(mutants=len(MUTANTS), detected=sum(r['detected'] for r in results), results=results,
                   original_header_sha256=before, original_unchanged=digest(HEADER) == before)
    with (args.output/'mutants.json').open('x', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2)
    print(json.dumps({k: v for k, v in summary.items() if k != 'results'}))
    for row in results:
        print(json.dumps(row))
    return 0 if summary['detected'] == len(MUTANTS) and summary['original_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
