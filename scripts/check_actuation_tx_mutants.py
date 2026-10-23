#!/usr/bin/env python3
"""송신 정책 시험의 검출력 확인: 의도적 결함 변이체마다 tx 정책 회귀가 실패하는지 본다. 원본은 수정하지 않는다."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PORTABLE = ROOT / 'src/arena_vehicle_interface/portable'
HEADER = PORTABLE / 'actuation_tx.hpp'
TESTS = 'tests/test_actuation_tx_policy.py'
# (이름, 원문, 대체문). 원문은 정확히 한 번 나타나야 한다.
MUTANTS = [
    ('stop_ack_not_first', 'if (acks_[i].stop) { index = i; break; }', 'if (false) { index = i; break; }'),
    ('stop_ack_does_not_evict', '      if (stop) {\n        for', '      if (false) {\n        for'),
    ('non_stop_overflow_evicts_oldest', 'if (victim == kTxAckCapacity) { ++stats_.ack_dropped_overflow; return; }',
     'if (victim == kTxAckCapacity) victim = 0;'),
    ('session_break_keeps_tail',
     '      stats_.aborted_bytes += frame_size_ - frame_offset_;\n      frame_size_ = frame_offset_ = 0;\n    }\n    stats_.ack_purged',
     '      stats_.aborted_bytes += frame_size_ - frame_offset_;\n    }\n    stats_.ack_purged'),
    ('session_break_keeps_acks', '    ack_count_ = 0;\n    consecutive_events_ = 0;', '    consecutive_events_ = 0;'),
    ('generation_change_no_urgent_offer', 'if (generation_changed && !urgent_offer_)',
     'if (false && generation_changed && !urgent_offer_)'),
    ('generation_change_replaces_in_flight', 'if (generation_changed && !urgent_offer_)',
     'if (generation_changed) frame_size_ = frame_offset_ = 0;\n    if (generation_changed && !urgent_offer_)'),
    ('starvation_guard_off', 'consecutive_events_ >= config_.max_consecutive_events;',
     'consecutive_events_ >= config_.max_consecutive_events + 1000;'),
    ('diag_ignores_offer_deadline', 'if (need_us > room_us)', 'if (false && need_us > room_us)'),
    ('uppercase_escape_hex', 'constexpr char digits[] = "0123456789abcdef";', 'constexpr char digits[] = "0123456789ABCDEF";'),
    ('sink_overrun_trusted', 'if (accepted > remaining) {', 'if (false) {'),
    ('tx_clock_regression_ignored', '(has_now_ && now < now_)', '(false && now < now_)'),
]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run_tests(folder, log):
    env = dict(os.environ, ARENA_TX_PORTABLE_OVERRIDE=str(folder))
    run = subprocess.run([sys.executable, '-B', '-m', 'pytest', '-q', '-x', '-p', 'no:cacheprovider', TESTS],
                         cwd=ROOT, env=env, capture_output=True, text=True, timeout=900)
    log.write_text(run.stdout[-20000:] + run.stderr[-5000:], encoding='utf-8', newline='\n')
    last = (run.stdout.strip().splitlines() or [''])[-1]
    return run.returncode, last


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    original = HEADER.read_text(encoding='utf-8')
    before = digest(HEADER)
    results = []
    for name, old, new in [('unmutated_copy', '', '')] + MUTANTS:
        folder = args.output / name
        folder.mkdir()
        for header in PORTABLE.glob('*.hpp'):
            shutil.copy(header, folder / header.name)
        if name != 'unmutated_copy':
            if original.count(old) != 1:
                results.append(dict(name=name, error='pattern not unique', detected=False))
                continue
            (folder / 'actuation_tx.hpp').write_text(original.replace(old, new), encoding='utf-8', newline='\n')
        code, last = run_tests(folder, folder / 'pytest.log')
        for header in folder.glob('*.hpp'):
            header.unlink()  # 사본 헤더는 원본/스냅숏과 같으므로 보존하지 않는다(변이 내용은 이 스크립트에 있다).
        results.append(dict(name=name, pytest_exit=code, last_line=last,
                            detected=(code != 0) if name != 'unmutated_copy' else None))
    mutants = [r for r in results if r['name'] != 'unmutated_copy']
    control = results[0]
    summary = dict(mutants=len(MUTANTS), detected=sum(bool(r['detected']) for r in mutants),
                   control_passed=control.get('pytest_exit') == 0, results=results,
                   original_header_sha256=before, original_unchanged=digest(HEADER) == before)
    with (args.output / 'mutants.json').open('x', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2, ensure_ascii=False)
    print(json.dumps({k: v for k, v in summary.items() if k != 'results'}))
    for row in results:
        print(json.dumps(row, ensure_ascii=False))
    ok = summary['detected'] == len(MUTANTS) and summary['control_passed'] and summary['original_unchanged']
    return 0 if ok else 2


if __name__ == '__main__':
    raise SystemExit(main())
