#!/usr/bin/env python3
"""FrameLink 대조의 검출력 확인: 의도적 결함 변이체가 모두 불일치로 잡히는지 본다. 원본은 수정하지 않는다."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

from actuation_conformance import ROOT, digest
from actuation_link_reference import make_policy_cases
import validate_actuation_link_cpp as bench

HEADER = 'src/arena_vehicle_interface/portable/actuation_link.hpp'
FLAGS = ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O1', '-g',
         '-fsanitize=address,undefined', '-fno-omit-frame-pointer']
# (이름, 원문, 대체문). 원문은 정확히 한 번 나타나야 한다.
MUTANTS = [
    ('deadline_off_by_one', 'now - anchor_ < timeout_', 'now - anchor_ <= timeout_'),
    ('trickle_refreshes_anchor', '} else if (remaining <= size) {', '} else {'),
    ('timeout_drops_without_drain',
     '    while (decoder_.buffered_bytes() != 0) {\n      decoder_.reject_head',
     '    drop_partial();\n    while (decoder_.buffered_bytes() != 0) {\n      decoder_.reject_head'),
    ('reconnect_keeps_partial', '    if (state_ == LinkState::kConnected) ++session_breaks_;\n    drop_partial();',
     '    if (state_ == LinkState::kConnected) ++session_breaks_;'),
    ('clock_regression_ignored', 'if (has_time_ && now < last_now_) {', 'if (has_time_ && now < last_now_ && false) {'),
    ('input_budget_not_latched',
     '      // 상위 수신 작업의 호출량 계약 위반. 같은 호출의 바이트는 하나도 처리하지 않는다.\n'
     '      latch(LinkFault::kInputBudget);',
     '      drop_partial();'),
    ('disconnect_not_a_session_break',
     '    if (state_ != LinkState::kConnected) return LinkResult::kNotConnected;\n    ++session_breaks_;',
     '    if (state_ != LinkState::kConnected) return LinkResult::kNotConnected;'),
    ('poll_does_not_expire',
     '    if (state_ != LinkState::kConnected) return LinkResult::kNotConnected;\n    expire(now, callback);',
     '    if (state_ != LinkState::kConnected) return LinkResult::kNotConnected;\n    (void)callback;'),
    ('clear_fault_reconnects', '    state_ = LinkState::kDisconnected;\n    last_now_ = now;',
     '    state_ = LinkState::kConnected;\n    last_now_ = now;'),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--compiler', default='g++')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    original = (ROOT/HEADER).read_text(encoding='utf-8')
    before = digest(ROOT/HEADER)
    link_cases, policy_cases = bench.make_link_cases(), make_policy_cases()
    results = []
    for name, old, new in MUTANTS:
        if original.count(old) != 1:
            results.append(dict(name=name, error='pattern not unique', detected=False))
            continue
        folder = args.output/name
        folder.mkdir()
        shutil.copy(ROOT/'src/arena_vehicle_interface/portable/actuation_wire.hpp', folder/'actuation_wire.hpp')
        (folder/'actuation_link.hpp').write_text(original.replace(old, new), encoding='utf-8', newline='\n')
        binary = folder/'link_driver'
        build = subprocess.run([args.compiler, *FLAGS, f'-I{folder}', str(ROOT/'scripts/actuation_link_cpp_driver.cpp'),
                                '-o', str(binary)], capture_output=True, text=True)
        (folder/'compiler.log').write_text(build.stdout + build.stderr, encoding='utf-8', newline='\n')
        if build.returncode:
            results.append(dict(name=name, error='compile failed', detected=False))
            continue
        report = bench.evaluate(binary, link_cases, policy_cases)[0]
        binary.unlink()  # 로컬 실행 파일은 보존 대상이 아니다.
        results.append(dict(name=name, detected=not report['passed'],
                            link_mismatches=len(report['link_mismatches']),
                            policy_failures=[f['name'] for f in report['policy_replay_failures']],
                            binary_exit_code=report['binary_exit_code']))
    summary = dict(mutants=len(MUTANTS), detected=sum(r['detected'] for r in results), results=results,
                   original_header_sha256=before, original_unchanged=digest(ROOT/HEADER) == before)
    with (args.output/'mutants.json').open('x', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2)
    print(json.dumps({k: v for k, v in summary.items() if k != 'results'}))
    for row in results:
        print(json.dumps(row))
    return 0 if summary['detected'] == len(MUTANTS) and summary['original_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
