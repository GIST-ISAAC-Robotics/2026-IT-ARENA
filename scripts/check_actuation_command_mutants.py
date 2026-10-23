#!/usr/bin/env python3
"""명령 parser 대조의 검출력 확인: 의도적 결함 변이체가 모두 불일치로 잡히는지 본다. 원본은 수정하지 않는다."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

from actuation_conformance import ROOT, digest
import validate_actuation_command_cpp as bench

PORTABLE = ROOT/'src/arena_vehicle_interface/portable'
HEADER = PORTABLE/'actuation_command.hpp'
FLAGS = ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O1', '-g',
         '-fsanitize=address,undefined', '-fno-omit-frame-pointer']
# (이름, [(원문, 대체문), ...]). 각 원문은 정확히 한 번 나타나야 한다.
MUTANTS = [
    ('duplicate_detection_disabled', [('set_bit(duplicate_, depth_ - 1, true);', '(void)0;')]),
    ('duplicate_reported_immediately_as_syntax', [('set_bit(duplicate_, depth_ - 1, true);', 'return false;')]),
    ('surrogate_pair_not_combined_in_scan',
     [('if (low >= 0xDC00 && low <= 0xDFFF) pos += 6;', 'if (low >= 0xDC00 && low > 0xDFFF) pos += 6;')]),
    ('control_characters_allowed', [('if (c < 0x20) return false;', 'if (c < 0x01) return false;')]),
    ('utf8_surrogates_allowed', [('else if (c == 0xED) { extra = 2; high = 0x9F; }', 'else if (c == 0xED) { extra = 2; }')]),
    ('constants_reported_as_syntax',
     [("return word[k] == '\\0' ? ParseStatus::kNonFiniteConstant : ParseStatus::kSyntax;", 'return ParseStatus::kSyntax;')]),
    ('bigint_saturates_to_int64',
     [('if (overflow || magnitude > limit) { out.type = ValueType::kBigInt; out.negative = negative; return; }',
       'if (overflow || magnitude > limit) { magnitude = limit; }')]),
    ('float_underflow_becomes_infinity', [('value = overflows(first, last) ? HUGE_VAL : 0.0;', 'value = HUGE_VAL;')]),
    ('drive_age_ttl_order_ignored', [('return speed_ok && angle_ok && age_ok && ttl_ok && age.integer < ttl.integer;',
                                      'return speed_ok && angle_ok && age_ok && ttl_ok;')]),
    ('seq_bound_off_by_one', [('seq.integer <= 0xFFFFFFFFLL', 'seq.integer <= 0x100000000LL')]),
    ('leading_zero_allowed', [("else if (s[pos] == '0') ++pos;",
                               "else if (s[pos] == '0') { ++pos; while (pos < n && detail::is_digit(s[pos])) ++pos; }")]),
    ('trailing_data_allowed', [('return ParseStatus::kSyntax;  // 최상위 값 뒤의 잔여 데이터',
                                'return top_type_ == ValueType::kObject ? ParseStatus::kObject : ParseStatus::kNotObject;')]),
    ('result_not_reset_before_parse', [('    out = Command{};\n    out.layout = layout;\n    out.status = scan',
                                        '    out.layout = layout;\n    out.status = scan')]),
    ('inner_keys_not_popped', [('      key_count_ = key_base_ - 1;\n      key_base_ = keys_[key_count_].begin;',
                                '      key_base_ = keys_[key_base_ - 1].begin;')]),
    ('invalid_drive_filtered_before_state_machine',
     [('    store(s, spans_[static_cast<std::size_t>(CommandField::kClient)], out.client);\n',
       '    store(s, spans_[static_cast<std::size_t>(CommandField::kClient)], out.client);\n'
       '    if (out.kind == KindName::kDrive && out.fields[7].type != ValueType::kFloat &&\n'
       '        out.fields[7].type != ValueType::kInt) out.status = ParseStatus::kSyntax;\n')]),
    ('bool_true_counts_as_version_one',
     [('if (v.type != ValueType::kInt || v.integer != 1)',
       'if ((v.type != ValueType::kInt && !(v.type == ValueType::kBool && v.boolean)) ||\n'
       '      (v.type == ValueType::kInt && v.integer != 1))')]),
    ('envelope_command_at_any_depth',
     [('layout == PayloadLayout::kEnvelope && slot == kEnvelopeCommand && depth_ == 2',
       'layout == PayloadLayout::kEnvelope && slot == kEnvelopeCommand && depth_ >= 2'),
      ('} else if (layout == PayloadLayout::kEnvelope && depth_ == 1) {', '} else if (layout == PayloadLayout::kEnvelope) {')]),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--compiler', default='g++')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    original = HEADER.read_text(encoding='utf-8')
    before = digest(HEADER)
    plan = bench.build_plan()
    results = []
    for name, edits in MUTANTS:
        text = original
        if any(text.count(old) != 1 for old, _ in edits):
            results.append(dict(name=name, error='pattern not unique', detected=False))
            continue
        for old, new in edits:
            text = text.replace(old, new)
        folder = args.output/name
        folder.mkdir()
        for other in ('actuation_wire.hpp', 'actuation_link.hpp'):
            shutil.copy(PORTABLE/other, folder/other)
        (folder/'actuation_command.hpp').write_text(text, encoding='utf-8', newline='\n')
        binary = folder/'command_driver'
        build = subprocess.run([args.compiler, *FLAGS, f'-I{folder}', str(ROOT/'scripts/actuation_command_cpp_driver.cpp'),
                                '-o', str(binary)], capture_output=True, text=True)
        (folder/'compiler.log').write_text(build.stdout + build.stderr, encoding='utf-8', newline='\n')
        if build.returncode:
            results.append(dict(name=name, error='compile failed', detected=False))
            continue
        report = bench.evaluate(binary, plan)[0]
        binary.unlink()  # 로컬 실행 파일은 보존 대상이 아니다.
        results.append(dict(name=name, detected=not report['passed'], binary_exit_code=report['binary_exit_code'],
                            representation_mismatches=len(report['representation_mismatches']),
                            manual_failures=len(report['manual_failures']),
                            receive_equivalence_failures=len(report['receive_equivalence_failures'])))
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
