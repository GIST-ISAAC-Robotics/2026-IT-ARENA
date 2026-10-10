#!/usr/bin/env python3
"""실행 소스·새/보존 기준선·원시 해시·회귀를 묶는 마감 감사."""
import hashlib
import json
from pathlib import Path
import re
import zipfile

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def main():
    summary = read(BASE/'analysis.json')
    new = BASE/'flat_legacy'
    old = BASE.parent/'braking_ab_v1/flat'
    a, b = read(old/'report.json'), read(new/'report.json')
    changed = {path for path, value in a['source_sha256'].items() if b['source_sha256'][path] != value}
    expected = {'src/arena_gazebo/include/arena_gazebo/drivetrain.hpp',
        'src/arena_gazebo/src/single_motor_drive.cpp', 'src/arena_bringup/launch/simulation.launch.py',
        'scripts/probe_braking_contact.py'}
    modes = {}
    for row in summary['runs']:
        run = ROOT/row['path']
        report = read(run/'report.json')
        with zipfile.ZipFile(run/'source_snapshot.zip') as source:
            source_ok = all(hashlib.sha256(source.read(path)).hexdigest() == value and sha(ROOT/path) == value
                            for path,value in report['source_sha256'].items())
        modes[run.name] = {'source_zip_and_current': source_ok,
            'analysis_inputs_unchanged': all(sha(run/path) == value for path,value in row['input_sha256'].items()),
            'passed': report['passed'], 'shutdown_clean': report['shutdown_clean'],
            'stop_ack': report['stop_acknowledged'], 'stop_latched': report['final_mcu']['reason'] == 'software_stop',
            'no_pids': not report['remaining_pids'],
            'mode_observed': report['mode_observed']}
    all_commands = all(d['samples'] >= 250 and d['max_abs'] == 0 for pair in summary['comparisons']
                       for d in pair['aligned_commands'].values())
    regression = (BASE/'pytest_final_v2.log').read_text()
    count = re.search(r'(\d+) passed', regression)
    passive_rows = [r for r in summary['runs'] if r['mode'] == 'passive_damping']
    plugin_hash = (BASE/'plugin.sha256').read_text().split()[0]
    checks = {'analysis_valid': summary['audit_passed'],
        'exact_five_registered_cases': set(modes) == {'flat_legacy','flat_reset_integral','flat_no_lag',
                                                     'flat_passive_damping','official_passive_damping'},
        'default_vehicle_byte_identical_to_previous': sha(old/'runtime_vehicle.sdf') == sha(new/'runtime_vehicle.sdf'),
        'only_four_expected_runtime_sources_changed': changed == expected,
        'prior_source_snapshot_preserved': sha(old/'source_snapshot.zip') ==
            read(BASE.parent/'braking_ab_v1/ab_analysis.json')['runs'][1]['input_sha256']['source_snapshot.zip'],
        'all_runs_source_stop_exit': all(all(v.values()) for v in modes.values()),
        'aligned_command_samples_equal': all_commands,
        'passive_no_positive_pre_power': len(passive_rows) == 2 and all(r['passive_invariant_passed'] for r in passive_rows),
        'regression_passed': bool(count) and ' failed' not in regression and 'ERROR' not in regression,
        'plugin_unchanged': sha(ROOT/'install/arena_gazebo/lib/libarena-single-motor-system.so') == plugin_hash}
    evidence = [BASE/'analysis.json', BASE/'README.md', BASE/'finalize.py', BASE/'preflight.log',
        BASE/'pytest_final.log', BASE/'pytest_final_v2.log', BASE/'build.log', BASE/'ctest.log', BASE/'plugin.sha256',
        ROOT/'scripts/analyze_zero_speed_modes.py', ROOT/'tests/test_zero_speed_modes.py',
        ROOT/'tests/test_zero_speed_analysis.py', ROOT/'config/tests/zero_speed_r0_sha256.json']
    result = {'checks': checks, 'audit_passed': all(checks.values()), 'runs': modes,
        'regression_count': int(count[1]) if count else None, 'plugin_sha256': plugin_hash,
        'evidence_sha256': {str(p.relative_to(ROOT)): sha(p) for p in evidence},
        'scope': '평가용 구동 모드 대조만 검증. 기본 모드 미채택, G1 재주행/실물 제동 미실시'}
    with (BASE/'final_audit.json').open('x', encoding='utf-8') as output:
        json.dump(result, output, ensure_ascii=False, indent=2)
    print(json.dumps(result['checks'], indent=2))
    raise SystemExit(0 if result['audit_passed'] else 1)


if __name__ == '__main__':
    main()
