#!/usr/bin/env python3
"""step 민감도 마감 감사. 원시 판정과 분석 결과를 바꾸지 않고 유효성·보존 상태만 모은다."""
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
RUNS = ['flat_passive_0p5ms', 'flat_passive_0p25ms', 'official_passive_0p5ms', 'official_passive_0p25ms',
        'flat_legacy_0p25ms', 'flat_legacy_0p25ms_retry1', 'official_passive_1ms_rerun', 'official_passive_0p25ms_repeat']
RAW = ['braking_diagnostics.jsonl', 'wheel_contacts.jsonl', 'requested_commands.jsonl', 'safe_commands.jsonl']
ANALYSES = ['analysis_passive', 'analysis_legacy_aux', 'analysis_official_rerun_ref', 'analysis_official_repeat025',
            'analysis_flat_legacy_failed_shutdown']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True)


runs = {}
for name in RUNS:
    run = BASE/name
    report = json.loads((run/'report.json').read_text())
    with zipfile.ZipFile(run/'source_snapshot.zip') as bundle:
        zip_ok = all(hashlib.sha256(bundle.read(k)).hexdigest() == v for k, v in report['source_sha256'].items())
    raw_paths = [str((run/f).relative_to(ROOT)) for f in RAW]
    ignored = git('check-ignore', *raw_paths).stdout.split()
    tracked = git('ls-files', *raw_paths).stdout.split()
    runs[name] = {
        'passed': report['passed'], 'input_valid': report['input_valid'], 'stopped': report['stopped'],
        'stop_acknowledged': report.get('stop_acknowledged'), 'final_reason': report.get('final_mcu', {}).get('reason'),
        'shutdown_clean': report['shutdown_clean'], 'remaining_pids': report['remaining_pids'],
        'source_unchanged_during_run': report['source_unchanged'], 'source_zip_matches': zip_ok,
        'raw_hashes_match': all(sha(run/f) == report[f[:-6]]['sha256'] for f in RAW),
        'raw_ignored': len(ignored) == len(RAW), 'raw_tracked': len(tracked),
        'physics_step_override_s': report.get('physics_step_override_s'), 'zero_speed_mode': report['zero_speed_mode'],
        'surface': report['surface'], 'error': report.get('error'),
        'shutdown_errors': report['shutdown_audit'].get('errors')}
analysis_hash = sha(ROOT/'scripts/analyze_physics_step.py')
analyses = {}
for name in ANALYSES:
    data = json.loads((BASE/f'{name}.json').read_text(encoding='utf-8'))
    analyses[name] = {'audit_passed': data['audit_passed'], 'analysis_sha256_current': data['analysis_sha256'] == analysis_hash,
                      'contract_sha256_current': data['contract_sha256'] == sha(ROOT/'config/tests/physics_step_r0_sha256.json'),
                      'classifications': {k: v['classification'] for k, v in data['classifications'].items()}}
freeze = {}
for line in (BASE/'pre_run_freeze.sha256').read_text().splitlines():
    value, path = line.split(' *', 1)
    freeze[path] = sha(ROOT/path) == value
expected_changed = {'artifacts/validation/2026-10-08/step_sensitivity_v1/README.md'}
checks = {
    'eight_new_gazebo_runs': len(RUNS) == 8,
    'seven_runs_valid_one_shutdown_failure_preserved': [k for k, v in runs.items() if not v['passed']] == ['flat_legacy_0p25ms'],
    'failed_run_dynamics_inputs_valid': runs['flat_legacy_0p25ms']['input_valid'] and runs['flat_legacy_0p25ms']['stopped'],
    'all_stop_acknowledged_software_stop': all(v['stop_acknowledged'] and v['final_reason'] == 'software_stop' for v in runs.values()),
    'no_remaining_pids': all(not v['remaining_pids'] for v in runs.values()),
    'source_zip_and_raw_hashes': all(v['source_zip_matches'] and v['raw_hashes_match'] for v in runs.values()),
    'raw_ignored_not_tracked': all(v['raw_ignored'] and v['raw_tracked'] == 0 for v in runs.values()),
    'runtime_sources_frozen_since_registration': {p for p, ok in freeze.items() if not ok} == expected_changed,
    'valid_analyses_audited': all(analyses[n]['audit_passed'] for n in ANALYSES if n != 'analysis_flat_legacy_failed_shutdown'),
    'analyses_use_current_script_and_contract': all(a['analysis_sha256_current'] and a['contract_sha256_current'] for a in analyses.values()),
    'plugin_unchanged': (BASE/'plugin_after.txt').read_text().strip().endswith(': OK'),
    'regression_passed': 'passed' in (BASE/'pytest_final.log').read_text() and 'failed' not in (BASE/'pytest_final.log').read_text(),
    'g1_comparison_not_run_by_gate': not any((BASE/n).exists() for n in ('g1_side_left_legacy', 'g1_side_left_passive')),
}
result = {'checks': checks, 'audit_passed': all(checks.values()), 'runs': runs, 'analyses': analyses,
          'freeze': freeze, 'analysis_sha256': analysis_hash,
          'scope': '평가 전용 step 민감도 보존 감사. 기본 모델 채택·실물 제동·G1 성공 판정이 아니다.'}
with (BASE/'final_audit.json').open('x', encoding='utf-8') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2)
print(json.dumps({'audit_passed': result['audit_passed'], 'checks': checks}, ensure_ascii=False, indent=2))
