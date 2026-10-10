#!/usr/bin/env python3
"""실행을 다시 하지 않는 최종 증거 감사. 기존 로그/판정은 수정하지 않는다."""
import ast
import hashlib
import json
from pathlib import Path
import re
import subprocess
import zipfile

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
RAW = ('braking_diagnostics', 'wheel_contacts', 'requested_commands', 'safe_commands',
       'actuation_status', 'output_status')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def command(*args):
    return subprocess.run(args, cwd=ROOT, capture_output=True, text=True)


def profile_block(tree, old):
    for node in ast.walk(tree):
        if not isinstance(node, ast.If) or not isinstance(node.test, ast.Name) or node.test.id != 'profile':
            continue
        if old and any(isinstance(n, ast.AugAssign) for n in node.body):
            return node.body
        if not old and isinstance(node.body[0], ast.If) and isinstance(node.body[0].test, ast.Name) \
                and node.body[0].test.id == 'grid_spec':
            return node.body[0].orelse + node.body[1:]
    raise AssertionError('profile validity block not found')


def recover_v1_analyzer(current, expected):
    """최종 검토의 작은 diff를 역적용하되, 기존 분석 SHA와 정확히 같을 때만 보존한다."""
    old = current.replace("'request': 0, 'request_observed': 0, 'safe': 0", "'request': 0, 'safe': 0")
    begin, end = old.index('def execution_passed('), old.index('def analyze_run(')
    old = old[:begin] + old[end:]
    old = old.replace("validity['passed'] = execution_passed(validity)",
                      "validity['passed'] = all(v for k, v in validity.items() if k != 'raw_and_source_checks')")
    old = old.replace("('request', 'request_observed', 'safe', 'mcu', 'output')", "('request', 'safe', 'mcu', 'output')")
    old = old.replace("if stage == 'output' and quantity == 'steering':  # 출력 상태에는 조향각이 없다.",
                      'if stage not in stages_a and stage not in stages_b:')
    old = old.replace('if (not stages_a.get(stage) or not stages_b.get(stage)\n'
                      '                    or len(stages_a[stage]) != len(stages_b[stage])):',
                      'if stages_a.get(stage) is None or stages_b.get(stage) is None:')
    old = old.replace('if holds_a and holds_b and len(holds_a) == len(holds_b):',
                      'if holds_a is not None and holds_b is not None:')
    old = old.replace("        else:\n            out[f'{quantity}_mcu_hold'] = {'max_abs_diff_ns': 'unknown', 'within': False}\n"
                      '            comparable = False\n', '')
    for newline in ('\n', '\r\n'):
        candidate = old.replace('\n', newline).encode('utf-8')
        if digest(candidate) == expected:
            return candidate
    raise AssertionError('v1 source reconstruction does not match recorded SHA; do not archive')


def main():
    destination = BASE/'final_review_v2.json'
    archive = BASE/'review_source_v2.zip'
    assert not destination.exists() and not archive.exists()
    previous = json.loads((BASE/'analysis_v1.json').read_text())
    latest = json.loads((BASE/'analysis_v2.json').read_text())
    checks, runs = {}, {}
    checks['metrics_unchanged'] = all(previous['runs'][k]['metrics'] == v['metrics'] for k, v in latest['runs'].items())
    checks['identification_unchanged'] = previous['identification'] == latest['identification']
    checks['comparison_verdicts_unchanged'] = all(all(a[k] == b[k] for k in
        ('valid', 'delivery_comparable', 'metrics_within_tolerance')) for a, b in zip(previous['comparisons'], latest['comparisons']))
    raw_paths = []
    for name, run in latest['runs'].items():
        folder = ROOT/run['path']
        report = json.loads((folder/'report.json').read_text())
        raw = {n: sha(folder/(n+'.jsonl')) == report[n]['sha256'] for n in RAW}
        raw_paths.extend(str((folder/(n+'.jsonl')).relative_to(ROOT)) for n in RAW)
        with zipfile.ZipFile(folder/'source_snapshot.zip') as bundle:
            sources = {p: digest(bundle.read(p)) == value for p, value in report['source_sha256'].items()}
        changed = [p for p, value in report['source_sha256'].items() if sha(ROOT/p) != value]
        runs[name] = {'raw_hashes': raw, 'source_zip_matches': all(sources.values()),
                      'source_files': len(sources), 'current_source_differences': changed,
                      'original_report_passed': report['passed'], 'execution_valid': run['validity']['passed'],
                      'schedule_valid': run['schedule']['passed'],
                      'registered_experiment_valid': run['validity']['passed'] and run['schedule']['passed']}
        checks[name+'_evidence_preserved'] = (all(raw.values()) and all(sources.values())
            and changed == ['scripts/probe_braking_contact.py'] and report['stopped'] and report['shutdown_clean']
            and report['final_mcu']['reason'] == 'software_stop' and not report['remaining_pids'])
    checks['raw_ignored'] = command('git', 'check-ignore', '--', *raw_paths).stdout.splitlines() == raw_paths
    checks['raw_untracked'] = not command('git', 'ls-files', '--', *raw_paths).stdout.strip()
    frozen = {}
    for line in (BASE/'pre_run_freeze.sha256').read_text().splitlines():
        expected, path = line.split(' ', 1)
        path = path.strip().lstrip('*')
        frozen[path] = sha(ROOT/path) == expected
    expected_changed = {'scripts/probe_braking_contact.py', str((BASE/'README.md').relative_to(ROOT))}
    checks['frozen_changes_expected'] = {p for p, ok in frozen.items() if not ok} == expected_changed
    expected_plugin, plugin = (BASE/'plugin_before.txt').read_text().strip().split(None, 1)
    checks['plugin_binary_unchanged'] = sha(ROOT/plugin) == expected_plugin
    with zipfile.ZipFile(BASE/'o1ms_p10_a/source_snapshot.zip') as bundle:
        old_probe = ast.parse(bundle.read('scripts/probe_braking_contact.py'))
    new_probe = ast.parse((ROOT/'scripts/probe_braking_contact.py').read_text())
    checks['poll_validity_statements_unchanged'] = ast.dump(ast.Module(body=profile_block(old_probe, True), type_ignores=[])) \
        == ast.dump(ast.Module(body=profile_block(new_probe, False), type_ignores=[]))
    processes = command('ps', '-eo', 'pid,args').stdout.splitlines()
    residual = [line for line in processes if re.search(
        r'gz sim|probe_braking_contact|parameter_bridge|rotating_lidar|actuation_ros|lidar_safety|run_batch_r2_r6', line)]
    checks['no_residual_simulation_process'] = not residual
    checks['diff_check'] = command('git', 'diff', '--check').returncode == 0
    checks['review_tests_pass'] = '190 passed' in (BASE/'unit_review_v2.log').read_text()
    checks['prior_full_tests_pass'] = '1176 passed' in (BASE/'pytest_final_v1.log').read_text()
    source_files = ['scripts/analyze_scheduled_phase.py', 'scripts/probe_braking_contact.py',
                    'scripts/scheduled_command.py', 'tests/test_scheduled_command.py',
                    'tests/test_scheduled_phase_analysis.py', 'config/tests/scheduled_command_r0_sha256.json',
                    'config/tests/scheduled_command_r1_sha256.json', str(Path(__file__).relative_to(ROOT))]
    recovered = recover_v1_analyzer((ROOT/source_files[0]).read_text(), previous['analysis_sha256'])
    with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in source_files:
            bundle.write(ROOT/path, path)
        bundle.writestr('before_review/scripts/analyze_scheduled_phase.py', recovered)
    artifact_hashes = {str(p.relative_to(ROOT)): sha(p) for p in sorted(BASE.rglob('*'))
                       if p.is_file() and p.suffix != '.jsonl' and p.name != destination.name}
    output = {'scope': '최종 증거/분석 검사이며 주행 통과 판정이 아니다. Gazebo 추가 실행 없음.',
              'checks': checks, 'passed': all(checks.values()), 'runs': runs, 'residual_processes': residual,
              'pre_run_freeze_matches': frozen, 'source_sha256': {p: sha(ROOT/p) for p in source_files},
              'preserved_v1_analysis_source_sha256': digest(recovered), 'artifact_sha256': artifact_hashes}
    with destination.open('x', encoding='utf-8') as stream:
        json.dump(output, stream, ensure_ascii=False, indent=2)
    print(json.dumps({'checks': checks, 'passed': output['passed'],
                      'registered_valid_runs': [n for n, r in runs.items() if r['registered_experiment_valid']],
                      'artifact_count': len(artifact_hashes)}, ensure_ascii=False, indent=2))
    return 0 if output['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
