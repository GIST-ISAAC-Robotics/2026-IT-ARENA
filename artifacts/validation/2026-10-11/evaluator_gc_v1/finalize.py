"""실행 후 근거 동결·불변/원시 제외/잔존 검사. 결과와 과거 증거를 수정하지 않는다."""
import json
from pathlib import Path
import re
import subprocess
import sys
import zipfile
from prepare import BASE, ROOT, sha, write

sys.path.insert(0, str(ROOT/'scripts'))
from run_probe_batch import residual_processes

parent = json.loads((ROOT/'config/tests/scheduled_command_r2_sha256.json').read_text())
unchanged = {**parent['sha256'], **parent['implementation_sha256']}
preservation = {name: sha(ROOT/name) == value for name, value in unchanged.items()}
preservation['plugin_binary'] = sha(ROOT/'install/arena_gazebo/lib/libarena-single-motor-system.so') == (
    'c48d4a99e42ba11cdf493439bf665495131aba89075c7e378a5c275597341f78')
raw = []
names = ('wheel_contacts.jsonl', 'braking_diagnostics.jsonl', 'requested_commands.jsonl', 'safe_commands.jsonl',
         'actuation_status.jsonl', 'output_status.jsonl', 'evaluator_timing.i64')
for run in ('a1_native', 'b1_defer', 'a2_native', 'b2_defer'):
    for name in names:
        p = BASE/run/name
        relative = p.relative_to(ROOT).as_posix()
        raw.append({'path': relative, 'exists': p.is_file(),
                    'ignored': subprocess.run(['git', 'check-ignore', '-q', relative], cwd=ROOT).returncode == 0,
                    'tracked': bool(subprocess.check_output(['git', 'ls-files', relative], cwd=ROOT, text=True))})
files = [ROOT/'scripts/evaluation_gc.py', ROOT/'scripts/probe_braking_contact.py',
         ROOT/'scripts/analyze_evaluator_gc.py', ROOT/'tests/test_evaluation_gc.py',
         ROOT/'tests/test_evaluation_gc_analysis.py', ROOT/'tests/test_physics_step_option.py',
         ROOT/'tests/test_scheduled_command.py',
         ROOT/'config/tests/scheduled_command_r3_sha256.json', ROOT/'config/tests/scheduled_command_r4_sha256.json']
files += sorted(BASE.glob('*.json'))+sorted(BASE.glob('*.md'))
files += [BASE/'pytest_final.log', BASE/'pytest_final_v2.log', BASE/'run.sh', BASE/'final_checks.sh', BASE/'finalize.py']
with zipfile.ZipFile(BASE/'final_source.zip', 'x', zipfile.ZIP_DEFLATED) as z:
    for p in files:
        z.write(p, p.relative_to(ROOT).as_posix())
residual = residual_processes()
pytest_last = (BASE/'pytest_final_v2.log').read_text().splitlines()[-1]
tests_passed = bool(re.fullmatch(r'\d+ passed in .+', pytest_last))
paired = json.loads((BASE/'analysis_ab2.json').read_text())['comparison']['accepted_bounded_comparison']
result = {'source_and_evidence_sha256': {p.relative_to(ROOT).as_posix(): sha(p) for p in files},
          'runtime_baseline_unchanged': preservation, 'raw_policy': raw, 'residual_processes': residual,
          'pytest': pytest_last, 'tests_passed': tests_passed, 'corrected_pair_accepted': paired,
          'passed': tests_passed and paired and all(preservation.values()) and not residual
                    and all(r['exists'] and r['ignored'] and not r['tracked'] for r in raw)}
write(BASE/'final_manifest.json', result)
print(json.dumps({'preservation_ok': all(preservation.values()), 'raw_files': len(raw),
                  'residual': residual, 'pytest': result['pytest'], 'passed': result['passed']}, indent=2))
