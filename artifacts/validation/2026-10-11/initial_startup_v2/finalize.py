"""초기 배치 반복 확인과 미해결 적용 변동을 구분하여 증거를 마감한다."""
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
OLD = BASE.parent/'initial_startup_v1'
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
contract = json.loads((ROOT/'config/tests/scheduled_command_r6_sha256.json').read_text())
runtime = {**contract['sha256'], **contract['implementation_sha256'], **contract['evaluation_tools_sha256']}
preserved = {p:sha(ROOT/p) == h for p,h in runtime.items()}
preserved['plugin_binary'] = sha(ROOT/'install/arena_gazebo/lib/libarena-single-motor-system.so') == 'c48d4a99e42ba11cdf493439bf665495131aba89075c7e378a5c275597341f78'
previous = json.loads((OLD/'final_manifest.json').read_text())['source_evidence_sha256']
historical = {p:sha(ROOT/p) == h for p,h in previous.items()
              if p.startswith(OLD.relative_to(ROOT).as_posix()+'/')}
pair = json.loads((BASE/'analysis_pair.json').read_text())
status = json.loads((BASE/'status_e3_embedded.json').read_text())
raw = []
for name in ('wheel_contacts','braking_diagnostics','requested_commands','safe_commands',
             'actuation_status','output_status','evaluator_timing'):
    p = BASE/'e3_embedded'/(name+('.i64' if name == 'evaluator_timing' else '.jsonl'))
    rel = p.relative_to(ROOT).as_posix()
    raw.append({'path':rel,'exists':p.is_file(),
                'ignored':subprocess.run(['git','check-ignore','-q',rel],cwd=ROOT).returncode == 0,
                'tracked':bool(subprocess.check_output(['git','ls-files',rel],cwd=ROOT,text=True))})
pytest = (BASE/'pytest_analysis.log').read_text().splitlines()[-1]
files = list(runtime) + ['scripts/analyze_initial_repeat.py','tests/test_initial_repeat_analysis.py',
        'config/tests/scheduled_command_r6_sha256.json','docs/PROJECT_CONTEXT.md',
        'docs/activity/2026-10-11.md','docs/autonomy/LOCAL_FOLLOW_G1B_2026_10_07.md']
files += [p.relative_to(ROOT).as_posix() for p in sorted(BASE.iterdir())
          if p.is_file() and p.suffix in ('.json','.md','.py','.sh','.log')]
files = sorted(set(files))
with zipfile.ZipFile(BASE/'final_source.zip','x',zipfile.ZIP_DEFLATED) as z:
    for path in files: z.write(ROOT/path,path)
checks = {'runtime_unchanged': all(preserved.values()), 'prior_evidence_unchanged': bool(historical) and all(historical.values()),
          'new_run_clean': status['exit_code'] == 0 and status['cases'][0]['residual_after'] == [],
          'raw_policy': all(x['exists'] and x['ignored'] and not x['tracked'] for x in raw),
          'regression_passed': pytest.startswith('80 passed in '),
          'initialization_repeat': pair['startup_pair']['bounded_initialization_repeat_confirmed'],
          'comparison_valid': pair['application_comparison_valid']}
result = {'scope':'초기 배치 반복·증거 보존 감사. 제동 재현성 통과가 아니다.',
          'checks':checks,'passed':all(checks.values()), 'runtime_preserved':preserved,
          'historical_evidence_preserved':historical,'raw_policy':raw,'pytest':pytest,
          'command_application_identical': all(pair['applications'][c]['comparable'] and
             pair['applications'][c]['differing'] == 0 for c in ('speed','steering')),
          'physics_metrics_within_tolerance':pair['metrics_within_tolerance'],
          'source_evidence_sha256':{p:sha(ROOT/p) for p in files}}
with (BASE/'final_manifest.json').open('x',encoding='utf-8') as f: json.dump(result,f,ensure_ascii=False,indent=2)
print(json.dumps({k:result[k] for k in ('passed','checks','pytest','command_application_identical',
                                     'physics_metrics_within_tolerance')},ensure_ascii=False,indent=2))
