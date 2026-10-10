"""최종 증거/원시 제외/기준 보존 감사. 과거 파일을 덮어쓰지 않는다."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'scripts'))
from run_probe_batch import residual_processes

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()

parent = json.loads((ROOT/'config/tests/scheduled_command_r4_sha256.json').read_text())
changed = {'src/arena_bringup/launch/simulation.launch.py', 'scripts/probe_braking_contact.py'}
preserved = {p:sha(ROOT/p) == h for p,h in {**parent['sha256'], **parent['implementation_sha256'],
                                         **parent['evaluation_tools_sha256']}.items() if p not in changed}
preserved['plugin_binary'] = sha(ROOT/'install/arena_gazebo/lib/libarena-single-motor-system.so') == 'c48d4a99e42ba11cdf493439bf665495131aba89075c7e378a5c275597341f78'
baseline = json.loads((ROOT/'artifacts/validation/2026-10-11/evaluator_gc_v1/b2_defer/report.json').read_text())
preserved.update({p:sha(ROOT/p) == h for p,h in baseline['source_sha256'].items() if p not in changed})
raw = []
for case in ('s1_service', 'e1_embedded', 'e2_embedded'):
    for name in ('wheel_contacts', 'braking_diagnostics', 'requested_commands', 'safe_commands', 'actuation_status', 'output_status', 'evaluator_timing'):
        p = BASE/case/(name+('.i64' if name == 'evaluator_timing' else '.jsonl'))
        rel = p.relative_to(ROOT).as_posix()
        raw.append({'path':rel, 'exists':p.is_file(),
                    'ignored':subprocess.run(['git','check-ignore','-q',rel],cwd=ROOT).returncode == 0,
                    'tracked':bool(subprocess.check_output(['git','ls-files',rel],cwd=ROOT,text=True))})
analysis = json.loads((BASE/'analysis_all.json').read_text())
pytest_line = (BASE/'pytest_finaltests_v2.log').read_text().splitlines()[-1]
residual = residual_processes()
files = [ROOT/'AGENTS.md', ROOT/'.gitignore', ROOT/'scripts/evaluation_startup.py', ROOT/'scripts/analyze_initial_startup.py',
         ROOT/'scripts/probe_braking_contact.py', ROOT/'src/arena_bringup/launch/simulation.launch.py',
         ROOT/'tests/test_evaluation_startup.py', ROOT/'tests/test_initial_startup_analysis.py',
         ROOT/'tests/test_physics_step_option.py', ROOT/'tests/test_scheduled_command.py',
         ROOT/'tests/test_r0_safety_contract.py', ROOT/'tests/test_multi_vehicle_g0.py',
         ROOT/'config/tests/scheduled_command_r5_sha256.json', ROOT/'docs/PROJECT_CONTEXT.md',
         ROOT/'config/tests/scheduled_command_r6_sha256.json',
         ROOT/'docs/activity/2026-10-11.md', ROOT/'docs/autonomy/LOCAL_FOLLOW_G1B_2026_10_07.md']
files += [p for p in sorted(BASE.glob('*')) if p.is_file() and p.suffix in ('.json','.md','.py','.sh','.log')]
with zipfile.ZipFile(BASE/'final_source.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files: z.write(p,p.relative_to(ROOT).as_posix())
result = {'source_evidence_sha256':{p.relative_to(ROOT).as_posix():sha(p) for p in files},
          'runtime_preserved':preserved, 'raw_policy':raw, 'residual':residual, 'pytest':pytest_line,
          'planned_repetition_acceptance':analysis['comparison']['passed'],
          'single_corrected_run_confirmed':analysis['bounded_confirmation']['single_corrected_run_confirmed'],
          'failed_evidence_verified':analysis['failed_attempt']['evidence_verified']}
result['passed'] = (all(preserved.values()) and all(r['exists'] and r['ignored'] and not r['tracked'] for r in raw)
                    and not residual and bool(re.fullmatch(r'\d+ passed in .+',pytest_line))
                    and result['single_corrected_run_confirmed'] and result['failed_evidence_verified'])
result['scope'] = '증거 보존·단회 기능 확인 감사. 원래 두 반복 수용 조건의 통과가 아니다.'
with (BASE/'final_manifest.json').open('x',encoding='utf-8') as f: json.dump(result,f,ensure_ascii=False,indent=2)
print(json.dumps({k:result[k] for k in ('passed','planned_repetition_acceptance','single_corrected_run_confirmed','pytest','residual')},indent=2))
