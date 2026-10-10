"""실행/분석 마감 증거. 원본·보호·구동/센서 보존과 raw Git 제외를 검사한다."""
import hashlib
import json
import subprocess
import zipfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[4]
BASE=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
registered=json.loads((BASE/'registered_manifest.json').read_text())
c=json.loads((ROOT/'config/tests/scheduled_command_r7_sha256.json').read_text())
allowed={**c['approved_changes'],**c['approved_evaluation_scripts'],**c['approved_implementation_changes']}
previous=json.loads((ROOT/'artifacts/validation/2026-10-11/initial_startup_v2/e3_embedded/report.json').read_text())
preserved={p:sha(ROOT/p)==value for p,value in previous['source_sha256'].items() if p not in allowed}
runtime={p:sha(ROOT/p)==value for p,value in registered['sources'].items()}
raw=[]
for run in (BASE/'n1_native',BASE/'t1_scheduled',BASE/'t2_scheduled'):
    for p in run.glob('*'):
        if p.suffix=='.i64' or p.name in ('wheel_contacts.jsonl','braking_diagnostics.jsonl',
            'requested_commands.jsonl','safe_commands.jsonl','actuation_status.jsonl','output_status.jsonl',
            'scheduled_sent.jsonl','scheduled_events.jsonl','scheduled_states.jsonl'):
            ignored=subprocess.run(['git','check-ignore','--quiet',str(p)],cwd=ROOT).returncode==0
            tracked=bool(subprocess.check_output(['git','ls-files','--',str(p)],cwd=ROOT,text=True).strip())
            raw.append({'path':p.relative_to(ROOT).as_posix(),'sha256':sha(p),'ignored':ignored,'tracked':tracked})
docs=['docs/PROJECT_CONTEXT.md','docs/activity/2026-10-11.md','docs/autonomy/LOCAL_FOLLOW_G1B_2026_10_07.md']
files=set(registered['sources'])|set(docs)|{
    'scripts/analyze_scheduled_apply.py','tests/test_scheduled_apply.py','tests/test_scheduled_apply_analysis.py',
    'tests/test_scheduled_command.py','tests/test_physics_step_option.py','tests/test_multi_vehicle_g0.py',
    'tests/test_r0_safety_contract.py','config/tests/scheduled_command_r7_sha256.json'}
with zipfile.ZipFile(BASE/'final_source.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(files):z.write(ROOT/p,p)
checks={'registered_sources_unchanged':all(runtime.values()),'protected_previous_sources_unchanged':all(preserved.values()),
        'plugin_unchanged':sha(ROOT/'install/arena_gazebo/lib/libarena-single-motor-system.so')==registered['plugin_sha256'],
        'raw_ignored_untracked':bool(raw) and all(r['ignored'] and not r['tracked'] for r in raw),
        'original_contract_unchanged':sha(ROOT/c['parent_contract'])==c['parent_contract_sha256']}
result={'checks':checks,'passed':all(checks.values()),'registered_source_checks':runtime,
        'preserved_previous_sources':preserved,'raw':raw,'final_source_sha256':sha(BASE/'final_source.zip'),
        'final_files':{p:sha(ROOT/p) for p in sorted(files)}}
with (BASE/'final_manifest.json').open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2)
print(json.dumps(checks,indent=2));assert all(checks.values())
