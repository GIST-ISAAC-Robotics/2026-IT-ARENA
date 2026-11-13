"""저장 비교 종료 후 읽기 전용 무결성/소유 PID 확인."""
from pathlib import Path
import hashlib
import json
import os
import zipfile

bundle=Path(__file__).resolve().parent;repo=bundle.parents[3]
root=Path('/home/jinhyeong/it-arena-r438-crop-a')
manifest=json.loads((bundle/'preparation.json').read_bytes())
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
checks={
    'r437_archive':digest(bundle.parent/'applied_limit_drive_r437_v1/sources.zip')==manifest['source_archive_sha256'],
    'r437_trace':digest(bundle.parent/'applied_limit_drive_r437_v1/run1/follow_decisions.jsonl')==manifest['trace_sha256'],
    'r426_currents':digest(bundle.parent/'crop_current_reach_r426_v1/inputs/currents.jsonl')==manifest['r426_currents_sha256'],
    'r426_expected':digest(bundle.parent/'crop_current_reach_r426_v1/verify_v1/conditional-full.zip')==manifest['r426_expected_archive_sha256'],
    'stage_copy_exact':digest(root/'stage/fixed_target_candidate.py')==digest(bundle/'stage/fixed_target_candidate.py'),
    'stage_prepared_lf_text_exact':hashlib.sha256((root/'stage/fixed_target_candidate.py').read_bytes().replace(b'\r\n',b'\n')).hexdigest()==manifest['stage_sha256'],
    'analyzer_exact':digest(root/'analyze_saved.py')==digest(bundle/'analyze_saved.py'),
    'inputs_exact':all(digest(root/'inputs'/name)==wanted==digest(bundle/'inputs'/name) for name,wanted in manifest['inputs'].items()),
    'frozen_packages_exact':all(digest(root/name)==wanted for name,wanted in manifest['frozen_package_files'].items()),
}
shared_changes=[name for name,wanted in manifest['frozen_package_files'].items() if digest(repo/name)!=wanted]
assert set(shared_changes)=={'src/arena_autonomy/arena_autonomy/race_ops_node.py','src/arena_autonomy/arena_autonomy/race_ops_page.py'}
checks['shared_core_r437_exact_except_known_r434_ui']=True
owned=[]
for entry in Path('/proc').glob('[0-9]*'):
    if int(entry.name)==os.getpid():continue
    try:command=(entry/'cmdline').read_bytes().replace(b'\x00',b' ').decode(errors='replace')
    except OSError:continue
    if 'it-arena-r438-crop-' in command or 'analyze_saved.py' in command:owned.append(dict(pid=int(entry.name),command=command))
result=dict(checks=checks,shared_differences_from_r437=shared_changes,owned_processes=owned,passed=all(checks.values()) and not owned)
(bundle/'handoff_audit.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result))
