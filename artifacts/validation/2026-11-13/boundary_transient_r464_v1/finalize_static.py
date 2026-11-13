from pathlib import Path
import ast
import hashlib
import json

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[3]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((HERE/'original_manifest.json').read_text())
original_changes=[name for name,value in manifest['files'].items() if sha(HERE/'original'/name)!=value]
parsed=[]
for path in sorted((HERE/'stage').rglob('*.py')):
    ast.parse(path.read_text(encoding='utf-8'),filename=str(path))
    parsed.append(str(path.relative_to(HERE)))
for name in ('inspect_saved.py','inspect_shape.py','geometry_diagnostic.py','prepare_stage.py','run_focused_tests.py'):
    ast.parse((HERE/name).read_text(encoding='utf-8'),filename=name)
    parsed.append(name)
source=HERE.parent/'solo_full_lap_r457_v1'
expected=json.loads((source/'analysis_v1/original_evidence_sha256.json').read_text())
trace_sha=sha(source/'run1/follow_decisions.jsonl')
assert trace_sha==expected['run1/follow_decisions.jsonl']
rel='src/arena_autonomy/arena_autonomy/local_follow.py'
assert sha(REPO/rel)==manifest['files'][rel]
assert not original_changes
result={'source_archive_sha256':sha(source/'sources.zip'),
        'source_archive_unchanged':sha(source/'sources.zip')==manifest['archive_sha256'],
        'trace_sha256':trace_sha,'trace_unchanged':True,
        'copied_source_changes':original_changes,'common_local_follow_unchanged':True,
        'stage_python_ast_passed':parsed,'prepared_pytest_cases':9,'pytest_executed':False,
        'native_evaluator_replay_executed':False,'new_drive':False,'WSL_used':False,'Jetson_used':False,
        'owned_background_processes_created':0,
        'changes_patch_sha256':sha(HERE/'changes.patch')}
(HERE/'static_review.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
