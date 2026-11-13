from pathlib import Path
import difflib
import hashlib
import json

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
rows=[];patch=[]
for path in sorted((A/'stage').rglob('*.py')):
    relative=path.relative_to(A/'stage');before=A/'base'/relative;common=ROOT/relative
    old=before.read_text(encoding='utf-8') if before.exists() else ''
    new=path.read_text(encoding='utf-8')
    match=(sha(common)==sha(before)) if before.exists() else not common.exists()
    assert match,relative
    rows.append(dict(path=relative.as_posix(),before_sha256=sha(before) if before.exists() else None,
        candidate_sha256=sha(path),common_before_matches=match))
    patch.extend(difflib.unified_diff(old.splitlines(keepends=True),new.splitlines(keepends=True),
        fromfile='a/'+relative.as_posix() if before.exists() else '/dev/null',tofile='b/'+relative.as_posix()))
    compile(new,str(path),'exec')
(A/'candidate.patch').write_text(''.join(patch),encoding='utf-8',newline='\n')
manifest=dict(status='ready_for_root_review_not_integrated',source_files=rows,
    common_source_unchanged=True,owned_running_pids=[],wsl_access=False,jetson_access=False,
    validation=dict(pytest_passed=33,subtests_passed=8,default_comparison_steps=15,
        default_comparison='exact except surface wall-time diagnostic; numeric tolerance 0',
        original_saved_cases=4,full_guard_replay=False),patch_sha256=sha(A/'candidate.patch'))
(A/'source_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(json.dumps(dict(files=len(rows),patch_sha256=manifest['patch_sha256'],common_source_unchanged=True)))
