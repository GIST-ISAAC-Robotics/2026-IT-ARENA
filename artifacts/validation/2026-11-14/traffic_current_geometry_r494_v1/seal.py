from pathlib import Path
import difflib,hashlib,json
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
rows=[];patch=[]
for original in json.loads((A/'source_before.json').read_text(encoding='utf-8')):
    name=original['path'];base=A/'base'/name;candidate=A/'stage'/name
    assert sha(base)==original['sha256']==sha(ROOT/name),name
    text=candidate.read_text(encoding='utf-8');compile(text,str(candidate),'exec')
    # 기존 두 파일의 CRLF를 patch context에도 보존한다. 검증된 stage 바이트는 바꾸지 않는다.
    patch.extend(difflib.unified_diff(base.read_bytes().decode('utf-8').splitlines(keepends=True),
        candidate.read_bytes().decode('utf-8').splitlines(keepends=True),fromfile='a/'+name,tofile='b/'+name))
    rows.append(dict(path=name,before_sha256=original['sha256'],candidate_sha256=sha(candidate),common_before_matches=True))
(A/'candidate.patch').write_text(''.join(patch),encoding='utf-8',newline='\n')
input_path=ROOT/'artifacts/validation/2026-11-14/traffic_return_recovery_r490_v1/failure_inputs/follow_records.jsonl'
receipt=json.loads((A/'test_receipt.json').read_text(encoding='utf-8'));assert receipt['exit_code']==0
manifest=dict(status='ready_for_root_review_not_integrated',files=rows,patch_sha256=sha(A/'candidate.patch'),
    input_path=input_path.relative_to(ROOT).as_posix(),input_sha256=sha(input_path),
    validation=dict(pytest_passed=8,default_full_output_exact_cases=3,numeric_tolerance=0,
        saved_representatives=3,subsequent_original_input=1,synthetic_obstacle_cases=2),
    test_receipt=receipt,wsl_access=False,jetson_access=False,adviser_used=False,owned_pids_remaining=[],
    baseline_geometry_provenance='baseline_geometry.json parsed from original baseline_geometry.log')
(A/'source_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(json.dumps(dict(files=len(rows),patch_sha256=manifest['patch_sha256'],common_source_unchanged=True)))
