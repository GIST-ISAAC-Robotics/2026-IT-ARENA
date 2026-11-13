"""실행 후 Windows 해시 봉인. WSL/장치 호출 없음."""
from pathlib import Path
import ast,hashlib,json

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[3]
EXEC=HERE/'execution_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((EXEC/'manifest.json').read_text())
source=HERE.parent/'solo_full_lap_r457_v1'
copied_changes={m:[n for n,h in manifest['files'][m].items() if sha(EXEC/m/n)!=h]
                for m in ('baseline','candidate')}
assert not any(copied_changes.values())
baseline=json.loads((EXEC/'baseline_result/summary.json').read_text())
candidate=json.loads((EXEC/'candidate_result/summary.json').read_text())
assert baseline['bit_exact']==405 and candidate['bit_exact']==402
assert baseline['input_digest_before_after']==candidate['input_digest_before_after']
assert not candidate['positive_to_zero']
assert candidate['zero_to_positive']==[643,644]
receipt=json.loads((EXEC/'bundle_receipt_v3.json').read_text())
assert receipt['success'] and all(s['exit_code']==0 and not s['pid_exists_after_wait'] for s in receipt['steps'])
assert not receipt['after']['matching_r464']
assert (EXEC/'bundle_v3.exit').read_text().strip()=='0'
rel='src/arena_autonomy/arena_autonomy/local_follow.py'
changes=json.loads((HERE/'changes.json').read_text())
assert sha(HERE/'stage'/rel)==changes['source_changes'][0]['stage_sha256']
assert sha(REPO/rel)==changes['source_changes'][0]['original_sha256']
assert sha(source/'sources.zip')==manifest['source_archive_sha256']
assert sha(source/'run1/follow_decisions.jsonl')==manifest['trace_sha256']
for f in (HERE/'stage').rglob('*.py'):ast.parse(f.read_text(encoding='utf-8'))
changes.update(tests_executed=True,tests_passed=9,tests_evidence='8 passed in initial runs + corrected fixture 1 passed; failures preserved',
               native_replay_executed=True,native_baseline_bit_exact=405,native_candidate_bit_exact=402,
               drive_executed=False,execution_result='RESULT.md',common_modified=False)
(HERE/'changes.json').write_text(json.dumps(changes,indent=2),encoding='utf-8')
result={'source_archive_unchanged':True,'trace_unchanged':True,'common_local_follow_unchanged':True,
        'copied_source_changes':copied_changes,'stage_source_sha256':sha(HERE/'stage'/rel),
        'patch_sha256':sha(HERE/'changes.patch'),'native_baseline_bit_exact':405,'native_candidate_bit_exact':402,
        'input_digest_equal':True,'positive_to_zero':0,'zero_to_positive':[643,644],
        'stage_ast_passed':True,'owned_children_exited':True,'owned_residual_audit_count':0,
        'carrier_shell_exit':0,'WSL_returned_to_parent':True,'Linux_hang_audit_retroactively_changed':False}
selected=['changes.patch','changes.json','RESULT.md','stage/tests/test_boundary_transient_r464.py',
    'execution_v1/manifest.json','execution_v1/bundle_receipt_v3.json','execution_v1/baseline_result/summary.json',
    'execution_v1/candidate_result/summary.json','execution_v1/comparison_summary.json']
result['evidence_sha256']={p:sha(HERE/p) for p in selected}
(HERE/'final_integrity.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k!='evidence_sha256'},indent=2))
