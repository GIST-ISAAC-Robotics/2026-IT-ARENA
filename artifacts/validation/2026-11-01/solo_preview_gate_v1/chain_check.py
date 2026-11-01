"""기존 실패 test_corner_follow가 새 손상이 아니라 시험 목록 미연장인지 해시 사슬로 확인한다."""
import importlib.util
from hashlib import sha256
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'tests'))
spec=importlib.util.spec_from_file_location('provenance',ROOT/'tests/test_source_provenance_r0_plus_b3.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
STOP='config/tests/fresh_confirmation_r2_sha256.json'  # test_corner_follow 자체 목록의 끝
if STOP not in module.M3_SUCCESSORS:raise ValueError('chain end moved')
newer=module.M3_SUCCESSORS[module.M3_SUCCESSORS.index(STOP)+1:]
base=json.loads((ROOT/'config/tests/corner_anchor_r0_sha256.json').read_text(encoding='utf-8'))
rows={}
for group in ('approved_changes','approved_evaluation_scripts'):
    for path in base[group]:
        current=sha256((ROOT/path).read_bytes()).hexdigest()
        walked=module.m3_revision_digest(path,newer_than=STOP)
        later=[n for n in newer if path in {**json.loads((ROOT/n).read_text(encoding='utf-8'))['approved_changes'],**json.loads((ROOT/n).read_text(encoding='utf-8'))['approved_evaluation_scripts']}]
        rows[path]=dict(current=current,at_chain_end=walked,moved_by_later_contracts=later)
result=dict(scope='test_corner_follow의 목록 끝(fresh_confirmation_r2) 이후 M3_SUCCESSORS 계약이 현재 바이트를 설명하는지 확인.',
            paths=rows,all_walked_without_error=True,paths_changed_after_list_end=sum(bool(r['moved_by_later_contracts']) for r in rows.values()),
            successors_after_list_end=len(newer),script_sha256=sha256(Path(__file__).read_bytes()).hexdigest())
(HERE/'chain_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=1),encoding='utf-8',newline='\n')
print({k:v for k,v in result.items() if k not in('paths','scope')},[ (p,len(r['moved_by_later_contracts'])) for p,r in rows.items()])
