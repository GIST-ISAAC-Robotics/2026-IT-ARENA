from pathlib import Path
import hashlib,json,zipfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
OUT=A.parent/'current_corner_fallback_r47_v1'
assert not OUT.exists()
sha=lambda b:hashlib.sha256(b).hexdigest()
path='src/arena_autonomy/arena_autonomy/local_follow.py'
old=(ROOT/path).read_bytes()
assert old==(A/'before_local_follow.py').read_bytes()
assert (A/'tests_v1.exit').read_text().strip()=='0' and (A/'replay_v1.exit').read_text().strip()=='0'
result=json.loads((A/'replay_summary.json').read_bytes())
assert result['calls']==1635 and result['first_difference']['t']>=37.52 and result['source_unchanged']
OUT.mkdir()
with zipfile.ZipFile(OUT/'before.zip','x',zipfile.ZIP_DEFLATED) as z:
    z.writestr(path,old)
    z.write(ROOT/'tests/test_source_provenance_r0_plus_b3.py','tests/test_source_provenance_r0_plus_b3.py')
(ROOT/path).write_bytes((A/'candidate_local_follow.py').read_bytes())
parent='config/tests/retained_corner_agreement_r46_sha256.json'
name='config/tests/current_corner_fallback_r47_sha256.json'
contract=dict(schema_version=1,scope='현재 유지 벽의 갱신 실패 후 충분한 현재 선호 코너를 같은 유효 이력과 대조. 관측/TTL/점유/기본 옵션 불변.',
    parent_contract=parent,parent_contract_sha256=sha((ROOT/parent).read_bytes()),
    before_snapshot=(OUT/'before.zip').relative_to(ROOT).as_posix(),before_snapshot_sha256=sha((OUT/'before.zip').read_bytes()),
    approved_changes={path:dict(before=sha(old),after=sha((ROOT/path).read_bytes()),
        reason='Prefer current retained wall, then compare an observed alternative corner with the same recent reference')},
    approved_evaluation_scripts={},new_dependencies={})
(ROOT/name).write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n',encoding='utf8',newline='\n')
p=ROOT/'tests/test_source_provenance_r0_plus_b3.py';s=p.read_text(encoding='utf8')
needle=f"M3_SUCCESSORS += ('{parent}',)";assert s.count(needle)==1
p.write_text(s.replace(needle,needle+f"\nM3_SUCCESSORS += ('{name}',)"),encoding='utf8',newline='\n')
s=(A/'test_candidate.py').read_text(encoding='utf8').replace('from loader import load','import arena_autonomy.local_follow as m').replace('m=load()','')
s=s.replace('A=Path(__file__).resolve().parent',"A=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-11-08/current_corner_fallback_probe_v2'")
(ROOT/'tests/test_current_corner_fallback.py').write_text(s,encoding='utf8',newline='\n')
oldA=A.parent/'retained_corner_agreement_r46_v1'
for f in ('check_runtime.py','assess.py','new_track_course.json','evaluation_source.json','geometry.json','run.py','review.py'):
    (OUT/f).write_bytes((oldA/f).read_bytes())
for f in ('tests.sh','run.sh','replay.sh'):
    s=(oldA/f).read_text(encoding='utf8').replace(oldA.name,OUT.name)
    if f=='tests.sh':s=s.replace('tests/test_retained_corner_agreement.py','tests/test_current_corner_fallback.py tests/test_retained_corner_agreement.py')
    (OUT/f).write_text(s,encoding='utf8',newline='\n')
s=(oldA/'register.py').read_text(encoding='utf8').replace('retained_corner_agreement_r46_sha256.json','current_corner_fallback_r47_sha256.json')
s=s.replace('corner_occlusion_connected_r45_v1/registered.json','retained_corner_agreement_r46_v1/registered.json')
s=s.replace('corner_occlusion_connected_r45_v1/new_track_course.json','retained_corner_agreement_r46_v1/new_track_course.json')
s=s.replace("'tests/test_retained_corner_agreement.py',", "'tests/test_current_corner_fallback.py', 'artifacts/validation/2026-11-08/current_corner_fallback_probe_v2/fixture.json', 'tests/test_retained_corner_agreement.py',")
s=s.replace("print('r46 sources'","print('r47 sources'").replace("scope='r46 common-arc agreement for current retained corner; r45/r41 same32s side actor'","scope='r47 current corner fallback after retained-current rejection; r46 same32s actor'")
(OUT/'register.py').write_text(s,encoding='utf8',newline='\n')
s=(oldA/'replay.py').read_text(encoding='utf8').replace('retained_corner_agreement_probe_v1/replay_rows.jsonl','current_corner_fallback_probe_v2/replay_rows.jsonl')
s=s.replace('corner_occlusion_connected_r45_v1/run1/follow_decisions.jsonl','retained_corner_agreement_r46_v1/run1/follow_decisions.jsonl')
(OUT/'replay.py').write_text(s,encoding='utf8',newline='\n')
print('Connected r47, unchanged scenario/criteria. Run tests and replay before registration.')
