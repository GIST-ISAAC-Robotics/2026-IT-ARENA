from pathlib import Path
import hashlib,json,zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3];OUT=A.parent/'retained_corner_agreement_r46_v1'
OUT.mkdir(exist_ok=False)
sha=lambda b:hashlib.sha256(b).hexdigest()
path='src/arena_autonomy/arena_autonomy/local_follow.py';old=(ROOT/path).read_bytes()
assert old==(A/'before_local_follow.py').read_bytes()
assert (A/'tests_v2.exit').read_text().strip()=='0' and (A/'replay_v1.exit').read_text().strip()=='0'
with zipfile.ZipFile(OUT/'before.zip','x',zipfile.ZIP_DEFLATED) as z:
    z.writestr(path,old);z.write(ROOT/'tests/test_source_provenance_r0_plus_b3.py','tests/test_source_provenance_r0_plus_b3.py')
(ROOT/path).write_bytes((A/'candidate_local_follow.py').read_bytes())
parent='config/tests/corner_occlusion_r45_sha256.json';name='config/tests/retained_corner_agreement_r46_sha256.json'
contract=dict(schema_version=1,scope='선택형 교통에서 현재 긴 직각 벽을 유지할 때 공통호길이 비교를 재사용한다. 기존 x비교 우선, 허용오차/관측/점유/TTL 불변.',
 parent_contract=parent,parent_contract_sha256=sha((ROOT/parent).read_bytes()),before_snapshot=(OUT/'before.zip').relative_to(ROOT).as_posix(),
 before_snapshot_sha256=sha((OUT/'before.zip').read_bytes()),approved_changes={path:dict(before=sha(old),after=sha((ROOT/path).read_bytes()),
 reason='Compare common arc of current observed piecewise corner when retaining the same recent wall; leave other comparisons unchanged')},
 approved_evaluation_scripts={},new_dependencies={})
(ROOT/name).write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
p=ROOT/'tests/test_source_provenance_r0_plus_b3.py';s=p.read_text(encoding='utf8');needle="M3_SUCCESSORS += ('config/tests/corner_occlusion_r45_sha256.json',)";assert s.count(needle)==1
p.write_text(s.replace(needle,needle+f"\nM3_SUCCESSORS += ('{name}',)"),encoding='utf8',newline='\n')
s=(A/'test_candidate.py').read_text(encoding='utf8').replace('from loader import load','import arena_autonomy.local_follow as m')
s=s.replace("m=load();A=Path(__file__).resolve().parent.parent/'corner_r45_transition_audit_v1'", "A=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-11-08/corner_r45_transition_audit_v1'")
(ROOT/'tests/test_retained_corner_agreement.py').write_text(s,encoding='utf8',newline='\n')
oldA=A.parent/'corner_occlusion_connected_r45_v1'
for f in ('check_runtime.py','assess.py','new_track_course.json','evaluation_source.json','geometry.json','run.py'):
    (OUT/f).write_bytes((oldA/f).read_bytes())
for f in ('tests.sh','run.sh'):
    s=(oldA/f).read_text(encoding='utf8').replace('corner_occlusion_connected_r45_v1','retained_corner_agreement_r46_v1')
    if f=='tests.sh':s=s.replace('tests/test_corner_occlusion_support.py','tests/test_retained_corner_agreement.py tests/test_corner_occlusion_support.py')
    (OUT/f).write_text(s,encoding='utf8',newline='\n')
s=(oldA/'register.py').read_text(encoding='utf8').replace('corner_occlusion_r45_sha256.json','retained_corner_agreement_r46_sha256.json')
s=s.replace("assert (A/'tests_v2.exit').read_text().strip() == '0'\nassert '341 passed' in (A/'tests_v1.log').read_text()", "assert (A/'tests_v1.exit').read_text().strip() == '0'")
s=s.replace("legacy = A.parent/'current_corner_support_v1/registered.json'", "legacy = A.parent/'corner_occlusion_connected_r45_v1/registered.json'")
s=s.replace("A.parent/'current_corner_support_v1/new_track_course.json'", "A.parent/'corner_occlusion_connected_r45_v1/new_track_course.json'")
s=s.replace("'tests/test_corner_occlusion_support.py',", "'tests/test_retained_corner_agreement.py', 'tests/test_corner_occlusion_support.py',")
s=s.replace("print('r45 sources'", "print('r46 sources'").replace("scope='r45 current foreground witness extends valid two-face corner; r41 same32s side actor'", "scope='r46 common-arc agreement for current retained corner; r45/r41 same32s side actor'")
(OUT/'register.py').write_text(s,encoding='utf8',newline='\n')
s=(oldA/'replay.py').read_text(encoding='utf8').replace('corner_occlusion_witness_probe_v1/replay_rows.jsonl','retained_corner_agreement_probe_v1/replay_rows.jsonl').replace("'current_corner_support_v1/run1/follow_decisions.jsonl'","'corner_occlusion_connected_r45_v1/run1/follow_decisions.jsonl'")
(OUT/'replay.py').write_text(s,encoding='utf8',newline='\n')
s=(oldA/'replay.sh').read_text(encoding='utf8').replace('corner_occlusion_connected_r45_v1','retained_corner_agreement_r46_v1');(OUT/'replay.sh').write_text(s,encoding='utf8',newline='\n')
s=(oldA/'review_window.py').read_text(encoding='utf8');(OUT/'review.py').write_text(s,encoding='utf8',newline='\n')
print('r46 same candidate connected; ready for tests then replay, no simulation launched')
