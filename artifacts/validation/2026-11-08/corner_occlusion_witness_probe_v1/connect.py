"""검토한 사본을 r45 선택형 연결로 옮기고 변경 전 바이트를 보존한다."""
from pathlib import Path
import hashlib
import json
import zipfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
OUT=A.parent/'corner_occlusion_connected_r45_v1'
OUT.mkdir(exist_ok=True)
assert not list(OUT.iterdir()), 'refuse to overwrite prepared evidence'
paths=['src/arena_autonomy/arena_autonomy/corner_boundary_support.py',
       'src/arena_autonomy/arena_autonomy/local_follow.py']
sha=lambda b:hashlib.sha256(b).hexdigest()
before={p:(ROOT/p).read_bytes() for p in paths}
assert before[paths[0]].decode('utf8').replace('\r\n','\n')==(A/'original.py').read_text(encoding='utf8')
with zipfile.ZipFile(OUT/'before.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p,b in before.items():z.writestr(p,b)
    for p in ('tests/test_source_provenance_r0_plus_b3.py','tests/test_corner_boundary_support.py'):
        z.write(ROOT/p,p)
code=(A/'candidate.py').read_text(encoding='utf8').replace('미연결 분석 후보.','선택형 교통 경계 갱신 후보.')
(ROOT/paths[0]).write_text(code,encoding='utf8',newline='\n')
p=ROOT/paths[1];s=p.read_text(encoding='utf8')
old='cfg, now-self.path_source)'
assert s.count(old)==1
s=s.replace(old,'cfg, now-self.path_source, scene=scene)')
old="""                    detail['current_corner_support'] = {k: v for k, v in support_evidence.items()
                        if not k.endswith('_indices')}"""
new="""                    detail['current_corner_support'] = {k: v for k, v in support_evidence.items()
                        if not k.endswith('_indices')}
                    far_support = support_evidence.get('far_occlusion_support')
                    if far_support is not None:
                        detail['current_corner_support']['far_occlusion_support'] = {
                            k: v for k, v in far_support.items() if not k.endswith('_indices')}"""
assert old in s;s=s.replace(old,new);p.write_text(s,encoding='utf8',newline='\n')
parent='config/tests/faster_entry_consistency_r44_sha256.json'
contract=dict(schema_version=1,scope='선택형 현재 코너 지지에서 먼 벽의 가림을 현재 광선으로 대조한다. 기존 면/기억/점유/독립 보호 기준은 유지.',
    parent_contract=parent,parent_contract_sha256=sha((ROOT/parent).read_bytes()),
    before_snapshot=(OUT/'before.zip').relative_to(ROOT).as_posix(),before_snapshot_sha256=sha((OUT/'before.zip').read_bytes()),
    approved_changes={p:dict(before=sha(b),after=sha((ROOT/p).read_bytes()),reason='Current foreground evidence for partially occluded far face, optional scene connection and compact evidence logging') for p,b in before.items()},
    approved_evaluation_scripts={},new_dependencies={})
name='config/tests/corner_occlusion_r45_sha256.json'
(ROOT/name).write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
p=ROOT/'tests/test_source_provenance_r0_plus_b3.py';s=p.read_text(encoding='utf8')
needle="M3_SUCCESSORS += ('config/tests/faster_entry_consistency_r44_sha256.json',)"
assert s.count(needle)==1
p.write_text(s.replace(needle,needle+f"\nM3_SUCCESSORS += ('{name}',)"),encoding='utf8',newline='\n')
# 실제 관측 fixture: 첫 차이가 발생한 call과 그때 이미 유효했던 경로만 보관한다.
capture=next(q for q in json.loads((A.parent/'corner_r41_cause_audit_v1/captures.json').read_bytes()) if abs(q['t']-35.92)<1e-8)
trace=A.parent/'current_corner_support_v1/run1/follow_decisions.jsonl'
with trace.open(encoding='utf8') as f:
    for line in f:
        e=json.loads(line)
        if e['kind']=='header':header=e['data']
        if e['kind']=='call' and abs(e['data']['input']['reference_s']-35.92)<1e-8:
            call=e['data'];break
fixture=dict(trace_sha256=sha(trace.read_bytes()),input=call['input'],follow_config=header['follow_config'],motion_config=header['motion_config'],
    reference=capture['carried_path'],side='right',reference_age_s=35.92-call['output'][2]['normal']['path_source_s'])
(OUT/'first_loss_fixture.json').write_text(json.dumps(fixture),encoding='utf8')
print('r45 source connection and before snapshot prepared')
