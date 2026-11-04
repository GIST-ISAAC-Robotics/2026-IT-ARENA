"""월드 생성 후 원본 대비 기둥 제거 범위·소스 출처를 기록한다."""
from pathlib import Path
from zipfile import ZipFile
import hashlib, json
import xml.etree.ElementTree as ET
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def canonical(e):return ET.canonicalize(ET.tostring(e,encoding="unicode"),strip_text=True)
world='src/arena_gazebo/worlds/it_arena_official/world.sdf'
with ZipFile(HERE/'before.zip') as z:
    before=ET.fromstring(z.read(world)).find("./world/model[@name='it_arena_track_static']")
after=ET.parse(ROOT/world).find("./world/model[@name='it_arena_track_static']")
left={x.get('name'):canonical(x) for x in before.findall('link')}
right={x.get('name'):canonical(x) for x in after.findall('link')}
removed=sorted(left.keys()-right.keys());added=sorted(right.keys()-left.keys())
changed=sorted(k for k in right.keys()&left.keys() if left[k]!=right[k])
assert removed==['tl_post'] and not added and not changed,(removed,added,changed)
assert not any('tl_post' in x.get('name','') for x in after.iter())
old=json.loads((HERE/'before_sha256.json').read_text())
paths={p:{'before':h,'after':sha(ROOT/p),'reason':'기둥 제거 파생본 출처'} for p,h in old.items() if sha(ROOT/p)!=h and not p.startswith('tests/')}
for row in paths.values():row['reason']='11/2 사용자가 실제 코스 안 기둥 없음 확인. 팀 파생본의 tl_post만 제거하고 나머지 형상 보존; 생성기·프로필·파생 산출물 출처 갱신.'
parent='config/tests/observed_gap_recovery_r1_sha256.json'
contract=dict(schema_version=1,scope='기둥 제거 파생본. 제어기·광원 표현 불변; 주행 성공/실차 검증 아님.',parent_contract=parent,parent_contract_sha256=sha(ROOT/parent),before_snapshot=(HERE/'before.zip').relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(HERE/'before.zip'),approved_changes=paths,approved_evaluation_scripts={},new_dependencies={})
with (ROOT/'config/tests/post_free_facility_r1_sha256.json').open('x',encoding='utf-8') as f:json.dump(contract,f,ensure_ascii=False,indent=2)
result=dict(removed_links=removed,added_links=added,other_changed_links=changed,remaining_links=len(right),changed_files=sorted(paths),scope='기둥 visual/collision 제거 이외의 SDF 링크는 정규화 비교에서 동일; 원본 ZIP은 builder에서 SHA 검증')
(HERE/'geometry_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
