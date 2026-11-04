"""Record pre-existing launch forwarding omitted from v2/v3 contracts; no runtime edit."""
import ast,hashlib,json,subprocess,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5]
HERE=Path(__file__).resolve().parent.parent
PARENT='config/tests/lidar_mount_speed_r4_sha256.json'
TARGET='config/tests/lidar_mount_launch_record_r5_sha256.json'
FILES=['src/arena_bringup/launch/'+n+'.launch.py' for n in ['g1_follow','demo','local_pursuit']]
def sha(b):return hashlib.sha256(b).hexdigest()
tree=ast.parse((ROOT/'tests/test_source_provenance_r0_plus_b3.py').read_text(encoding='utf8'))
chain=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='M3_SUCCESSORS' for t in n.targets))
expected={}
for c in chain:
    if c==TARGET:continue
    data=json.loads((ROOT/c).read_text(encoding='utf8'))
    expected.update({p:r['after'] for g in ['approved_changes','approved_evaluation_scripts'] for p,r in data[g].items()})
before={}
pilot=json.loads((ROOT/'artifacts/validation/2026-11-04/lidar_mount_pilot_v3/c50_2_registered.json').read_text(encoding='utf8'))['source_sha256']
rows={}
for p in FILES:
    blob=subprocess.run(['git','show','HEAD:'+p],cwd=ROOT,capture_output=True,check=True).stdout
    before[p]=next(b for b in [blob,blob.replace(b'\r\n',b'\n'),blob.replace(b'\r\n',b'\n').replace(b'\n',b'\r\n')] if sha(b)==expected[p])
    after=sha((ROOT/p).read_bytes());assert after==pilot[p]
    rows[p]=dict(before=expected[p],after=after,reason='v2/v3 시점에 적용한 센서 시험 프로필 전달을 뒤늦게 출처 사슬에 기록. v3 등록 해시와 동일하고 이번 주행 코드 변경 아님')
snap=HERE/'launch_forwarding_before.zip'
if not snap.exists():
    with zipfile.ZipFile(snap,'x',compression=zipfile.ZIP_DEFLATED) as z:
        for p,b in before.items():z.writestr(p,b)
out=dict(schema_version=1,scope='기존 미기록 launch 전달의 증거 보완. 과거 실패는 보존; 새로운 동작 변경·승인 아님',
         parent_contract=PARENT,parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
         before_snapshot=snap.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(snap.read_bytes()),
         approved_changes=rows,approved_evaluation_scripts={},new_dependencies={})
text=json.dumps(out,ensure_ascii=False,indent=2)+'\n'
dst=ROOT/TARGET
if dst.exists():assert dst.read_text(encoding='utf8')==text
else:dst.write_text(text,encoding='utf8',newline='\n')
test=ROOT/'tests/test_source_provenance_r0_plus_b3.py';b=test.read_bytes()
if TARGET.encode() not in b:
    old=("'"+PARENT+"')").encode();assert b.count(old)==1
    test.write_bytes(b.replace(old,("'"+PARENT+"', '"+TARGET+"')").encode()))
print(TARGET)
