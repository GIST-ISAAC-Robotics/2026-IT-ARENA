"""검증된 감속 재검사 사본만 연결하고 r37→r38 출처를 보존한다."""
from pathlib import Path
import ast,hashlib,json,subprocess,sys,zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3];probe=A.parent/'released_masked_speed_probe_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
m=json.loads((probe/'candidate_manifest.json').read_bytes());p=ROOT/m['source'];candidate=probe/'pass_release_candidate.py'
parent='config/tests/query_local_side_scope_r37_sha256.json';contract='config/tests/masked_motion_speed_r38_sha256.json'
helper='src/arena_autonomy/arena_autonomy/masked_motion_speed.py'
chain=ROOT/'tests/test_source_provenance_r0_plus_b3.py';test=ROOT/'tests/test_masked_motion_speed.py'
assert sha(p)==m['before_sha256'] and sha(candidate)==m['after_sha256']
assert sha(probe/'masked_motion_speed.py')==m['helper_sha256']
assert not any(x.exists() for x in (ROOT/contract,ROOT/helper,test,A/'before.zip'))
for f in (candidate,probe/'masked_motion_speed.py'):ast.parse(f.read_text(encoding='utf8'))
parent_data=json.loads((ROOT/parent).read_bytes())
for path,row in parent_data['approved_changes'].items():assert sha(ROOT/path)==row['after'],path
for path,h in parent_data['new_dependencies'].items():assert sha(ROOT/path)==h,path
text=chain.read_bytes().decode();needle=f"M3_SUCCESSORS += ('{parent}',)";assert text.count(needle)==1
updated=text.replace(needle,needle+f"\nM3_SUCCESSORS += ('{contract}',)").encode()
check=subprocess.run(['pgrep','-af',r'^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)'],capture_output=True,text=True)
assert check.returncode==1,('simulation active or check failed',check.stdout,check.stderr)
replay=json.loads((probe/'replay_result.json').read_bytes())
assert replay['r37_B']['original_matched']==3041 and replay['r34_A']['original_matched']==3051
assert replay['r34_A']['first_difference'] is None
assert all(row['candidate_sha256']==m['after_sha256'] for row in replay.values())
assert replay['r37_B']['first_difference']['new'][0]>0 and replay['r37_B']['first_difference']['old'][0]==0
if '--check' in sys.argv:print('r38 preconditions passed; no simulation');raise SystemExit(0)
staged=(probe/'test_candidate.py').read_text(encoding='utf8')
start=staged.index('A=Path(__file__)');end=staged.index('REGION=',start)
staged=staged[:start]+"A=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-11-08/released_masked_speed_probe_v1'\nimport arena_autonomy.masked_motion_speed as m\n"+staged[end:];ast.parse(staged)
with zipfile.ZipFile(A/'before.zip','x',zipfile.ZIP_DEFLATED) as z:
    z.writestr(m['source'],p.read_bytes());z.writestr(chain.relative_to(ROOT).as_posix(),chain.read_bytes())
c=dict(schema_version=1,scope='완전관측 기존 경로의 RELEASED 후방 꼬리 가림 거부에서 낮은 목표만 최대3회 재검사. 현재/이전 조향·측정속도·1cm/미관측/점유/STOP 불변. 기본 활성화 승인 아님.',
    parent_contract=parent,parent_contract_sha256=sha(ROOT/parent),
    before_snapshot=(A/'before.zip').relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(A/'before.zip'),
    approved_changes={m['source']:dict(before=m['before_sha256'],after=m['after_sha256'],reason='try lower target with unchanged measured motion and existing region rejection')},
    approved_evaluation_scripts={},new_dependencies={helper:m['helper_sha256']})
p.write_bytes(candidate.read_bytes());(ROOT/helper).write_bytes((probe/'masked_motion_speed.py').read_bytes())
test.write_bytes(staged.encode());chain.write_bytes(updated)
(ROOT/contract).write_bytes((json.dumps(c,ensure_ascii=False,indent=2)+'\n').encode())
print('applied r38 selector and helper; run related tests before simulation')
