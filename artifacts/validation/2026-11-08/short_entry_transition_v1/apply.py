"""r38 시험 종료 뒤 검토된 최초 전이 사본을 r39로 연결한다. 자동 실행 없음."""
from pathlib import Path
import ast,hashlib,json,subprocess,sys,zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3];probe=A.parent/'entry_preview_continuity_probe_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
m=json.loads((probe/'corridor_candidate_manifest.json').read_bytes());p=ROOT/m['source'];candidate=probe/'bypass_corridor_candidate.py'
parent='config/tests/masked_motion_speed_r38_sha256.json';contract='config/tests/short_entry_transition_r39_sha256.json'
helper='src/arena_autonomy/arena_autonomy/short_entry_transition.py'
chain=ROOT/'tests/test_source_provenance_r0_plus_b3.py';test=ROOT/'tests/test_short_entry_transition.py'
assert sha(p)==m['before_sha256'] and sha(candidate)==m['after_sha256']
assert sha(probe/'short_transition_candidate.py')==m['helper_sha256']
assert not any(x.exists() for x in (ROOT/contract,ROOT/helper,test,A/'before.zip'))
for f in (candidate,probe/'short_transition_candidate.py'):ast.parse(f.read_text(encoding='utf8'))
parent_data=json.loads((ROOT/parent).read_bytes())
for path,row in parent_data['approved_changes'].items():assert sha(ROOT/path)==row['after'],path
for path,h in parent_data['new_dependencies'].items():assert sha(ROOT/path)==h,path
text=chain.read_bytes().decode();needle=f"M3_SUCCESSORS += ('{parent}',)";assert text.count(needle)==1
updated=text.replace(needle,needle+f"\nM3_SUCCESSORS += ('{contract}',)").encode()
check=subprocess.run(['pgrep','-af',r'^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)'],capture_output=True,text=True)
assert check.returncode==1,('simulation active or check failed',check.stdout,check.stderr)
r=json.loads((probe/'short_transition_replay.json').read_bytes());staged=json.loads((probe/'staged_corridor_replay.json').read_bytes())
assert r['matched']==1370 and staged['matched']==212
assert r['first_command_difference']['t']==staged['first_command_difference']['t']==13.16
assert r['first_command_difference']['old'][:2]==staged['first_command_difference']['old'][:2]
assert r['first_command_difference']['new'][:2]==staged['first_command_difference']['new'][:2]
if '--check' in sys.argv:print('r39 preconditions passed; no simulation');raise SystemExit(0)
tests=(probe/'test_short_transition.py').read_text(encoding='utf8')
tests=tests[:tests.index("@pytest.mark.parametrize('enabled'")]
tests=tests.replace('from short_transition_candidate import compare_short_transition','from arena_autonomy.short_entry_transition import compare_short_transition')
tests=tests.replace('A=Path(__file__).resolve().parent',"A=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-11-08/entry_preview_continuity_probe_v1'")
ast.parse(tests)
with zipfile.ZipFile(A/'before.zip','x',zipfile.ZIP_DEFLATED) as z:
    z.writestr(m['source'],p.read_bytes());z.writestr(chain.relative_to(ROOT).as_posix(),chain.read_bytes())
c=dict(schema_version=1,scope='기본 OFF 짧은 목표점 선택형 안의 가까운 최초 우회 전이만 비교. 기존 조향 한계/원시 반환/점유/관측/STOP 유지. 기본 활성화 승인 아님.',
    parent_contract=parent,parent_contract_sha256=sha(ROOT/parent),
    before_snapshot=(A/'before.zip').relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(A/'before.zip'),
    approved_changes={m['source']:dict(before=m['before_sha256'],after=m['after_sha256'],reason='one intermediate initial transition ranked with existing observed progress and gap rules')},
    approved_evaluation_scripts={},new_dependencies={helper:m['helper_sha256']})
p.write_bytes(candidate.read_bytes());(ROOT/helper).write_bytes((probe/'short_transition_candidate.py').read_bytes())
test.write_bytes(tests.encode());chain.write_bytes(updated)
(ROOT/contract).write_bytes((json.dumps(c,ensure_ascii=False,indent=2)+'\n').encode())
print('applied r39 first transition only; run related tests before simulation')
