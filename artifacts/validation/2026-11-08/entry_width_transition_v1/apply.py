"""r39 종료 후 검토된 r40 사본을 연결한다. 시뮬레이션 실행은 하지 않는다."""
from pathlib import Path
import ast,hashlib,json,subprocess,zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3];probe=A.parent/'entry_width_transition_probe_v2'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
parent='config/tests/short_entry_transition_r39_sha256.json'
contract='config/tests/adaptive_corridor_offset_r40_sha256.json'
helper='src/arena_autonomy/arena_autonomy/adaptive_corridor_offset.py'
pairs={
 'src/arena_autonomy/arena_autonomy/bypass_handoff.py':'bypass_handoff_candidate.py',
 'src/arena_autonomy/arena_autonomy/bypass_corridor_trial.py':'bypass_corridor_candidate.py'}
assert not any(p.exists() for p in (A/'before.zip',ROOT/contract,ROOT/helper))
check=subprocess.run(['pgrep','-af',r'^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)'],capture_output=True,text=True)
assert check.returncode==1,('simulation active',check.stdout,check.stderr)
integrity=json.loads((A.parent/'short_entry_transition_v1/integrity.json').read_bytes())
assert integrity['changed']==[] and integrity['snapshot_unchanged']
prior=json.loads((ROOT/parent).read_bytes())
for path,row in prior['approved_changes'].items():assert sha(ROOT/path)==row['after'],path
for path,h in prior['new_dependencies'].items():assert sha(ROOT/path)==h,path
replay=json.loads((probe/'staged_replay.json').read_bytes())
assert replay['matched']==208 and replay['first_command_difference']['t']==13.08
for name in (*pairs.values(),'roomy_candidate.py'):ast.parse((probe/name).read_text())
chain=ROOT/'tests/test_source_provenance_r0_plus_b3.py'
text=chain.read_text();needle=f"M3_SUCCESSORS += ('{parent}',)";assert text.count(needle)==1
newchain=text.replace(needle,needle+f"\nM3_SUCCESSORS += ('{contract}',)")
before={path:sha(ROOT/path) for path in pairs}
with zipfile.ZipFile(A/'before.zip','x',zipfile.ZIP_DEFLATED) as z:
 for path in (*pairs,chain.relative_to(ROOT).as_posix()):z.writestr(path,(ROOT/path).read_bytes())
c=dict(schema_version=1,scope='기본 OFF adaptive_entry_preview의 최초 가까운 우회에 관측 통로 편이 중간값 비교. 기존 외벽 가정/스캔 재사용 여유만 순위에 적용하며 실제 구동 검사는 보존.',
 parent_contract=parent,parent_contract_sha256=sha(ROOT/parent),
 before_snapshot=(A/'before.zip').relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(A/'before.zip'),
 approved_changes={path:dict(before=before[path],after=sha(probe/name),reason='bounded initial corridor offset candidate in existing adaptive trial') for path,name in pairs.items()},
 approved_evaluation_scripts={},new_dependencies={helper:sha(probe/'roomy_candidate.py')})
for path,name in pairs.items():(ROOT/path).write_bytes((probe/name).read_bytes())
(ROOT/helper).write_bytes((probe/'roomy_candidate.py').read_bytes())
tests=(probe/'test_roomy_candidate.py').read_text().replace('from roomy_candidate import','from arena_autonomy.adaptive_corridor_offset import')
tests=tests.replace('A=Path(__file__).resolve().parent',"A=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-11-08/entry_width_transition_probe_v2'")
(ROOT/'tests/test_adaptive_corridor_offset.py').write_text(tests,encoding='utf8')
chain.write_text(newchain,encoding='utf8');(ROOT/contract).write_text(json.dumps(c,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
print('r40 applied; defaults and physical/guard criteria unchanged; run related checks')
