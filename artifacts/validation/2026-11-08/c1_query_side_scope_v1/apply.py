"""검토한 same-path 범위 수정만 적용하고 r36→r37 출처를 보존한다."""
from pathlib import Path
import ast,hashlib,json,subprocess,sys,zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3];probe=A.parent/'c1_release_observation_probe_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
m=json.loads((probe/'candidate_manifest.json').read_bytes());p=ROOT/m['path'];candidate=probe/'same_path_gap_trial.py'
parent='config/tests/adaptive_entry_preview_r36_sha256.json';contract='config/tests/query_local_side_scope_r37_sha256.json'
chain=ROOT/'tests/test_source_provenance_r0_plus_b3.py';test=ROOT/'tests/test_query_local_side_scope.py'
assert sha(p)==m['before'] and sha(candidate)==m['after']
assert not (ROOT/contract).exists() and not test.exists() and not (A/'before.zip').exists()
ast.parse(candidate.read_text(encoding='utf8'))
parent_data=json.loads((ROOT/parent).read_bytes())
for path,row in parent_data['approved_changes'].items():assert sha(ROOT/path)==row['after'],path
for path,h in parent_data['new_dependencies'].items():assert sha(ROOT/path)==h,path
text=chain.read_bytes().decode();needle=f"M3_SUCCESSORS += ('{parent}',)";assert text.count(needle)==1
updated=text.replace(needle,needle+f"\nM3_SUCCESSORS += ('{contract}',)").encode()
check=subprocess.run(['pgrep','-af',r'^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)'],capture_output=True,text=True)
assert check.returncode==1,('simulation active or check failed',check.stdout,check.stderr)
replay=json.loads((probe/'replay_result.json').read_bytes())
assert replay['r35_B']['original_matched']==3040 and replay['r34_A']['original_matched']==3051
assert replay['r34_A']['first_difference'] is None
assert all(row['candidate_sha256']==m['after'] for row in replay.values())
if '--check' in sys.argv:print('r37 preconditions passed; no simulation');raise SystemExit(0)
staged=(probe/'test_query_local_side_staged.py').read_text(encoding='utf8')
start=staged.index("spec=importlib.util.spec_from_file_location(")
end=staged.index('\ndef build(',start)
legacy="""import types,zipfile
_source=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-11-08/c1_query_side_scope_v1/before.zip'
with zipfile.ZipFile(_source) as _z:
    _before=_z.read('src/arena_autonomy/arena_autonomy/same_path_gap_trial.py')
_legacy=types.ModuleType('preserved_query_scope_before_r37')
exec(compile(_before,str(_source),'exec'),_legacy.__dict__)
original=_legacy.bounded_same_path_gap
from arena_autonomy.same_path_gap_trial import bounded_same_path_gap as candidate
"""
staged=staged[:start]+legacy+staged[end:];ast.parse(staged)
with zipfile.ZipFile(A/'before.zip','x',zipfile.ZIP_DEFLATED) as z:
    z.writestr(m['path'],p.read_bytes());z.writestr(chain.relative_to(ROOT).as_posix(),chain.read_bytes())
c=dict(schema_version=1,scope='기존 같은-경로 측방 증거에서 실제 이동 질의와 무관한 후방 결측만 분리. 점유/원시반환/전방/다른 명령/STOP 불변.',
    parent_contract=parent,parent_contract_sha256=sha(ROOT/parent),
    before_snapshot=(A/'before.zip').relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(A/'before.zip'),
    approved_changes={m['path']:dict(before=m['before'],after=m['after'],reason='scope retry only after all actual queries pass original nonreturn checks')},
    approved_evaluation_scripts={},new_dependencies={})
p.write_bytes(candidate.read_bytes());test.write_bytes(staged.encode());chain.write_bytes(updated)
(ROOT/contract).write_bytes((json.dumps(c,ensure_ascii=False,indent=2)+'\n').encode())
print('applied r37 one runtime module; build and tests not performed by this script')
