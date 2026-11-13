from pathlib import Path
import ast,difflib,hashlib,json,shutil
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
old=json.loads((A/'source_manifest.json').read_bytes())['files']
stage={p:sha(A/'stage'/p) for p in old}
changed=[p for p in old if old[p]!=stage[p]]
assert set(changed)=={'src/arena_autonomy/arena_autonomy/local_follow.py',
    'src/arena_autonomy/arena_autonomy/race_runtime_policy/current_route_priority.py'}
assert all(sha(ROOT/p)==h for p,h in old.items())
test=A/'test_inputs/test_runtime_dual_contract.py';test.parent.mkdir(exist_ok=True)
shutil.copyfile(ROOT/'tests/test_runtime_dual_contract.py',test)
changes=[];diff=[]
for p in changed:
    before=A/'originals'/p;after=A/'stage'/p
    assert sha(before)==old[p]
    ast.parse(after.read_text(encoding='utf-8'))
    changes.append(dict(path=p,before_sha256=old[p],after_sha256=stage[p]))
    diff.extend(difflib.unified_diff(before.read_text(encoding='utf-8').splitlines(True),
        after.read_text(encoding='utf-8').splitlines(True),fromfile='a/'+p,tofile='b/'+p))
(A/'changes.json').write_text(json.dumps(changes,indent=2)+'\n',encoding='utf-8')
(A/'change.patch').write_text(''.join(diff),encoding='utf-8',newline='\n')
trace=A.parent/'solo_variations_r477_v1/cases/start_left20_yaw_plus3/run1/follow_decisions.jsonl'
native='src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-312-x86_64-linux-gnu.so'
helpers=['replay.py','run_native_tests.py','test_route_observation.py','test_inputs/test_runtime_dual_contract.py','execution_v2/run_once.sh']
for p in helpers:
    if p.endswith('.py'):ast.parse((A/p).read_text(encoding='utf-8'))
result=dict(stage_files=stage,original_files={p:old[p] for p in changed},
    helper_files={p:sha(A/p) for p in helpers},native_sha256=stage[native],
    trace_sha256=sha(trace),baseline_archive_sha256=json.loads((A/'source_manifest.json').read_bytes())['archive_sha256'],
    baseline_common_231_unchanged=True,changed_sources=len(changed),
    plan_count=540,comparison='float abs 1e-12; integer/bool/string/shape/key exact',
    command="wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/solo_finish_r481_v1/execution_v2/run_once.sh'",
    scope='corrected candidate only; exact baseline540/current854 from native_run_v1 reused; no build/ROS/Gazebo/device')
(A/'prepared.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(changed_sources=changes,files=len(stage),prepared_sha256=sha(A/'prepared.json'),native_sha256=stage[native])))
