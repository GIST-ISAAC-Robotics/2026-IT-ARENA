"""r474 동결 소스와 r469 저장 입력의 제한 ARM 비용 묶음. 실행 없음."""
from pathlib import Path
import json,hashlib,shutil,zipfile
A=Path(__file__).resolve().parent;D=A.parent;R474=D/'current_route_priority_r474_v1';B=A/'bundle'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
load=lambda p:json.loads(p.read_text(encoding='utf-8'))
manifest=load(R474/'source_manifest.json')
assert not B.exists();B.mkdir()
for p,h in manifest['files'].items():
    source=R474/'stage'/p;assert sha(source)==h
    if p.endswith('.so'):continue
    target=B/p;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,target)
trace=D/'solo_full_lap_r469_v1/run1/follow_decisions.jsonl'
rows=[json.loads(s) for s in trace.open(encoding='utf-8')]
header=next(r['data']['header'] for r in rows if r['kind']=='runtime_evaluator_header')
assert not any(header.get(k) for k in ('vehicle_marker_enabled','vehicle_marker_trial'))
plans=[r['data'] for r in rows if r['kind']=='runtime_plan_submitted']
currents=[]
for r in rows:
    if r['kind']!='call' or not r['data'].get('current_rebase'):continue
    meta=r['data']['current_rebase']['compute'];plan=meta['plan']
    currents.append(dict(trace_seq=r['seq'],key=[plan['generation'],plan['sequence']],
        submitted=plan['input'],current=meta['input'],retained=meta['route_check']['retained']))
assert len(plans)==217 and len(currents)==113
data=dict(header=header,plans=plans,currents=currents,scope='saved simulation sensor values; no actual sensor, ROS or driving')
(B/'inputs.json').write_text(json.dumps(data,allow_nan=False)+'\n',encoding='utf-8')
shutil.copyfile(R474/'replay_v2/outputs.jsonl',B/'expected_plan_outputs.jsonl')
expected={str(r['trace_seq']):r['output'] for r in load(R474/'replay_v2/new_plan_actual_current_rows.json')}
(B/'expected_current_outputs.json').write_text(json.dumps(expected,allow_nan=False)+'\n',encoding='utf-8')
for name in ('run_cost.py','execute.py'):shutil.copyfile(A/name,B/name)
shutil.copyfile(D/'current_mask_native_r470_v1/bundle/environment_reference.py',B/'environment_reference.py')
native_files=[B/'src/arena_autonomy/setup.py',*(B/'src/arena_autonomy/arena_autonomy/_native').rglob('*')]
native_sha={p.relative_to(B).as_posix():sha(p) for p in native_files if p.is_file()}
for p,h in native_sha.items():assert sha(D/'current_mask_native_r470_v1/bundle/stage'/p)==h,p
files={p.relative_to(B).as_posix():sha(p) for p in B.rglob('*') if p.is_file()}
prepared=dict(files=files,remote_root='/home/q/it-arena-r476-a',plan_calls=217,current_calls=113,warmup_calls=0,
    marker_default_off=True,source_r474_manifest_sha256=sha(R474/'source_manifest.json'),trace_sha256=sha(trace),
    native_reuse=dict(source='/home/q/it-arena-r470-a/stage/src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-310-aarch64-linux-gnu.so',
        sha256='5654cd2f52af838b52384a084f159bb1319925db6690945b4403cb7ae33f5bff',
        source_sha256=native_sha),jetson_exclusive=True,root_authorized=True,common_modified=False,
    scope=data['scope'])
(B/'source_manifest.json').write_text(json.dumps(prepared,indent=2)+'\n',encoding='utf-8')
archive=A/'bundle.zip'
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
    for p in B.rglob('*'):
        if p.is_file():z.write(p,p.relative_to(B).as_posix())
(A/'prepared.json').write_text(json.dumps(dict(files=len(files),archive_sha256=sha(archive),
    manifest_sha256=sha(B/'source_manifest.json'),plan_calls=217,current_calls=113,native_rebuild=False,
    source_changes=0,remote_root=prepared['remote_root']),indent=2)+'\n',encoding='utf-8')
print((A/'prepared.json').read_text(encoding='utf-8'))
