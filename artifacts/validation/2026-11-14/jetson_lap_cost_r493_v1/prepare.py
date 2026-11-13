"""센서 분리 중 Jetson 저장 전체 plan 비용 확인. 공개 원시 기록은 추가하지 않는다."""
from pathlib import Path
import hashlib
import json
import shutil
import zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
B = A / 'bundle'
B.mkdir()
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
for pkg in ('arena_autonomy', 'arena_vehicle_interface'):
    for p in (ROOT / 'src' / pkg / pkg).rglob('*'):
        if p.is_file() and p.suffix in ('.py', '.cpp', '.hpp', '.h'):
            d = B / p.relative_to(ROOT)
            d.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(p, d)
setup = Path('src/arena_autonomy/setup.py')
shutil.copy2(ROOT / setup, B / setup)
trace = A.parent / 'solo_full_lap_r475_v1/run1/follow_decisions.jsonl'
header = None
plans, outputs = [], []
with trace.open(encoding='utf-8') as stream:
    for line in stream:
        row = json.loads(line)
        if row['kind'] == 'runtime_evaluator_header':
            assert header is None
            header = row['data']['header']
        elif row['kind'] == 'runtime_plan_submitted':
            plans.append(row['data'])
        elif row['kind'] == 'runtime_plan_completed':
            item = row['data']
            outputs.append(dict(key=[item['generation'], item['sequence']], output=item['output']))
assert len(plans) == len(outputs) == 508
(B / 'inputs.json').write_text(json.dumps(dict(header=header, plans=plans, recorded_outputs=outputs), allow_nan=False)+'\n', encoding='utf-8')
prior = A.parent / 'r476_runtime_route_cost_v1'
script = (prior/'execute.py').read_text(encoding='utf-8').replace('it-arena-r476-a', 'it-arena-r493-a').replace('r476 단일', 'r493 단일')
(A/'execute.py').write_text(script, encoding='utf-8')
for name in ('run_cost.py', 'execute.py'):
    shutil.copy2(A / name, B / name)
shutil.copy2(prior/'bundle/environment_reference.py', B/'environment_reference.py')
reuse = json.loads((prior/'bundle/source_manifest.json').read_bytes())['native_reuse']
for p, h in reuse['source_sha256'].items():
    assert sha(B/p) == h, p
files = {p.relative_to(B).as_posix(): sha(p) for p in B.rglob('*') if p.is_file()}
manifest = dict(files=files, remote_root='/home/q/it-arena-r493-a', native_reuse=reuse,
    root_authorized=True, jetson_exclusive=True, plan_calls=508, trace_sha256=sha(trace),
    source_scope='current common before r489 integration; guard is not invoked', current_lane_calls=0,
    devices_opened=False, driving=False, settings_changed=False)
(B/'source_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
shutil.copy2(B/'source_manifest.json', A/'source_manifest.json')
with zipfile.ZipFile(A/'bundle.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
    for p in B.rglob('*'):
        if p.is_file():
            archive.write(p, p.relative_to(B).as_posix())
(A/'prepared.json').write_text(json.dumps(dict(files=len(files), plans=508, archive_sha256=sha(A/'bundle.zip')), indent=2)+'\n', encoding='utf-8')
print((A/'prepared.json').read_text(encoding='utf-8'))
