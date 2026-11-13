"""r457 종료 원본에서 세 current 입력과 당시 코드를 추출한다. 주행/장치 변경 없음."""
import hashlib,json,zipfile
from pathlib import Path
A=Path(__file__).resolve().parent
B=A.parent/'solo_full_lap_r457_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert not (A/'base').exists()
with zipfile.ZipFile(B/'sources.zip') as z:
    for n in z.namelist():
        if n.startswith(('src/arena_autonomy/arena_autonomy/','src/arena_vehicle_interface/arena_vehicle_interface/')):
            p=A/'base'/n;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(z.read(n))
selected=[];header=None
for line in (B/'run1/follow_decisions.jsonl').open(encoding='utf8'):
    r=json.loads(line)
    if r['kind']=='runtime_evaluator_header':header=r['data']['header']
    if r['kind']=='call' and r['seq'] in (1360,1590,1986):selected.append(r)
assert len(selected)==3 and header
(A/'inputs.json').write_text(json.dumps(dict(header=header,calls=selected)),encoding='utf8')
manifest={p.relative_to(A).as_posix():sha(p) for p in (A/'base').rglob('*') if p.is_file()}
manifest['inputs.json']=sha(A/'inputs.json')
(A/'manifest.json').write_text(json.dumps(dict(files=manifest,source_archive_sha256=sha(B/'sources.zip'),source_trace_sha256=sha(B/'run1/follow_decisions.jsonl')),indent=2),encoding='utf8')
print(json.dumps(dict(files=len(manifest),sequences=[r['seq'] for r in selected])))
