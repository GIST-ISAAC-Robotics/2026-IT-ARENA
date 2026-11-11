"""기존 기록에서 비교 대상만 추출하고 현재 코드와 함께 별도 동결한다."""
import hashlib
import io
import json
from pathlib import Path
import tarfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
old=ROOT/'artifacts/validation/2026-11-11/guard_recheck_r139_v1'
events=[json.loads(line) for line in (old/'two_actor_run1/run1/follow_decisions.jsonl').open()]
records=json.loads((A.parent/'guard_current_r142_v1/comparison.json').read_bytes())['results']
with (A/'samples.jsonl').open('x',encoding='utf8') as stream:
    stream.write(json.dumps(dict(header=events[0]['data']))+'\n')
    for row in records:
        saved=dict(seq=row['seq'],original=events[row['seq']]['data'],
                   current=events[row['next_seq']]['data']['input'],expected=row['current'])
        stream.write(json.dumps(saved)+'\n')
files=[p.relative_to(ROOT).as_posix() for package in ('arena_autonomy','arena_vehicle_interface')
       for p in (ROOT/'src'/package/package).glob('*.py')]
files += [p.relative_to(ROOT).as_posix() for p in (A/'measure.py',A/'samples.jsonl',
    A.parent/'guard_current_r142_v1/candidate.py',
    ROOT/'artifacts/validation/2026-11-11/body_nonreturn_r136_v1/candidate.py',
    ROOT/'artifacts/validation/2026-11-11/follow_rebase_r126_v1/rebase.py')]
registration=json.loads((A.parent/'guard_current_r143_v1/two_actor_run1/registered.json').read_bytes())
inventory={};archive=ROOT/'build/jetson_guard_current_r144_20261112.tar.gz'
with tarfile.open(archive,'x:gz',compresslevel=1) as tar:
    for name in files:
        data=(ROOT/name).read_bytes();sha=hashlib.sha256(data).hexdigest()
        if name in registration['extra_sources']:assert sha==registration['extra_sources'][name]
        inventory[name]=dict(size=len(data),sha256=sha)
        item=tarfile.TarInfo(name);item.size=len(data);tar.addfile(item,io.BytesIO(data))
    manifest=dict(files=inventory,source='r139/r142 saved comparison',scope='CPU-only no actuation')
    data=json.dumps(manifest,indent=2).encode()
    name=(A/'bundle_manifest.json').relative_to(ROOT).as_posix()
    item=tarfile.TarInfo(name);item.size=len(data);tar.addfile(item,io.BytesIO(data))
    (A/'bundle_manifest.json').write_bytes(data)
result=dict(archive=archive.relative_to(ROOT).as_posix(),files=len(files),size=archive.stat().st_size,
            sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
(A/'bundle_archive.json').write_text(json.dumps(result,indent=2),'utf8')
print(json.dumps(result))
