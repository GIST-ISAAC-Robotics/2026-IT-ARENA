"""실험 소스와 시뮬 센서 입력만 격리한다. 실제 센서/인증 정보 제외."""
import hashlib
import io
import json
from pathlib import Path
import tarfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
PREVIOUS=A.parent/'follow_rebase_r126_v1'
trace=(PREVIOUS/'two_actor_run1/run1/follow_decisions.jsonl').relative_to(ROOT).as_posix()
registration=json.loads((PREVIOUS/'two_actor_run1/registered.json').read_bytes())
files=[p.relative_to(ROOT).as_posix() for package in ('arena_autonomy','arena_vehicle_interface')
    for p in (ROOT/'src'/package/package).glob('*.py')]
files.extend([trace,(PREVIOUS/'rebase.py').relative_to(ROOT).as_posix(),
              (A/'measure.py').relative_to(ROOT).as_posix()])
archive=ROOT/'build/jetson_current_rebase_r128_20261111.tar.gz'
frozen=ROOT/'build/r128_current_rebase_frozen'
assert not archive.exists() and not frozen.exists() and len(files)==len(set(files))
frozen.mkdir()
inventory={}
with tarfile.open(archive,'x:gz',compresslevel=1) as tar:
    for name in files:
        data=(ROOT/name).read_bytes();digest=hashlib.sha256(data).hexdigest()
        if name in registration['extra_sources']:assert digest==registration['extra_sources'][name]
        inventory[name]=dict(size=len(data),sha256=digest)
        item=tarfile.TarInfo(name);item.size=len(data);tar.addfile(item,io.BytesIO(data))
        target=frozen/name;target.parent.mkdir(exist_ok=True,parents=True);target.write_bytes(data)
    manifest=dict(files=inventory,trace=trace,scope='r126 recorded current rebase, isolated CPU only')
    data=(json.dumps(manifest,indent=2)+'\n').encode()
    name=(A/'bundle_manifest.json').relative_to(ROOT).as_posix()
    item=tarfile.TarInfo(name);item.size=len(data);tar.addfile(item,io.BytesIO(data))
    (frozen/name).write_bytes(data);(A/'bundle_manifest.json').write_bytes(data)
result=dict(archive=archive.relative_to(ROOT).as_posix(),size=archive.stat().st_size,
    sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),files=len(inventory))
(A/'bundle_archive.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
