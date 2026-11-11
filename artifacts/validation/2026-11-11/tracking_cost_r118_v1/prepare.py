"""센서 궤적을 명시한 r117 단일 비용 묶음. 자격 증명/실물 로그는 포함하지 않는다."""
import hashlib
import io
import json
from pathlib import Path
import tarfile

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sha=lambda b:hashlib.sha256(b).hexdigest()
trace='artifacts/validation/2026-11-11/fresh_submission_r114_v1/two_actor_run1/run1/follow_decisions.jsonl'
files=[p.relative_to(ROOT).as_posix() for package in ('arena_autonomy','arena_vehicle_interface')
       for p in (ROOT/'src'/package/package).glob('*.py')]
files += [trace]
files += ['artifacts/validation/2026-11-11/current_tracking_r117_v1/'+n
          for n in ('candidate.py','probe.py','details.jsonl')]
files += [A.relative_to(ROOT).as_posix()+'/measure.py']
archive=ROOT/'build/jetson_tracking_r118_20261111.tar.gz'
frozen=ROOT/'build/r118_tracking_frozen'
assert len(files)==len(set(files)) and not archive.exists() and not frozen.exists()
inventory={}
frozen.mkdir()
with tarfile.open(archive,'x:gz',compresslevel=1) as tar:
    for name in files:
        data=(ROOT/name).read_bytes()
        inventory[name]=dict(size=len(data),sha256=sha(data))
        item=tarfile.TarInfo(name);item.size=len(data)
        tar.addfile(item,io.BytesIO(data))
        out=frozen/name;out.parent.mkdir(parents=True,exist_ok=True);out.write_bytes(data)
manifest=dict(files=inventory,archive_sha256=sha(archive.read_bytes()),archive_size=archive.stat().st_size,
    scope='r117 exact candidate and r114 saved trace; no live sensors/ROS actuation or power settings')
data=(json.dumps(manifest,indent=2)+'\n').encode()
(A/'bundle_manifest.json').write_bytes(data)
dest=frozen/A.relative_to(ROOT)/'bundle_manifest.json';dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(data)
print(json.dumps({k:v for k,v in manifest.items() if k!='files'}))
