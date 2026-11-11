"""재현용 소스·원 저장 궤적만 담은 휴대 묶음. 인증정보/장치 로그 미포함."""
import hashlib
import io
import json
from pathlib import Path
import tarfile

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sha=lambda data:hashlib.sha256(data).hexdigest()
files=[p.relative_to(ROOT).as_posix() for package in ('arena_autonomy','arena_vehicle_interface')
       for p in (ROOT/'src'/package/package).glob('*.py')]
prefix='artifacts/validation/2026-11-11/'
files += [prefix+'fresh_submission_r114_v1/two_actor_run1/run1/follow_decisions.jsonl']
files += [prefix+'current_tracking_r117_v1/'+n for n in ('candidate.py','probe.py','details.jsonl')]
files += [prefix+'native_depth_r121_v1/'+n for n in ('evaluate.cpp','native_candidate.py','compare.py')]
files += [A.relative_to(ROOT).as_posix()+'/measure.py']
archive=ROOT/'build/jetson_native_depth_r122_20261111.tar.gz'
frozen=ROOT/'build/r122_native_depth_frozen'
assert len(files)==len(set(files)) and not archive.exists() and not frozen.exists()
inventory={};frozen.mkdir()
with tarfile.open(archive,'x:gz',compresslevel=1) as tar:
    for name in files:
        data=(ROOT/name).read_bytes()
        inventory[name]=dict(size=len(data),sha256=sha(data))
        item=tarfile.TarInfo(name);item.size=len(data);tar.addfile(item,io.BytesIO(data))
        path=frozen/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
manifest=dict(files=inventory,archive_sha256=sha(archive.read_bytes()),archive_size=archive.stat().st_size,
    scope='r121 C++ depth experiment; source-bound r114 saved trace; no sensors or actuation')
data=(json.dumps(manifest,indent=2)+'\n').encode()
(A/'bundle_manifest.json').write_bytes(data)
(frozen/A.relative_to(ROOT)/'bundle_manifest.json').write_bytes(data)
print(json.dumps({k:v for k,v in manifest.items() if k!='files'}))
