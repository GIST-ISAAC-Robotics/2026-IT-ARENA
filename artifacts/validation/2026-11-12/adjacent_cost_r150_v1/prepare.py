"""현재 소스·저장 입력·PC 기대 결과를 CPU 전용 묶음으로 보존한다."""
import hashlib
import io
import json
from pathlib import Path
import tarfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
files=[p.relative_to(ROOT).as_posix() for package in ('arena_autonomy','arena_vehicle_interface')
    for p in (ROOT/'src'/package/package).glob('*.py')]
files += [p.relative_to(ROOT).as_posix() for p in (A/'measure.py',A/'prepare.py',
    A/'baseline_expected.jsonl',A/'candidate_expected.jsonl',
    A.parent/'guard_current_r143_v1/two_actor_run1/run1/follow_decisions.jsonl',
    A.parent/'adjacent_mount_gap_r149_v1/candidate.py',A.parent/'guard_horizon_r148_v1/candidate.py',
    A.parents[1]/'2026-11-11/guard_recheck_r139_v1/candidate.py',
    A.parents[1]/'2026-11-11/body_nonreturn_r136_v1/candidate.py')]
inventory={};archive=ROOT/'build/jetson_adjacent_cost_r150_20261112.tar.gz'
with tarfile.open(archive,'x:gz',compresslevel=1) as tar:
    for name in files:
        data=(ROOT/name).read_bytes();inventory[name]=dict(size=len(data),sha256=hashlib.sha256(data).hexdigest())
        item=tarfile.TarInfo(name);item.size=len(data);tar.addfile(item,io.BytesIO(data))
    manifest=dict(files=inventory,scope='CPU-only saved simulation inputs, no physical device access')
    data=json.dumps(manifest,indent=2).encode();name=(A/'bundle_manifest.json').relative_to(ROOT).as_posix()
    item=tarfile.TarInfo(name);item.size=len(data);tar.addfile(item,io.BytesIO(data))
    (A/'bundle_manifest.json').write_bytes(data)
result=dict(archive=archive.relative_to(ROOT).as_posix(),files=len(files),size=archive.stat().st_size,
    sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
(A/'bundle_archive.json').write_text(json.dumps(result,indent=2),'utf8');print(json.dumps(result))
