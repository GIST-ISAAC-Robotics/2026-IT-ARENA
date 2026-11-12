from pathlib import Path
import hashlib,json,tarfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
parent=ROOT/'artifacts/validation/2026-11-11/ros_ingress_r100_v1/probe.py'
helper=A/'publisher_source.py'
assert not helper.exists()
helper.write_text(parent.read_text().replace("TOPIC='/arena_probe_r100/imu'","TOPIC='/arena_probe_r232/imu'"))
trace=ROOT/'artifacts/validation/2026-11-12/traffic_solo_gap_runtime_r225_v1/two_actor_run1/run1/follow_decisions.jsonl'
files=[p for folder in ('arena_autonomy','arena_vehicle_interface')
       for p in (ROOT/'src'/folder/folder).rglob('*.py')]
files += [A/'probe.py',helper,trace]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=dict(trace=trace.relative_to(ROOT).as_posix(),trace_sha256=sha(trace),
    publisher_parent=parent.relative_to(ROOT).as_posix(),publisher_parent_sha256=sha(parent),
    files={p.relative_to(ROOT).as_posix():dict(size=p.stat().st_size,sha256=sha(p)) for p in files})
(A/'bundle_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
bundle=ROOT/'build/combined_ros_cost_r232_20261112.tar.gz'
with tarfile.open(bundle,'x:gz') as out:
    for p in files+[A/'bundle_manifest.json']:out.add(p,arcname=p.relative_to(ROOT).as_posix())
print(json.dumps(dict(files=len(files),bytes=bundle.stat().st_size,sha256=sha(bundle))))
