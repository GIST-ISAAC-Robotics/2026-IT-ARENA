"""현 M2 주입 시험 소스/설치본·자기 시험 잔존 확인. 새 주행을 시작하지 않는다."""
import hashlib
import importlib
import json
import os
from pathlib import Path
import sys
import zipfile

root=Path.cwd(); sys.path.insert(0,str(root/'scripts'))
import validate_g1_traffic as runner
out=Path(sys.argv[2]); phase=sys.argv[1]
digest=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
paths=runner.source_paths('config/tests/g1_curve_stop_restart_v1.json',motion_profile='config/tests/motion_combined.json')
paths+=['config/tests/m2_motion_stress_r0_sha256.json','scripts/replay_follow_trace.py','scripts/surface_contact_eval.py',
        'tests/test_g1_motion_stress.py','src/arena_autonomy/arena_autonomy/m3_guarded_return_trial.py',
        'src/arena_autonomy/arena_autonomy/m3_return_guard.py',
        'artifacts/validation/2026-10-25/m3_execution_review_v1/evaluate_curve_stop.py',
        'artifacts/validation/2026-10-25/m3_execution_review_v1/curve_stop_preflight.py']
paths=sorted(set(paths))
current={p:digest(root/p) for p in paths}
if phase=='before':
    out.mkdir(parents=True,exist_ok=True)
    installed={}
    for package,names in [('arena_autonomy',('local_pursuit','local_follow','follow_trace','lateral_intent',
            'observation_support','scan_topology','m3_execution_trial','m3_guarded_return_trial','race_session','lidar_safety')),
            ('arena_vehicle_interface',('motion_fault_relay','motion_impairment','drive_feedback','sim_wheel_encoder','actuation_ros'))]:
        for name in names:
            module=importlib.import_module(package+'.'+name); path=Path(module.__file__)
            source=root/'src'/package/package/(name+'.py')
            installed[package+'.'+name]=dict(path=str(path),sha256=digest(path),matches_source=digest(path)==digest(source))
    (out/'installed_modules.json').write_text(json.dumps(installed,indent=2))
    if not all(v['matches_source'] for v in installed.values()): raise SystemExit('installed source mismatch')
    with zipfile.ZipFile(out/'execution_sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in paths:z.write(root/p,p)
    (out/'pre_run_hashes.json').write_text(json.dumps(current,indent=2))
elif phase=='after':
    old=json.loads((out/'pre_run_hashes.json').read_text())
    differences={p:[old.get(p),current.get(p)] for p in old.keys()|current.keys() if old.get(p)!=current.get(p)}
    (out/'source_hash_diff.json').write_text(json.dumps(differences,indent=2))
    if differences:raise SystemExit('source changed during run')
else: raise SystemExit('unknown phase')
found=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name)==os.getpid():continue
    try:
        cmd=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        comm=(p/'comm').read_text().strip()
    except OSError:continue
    if comm in ('bash','sh','timeout'):continue
    if any(t in cmd for t in ('gz sim','ros2 launch','ros_gz_bridge','validate_g1_traffic.py','local_pursuit')):
        found.append(dict(pid=int(p.name),cmd=cmd))
(out/(phase+'_processes.json')).write_text(json.dumps(found,indent=2))
print(json.dumps(dict(phase=phase,sources=len(current),processes=found)))
if found:raise SystemExit('existing or residual processes; inspect ownership')
