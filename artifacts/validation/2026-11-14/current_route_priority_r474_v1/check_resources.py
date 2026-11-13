"""r474 자신의 계산 작업 회수와 동결 무결성만 확인한다."""
from pathlib import Path
import os,json,hashlib,datetime
A=Path(__file__).resolve().parent;root=A.parents[3]
manifest=json.loads((A/'source_manifest.json').read_text(encoding='utf-8'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
changed=[p for p,h in manifest['files'].items() if sha(A/'stage'/p)!=h]
owned=[]
for path in Path('/proc').glob('[0-9]*/cmdline'):
    try:
        argv=path.read_bytes().decode(errors='replace').split('\0')
        if int(path.parent.name)==os.getpid():continue
        if not argv or not Path(argv[0]).name.startswith(('python','pytest')):continue
        if any('current_route_priority_r474_v1' in arg for arg in argv):
            owned.append(dict(pid=int(path.parent.name),argv=argv))
    except (FileNotFoundError,PermissionError,ProcessLookupError):continue
implementation=['src/arena_autonomy/arena_autonomy/'+p for p in (
    'race_runtime_policy/current_route_priority.py','pass_release_trial.py',
    'race_runtime_policy/observation_policy.py','race_runtime_policy/current_region.py')]
result=dict(time_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),owned_processes=owned,
    stage_files=len(manifest['files']),stage_changed=changed,
    common_implementation_exact={p:sha(root/p)==sha(A/'stage'/p) for p in implementation},
    trace_sha256=sha(A.parent/'solo_full_lap_r469_v1/run1/follow_decisions.jsonl'),
    statuses={p.name:p.read_text(encoding='utf-8').strip() for p in A.glob('*.exit')},
    scope='r474 pure saved evaluator/pytest only; no ROS, Gazebo, device or Jetson started')
(A/'resource_return.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result));assert not owned and not changed and all(result['common_implementation_exact'].values())
