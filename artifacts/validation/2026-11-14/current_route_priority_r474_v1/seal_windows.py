"""Windows에서 산출물 봉인. WSL/계산 재실행 없음."""
from pathlib import Path
import json,hashlib,datetime
A=Path(__file__).resolve().parent;root=A.parents[3]
load=lambda p:json.loads(p.read_text(encoding='utf-8'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=load(A/'source_manifest.json');summary=load(A/'replay_v2/summary.json')
stage_changed=[p for p,h in manifest['files'].items() if sha(A/'stage'/p)!=h]
common={p:sha(root/p)==h for p,h in manifest['files'].items()}
resource=dict(time_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    foreground_jobs_completed=True,foreground_exits={p.name:p.read_text(encoding='utf-8').strip() for p in A.glob('*.exit')},
    background_jobs_started=False,ros_or_gazebo_started=False,
    final_proc_query_verified=False,owned_pid_count=None,
    final_return_attempts=[dict(attempt=i,error='Wsl/Service/E_UNEXPECTED',exit_code=1,script_started=False) for i in (1,2)],
    restart_performed=False,returned_to_root=True)
(A/'resource_return.json').write_text(json.dumps(resource,indent=2)+'\n',encoding='utf-8')
receipt=dict(time_utc=resource['time_utc'],source_manifest_sha256=sha(A/'source_manifest.json'),
    stage_files=len(manifest['files']),stage_changed=stage_changed,
    common_stage_mismatches=[p for p,ok in common.items() if not ok],
    trace_sha256=sha(A.parent/'solo_full_lap_r469_v1/run1/follow_decisions.jsonl'),
    replay_trace_unchanged=summary['trace_unchanged'],marker_default_off=summary['marker_default_off'],
    root_owned_implementation_sha256={p:sha(A/'stage/src/arena_autonomy/arena_autonomy'/p) for p in manifest['root_owned_implementation']},
    native_sha256={p:h for p,h in manifest['files'].items() if p.endswith('.so')},
    focused_tests=16,existing_stop_tests=2,resource_return=resource)
(A/'final_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(stage_files=receipt['stage_files'],stage_changed=stage_changed,
    common_stage_mismatches=receipt['common_stage_mismatches'],source_manifest_sha256=receipt['source_manifest_sha256'])))
assert not stage_changed and not receipt['common_stage_mismatches'] and summary['trace_unchanged']
