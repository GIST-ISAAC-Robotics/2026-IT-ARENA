"""현재 snapshot/투영/재검사의 저장 입력 CPU 대조. ROS·장치 접근 없음."""
from pathlib import Path
import hashlib,json,platform,statistics,time
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call,snapshot_call,freeze,equal_output
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy import guard_observation_policy as policy
from arena_autonomy.lidar_motion import MotionUnavailable

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
sources={p.relative_to(ROOT).as_posix():sha(p) for p in (ROOT/'src').rglob('*.py')}
sample_sha=sha(A/'samples.jsonl')
rows=[json.loads(line) for line in (A/'samples.jsonl').open()]
owner,motion=make_controller(rows[0]['header'])
prepared=[(row,restore_call(row['current'],motion)) for row in rows[1:]]
baseline={};records=[];errors=[];commands=[];snapshots=[];repeat_errors=[]
for repeat in range(2):
    for row,(data,scan,history) in prepared:
        began=time.perf_counter()
        current=snapshot_call(scan,history,data['reference_s'],data['epoch'],data['speed_mps'],
            data['side'],data['previous_steering_rad'],data['dt_s'],data['timing_verified'],data['rear_half_angle_deg'])
        copied=time.perf_counter()
        try:
            scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
            projected=time.perf_counter()
            answer=policy.rebase(row['original']['output'],row['original']['input'],current,scene,history,owner.config)
        except (MotionUnavailable,ValueError) as error:
            projected=time.perf_counter()
            answer=dict(speed_mps=0.,steering_rad=data['previous_steering_rad'],reference_s=data['reference_s'],
                reason='rebase_input_stop',input_error=str(error),independent_guard_required=True,runtime_authorized=False)
        answer=freeze(answer);finished=time.perf_counter()
        if repeat==0:
            baseline[row['seq']]=answer
            if not equal_output(current,row['current']):snapshots.append(row['seq'])
            if not equal_output(answer,row['expected']):errors.append(row['seq'])
            keys=('speed_mps','steering_rad','reason','reference_s')
            if not equal_output([answer.get(k) for k in keys],[row['expected'].get(k) for k in keys]):commands.append(row['seq'])
        else:
            if not equal_output(answer,baseline[row['seq']]):repeat_errors.append(row['seq'])
            elapsed=finished-began
            first=current['scan']['measurements']['acquired_at_s'][0]
            records.append(dict(seq=row['seq'],positive=answer['speed_mps']>0,reason=answer['reason'],
                total_ms=elapsed*1000,snapshot_ms=(copied-began)*1000,projection_ms=(projected-copied)*1000,
                recheck_ms=(finished-projected)*1000,current_first_age_s=data['reference_s']-first,
                hypothetical_age_after_cpu_s=data['reference_s']-first+elapsed))
def stats(values):
    v=sorted(values)
    return dict(count=len(v),p50=statistics.median(v),p95=v[min(len(v)-1,int(.95*len(v)))],maximum=max(v),
                over20=sum(t>20 for t in v),over100=sum(t>100 for t in v)) if v else dict(count=0)
after={p.relative_to(ROOT).as_posix():sha(p) for p in (ROOT/'src').rglob('*.py')}
result=dict(calls=len(records),pc_full_mismatches=errors,pc_command_mismatches=commands,
    snapshot_mismatches=snapshots,same_platform_output_mismatches=repeat_errors,
    source_unchanged=sources==after,input_unchanged=sha(A/'samples.jsonl')==sample_sha,
    input_sha256=sample_sha,source_sha256=sources,
    platform=dict(machine=platform.machine(),python=platform.python_version()),
    all_ms=stats([r['total_ms'] for r in records]),positive_ms=stats([r['total_ms'] for r in records if r['positive']]),
    positive_stages={k:stats([r[k] for r in records if r['positive']]) for k in ('snapshot_ms','projection_ms','recheck_ms')},
    hypothetical_current_scan_over_0p3=sum(r['hypothetical_age_after_cpu_s']>=.3 for r in records),
    rows=records,scope='preloaded recorded current recheck CPU only; excludes restore/ROS/vision/guard/planner; no live timing or actuation')
result['passed']=result['source_unchanged'] and result['input_unchanged'] and not any((errors,commands,snapshots,repeat_errors))
with (A/'report.json').open('x') as out:json.dump(result,out,indent=2);out.write('\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('rows','source_sha256')}),flush=True)
raise SystemExit(0 if result['passed'] else 1)
