"""실패 당시 소스로 재생해 첫 코너의 경로와 미관측 이동 표본을 분리한다."""
import hashlib,json,sys,zipfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
BASE=HERE/'two_actor_run1/run1'
SNAP=ROOT/'build/r78_frozen_diagnosis'
OUT=HERE/'diagnosis_summary.json'
if SNAP.exists() or OUT.exists():raise SystemExit('preserve prior diagnosis')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
archive=HERE/'two_actor_run1/extra_sources.zip';archive_sha=sha(archive)
with zipfile.ZipFile(archive) as z:
    members=z.namelist()
    for name in members:
        p=(SNAP/name).resolve()
        if not p.is_relative_to(SNAP.resolve()):raise ValueError('outside snapshot')
        p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(z.read(name))
hashes={name:sha(SNAP/name) for name in members}
sys.path[:0]=[str(SNAP/'src/arena_autonomy'),str(SNAP/'src/arena_vehicle_interface'),str(SNAP/'scripts')]
import numpy as np
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
import arena_autonomy.pass_release_trial as selector
capture={'on':False,'parts':[]}
original_observed=LocalFollow._observed_command
original_region=selector.motion_region
def observed(self,*args,**kw):
    out=original_observed(self,*args,**kw)
    if capture['on']:capture['parts'].append(dict(stage='before_parallel',output=freeze(out)))
    return out
def region(scene,*args,**kw):
    context=kw.get('context')
    if capture['on'] and context is None:
        context={};kw['context']=context
    result=original_region(scene,*args,**kw)
    if capture['on']:
        details=dict(stage='motion_region',args=freeze(args[:5]),result=result)
        if context and 'queries' in context:
            free,masked=context['free'],context['masked'];queries=context['queries']
            unknown=queries[~free & ~masked]
            details.update(unobserved_queries=freeze(unknown),masked_queries=freeze(queries[masked]),
                rays=freeze([dict(status=r.status,origin=r.ray_origin_xy_m,direction=r.direction_xy,
                    hit=r.hit_xy_m,source_index=r.source_index,acquired_at_s=r.acquired_at_s)
                    for r in scene.rays]),padding=freeze(context['padding']))
        capture['parts'].append(details)
    return result
LocalFollow._observed_command=observed;selector.motion_region=region
original_view=selector.threat_view
def view(scene,path,cfg):
    answer=original_view(scene,path,cfg)
    if capture['on'] and answer is not None:
        light=selector.mx.ReturnGrid(scene,cfg)
        ds,masks=selector.mx.side_return_masks(answer['frame'],answer['state'],light,cfg)
        returns=[r for r in scene.rays if r.status=='return']
        record=dict(stage='side_view',time_s=scene.t_ref_s,path_points=freeze(path),state=freeze(answer['state']))
        for (name,_),mask in zip(selector.SIDES,masks):
            record[name]=dict(summary=freeze(answer[name]),points=freeze(light.returns[mask]),
                source_indices=[r.source_index for r,m in zip(returns,mask) if m],
                ds=freeze(ds[mask]))
        capture['parts'].append(record)
    return answer
selector.threat_view=view

targets=[32.44,32.46,33.06,40.,60.];rows=[];mismatches=[];calls=0
trace=BASE/'follow_decisions.jsonl';trace_sha=sha(trace)
for line in trace.open(encoding='utf-8'):
    e=json.loads(line);d=e['data']
    if e['kind']=='header':
        motion=MotionConfig(**d['motion_config'])
        follow=BypassHandoffFollow(FollowConfig(**d['follow_config']),
            HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
    elif e['kind']=='reset':follow.reset()
    elif e['kind']=='call':
        t=d['input']['reference_s'];capture.update(on=any(abs(t-v)<1e-6 for v in targets),parts=[])
        out=freeze(replay_call(follow,d['input'],motion));calls+=1
        if not equal_output(comparable_decision(out),comparable_decision(d['output'])):mismatches.append(e['seq'])
        if capture['on']:rows.append(dict(seq=e['seq'],time_s=t,parts=capture['parts'],output=out,
            speed_mps=d['input']['speed_mps'],previous_steering_rad=d['input']['previous_steering_rad']))
loaded={k:str(v.__file__) for k,v in sys.modules.items() if k.startswith(('arena_autonomy.','arena_vehicle_interface.')) and getattr(v,'__file__',None)}
outside={k:p for k,p in loaded.items() if not Path(p).is_relative_to(SNAP)}
summary=dict(scope='frozen r78 replay with observational hooks, no new commands or simulation',calls=calls,
    baseline_mismatches=mismatches,selected_times=[r['time_s'] for r in rows],archive_sha256=archive_sha,
    trace_sha256=trace_sha,snapshot_members=len(members),loaded_outside_snapshot=outside,
    unchanged=sha(trace)==trace_sha and sha(archive)==archive_sha and all(sha(SNAP/n)==h for n,h in hashes.items()))
(ROOT/'build/r78_diagnosis_details.json').write_text(json.dumps(freeze(rows),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
OUT.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False))
raise SystemExit(int(bool(mismatches) or not summary['unchanged']))
