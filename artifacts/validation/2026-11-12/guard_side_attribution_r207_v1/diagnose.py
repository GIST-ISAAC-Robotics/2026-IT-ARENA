"""r203의 곡선 정체 입력만 관찰한다. 정답 기하는 평가에만 쓰며 제어 출력을 바꾸지 않는다."""
from collections import Counter
from pathlib import Path
import hashlib,importlib.util,json,math
import numpy as np
from shapely.geometry import Point
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from surface_contact_eval import SurfaceMap
from build_experimental_track import rectangle

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
RUN=A.parent/'released_mount_runtime_r203_v1/two_actor_run1/run1'
trace=RUN/'follow_decisions.jsonl'
digest=hashlib.sha256(trace.read_bytes()).hexdigest()
assert digest=='f4c939829cb7b0a90d07f13f319eac69054c1e87211edc29d06720fa11ad6a93'
events=[json.loads(x) for x in trace.open()]
targets={e['seq']:e['data'] for e in events if e['kind']=='call' and
    e['data']['output'][2].get('normal',{}).get('selector_reason')=='guard_curve_rejected_toward_left'}
assert len(targets)==52
spec=importlib.util.spec_from_file_location('r207_route',A.parent/'expiry_route_reference_r186_v1/candidate_v2.py')
route=importlib.util.module_from_spec(spec);spec.loader.exec_module(route);route.install()
owner,motion=make_controller(events[0]['data'])
original_view,original_approach=ps.threat_view,ps.observed_guard_approach
current_seq=None;views=[];approaches=[]
def view(scene,path,cfg):
    result=original_view(scene,path,cfg)
    if current_seq in targets and result is not None:
        grid=ps.mx.ReturnGrid(scene,cfg)
        ds,masks=ps.mx.side_return_masks(result['frame'],result['state'],grid,cfg)
        rays=[r for r in scene.rays if r.status=='return']
        _,lateral=result['frame'].project(grid.returns)
        sides={}
        for name,mask in zip(('left','right'),masks):
            sides[name]=[dict(source_index=rays[i].source_index,acquired_at_s=rays[i].acquired_at_s,
                xy=list(rays[i].hit_xy_m),ds_m=float(ds[i]),lateral_m=float(lateral[i]),
                assumed_wall_distance_m=abs(abs(float(lateral[i]))-cfg.wall_offset_m))
                for i in np.flatnonzero(mask)]
        views.append(dict(seq=current_seq,reference_s=scene.t_ref_s,sides=sides,
                          wall_offset_m=cfg.wall_offset_m,wall_tolerance_m=cfg.wall_tolerance_m))
    return result
def approach(frame,context,guarded,rejected):
    result=original_approach(frame,context,guarded,rejected)
    if current_seq in targets and context and len(context['queries']):
        _,lat=frame.project(context['queries']);rows={}
        for name in rejected:
            sign=1 if name=='left' else -1
            beyond=sign*lat>guarded[name]['current_m']+1e-9
            masked=context.get('masked',np.zeros(len(lat),dtype=bool))
            rows[name]=dict(total=int(beyond.sum()),free=int(np.count_nonzero(beyond&context['free'])),
                masked=int(np.count_nonzero(beyond&~context['free']&masked)),
                unknown=int(np.count_nonzero(beyond&~context['free']&~masked)))
        approaches.append(dict(seq=current_seq,accepted=result['accepted'],sides=rows))
    return result
ps.threat_view,ps.observed_guard_approach=view,approach
mismatch=[];count=0
for e in events[1:]:
    d=e['data'];current_seq=e['seq']
    if e['kind']=='reset':
        if d.get('reason')=='decision_source_expired':route.reset_reference(owner)
        else:owner.reset()
    if e['kind']!='call':continue
    output=freeze(replay_call(owner,d['input'],motion));count+=1
    if not equal_output(comparable_decision(output),comparable_decision(d['output'])):mismatch.append(current_seq)
ps.threat_view,ps.observed_guard_approach=original_view,original_approach
assert not mismatch,mismatch
# 제어기 재생을 모두 마친 뒤에만 평가용 정답 자세를 불러온다.
ego=np.asarray([json.loads(x) for x in (RUN/'ego_poses.jsonl').open()]);ego[:,3]=np.unwrap(ego[:,3])
actors={'primary':np.asarray([json.loads(x) for x in (RUN/'actor_poses.jsonl').open()]),
        **{name:np.asarray(rows) for name,rows in json.loads((RUN/'additional_actors.json').read_bytes())['poses'].items()}}
for rows in actors.values():rows[:,3]=np.unwrap(rows[:,3])
def pose(rows,t):
    i=int(np.searchsorted(rows[:,0],t));assert 0<i<len(rows) and rows[i,0]-rows[i-1,0]<=.05
    return [float(np.interp(t,rows[:,0],rows[:,axis])) for axis in (1,2,3)]
walls=SurfaceMap.from_world(RUN/'runtime_world.sdf').walls
selected=[]
for seq,data in targets.items():
    expected=data['output'][2]['normal']['selector_evidence']
    choices=[v for v in views if v['seq']==seq and all(len(v['sides'][n])==expected[n]['points'] for n in ('left','right'))]
    assert len(choices)==1,(seq,len(choices))
    v=choices[0];x,y,yaw=pose(ego,v['reference_s']);c,s=math.cos(yaw),math.sin(yaw)
    for points in v['sides'].values():
        for p in points:
            px,py=p['xy'];q=Point(x+c*px-s*py,y+s*px+c*py)
            p['evaluation_wall_distance_m']=q.distance(walls)
            p['evaluation_actor_distance_m']={name:q.distance(rectangle(*pose(rows,p['acquired_at_s']),.2,.15))
                                              for name,rows in actors.items()}
    selected.append(v)
points=[p for v in selected for side in v['sides'].values() for p in side]
wall_dist=[p['evaluation_wall_distance_m'] for p in points]
actor_dist=[min(p['evaluation_actor_distance_m'].values()) for p in points]
summary=dict(replayed_calls=count,mismatches=mismatch,target_calls=len(targets),side_points=len(points),
    wall_distance_m=dict(min=min(wall_dist),median=float(np.median(wall_dist)),max=max(wall_dist)),
    actor_distance_m=dict(min=min(actor_dist),median=float(np.median(actor_dist)),max=max(actor_dist)),
    wall_near_actor_far_points=sum(w<=.02 and a>=.3 for w,a in zip(wall_dist,actor_dist)),
    approach_checks=len(approaches),approach_accepted=sum(a['accepted'] for a in approaches),
    trace_sha256=digest,world_sha256=hashlib.sha256((RUN/'runtime_world.sdf').read_bytes()).hexdigest(),
    scope='evaluation association only; projected scan includes odometry error; no controller truth input or new runtime')
with (A/'diagnosis.json').open('x') as f:json.dump(dict(summary=summary,views=selected,approaches=approaches),f,indent=2);f.write('\n')
print(json.dumps(summary))
