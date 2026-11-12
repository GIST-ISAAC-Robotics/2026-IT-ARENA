"""r212 후반 경계 거절의 저장 재현과 센서/경로 그림. 평가 정답 입력 없음."""
from pathlib import Path
from dataclasses import asdict
from collections import Counter
import hashlib,importlib.util,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy import local_follow as local
from arena_autonomy.local_path import local_path
from arena_autonomy.corner_distance import corner_distance_evidence
A=Path(__file__).resolve().parent;P=A.parent
trace=P/'static_side_runtime_r212_v1/two_actor_run1/run1/follow_decisions.jsonl'
events=[json.loads(s) for s in trace.open()]
case=json.loads((trace.parent/'report.json').read_bytes())
end=case['case_window']['end_sim_s'];calls=[e for e in events if e['kind']=='call' and e['data']['input']['reference_s']<=end]
blocks=[];block=[]
for e in calls:
    d=e['data'];normal=d['output'][2].get('normal',{})
    if d['output'][0]<=0 and normal.get('reason')=='follow_boundary_unconfirmed' and d['input']['reference_s']>30:
        block.append(e)
    else:
        if block:blocks.append(block);block=[]
if block:blocks.append(block)
blocks.sort(key=lambda b:len(b),reverse=True)
assert blocks
selected=[blocks[0][0]['seq'],blocks[0][-1]['seq']]
spec=importlib.util.spec_from_file_location('r215_route',P/'expiry_route_reference_r186_v1/candidate_v2.py')
route=importlib.util.module_from_spec(spec);spec.loader.exec_module(route);route.install()
original=local.LocalFollow._baseline_command
current_seq=None;captures=[]
def wrapped(owner,scene,history,speed,side,*args,**kwargs):
    if current_seq in selected:
        config=owner.config;points=np.asarray(scene.occupied_endpoints_xy_m,dtype=float)
        row=dict(seq=current_seq,reference_s=scene.t_ref_s,side=side,speed=speed,points=points.tolist(),
            config=asdict(config),old_path_wall=owner.path_wall,old_path_source=owner.path_source,
            old_path=None if owner.path is None else owner.path.tolist(),candidates={})
        ray_index=dict(zip(map(tuple,points.tolist()),(r.source_index for r in scene.rays if r.status=='return')))
        age=scene.t_ref_s-scene.source_first_s;v=config.open_route_speed_mps
        horizon=max(1.8,.06+config.standstill_gap_m+v*(age+.1)+v*v/4)
        for wall in ('left','right'):
            path,info=local_path(points,wall,config.wall_offset_m,horizon=horizon,
                rays=[ray_index[tuple(p)] for p in points.tolist()] if config.ray_wall_continuity else None,
                piecewise_corner=config.piecewise_wall_trial,
                minimum_radius=config.wheelbase_m/np.tan(config.max_steering_rad),
                narrow_occluder_bridge=config.narrow_occluder_bridge)
            candidate=dict(info=freeze(info),path=None if path is None else path.tolist())
            if path is not None:
                candidate['inner_evidence']=freeze(corner_distance_evidence(points,path,'right' if wall=='left' else 'left',config,detailed=True))
            row['candidates'][wall]=candidate
        captures.append(row)
    return original(owner,scene,history,speed,side,*args,**kwargs)
local.LocalFollow._baseline_command=wrapped
owner,mode=make_controller(events[0]['data']);errors=[];count=0
for event in events[1:]:
    d=event['data'];current_seq=event['seq']
    if event['kind']=='reset':
        if d.get('reason')=='decision_source_expired':route.reset_reference(owner)
        else:owner.reset()
    if event['kind']!='call':continue
    out=freeze(replay_call(owner,d['input'],mode));count+=1
    if not equal_output(comparable_decision(out),comparable_decision(d['output'])):errors.append(current_seq)
summary=dict(calls=count,mismatches=errors,selected=selected,
    blocks=[dict(count=len(b),first_s=b[0]['data']['input']['reference_s'],last_s=b[-1]['data']['input']['reference_s']) for b in blocks],
    trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),scope='saved sensor inputs and model hypotheses only; no new driving permission')
with (A/'diagnosis.json').open('x') as f:json.dump(dict(summary=summary,captures=captures),f,indent=2);f.write('\n')
for row in captures:
    fig,ax=plt.subplots(figsize=(7,7));p=np.asarray(row['points']);ax.scatter(p[:,0],p[:,1],s=9,c='black',label='C1 returns')
    if row['old_path'] is not None:
        old=np.asarray(row['old_path']);ax.plot(old[:,0],old[:,1],color='gray',ls='--',label='previous path (old frame)')
    for wall,color in [('left','royalblue'),('right','darkorange')]:
        path=row['candidates'][wall]['path']
        if path is not None:
            q=np.asarray(path);ax.plot(q[:,0],q[:,1],color=color,lw=2,label=wall+' candidate')
    ax.add_patch(plt.Rectangle((-.1,-.075),.2,.15,fill=False,color='crimson',lw=2,label='ego footprint'))
    ax.set(xlim=(-.6,2.3),ylim=(-1.5,1.5),xlabel='forward x (m)',ylabel='left y (m)',title=f"Recorded input t={row['reference_s']:.2f}s")
    ax.set_aspect('equal');ax.grid(alpha=.3);ax.legend(fontsize=8);fig.tight_layout();fig.savefig(A/f"sensor_paths_{row['seq']}.png",dpi=150);plt.close(fig)
print(json.dumps(summary));assert not errors
