"""r279 현재 영역 거절의 현재 광선/기존 저속 위험 판정 대조. 주행 미연결."""
from pathlib import Path
import json,sys
import numpy as np
A=Path(__file__).resolve().parent;R=A.parents[3];B=A.parent/'opposite_reacquire_runtime_r279_v1'
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface'),
             str(A.parent/'retained_route_execution_r266_v1')]
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,forward_planning_speed
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.released_body_gap import check
from overlap import plan_reference
import candidate_policy

with (B/'two_actor_run1/run1/follow_decisions.jsonl').open() as f:
    header=json.loads(next(f))['data'];calls=[]
    for line in f:
        e=json.loads(line)
        if e['kind']!='call':continue
        c=e['data'];cur=c.get('current_rebase')
        if cur and cur['output'].get('reason')=='route_current_large_motion_region_unobserved':calls.append(c)
cfg=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
chosen=[calls[0],next(c for c in calls if c['input']['reference_s']>=54.),calls[-1]]
results=[]
for index,c in enumerate(chosen):
    current=c['current_rebase']['input'];d,scan,h=restore_call(current,motion)
    scene=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
    pre=candidate_policy.rebase(c['output'],plan_reference(c['input']),current,scene,h,cfg)
    assert pre and pre['speed_mps']>0
    speed=forward_planning_speed(cfg,d['speed_mps']);context={}
    region=ps.motion_region(scene,speed,pre['speed_mps'],pre['steering_rad'],d['previous_steering_rad'],d['dt_s'],cfg,ps.lateral_config(cfg),context=context)
    bounded=check(scene,scan,speed,pre['speed_mps'],pre['steering_rad'],d['previous_steering_rad'],d['dt_s'],cfg,ps.lateral_config(cfg),ps.SelectorConfig())
    unknown=context['queries'][~context['free']&~context['masked']]
    row=dict(reference_s=d['reference_s'],speed_mps=d['speed_mps'],pre={k:v for k,v in pre.items() if k!='path_points'},
        region=region,current_bounded_check=bounded,unknown_min_xy=unknown.min(axis=0).tolist(),unknown_max_xy=unknown.max(axis=0).tolist(),
        normal={k:c['output'][2].get('normal',{}).get(k) for k in ('selector_mode','selector_reason','selector_choice','partial_observation','selector_same_path_gap_risk_accepted','selector_released_body_gap')})
    results.append(row)
    if index==1:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        q=context['queries'];free=context['free'];masked=context['masked'];p=np.asarray(scene.occupied_endpoints_xy_m)
        fig,ax=plt.subplots(figsize=(9,7));ax.scatter(p[:,0],p[:,1],s=8,c='#526778',label='Current returns')
        for m,color,label in [(free,'#4b9b72','Observed'),(masked,'#b18a2f','Mount mask'),(~free&~masked,'#d33682','Unobserved')]:
            ax.scatter(q[m,0],q[m,1],s=7,c=color,label=label)
        ax.add_patch(plt.Rectangle((-.1,-.075),.2,.15,fill=False,color='black',linewidth=1.5))
        ax.set(xlim=(-.3,.7),ylim=(-.45,.45),xlabel='Forward x (m)',ylabel='Left y (m)',title=f'Current query region at {scene.t_ref_s:.3f}s')
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend();fig.tight_layout();fig.savefig(A/'current_query_region.png',dpi=160)
        (A/'example.json').write_text(json.dumps(c,indent=2)+'\n')
(A/'inspection.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps([dict(reference_s=r['reference_s'],speed_mps=r['speed_mps'],before_reason=r['pre']['reason'],before_speed=r['pre']['speed_mps'],region=r['region'],bounded=r['current_bounded_check'],unknown_min_xy=r['unknown_min_xy'],unknown_max_xy=r['unknown_max_xy']) for r in results]))
