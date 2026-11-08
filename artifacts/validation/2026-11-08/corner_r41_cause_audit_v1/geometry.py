"""같은 기록의 대표 관측 그림과 조기 측방 거부 뒤 실제 이동 영역 검사."""
import json
from collections import Counter
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

from arena_autonomy.follow_trace import FollowConfig, MotionConfig, restore_call, freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.pass_release_trial import motion_region, lateral_config, region_blocked, SelectorConfig

A=Path(__file__).resolve().parent
captures=json.loads((A/'captures.json').read_bytes())
fig,axes=plt.subplots(2,2,figsize=(12,10),constrained_layout=True)
chosen=[]
for ax,t in zip(axes.flat,[33.52,35.82,36.02,39.42]):
    rec=min(captures,key=lambda q:abs(q['t']-t)); chosen.append(rec['t'])
    pts=np.asarray([r['hit'] for r in rec['rays'] if r['status']=='return'])
    ax.scatter(pts[:,0],pts[:,1],s=7,c='#202a32',label='Measured LiDAR returns',zorder=3)
    for status,color in [('masked','#ffbd59'),('unknown','#ea67c7')]:
        first=True
        for ray in rec['rays']:
            if ray['status']!=status:continue
            o=np.asarray(ray['origin']);e=o+1.4*np.asarray(ray['direction'])
            ax.plot([o[0],e[0]],[o[1],e[1]],color=color,alpha=.15,
                    label=(status+' ray') if first else None)
            first=False
    for path,label,color,style in [(rec['carried_path'],'Transported previous path','#c536a7','--'),
        (rec['baseline']['detail'].get('path_points'),'Current accepted path','#2375c4','-')]:
        if path is not None:
            p=np.asarray(path);ax.plot(p[:,0],p[:,1],style,color=color,lw=2,label=label)
    ax.add_patch(Rectangle((-.1,-.075),.2,.15,fc='#71bb93',ec='black',zorder=5))
    ax.annotate('',xy=(.24,0),xytext=(.1,0),arrowprops={'arrowstyle':'->','color':'#006d42'})
    ax.set(xlim=(-.45,2.),ylim=(-1.05,1.7),aspect='equal',xlabel='Forward x (m)',ylabel='Left y (m)',
        title=f"t={rec['t']:.2f}s | {rec['baseline']['detail'].get('reason')}")
    ax.grid(alpha=.2);ax.legend(loc='upper right',fontsize=7)
fig.savefig(A/'corner_observations.png',dpi=150);plt.close(fig)

trace=A.parent/'current_corner_support_v1/run1/follow_decisions.jsonl'
rows=[];counts=Counter();scans={}
for line in trace.open():
    e=json.loads(line);d=e['data']
    if e['kind']=='header':cfg=FollowConfig(**d['follow_config']);mc=MotionConfig(**d['motion_config'])
    if e['kind']!='call':continue
    out=d['output'];normal=out[2].get('normal',{});t=d['input']['reference_s']
    if t>=33 and out[0]==0:
        key=(normal.get('baseline_reason'),normal.get('selector_reason'))
        counts[str(key)]+=1
        src=normal.get('source_first_s');scans.setdefault(str(key),set()).add(src)
    why=normal.get('selector_reason','')
    if not why.startswith('guard_curve_rejected_toward_'):continue
    info=normal['selector_guard_candidate'];args,scan,hist=restore_call(d['input'],mc)
    scene=motion_scene(scan,hist,t,timing_verified=args['timing_verified'],epoch=args['epoch'],
        rear_half_angle_deg=args['rear_half_angle_deg'])
    region=motion_region(scene,args['speed_mps'],info['target_speed_mps'],info['steering_rad'],
        info['previous_steering_rad'],args['dt_s'],cfg,lateral_config(cfg))
    rows.append(dict(t=t,source=scene.source_first_s,guarded_sides=info['guarded_sides'],region=region,
        region_blocked=region_blocked(region,SelectorConfig()),
        fully_observed=bool(region['queries'] and not any(region[k] for k in ('unobserved','masked_left','masked_right')))))
(A/'geometry_summary.json').write_text(json.dumps(freeze(dict(fig_times=chosen,zero_calls=counts,
    zero_scans={k:len(v) for k,v in scans.items()},toward_side_regions=rows)),indent=2))
print(json.dumps(dict(fig_times=chosen,toward_side_calls=len(rows),
    fully_observed=sum(r['fully_observed'] for r in rows),region_allowed=sum(r['region_blocked'] is None for r in rows))))
