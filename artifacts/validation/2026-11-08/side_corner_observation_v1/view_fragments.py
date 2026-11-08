"""Saved sensor-only frames, independent wall fitting display. No controller or Gazebo execution."""
import hashlib,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_path import local_path
from fragment_candidate import propose
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sources=[('r28','side_corner_live_v1',[33.52,35.52,36.42]),('r29','traffic_boundary_refresh_v1',[34.04,36.02,37.42])]
fig,axes=plt.subplots(2,3,figsize=(14,9),constrained_layout=True)
rows=[]
for r,(name,folder,targets) in enumerate(sources):
    src=HERE.parent/folder/'right_parallel_first_curve/run1/follow_decisions.jsonl'
    digest=hashlib.sha256(src.read_bytes()).hexdigest()
    chosen={}
    for line in src.open():
        e=json.loads(line)
        if e['kind']=='header':mc=MotionConfig(**e['data']['motion_config']);cfg=e['data']['follow_config']
        if e['kind']=='call':
            d=e['data'];t=d['input']['reference_s']
            for target in targets:
                if abs(t-target)<.011 and (target not in chosen or abs(t-target)<abs(chosen[target]['input']['reference_s']-target)):chosen[target]=d
    for j,target in enumerate(targets):
        d=chosen[target];i,scan,h=restore_call(d['input'],mc)
        s=motion_scene(scan,h,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
        pts=np.asarray(s.occupied_endpoints_xy_m);ax=axes[r,j]
        ax.scatter(pts[:,0],pts[:,1],s=3,c='#444444',label='returned points')
        row=dict(source=src.relative_to(ROOT).as_posix(),sha256=digest,t=i['reference_s'],v=i['speed_mps'],paths={})
        for side,color in [('left','#d97706'),('right','#2563eb')]:
            path,info=local_path(pts,side,cfg['wall_offset_m'],horizon=1.8,piecewise_corner=True,minimum_radius=cfg['wheelbase_m']/math.tan(cfg['max_steering_rad']))
            row['paths'][side]={k:v for k,v in info.items() if k!='path_points'}
            if path is not None:ax.plot(path[:,0],path[:,1],c=color,lw=2,label=side+' path')
        candidate,evidence=propose(pts,'right',cfg['wall_offset_m'],cfg['wheelbase_m']/math.tan(cfg['max_steering_rad']))
        row['fragment_candidate']={k:v for k,v in evidence.items() if k not in ('source_indices','occluder_indices')}
        if candidate is not None:ax.plot(candidate[:,0],candidate[:,1],c='#be185d',lw=2,ls='--',label='fragment hypothesis')
        ax.add_patch(Rectangle((-.1,-.075),.2,.15,facecolor='#14b8a6',alpha=.8))
        ax.scatter([.06],[0],c='red',s=25)
        ax.set(xlim=(-.3,2.5),ylim=(-1.2,1.4),aspect='equal',title=f'{name} t={i["reference_s"]:.2f}s v={i["speed_mps"]:.2f}',xlabel='forward x (m)',ylabel='left y (m)')
        ax.grid(alpha=.25);ax.legend(fontsize=7,loc='upper right');rows.append(row)
    assert hashlib.sha256(src.read_bytes()).hexdigest()==digest
fig.savefig(HERE/'fragment_walls_v4.png',dpi=170)
(HERE/'fragment_summary_v4.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(rows,ensure_ascii=False))
