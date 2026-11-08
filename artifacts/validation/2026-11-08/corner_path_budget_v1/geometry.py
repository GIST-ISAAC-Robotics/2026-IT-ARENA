"""실제 반환으로 벽 탐색 길이와 오프셋 경로 길이를 비교한다. 제어 변경 없음."""
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_path import local_path
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.corner_distance import corner_distance_evidence

HERE=Path(__file__).resolve().parent
SOURCE=HERE.parent/'traffic_lap_newcourse_v1/two_actor_course_lap/run1/follow_decisions.jsonl'
targets=[27.92,28.22,33.02]
rows=[]
fig,axes=plt.subplots(1,3,figsize=(15,5))
with SOURCE.open() as f:
    h=json.loads(next(f))['data'];mc=MotionConfig(**h['motion_config']);cfg=FollowConfig(**h['follow_config'])
    for line in f:
        e=json.loads(line)
        if e['kind']!='call':continue
        d=e['data'];t=d['input']['reference_s']
        if not targets or t<targets[0]-1e-6:continue
        target=targets.pop(0);ax=axes[len(rows)]
        raw,scan,history=restore_call(d['input'],mc)
        sc=motion_scene(scan,history,t,timing_verified=raw['timing_verified'],epoch=raw['epoch'],rear_half_angle_deg=raw['rear_half_angle_deg'])
        p=np.asarray(sc.occupied_endpoints_xy_m)
        ax.scatter(p[:,0],p[:,1],s=4,c='gray',label='actual returns')
        ax.plot(0,0,'ko');ax.arrow(0,0,.25,0,width=.01,color='black')
        entry=dict(t=t,paths=[])
        for horizon in (1.8,2.4,3.1):
            path,info=local_path(p,'right',cfg.wall_offset_m,horizon=horizon,piecewise_corner=True,minimum_radius=cfg.wheelbase_m/np.tan(cfg.max_steering_rad))
            evidence=None;ahead=None
            if path is not None:
                arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
                ahead=float(arc[-1]-arc[np.argmin(np.linalg.norm(path,axis=1))])
                evidence=corner_distance_evidence(p,path,'left',cfg,detailed=True)
                ax.plot(path[:,0],path[:,1],label=f'wall {horizon:.1f} / path {ahead:.2f}m')
            entry['paths'].append(dict(horizon=horizon,path_ahead_m=ahead,info=info,evidence=evidence))
        ax.set(xlim=(-.4,1.8),ylim=(-.8,1.8),title=f't={t:.2f}s',xlabel='forward x (m)',ylabel='left y (m)')
        ax.axis('equal');ax.set_xlim(-.4,1.8);ax.set_ylim(-.8,1.8);ax.grid();ax.legend(fontsize=7)
        rows.append(entry)
        if not targets:break
assert not targets
fig.tight_layout();fig.savefig(HERE/'observed_geometry.png',dpi=160)
(HERE/'geometry.json').write_text(json.dumps(dict(source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),rows=rows),indent=1)+'\n')
for row in rows:
    print(json.dumps(dict(t=row['t'],paths=[dict(horizon=v['horizon'],ahead=v['path_ahead_m'],model=v['info'].get('wall_model'),evidence=(v['evidence'] or {}).get('reason'),supported=(v['evidence'] or {}).get('supported')) for v in row['paths']])))
