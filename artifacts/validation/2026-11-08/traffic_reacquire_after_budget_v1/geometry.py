"""코너의 수평 벽좌표와 실제 경로 최근접 거리를 구별한다. 판단 변경 없음."""
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, path_agrees
from arena_autonomy.local_path import local_path

HERE=Path(__file__).resolve().parent
SOURCE=HERE.parent/'traffic_lap_budget_v1/two_actor_course_lap/run1/follow_decisions.jsonl'
targets=[43.42,43.72,44.12,47.92]
fig,axes=plt.subplots(1,4,figsize=(16,5));rows=[]
with SOURCE.open() as f:
    h=json.loads(next(f))['data'];cfg=FollowConfig(**h['follow_config']);mc=MotionConfig(**h['motion_config'])
    for line in f:
        e=json.loads(line)
        if e['kind']!='call':continue
        d=e['data'];t=d['input']['reference_s']
        if t<targets[0]-1e-6:continue
        targets.pop(0);raw,scan,history=restore_call(d['input'],mc)
        sc=motion_scene(scan,history,t,timing_verified=raw['timing_verified'],epoch=raw['epoch'],rear_half_angle_deg=raw['rear_half_angle_deg'])
        points=np.asarray(sc.occupied_endpoints_xy_m)
        ax=axes[len(rows)];ax.scatter(*points.T,s=3,c='gray');ax.arrow(0,0,.25,0,width=.01,color='black')
        r=dict(t=t,speed=raw['speed_mps'],walls=[]);paths=[]
        for side in ('left','right'):
            path,info=local_path(points,side,cfg.wall_offset_m,horizon=1.8,piecewise_corner=True,minimum_radius=cfg.wheelbase_m/math.tan(cfg.max_steering_rad))
            row=dict(side=side,info={k:v for k,v in info.items() if k!='path_points'})
            paths.append(path)
            if path is not None:
                near=int(np.argmin(np.linalg.norm(path,axis=1)))
                row.update(path_nearest_m=float(np.linalg.norm(path[near])),nearest_xy=path[near].tolist())
                ax.plot(*path.T,label=f'{side}: |path|={row["path_nearest_m"]:.3f}')
            r['walls'].append(row)
        r['path_agreement']=bool(path_agrees(paths[0],paths[1],cfg.boundary_agreement_m))
        ax.set_title(f't={t:.2f}s');ax.axis('equal');ax.set_xlim(-.7,2.2);ax.set_ylim(-1.6,1.3);ax.legend(fontsize=7);ax.grid()
        rows.append(r)
        if not targets:break
fig.tight_layout();fig.savefig(HERE/'anchor_geometry.png',dpi=160)
(HERE/'geometry.json').write_text(json.dumps(rows,indent=1)+'\n')
for r in rows:
    print(json.dumps(dict(t=r['t'],path_agreement=r['path_agreement'],walls=[dict(side=w['side'],distance=w['info'].get('wall_distance_m'),path_nearest=w.get('path_nearest_m'),model=w['info'].get('wall_model')) for w in r['walls']])))
