"""실패 입력의 현재 벽 경계 후보를 복원. 운영 상태/주행 판정은 변경하지 않는다."""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
for part in ('src/arena_autonomy','src/arena_vehicle_interface'):
    sys.path.insert(0,str(ROOT/part))
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,path_agrees
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.corner_corridor import corridor_evidence


def main():
    out=OUT/'boundary_diagnosis_v1';out.mkdir(exist_ok=False)
    wanted=[113.4,113.62,114.,120.]
    trace=OUT/'run1_course/follow_decisions.jsonl'
    found=[]
    with trace.open() as f:
        header=json.loads(next(f))['data']
        cfg=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
        for line in f:
            row=json.loads(line)
            if row['kind']!='call':continue
            if not any(abs(row['data']['input']['reference_s']-t)<1e-7 for t in wanted):continue
            d,scan,history=restore_call(row['data']['input'],motion)
            scene=motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],
                epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
            points=np.asarray(scene.occupied_endpoints_xy_m)
            paths={};sides={}
            for side in ('left','right'):
                path,info=local_path(points,side,cfg.wall_offset_m);paths[side]=path
                sides[side]=dict(info,sufficient=bool(path is not None and
                    info['boundary_observed_ahead_m']>=cfg.boundary_min_ahead_m and
                    abs(info['wall_distance_m']-cfg.wall_offset_m)<=cfg.boundary_anchor_error_m))
            corridor=corridor_evidence(points,d['side'],cfg)
            detail=dict(t_s=d['reference_s'],preferred=d['side'],speed_mps=d['speed_mps'],
                raw_recorded_reason=row['data']['output'][2]['reason'],
                both_agree=path_agrees(paths['left'],paths['right'],cfg.boundary_agreement_m)
                    if all(p is not None for p in paths.values()) else False,
                points=points.tolist(),sides=sides,corridor=corridor)
            found.append(detail)
    assert len(found)==len(wanted)
    fig,axes=plt.subplots(2,2,figsize=(12,10))
    for row,ax in zip(found,axes.flat):
        p=np.asarray(row['points']);ax.scatter(p[:,0],p[:,1],s=3,c='black',label='measured returns')
        for side,color in (('left','tab:blue'),('right','tab:orange')):
            s=row['sides'][side]
            if 'path_points' in s:
                q=np.asarray(s['path_points']);ax.plot(q[:,0],q[:,1],c=color,label=side+' offset path')
        ax.plot([0,.25],[0,0],c='tab:red',lw=2)
        ax.set(xlim=(-.6,2.),ylim=(-1.3,1.3),aspect='equal',
            title=f"t={row['t_s']:.2f}s preferred={row['preferred']}",xlabel='forward (m)',ylabel='left (m)')
        ax.grid(alpha=.3);ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(out/'sensor_paths.png',dpi=150);plt.close(fig)
    inputs=[Path(__file__),trace,ROOT/'src/arena_autonomy/arena_autonomy/local_path.py',
            ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py',
            ROOT/'src/arena_autonomy/arena_autonomy/corner_corridor.py']
    result=dict(scope='stateless current scan candidate diagnosis; no command or closed-loop claim',rows=found,
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs})
    (out/'report.json').write_text(json.dumps(freeze(result),ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps([dict(t=r['t_s'],sides={k:{key:v for key,v in d.items() if key!='path_points'}
        for k,d in r['sides'].items()},corridor_reason=r['corridor']['reason'],agree=r['both_agree']) for r in found]))


if __name__=='__main__':main()
