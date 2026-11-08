"""첫 갱신 실패의 현재 경로와 보존 경로가 실제 공통 구간에서 같은지 비교."""
from pathlib import Path
import json
import numpy as np
from arena_autonomy.follow_trace import FollowConfig,freeze
from arena_autonomy.local_path import local_path,_path_ahead,_origin_error
from arena_autonomy.local_follow import path_agrees,path_agrees_arc,anchor_error,boundary_support,drop_protrusions
from arena_autonomy.fragmented_wall_hypothesis import propose

A=Path(__file__).resolve().parent
trace=A.parent/'corner_slowing_actor_v1/run1/follow_decisions.jsonl'
with trace.open() as f:cfg=FollowConfig(**json.loads(next(f))['data']['follow_config'])
captures=json.loads((A/'captures.json').read_bytes());results=[]
for q in captures:
    if not 34.8 <= q['t'] <= 43.8: continue
    t=q['t']
    p=np.asarray([r['hit'] for r in q['rays'] if r['status']=='return']);ref=q['carried_path']
    index={tuple(r['hit']):r['index'] for r in q['rays'] if r['status']=='return'}
    def observed(cloud,side):
        path,info=local_path(cloud,side,cfg.wall_offset_m,horizon=1.8,
            rays=[index[tuple(v)] for v in cloud] if cfg.ray_wall_continuity else None,
            piecewise_corner=True,minimum_radius=cfg.wheelbase_m/np.tan(cfg.max_steering_rad))
        if info.get('wall_model')!='piecewise_corner':
            alt,proof=propose(cloud,side,cfg.wall_offset_m,cfg.wheelbase_m/np.tan(cfg.max_steering_rad))
            if alt is not None and (path is None or path_agrees(alt,path,cfg.boundary_agreement_m)):
                path=alt;info=dict(reason='local_path',wall_model='observed_fragment_corner',
                    boundary_observed_ahead_m=_path_ahead(alt),path_anchor_error_m=_origin_error(alt),
                    wall_distance_m=abs(float(cloud[np.argmin(np.linalg.norm(cloud-[0.,(1 if side=='left' else -1)*cfg.wall_offset_m],axis=1)),1])))
        return path,info
    row=dict(t=t,reference_present=ref is not None,prior=q['prior'],baseline=q['baseline'],candidates=[])
    for side in ('left','right'):
        clouds={'all':p}
        if ref is not None:
            clouds['support']=boundary_support(p,np.asarray(ref),side,cfg)
            clouds['refined']=drop_protrusions(clouds['support'],np.asarray(ref),side,cfg)
        for kind,cloud in clouds.items():
            path,info=observed(cloud,side)
            item=dict(side=side,kind=kind,points=len(cloud),info=info)
            if path is not None:
                item.update(path=path.tolist(),anchor=anchor_error(info,cfg),ahead=info.get('boundary_observed_ahead_m'),
                    x_agrees=ref is not None and path_agrees(path,np.asarray(ref),cfg.boundary_agreement_m),
                    arc_agrees=ref is not None and path_agrees_arc(path,np.asarray(ref),cfg.boundary_agreement_m))
            row['candidates'].append(item)
    results.append(row)
(A/'path_comparison.json').write_text(json.dumps(freeze(results),indent=2))
print('Saved path_comparison.json:',len(results),'scans')
