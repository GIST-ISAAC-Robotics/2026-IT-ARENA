"""실패 전후 실제 관측과 선택형 앵커를 보존한다. 운영/입력 변경 없음."""
import hashlib
import json
from pathlib import Path
import zipfile
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.corner_evidence import corner_anchor_evidence

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def main():
    output=BASE/'geometry.json'
    if output.exists():raise FileExistsError(output)
    raw=BASE/'run/follow_decisions.jsonl'
    before=digest(raw)
    selected=(1606,1611,1616,1861)
    events=[]
    with raw.open() as stream:
        for line in stream:
            e=json.loads(line)
            if e['kind']=='header':
                motion=MotionConfig(**e['data']['motion_config'])
                cfg=FollowConfig(**e['data']['follow_config'])
            elif e['kind']=='call' and e['seq'] in selected:events.append(e)
    rows=[]
    fig,axes=plt.subplots(2,2,figsize=(12,9),constrained_layout=True)
    for event,ax in zip(events,axes.flat):
        data,scan,h=restore_call(event['data']['input'],motion)
        scene=motion_scene(scan,h,data['reference_s'],timing_verified=data['timing_verified'],
                           epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
        points=np.asarray(scene.occupied_endpoints_xy_m)
        paths={}
        ax.scatter(points[:,0],points[:,1],s=5,c='#334155',label='All returns')
        for side,color in [('left','#de4242'),('right','#2284d2')]:
            path,info=local_path(points,side,cfg.wall_offset_m)
            paths[side]=dict(path=None if path is None else path.tolist(),info=info)
            if path is not None:ax.plot(path[:,0],path[:,1],color=color,lw=2,label=side+' path')
        evidence=corner_anchor_evidence(points,paths['right']['path'],'left',cfg) if paths['right']['path'] else None
        inner=points[(points[:,1]>.1)&(points[:,1]<.65)&(points[:,0]<.5)]
        rows.append(dict(seq=event['seq'],t=data['reference_s'],points=points.tolist(),
            paths=paths,evidence=evidence,recorded=event['data']['output'],
            inner_bounds=None if not len(inner) else dict(min_xy=inner.min(axis=0).tolist(),max_xy=inner.max(axis=0).tolist())))
        ax.add_patch(plt.Rectangle((-.1,-.075),.2,.15,color='#555',alpha=.3))
        ax.set(xlim=(-1.,1.6),ylim=(-.8,1.2),aspect='equal',xlabel='Forward x [m]',ylabel='Left y [m]',
               title=f"t={data['reference_s']:.2f}s; {event['data']['output'][2]['reason']}")
        ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.savefig(BASE/'corner_loss.png',dpi=150)
    sources=['src/arena_autonomy/arena_autonomy/'+m+'.py' for m in ('corner_evidence','local_path','scene_motion','follow_trace','lidar_motion','local_scene','local_follow','boundary_evidence','occupancy_timing','lidar_observation')]
    sources+=[str(Path(__file__).relative_to(ROOT))]
    hashes={p:digest(ROOT/p) for p in sources}
    with zipfile.ZipFile(BASE/'geometry_source.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in sources:z.write(ROOT/p,p)
    output.write_text(json.dumps(dict(scope='saved_sensor_geometry_not_new_drive',rows=rows,
        input_sha256=before,input_unchanged=before==digest(raw),source_sha256=hashes),indent=2,allow_nan=False)+'\n')
    print(json.dumps([dict(seq=r['seq'],t=r['t'],inner=r['inner_bounds'],evidence=r['evidence']) for r in rows]))


if __name__=='__main__':main()
