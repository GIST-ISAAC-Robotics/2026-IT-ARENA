"""저장 광선에서 가까운 안쪽 연결 성분 단절을 진단. 입력·운영 변경 없음."""
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.corner_distance import corner_distance_evidence
from inspect_corner_geometry import components, digest
from profile_follow_trace import SNAPSHOT_MODULES

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
SELECTED=(2166,2171,2176,2181,2201,2231)


def inspect(scene, scan, side, cfg):
    p=np.asarray(scene.occupied_endpoints_xy_m)
    rays=[r for r in scene.rays if r.status=='return']
    sign=1 if side=='left' else -1
    candidates=components(p,side)
    if not candidates:return dict(component=None)
    close=min(candidates,key=lambda c:float(np.min(np.linalg.norm(p[c['indices']]-[0,sign*cfg.wall_offset_m],axis=1))))
    order=np.argsort(-sign*np.arctan2(p[:,1],p[:,0]),kind='stable')
    rank={int(index):i for i,index in enumerate(order)}
    neighbors=[]
    for own,step in ((close['indices'][0],-1),(close['indices'][-1],1)):
        k=rank[own]+step
        if k<0 or k>=len(order):
            neighbors.append(dict(reason='end_of_return_sequence'));continue
        other=int(order[k]);a,b=p[own],p[other]
        ar,br=rays[own],rays[other]
        gap=float(np.linalg.norm(a-b))
        limit=.10+.035*min(float(np.linalg.norm(a)),float(np.linalg.norm(b)))
        neighbors.append(dict(own_xy=a.tolist(),next_xy=b.tolist(),gap_m=gap,limit_m=limit,
            connection_split=gap>limit,
            next_outside_roi=bool(sign*b[1]<=.10 or np.linalg.norm(b)>2.),
            source_indices=[ar.source_index,br.source_index],
            acquisition_s=[ar.acquired_at_s,br.acquired_at_s],
            source_angles_deg=[math.degrees(scan.angle_min+r.source_index*scan.angle_increment) for r in (ar,br)],
            source_index_gap=abs(ar.source_index-br.source_index)))
    path,info=local_path(p,'right' if side=='left' else 'left',cfg.wall_offset_m)
    evidence=corner_distance_evidence(p,path,side,cfg) if path is not None else None
    return dict(component={k:v for k,v in close.items() if k!='indices'},neighbors=neighbors,
                returns=len(p),evidence=None if evidence is None else {k:v for k,v in evidence.items() if k!='source_indices'})


def main():
    target=BASE/'report.json'
    if target.exists():raise FileExistsError(target)
    raw=BASE.parent/'g1_solo_lap_v4/run/follow_decisions.jsonl'
    before=digest(raw);events=[]
    with raw.open() as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']=='header':
                motion=MotionConfig(**event['data']['motion_config'])
                cfg=FollowConfig(**event['data']['follow_config'])
            elif event['kind']=='call' and event['seq'] in SELECTED:events.append(event)
    if len(events)!=len(SELECTED):raise ValueError('selected records missing')
    rows=[]
    for event in events:
        data,scan,h=restore_call(event['data']['input'],motion)
        views={}
        for name,mask in (('recorded',data['rear_half_angle_deg']),('no_rear_mask_diagnostic',0.)):
            scene=motion_scene(scan,h,data['reference_s'],timing_verified=data['timing_verified'],
                               epoch=data['epoch'],rear_half_angle_deg=mask)
            views[name]=inspect(scene,scan,data['side'],cfg)
        rows.append(dict(seq=event['seq'],t=data['reference_s'],views=views))
    paths=[f'src/{pkg}/{pkg}/{mod}.py' for pkg,mods in SNAPSHOT_MODULES.items() for mod in mods]
    paths+=['scripts/inspect_corner_geometry.py','scripts/profile_follow_trace.py',str(Path(__file__).relative_to(ROOT))]
    hashes={p:digest(ROOT/p) for p in paths}
    with zipfile.ZipFile(BASE/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in paths:z.write(ROOT/p,p)
    result=dict(scope='saved_scan_visibility_diagnostic_only',input_sha256=before,input_unchanged=before==digest(raw),
        source_sha256=hashes,source_unchanged=all(digest(ROOT/p)==value for p,value in hashes.items()),
        driving_authorized=False,physical_mount_validated=False,rows=rows)
    target.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps([dict(seq=r['seq'],t=r['t'],views={k:dict(extent=v['component']['extent_m'],
        supported=v['evidence']['supported'],neighbors=v['neighbors']) for k,v in r['views'].items()}) for r in rows]))


if __name__=='__main__':main()
