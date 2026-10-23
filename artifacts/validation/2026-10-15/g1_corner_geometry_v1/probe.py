"""첫 반환 광선으로 만든 L자 회랑/가림 반례. 동체 회전 취득이나 물리 시험이 아니다."""
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from test_local_follow import cast_scene,history,box
from profile_follow_trace import SNAPSHOT_MODULES

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]


def geometry(entry,kind):
    theta=np.linspace(-math.pi/2,0.,121)
    arc=np.column_stack((entry+.85*np.cos(theta),.425+.85*np.sin(theta)))
    segments=[((-3.,-.425),(entry,-.425)),*list(zip(arc[:-1],arc[1:])),
              ((entry+.85,.425),(entry+.85,3.)),((entry+.85,3.),(entry,3.))]
    if kind!='missing_inner':
        segments += [((-3.,.425),(entry,.425)),((entry,.425),(entry,3.))]
    if kind=='center_block': segments += box(.26,0.,.20,.20)
    if kind=='side_block': segments += box(.35,.18,.20,.20)
    if kind=='short_fake_inner':
        segments=[s for s in segments if s!=((-3.,.425),(entry,.425))]
        segments+=box(0.,.50,.20,.15)
    return segments


def main():
    report=BASE/'report.json'
    if report.exists(): raise FileExistsError(report)
    sources=[f'src/{p}/{p}/{m}.py' for p,mods in SNAPSHOT_MODULES.items() for m in mods]
    sources+=['tests/test_local_follow.py',str(Path(__file__).relative_to(ROOT))]
    hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}
    with zipfile.ZipFile(BASE/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in sources:archive.write(ROOT/p,p)
    rows=[]
    for kind in ('open','center_block','side_block','missing_inner','short_fake_inner'):
        for entry in (.15,.30,.50):
            for sign in (1,-1):
                world=(np.asarray(geometry(entry,kind))*[1,sign]).tolist()
                old=LocalFollow(FollowConfig(boundary_evidence_mode='fragmented_candidate',occupancy_timing_mode='per_ray_candidate'))
                new=LocalFollow(FollowConfig(boundary_evidence_mode='corner_anchor_candidate',occupancy_timing_mode='per_ray_candidate'))
                values=[]
                for t in (.4,.5,.6,.7):
                    scene=cast_scene(t,world)
                    # 후방 60도는 실제 기본 구성처럼 미관측. 정지 시각의 이상적 광선이다.
                    rays=tuple(replace(r,status='masked',hit_xy_m=None) if
                               abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))>math.radians(150) else r
                               for r in scene.rays)
                    scene=replace(scene,rays=rays)
                    side='left' if sign==1 else 'right'
                    a=old.command(scene,history(),0.,side)
                    b=new.command(scene,history(),0.,side)
                    values.append(dict(time=t,old=a,new=b))
                rows.append(dict(kind=kind,entry=entry,side=side,values=values))
    result=dict(scope='static_first_return_raycast_not_dynamic_physics',rows=rows,
                source_sha256=hashes,source_unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items()),
                driving_authorized=False)
    report.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps([dict(kind=r['kind'],entry=r['entry'],side=r['side'],
        old=r['values'][-1]['old'][0],new=r['values'][-1]['new'][0],
        reason=r['values'][-1]['new'][2]['reason'],selection=r['values'][-1]['new'][2].get('boundary_selection'),
        corner=r['values'][-1]['new'][2].get('corner_boundary_evidence')) for r in rows]))


if __name__=='__main__':main()
