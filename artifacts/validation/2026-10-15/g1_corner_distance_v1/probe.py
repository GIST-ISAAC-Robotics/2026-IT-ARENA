"""저장 실패 및 정적 회전 코너를 대조한다. 운영 연결/폐루프 시험 아님."""
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.local_path import local_path
from corner_distance_probe import corner_distance_evidence
from test_corner_raycast import elbow
from test_local_follow import cast_scene, history

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]


def main():
    target=BASE/'report.json'
    if target.exists():raise FileExistsError(target)
    sources=['scripts/corner_distance_probe.py','tests/test_corner_raycast.py',
             'tests/test_local_follow.py','src/arena_autonomy/arena_autonomy/local_path.py',
             'src/arena_autonomy/arena_autonomy/local_follow.py',str(Path(__file__).relative_to(ROOT))]
    hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}
    with zipfile.ZipFile(BASE/'source.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in sources:z.write(ROOT/p,p)
    cfg=FollowConfig(boundary_evidence_mode='corner_anchor_candidate')
    saved=json.loads((BASE.parent/'g1_solo_lap_v3/geometry.json').read_text())
    actual=[]
    for row in saved['rows']:
        actual.append(dict(seq=row['seq'],time=row['t'],evidence=corner_distance_evidence(
            row['points'],row['paths']['right']['path'],'left',cfg)))
    cases=[]
    # 같은 중심 궤도의 자세에서 정적 첫 반환을 만든다. 정답 자세는 시험 장면 생성에만 사용한다.
    phases=[('entry',float(x)) for x in (-.30,-.15,-.05,0.)]
    phases += [('turn',float(x)) for x in (5.,15.,30.,45.,60.,75.,90.)]
    phases += [('exit',float(x)) for x in (.15,.30)]
    for name,value in phases:
        if name=='entry':position=np.array([value,0.]);yaw=0.
        elif name=='turn':
            yaw=math.radians(value);position=np.array([.425*math.sin(yaw),.425*(1-math.cos(yaw))])
        else:yaw=math.pi/2;position=np.array([.425,.425+value])
        rotation=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        for kind in ('open','missing_inner','short_fake_inner'):
            for sign in (1,-1):
                world=((np.asarray(elbow(0.,kind))-position)@rotation)*[1,sign]
                scene=cast_scene(.5,world.tolist())
                scene=replace(scene,rays=tuple(replace(r,status='masked',hit_xy_m=None)
                    if abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))>math.radians(150) else r for r in scene.rays))
                points=np.asarray(scene.occupied_endpoints_xy_m)
                side='left' if sign==1 else 'right'
                other='right' if sign==1 else 'left'
                reference,info=local_path(points,other,cfg.wall_offset_m)
                evidence=corner_distance_evidence(points,reference,side,cfg) if reference is not None else None
                ctl=LocalFollow(cfg)
                for now in (.4,.5,.6,.7):
                    current=replace(scene,t_ref_s=now,source_first_s=now-.1,source_last_s=now-.1,
                                    rays=tuple(replace(r,acquired_at_s=now-.1) for r in scene.rays))
                    result=ctl.command(current,history(),0.,side)
                cases.append(dict(phase=name,value=value,kind=kind,side=side,evidence=evidence,
                                  original_speed=result[0],original_reason=result[2]['reason'],
                                  reference_info=info))
    output=dict(scope='offline_geometry_diagnosis_not_drive_permission',actual=actual,cases=cases,
                source_sha256=hashes,source_unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items()),
                driving_authorized=False,closed_loop_pass=False)
    target.write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(actual=actual,cases=[{k:v for k,v in r.items() if k!='reference_info'} for r in cases])))


if __name__=='__main__':main()
