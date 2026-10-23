#!/usr/bin/env python3
"""정적 코스에서 짧은 직진 후 무응답 감소 여부만 평가. 제어 입력/주행 시험 아님."""
import argparse
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np

from diagnose_front_unknown import ROOT, digest, wall_geometry
from g0_geometry import ray_box_distance


def front_cast(boxes, origin, yaw, indices, angle_min, increment, limit):
    missing=[]
    for i in indices:
        angle=yaw+angle_min+i*increment
        direction=(math.cos(angle),math.sin(angle),0.)
        hit=min((d for b in boxes if (d:=ray_box_distance(origin,direction,b,0.,100.)) is not None),default=math.inf)
        if hit>limit or hit<.05: missing.append(dict(index=i,distance_m=hit if math.isfinite(hit) else None))
    return missing


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('bounded',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    selected=next(r for r in json.loads(args.bounded.read_text())['rows'] if r['bounded_recovery']['eligible'])
    with (args.run/'ego_poses.jsonl').open() as stream:
        poses=np.array([json.loads(line) for line in stream])
    index=np.argmin(abs(poses[:,0]-selected['time_s']))
    t,x,y,yaw=poses[index]
    if abs(t-selected['time_s'])>.006: raise ValueError('nearby recorded pose unavailable')
    # 런의 실제 센서 pose를 읽는다. 코드로 임의의 센서 위치를 다시 정하지 않는다.
    import xml.etree.ElementTree as ET
    from diagnose_front_unknown import pose
    vehicle=ET.parse(args.run/'runtime_vehicle.sdf').find('model')
    sensor=pose(vehicle.find("link[@name='lidar_link']"))
    chassis=pose(vehicle.find("link[@name='chassis']"))
    report=json.loads((args.run/'report.json').read_text())
    z=report['last_observation']['dynamics']['truth_z_m']+sensor[2]-chassis[2]
    c,s=math.cos(yaw),math.sin(yaw)
    base=(x+c*sensor[0]-s*sensor[1],y+s*sensor[0]+c*sensor[1],z)
    boxes,_,meshes=wall_geometry(args.run/'runtime_world.sdf')
    inputs=[args.bounded,args.run/'ego_poses.jsonl',args.run/'runtime_world.sdf',args.run/'runtime_vehicle.sdf',args.run/'report.json',*meshes]
    hashes={str(p):digest(p) for p in inputs}
    # 같은 500점/12 m 평가. 취득 시간·잡음·반사는 이 정적 비교에 포함되지 않는다.
    indices=[i for i in range(500) if abs(-math.pi+i*math.tau/500)<=math.radians(100)]
    rows=[]
    for distance in (0.,.025,.05,.1):
        origin=(base[0]+c*distance,base[1]+s*distance,base[2])
        missing=front_cast(boxes,origin,yaw+sensor[5],indices,-math.pi,math.tau/500,12.)
        rows.append(dict(straight_translation_m=distance,front_unknown_count=len(missing),unknown_rays=missing))
    sources={}
    for module in tuple(sys.modules.values()):
        name=getattr(module,'__file__',None)
        if name:
            path=Path(name).resolve()
            if path.is_relative_to(ROOT) and path.suffix=='.py': sources[path.relative_to(ROOT).as_posix()]=digest(path)
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name in sources: archive.write(ROOT/name,name)
    result=dict(scope='evaluation_only_static_viewpoint_counterfactual',source_seq=selected['seq'],
        recorded_pose=[float(v) for v in poses[index]],rows=rows,input_sha256=hashes,source_sha256=sources,
        input_unchanged=all(digest(p)==h for p,h in hashes.items()),
        source_unchanged=all(digest(ROOT/p)==h for p,h in sources.items()),
        runtime_changed=False,controller_received_truth=False,driving_verified=False)
    (args.output/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('input_sha256','source_sha256')}))


if __name__=='__main__': main()
