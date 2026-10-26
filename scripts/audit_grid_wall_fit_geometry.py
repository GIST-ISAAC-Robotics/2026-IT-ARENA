"""저장 반환의 알려진 편향을 뺀 점을 SDF 벽 조각에 대조하는 평가 전용 감사."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from probe_grid_follow import ROOT,RUNTIME,VEHICLE,CAMERA,registry,wall_sections


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    paths=[args.input,RUNTIME/'world.sdf',RUNTIME/'scene.json',VEHICLE,CAMERA,Path(__file__),
           ROOT/'scripts/probe_grid_follow.py']
    frozen={p.resolve().relative_to(ROOT).as_posix():p.read_bytes() for p in paths}
    source=json.loads(args.input.read_text(encoding='utf-8'))
    vehicle=yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    sensor=yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    slots=registry(json.loads((RUNTIME/'scene.json').read_text(encoding='utf-8')),(RUNTIME/'world.sdf').read_bytes(),vehicle)
    slot=next(s for s in slots if s['index']==5)
    segments=wall_sections((RUNTIME/'world.sdf').read_bytes(),sensor['xyz_m'][2])
    c,s=math.cos(slot['yaw_rad']),math.sin(slot['yaw_rad'])
    rotation=np.array([[c,-s],[s,c]])
    local=(segments-np.array([slot['x'],slot['y']]))@rotation
    a,b=local[:,0],local[:,1]; edges=b-a; sq=np.sum(edges**2,axis=1)
    rows=[]
    for case in source['cases']:
        rays=case.get('evaluation_selected_rays')
        if not rays:continue
        measured=np.array([r['hit_xy_m'] for r in rays]);directions=np.array([r['direction_xy'] for r in rays])
        true=measured-case['bias_m']*directions
        delta=true[:,None,:]-a[None,:,:]
        t=np.clip(np.sum(delta*edges[None,:,:],axis=2)/sq[None,:],0.,1.)
        closest=a[None,:,:]+t[:,:,None]*edges[None,:,:]
        distance=np.linalg.norm(true[:,None,:]-closest,axis=2)
        selected=np.argmin(distance,axis=1); best=np.min(distance,axis=1)
        segments_used=[]
        for index in sorted(set(map(int,selected))):
            indices=np.flatnonzero(selected==index)
            tangent=edges[index]/math.sqrt(sq[index]);theta=math.atan2(tangent[1],tangent[0])
            theta=(theta+math.pi/2)%math.pi-math.pi/2
            normal=np.array([-math.sin(theta),math.cos(theta)])
            signed=local[index]@normal
            segments_used.append(dict(segment_index=index,points=len(indices),
                source_indices=[rays[i]['source_index'] for i in indices],
                endpoints_base_link=local[index].tolist(),tangent_rad=theta,
                signed_offset_m=float(np.mean(signed)),max_point_distance_m=float(max(best[indices]))))
        common_normal=np.array([-math.sin(case['evaluation_known_bias_removed_fit']['theta_rad']),
                                 math.cos(case['evaluation_known_bias_removed_fit']['theta_rad'])])
        inferred=true@common_normal-np.array([r['side'] for r in rays])*.425
        rows.append(dict(bias_m=case['bias_m'],frame=case['frame'],selected_points=len(rays),
            max_point_to_wall_distance_m=float(max(best)),segments=segments_used,
            common_center_span_m=float(max(inferred)-min(inferred)),
            common_center_low_m=float(min(inferred)),common_center_high_m=float(max(inferred))))
    unchanged=all((ROOT/n).read_bytes()==raw for n,raw in frozen.items())
    report=dict(scope='evaluation-only SDF match after known injected bias removal; not controller input or real wall calibration',
        cases=rows,inputs_unchanged=unchanged,source_sha256={n:hashlib.sha256(v).hexdigest() for n,v in frozen.items()},
        all_selected_points_on_wall=all(r['max_point_to_wall_distance_m']<1e-8 for r in rows))
    with args.output.open('x',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2)
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},ensure_ascii=False))
    return 0 if unchanged and report['all_selected_points_on_wall'] and len(rows)==4 else 1


if __name__=='__main__':raise SystemExit(main())
