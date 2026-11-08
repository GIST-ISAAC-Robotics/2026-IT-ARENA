"""고정된 경로의 평가 전용 SDF/자세 대조. 결과로 경로를 조정하거나 주행에 연결하지 않는다."""
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
from surface_contact_eval import SurfaceMap, VehicleGeometry, evaluate_pose, steering_footprint

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
OBS=HERE.parent/'p22_sensor_preview_v1'
sys.path.insert(0,str(OBS))
from evaluate import Poses,to_world,ENTRY,EXIT

digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def score(one, pose, surface):
    res={k:v for k,v in one.items() if k!='poses'}
    if one['reason']!='candidate':return res
    g=VehicleGeometry()
    maxoff=dict(any=0,center=0,full=0); nonroad_other=False; gap=100.
    first_three=None
    for rx,ry,h,steer,s in one['poses']:
        xy=to_world(pose,(rx+g.wheelbase_m/2*math.cos(h),ry+g.wheelbase_m/2*math.sin(h)))
        yaw=pose[2]+h
        footprint=steering_footprint(*xy,yaw,steer,g)
        gap=min(gap,footprint.distance(surface.walls))
        e=evaluate_pose(surface,*xy,yaw,steer,g)
        for k,v in e['off_counts'].items():maxoff[k]=max(maxoff[k],v)
        if e['off_counts']['center']>2 and first_three is None:first_three=s
        nonroad_other |= any(w['fractions']['other']>1e-9 or w['fractions']['wall']>1e-9 for w in e['wheels'].values())
    return dict(**res,samples=len(one['poses']),end_body_xy_m=xy,first_three_center_s_m=first_three,
                min_sampled_wall_clearance_m=gap,max_off_road_wheels=maxoff,
                any_patch_on_wall_or_non_grass_offroad=nonroad_other,
                strict_sampled_geometry_ok=maxoff['any']<=2 and not nonroad_other and gap>0,
                center_sampled_geometry_ok=maxoff['center']<=2 and not nonroad_other and gap>0)

def main():
    p=HERE/'paths.json'; paths=json.loads(p.read_bytes())
    hashes={p.name:digest(p)}; result={}
    for n,record in paths.items():
        d=(ROOT/record['source']).parent
        obs=json.loads((OBS/f'observations_{n}.json').read_bytes())
        by_t={r['t']:r for r in obs['rows']}
        poses_file=d/'ego_poses.jsonl';world=d/'runtime_world.sdf'
        hashes.update({str(f.relative_to(ROOT)):digest(f) for f in (world,poses_file)})
        with poses_file.open() as f:poses=Poses([json.loads(l) for l in f if l.strip()])
        surface=SurfaceMap.from_world(world)
        scored=[]
        for path in record['paths']:
            t=path['t'];pose=poses.at(t)
            assert poses.gap(t)<.01
            feat=by_t[t]['right']['notches'][0]
            error={label:max(math.dist(to_world(pose,feat[k.lower()]),pair[k]) for k in ('E1','E2'))
                   for label,pair in [('entrance',ENTRY),('exit',EXIT)]}
            match=min(error,key=error.get)
            scored.append(dict(eval_gate=match if error[match]<=.15 else 'other',
                               eval_start_pose=pose,eval_edge_error_m=error[match],**score(path,pose,surface)))
        result[n]=scored
    assert digest(p)==hashes[p.name]
    out=dict(scope='sampled open-loop static geometry only, no actor/contact dynamics/observation permission/exit traversal',
             evidence_sha256=hashes,runs=result)
    (HERE/'summary.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf8')
    for n,rows in result.items():
        for r in rows:
            print(n,r['eval_gate'],r['t'],r['radius_m'],r['speed_mode'],r['reason'],r.get('max_off_road_wheels'),r.get('min_sampled_wall_clearance_m'))
if __name__=='__main__':main()
