"""저장 센서 무반환과 공식 기하의 평가 전용 대조. 제어 입력은 바꾸지 않는다."""
from pathlib import Path
import collections
import hashlib
import json
import math
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
RUN = ROOT/'artifacts/validation/2026-11-02/official_facility_v1/run1'
WORLD = ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf'


def rotation(r, p, y):
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def transform(text):
    values=list(map(float,(text or '0 0 0 0 0 0').split()))
    result=np.eye(4);result[:3,:3]=rotation(*values[3:]);result[:3,3]=values[:3]
    return result


def boxes():
    found=[]
    def visit(node,parent):
        here=parent
        if node.tag in ('model','link','visual'):
            pose=node.find('pose')
            assert pose is None or not pose.get('relative_to'), 'relative frame unsupported'
            here=parent@transform(node.findtext('pose'))
        if node.tag=='visual':
            box=node.find('geometry/box/size')
            if box is not None:
                found.append((node.get('name'),here[:3,3],here[:3,:3],np.array(list(map(float,box.text.split())))/2))
            return
        for child in node:
            if child.tag in ('sdf','world','model','link','visual'):
                visit(child,here)
    visit(ET.parse(WORLD).getroot(),np.eye(4))
    return [v[0] for v in found],np.array([v[1] for v in found]),np.array([v[2] for v in found]),np.array([v[3] for v in found])


def main():
    labels,centers,rotations,half=boxes()
    rows=[json.loads(line) for line in (RUN/'trajectory.jsonl').open()]
    times=np.array([row['time'] for row in rows])
    progress=np.array([row['progress_m'] for row in rows])
    ds=[row['dynamics'] for row in rows if row.get('dynamics') and 'truth_z_m' in row['dynamics']]
    td=np.array([d['sim_time_s'] for d in ds])
    pose=np.array([[d['truth_'+k] for k in ['x_m','y_m','z_m','roll_rad','pitch_rad','yaw_rad']] for d in ds])
    pose[:,5]=np.unwrap(pose[:,5])
    result=[];seen=set();summary=collections.Counter();errors=[];diagnostics=[]
    for line in (RUN/'follow_decisions.jsonl').open():
        event=json.loads(line)
        if event['kind']!='call':continue
        data=event['data']; inp=data['input']; detail=data['output'][2];ref=inp['reference_s']
        s=float(np.interp(ref,times,progress))
        if not (42<=s<=44 or .5<=s<=4):continue
        scan=inp['scan']; first=scan['sec']+scan['nanosec']*1e-9
        part=detail.get('partial_observation') or {}
        diagnostics.append(dict(progress_m=s,reference_s=ref,speed=data['output'][0],reason=detail.get('reason'),
            path_wall=detail.get('path_wall'),partial_reason=part.get('reason'),cap=part.get('speed_cap_mps'),
            boundary=detail.get('boundary_selection'),corridor=detail.get('corridor_selected')))
        if first in seen:continue
        seen.add(first)
        raw=scan['ranges'];angles=scan['angle_min']+np.arange(len(raw))*scan['angle_increment']
        values=np.array([float(v['nonfinite']) if isinstance(v,dict) else v for v in raw])
        unknown=(~np.isfinite(values))|(values<scan['range_min'])|(values>scan['range_max'])
        front=np.abs((angles+np.pi)%(2*np.pi)-np.pi)<=math.radians(100)
        ids=np.flatnonzero(unknown&front)
        neighbor_ids=set()
        for index in ids:
            for n in (index-1,index+1):
                if 0<=n<len(raw) and not unknown[n]:neighbor_ids.add(n)
        # 기둥 근처는 현재 실제 반환도 기하 대조한다.
        if s<4:neighbor_ids.update(np.flatnonzero(front&~unknown)[::10])
        for index in sorted(set(ids)|neighbor_ids):
            acquire=first+index*scan['time_increment']
            p=np.array([np.interp(acquire,td,pose[:,j]) for j in range(6)])
            r=rotation(*p[3:]);origin=p[:3]+r@np.array([.06,0,.095-.0625])
            direction=r@np.array([math.cos(angles[index]),math.sin(angles[index]),0])
            local_origin=np.einsum('ni,nij->nj',origin-centers,rotations)
            local_direction=np.einsum('i,nij->nj',direction,rotations)
            with np.errstate(divide='ignore',invalid='ignore'):
                a=(-half-local_origin)/local_direction;b=(half-local_origin)/local_direction
                enter=np.max(np.minimum(a,b),axis=1);leave=np.min(np.maximum(a,b),axis=1)
            distance=np.where((leave>=np.maximum(enter,0))&(leave>0),np.maximum(enter,0),np.inf)
            best=int(np.argmin(distance));hit=float(distance[best]);unknown_ray=bool(unknown[index])
            category=('beyond_range' if hit>scan['range_max'] else 'inside_range_hit') if unknown_ray else 'valid_neighbor'
            summary[category]+=1
            residual=None if unknown_ray or not math.isfinite(hit) else float(values[index]-hit)
            if residual is not None:errors.append(abs(residual))
            result.append(dict(source_first_s=first,acquired_s=acquire,reference_s=ref,progress_m=s,
                index=int(index),angle_deg=math.degrees(angles[index]),raw=raw[index],unknown=unknown_ray,
                class_name=category,expected_hit_m=hit if math.isfinite(hit) else None,hit_label=labels[best] if math.isfinite(hit) else None,
                residual_m=residual,origin=origin.tolist(),direction=direction.tolist()))
    summary=dict(summary)
    summary.update(unique_scans=len(seen),calls=len(diagnostics),valid_residual_median_m=float(np.median(errors)),
        valid_residual_max_m=max(errors),world_sha256=hashlib.sha256(WORLD.read_bytes()).hexdigest(),
        note='評価専用: captured chassis truth interpolation + nominal sensor transform; box visuals only, no ego geometry/plane/spheres. No controller input.')
    (OUT/'rays.json').write_text(json.dumps(dict(summary=summary,rays=result,calls=diagnostics),ensure_ascii=False,indent=1),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False))


if __name__=='__main__':main()
