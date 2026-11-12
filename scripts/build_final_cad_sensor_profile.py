"""11/13 확정 STEP의 두 지주 mesh/단면만 선택형 센서 프로필로 옮긴다. 실물 조작 없음."""
import copy
import hashlib
import json
from pathlib import Path
import struct

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'outputs/cad_final_2026_11_13'
DEST=ROOT/'src/arena_description'
STEP_SHA='c9c203b5a66585ccde71313ea888c16804a1ddb71e6e91514a46bebededbfce6'


def loops(lines):
    """B-rep 수평 직선 edge를 연결한다. U 홈/오목한 단면을 볼록껍질로 채우지 않는다."""
    pending=[np.asarray(line,float) for line in lines]
    if any(p.shape!=(2,2) for p in pending):raise ValueError('straight two-point section expected')
    result=[]
    while pending:
        path=list(pending.pop(0))
        while np.linalg.norm(path[-1]-path[0])>1e-6:
            matches=[]
            for i,edge in enumerate(pending):
                for j in (0,1):
                    if np.linalg.norm(path[-1]-edge[j])<=1e-6:matches.append((i,j))
            if len(matches)!=1:raise ValueError('section is open or branched')
            i,j=matches[0];edge=pending.pop(i);path.append(edge[1-j])
        p=np.asarray(path[:-1])/1000.
        area=np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1))
        result.append(p if area>0 else p[::-1])
    return sorted(result,key=lambda p:float(p[:,1].mean()),reverse=True)


def wire_strips(p,thickness,direction):
    """차량 전방 또는 차체 안쪽으로만 늘어나는 표면 예약. 전방향 팽창 아님."""
    if thickness==0:return []
    d=np.array([1.,0.]) if direction=='front' else np.array([0.,-np.sign(p[:,1].mean())])
    result=[]
    for a,b in zip(p,np.roll(p,-1,axis=0)):
        edge=b-a;outward=np.array([edge[1],-edge[0]])
        if outward@d>1e-12:result.append(np.array([a,b,b+thickness*d,a+thickness*d]))
    return result


def write_stl(path,triangles):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('wb') as out:
        out.write(b'IT ARENA CAD 2026-11-12, axle midpoint ground frame metres'.ljust(80,b' '))
        out.write(struct.pack('<I',len(triangles)))
        for t in triangles:
            # STL에 기록되는 float32 꼭짓점에서 법선을 구해 얇은 면의 반올림 불일치를 피한다.
            t=np.asarray(t,dtype=np.float32).astype(float)
            n=np.cross(t[1]-t[0],t[2]-t[0]);length=np.linalg.norm(n)
            if length:n/=length
            out.write(struct.pack('<12fH',*n,*t.ravel(),0))


def prism_triangles(p,low,high):
    """여기 입력은 부착면의 볼록 사각형이다. 원 U/트러스는 실제 mesh를 쓴다."""
    area=np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1))
    if area<0:p=p[::-1]
    vertices=np.vstack((np.c_[p,np.full(4,low)],np.c_[p,np.full(4,high)]))
    faces=[[0,2,1],[0,3,2],[4,5,6],[4,6,7]]
    for i in range(4):j=(i+1)%4;faces.extend([[i,j,j+4],[i,j+4,i+4]])
    return vertices[np.array(faces)]


def main():
    sections=json.loads((SOURCE/'sections.json').read_text(encoding='utf-8'))
    base=yaml.safe_load((DEST/'config/b0390_c1_jr60p20_sim.yaml').read_text(encoding='utf-8'))
    meshdir=DEST/'models/arena_car/meshes/cad_20261113_final'
    parts=[]
    for entry in sections['134.6']:
        part=entry['id'];assert part in (2372,2385)
        raw=np.load(SOURCE/f'mesh_{part}.npz')
        v=raw['vertices'];v=np.c_[-v[:,2]-64.,21.2-v[:,0],v[:,1]+32.2]/1000.
        # 변환의 determinant +1. 원 mesh 삼각형 순서를 유지한다.
        write_stl(meshdir/f'part_{part}.stl',v[raw['faces']])
        for index,p in enumerate(loops(entry['lines'])):parts.append(dict(part_id=part,index=index,polygon=p))
    summaries=[]
    for direction,mm in [('front',0),('front',6),('inboard',6)]:
        cfg=copy.deepcopy(base);lidar=cfg['lidar']
        for key in ('scan_plane_posts','scan_plane_wires','chassis_visual_top_m'):lidar.pop(key,None)
        lidar['xyz_m']=[.0754,0.,.1346]
        lidar['status']='cad_20261113_final_axis_estimate_not_measured'
        lidar['scan_plane_sections']=[]
        lidar['scan_plane_meshes']=[f'../models/arena_car/meshes/cad_20261113_final/part_{p}.stl' for p in (2372,2385)]
        lidar['mount_motion_margin_m']=.003 # 사용자 요청: 실측 전 임시 흔들림/장착 여유3mm
        lidar['self_return_distance_margin_m']=.002 # mesh/이상적 시뮬 근사 여유. 실물 잡음 교정 아님.
        wire=[]
        for part in parts:
            p=part['polygon'];strips=wire_strips(p,mm/1000.,direction)
            lidar['scan_plane_sections'].append(dict(part_id=part['part_id'],side='left' if p[:,1].mean()>0 else 'right',
                polygon_xy_m=p.tolist(),wire_polygons_xy_m=[q.tolist() for q in strips]))
            for q in strips:wire.extend(prism_triangles(q,.1246,.1446))
        if wire:
            name=f'wire_{direction}_{mm}mm.stl';write_stl(meshdir/name,np.asarray(wire))
            lidar['scan_plane_meshes'].append(f'../models/arena_car/meshes/cad_20261113_final/{name}')
        cfg['camera']['xyz_m']=[.055,0.,.198363]
        cfg['camera']['status']='B0390_with_old_camera_CAD_axis_height_only_X_provisional'
        cfg['mount_provenance']={
            'source_step_sha256':STEP_SHA,'status':'user_final_design_sensor_geometry_not_installed_calibration',
            'coordinate_frame':'base_link_at_axle_midpoint_ground; chassis_visual_z subtract wheel_radius+body_height/2',
            'step_to_vehicle_mm':'X=-c-64, Y=21.2-a, Z=b+32.2',
            'pillar_geometry':'original two part meshes, exact B-rep section at estimated optical Z134.6mm',
            'wire_direction':direction,'wire_extra_thickness_mm':mm,'wire_status':'front_or_inboard_6mm_reserved_not_measured',
            'motion_margin_mm':3,'motion_margin_status':'provisional_remaining_slack_after_calibration_not_verified_bound',
            'wire_vertical_extent_m':[.1246,.1446],'wire_vertical_status':'scan_band_proxy_not_whole_harness',
            'camera_height_source':'B0183 substitute lens cylinder axis b=166.163mm; B0390 unmeasured',
            'camera_x_source':'old simulation 55mm retained; neither lens tip nor STEP optical projection centre',
            'camera_y_source':'old zero matches substitute lens axis a=21.2mm',
            'camera_intrinsics':'B0390 unchanged; mount pitch 0 temporary',
            'retained':'mass,inertia,wheelbase,track,wheel_diameter,drivetrain,IMU pose',
            'rear_id49':'not in CAD; existing 140mm centre intersects lidar plane; placement remains separate test argument',
            'self_filter':'distance-limited within CAD angular sectors; distant valid hits preserved; no-return remains unknown',
        }
        label='nowire' if mm==0 else f'{direction}wire_{mm}mm'
        output=DEST/f'config/b0390_c1_cad_20261113_final_{label}.yaml'
        output.write_text('# 사용자 확정 STEP의 선택형 센서 모델. 배선/흔들림 여유는 가정, 실장 교정은 미완료.\n'+yaml.safe_dump(cfg,allow_unicode=True,sort_keys=False),encoding='utf-8')
        summaries.append(dict(profile=str(output.relative_to(ROOT)),direction=direction,thickness_mm=mm))
    evidence=ROOT/'artifacts/validation/2026-11-13/cad_mount_r347_v1'
    evidence.mkdir(parents=True,exist_ok=True)
    (evidence/'generated.json').write_text(json.dumps(dict(profiles=summaries,
        sections_sha256=hashlib.sha256((SOURCE/'sections.json').read_bytes()).hexdigest(),
        source_meshes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (SOURCE/'mesh_2372.npz',SOURCE/'mesh_2385.npz')}),indent=2)+'\n')
    print(json.dumps(summaries))


if __name__=='__main__':main()
