"""10/29 시험본의 무결성·6그리드 정지 시야·시선 가림 표본. 주행·연속 가시면적·픽셀 검출 증거는 아니다."""
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from audit_start_signal_geometry import pose, rotation, project

CAMERAS={
    # 기존5랩 시험 카메라와 현재 차량 설정의 임시 카메라. 둘 다 B0183 1280x720/수평75°.
    'trial_rear_30':dict(xyz_m=[-.075,0.,.30],rpy_rad=[0.,0.,0.]),
    'baseline_front_145':dict(xyz_m=[.055,0.,.145],rpy_rad=[0.,0.,0.]),
}
PEER_HEIGHTS=(None,.25,.40)


def boxes(world):
    rows=[]
    for model in world.findall('model'):
        m,mr=pose(model)
        for link in model.findall('link'):
            l,lr=pose(link)
            for visual in link.findall('visual'):
                box=visual.find('geometry/box/size')
                if box is None:continue
                v,vr=pose(visual)
                rows.append((link.get('name'),m+mr@(l+lr@v),mr@lr@vr,np.array(list(map(float,box.text.split())))/2))
    return rows


def hits(origin,target,obstacles):
    centers=np.array([r[1] for r in obstacles]);rs=np.array([r[2] for r in obstacles]);half=np.array([r[3] for r in obstacles])
    a=np.einsum('nji,nj->ni',rs,origin-centers)
    d=np.einsum('nji,nj->ni',rs,target-origin+np.zeros_like(centers))
    parallel=np.abs(d)<1e-12
    outside=np.any(parallel&(np.abs(a)>half),axis=1)
    nonzero=np.where(parallel,1.,d)
    t1,t2=(-half-a)/nonzero,(half-a)/nonzero
    lower=np.where(parallel,-np.inf,np.minimum(t1,t2)).max(axis=1)
    upper=np.where(parallel,np.inf,np.maximum(t1,t2)).min(axis=1)
    mask=(~outside)&(np.maximum(lower,0.)<=np.minimum(upper,1.))&(upper>1e-6)&(lower<1.-1e-6)
    return [obstacles[i][0] for i in np.flatnonzero(mask)]


def signature(e):return (e.tag,sorted(e.attrib.items()),(e.text or '').strip(),[signature(c) for c in e])


def peer_boxes(slots,ego,height):
    if height is None:return []
    return [(f"peer_slot{s['index']}",np.array([s['x'],s['y'],height/2]),rotation(0.,0.,s['yaw_rad']),np.array([.10,.075,height/2]))
            for s in slots if s['index']!=ego]


def main():
    output=BASE/'preflight.json'
    if output.exists():raise FileExistsError(output)
    fixture=BASE/'fixture';scene=json.loads((fixture/'scene.json').read_text(encoding='utf-8'))
    manifest=json.loads((fixture/'manifest.json').read_text(encoding='utf-8'))
    world=ET.parse(fixture/'world.sdf').find('world');model=world.find("model[@name='it_arena_track_static']")
    old=ET.parse(ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf').find("world/model[@name='it_arena_track_static']")
    for link in old.findall('link'):
        if link.get('name') not in manifest['removed_links']:
            assert signature(link)==signature(model.find(f"link[@name='{link.get('name')}']"))
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in manifest['sources'].items())
    assert all(hashlib.sha256((fixture/p).read_bytes()).hexdigest()==h for p,h in manifest['assets_sha256'].items())
    n=np.array(manifest['face_normal']);center=np.array(manifest['array_center_xyz_m'])
    geometry=dict(module_bottom_z_m=manifest['module_bottom_z_m'],array_center_z_m=float(center[2]),
                  normal_below_horizontal_deg=math.degrees(math.asin(-n[2])),
                  lamp_center_mean_z_m=float(np.mean([l['xyz_m'][2] for l in manifest['lamps']])))
    assert abs(geometry['module_bottom_z_m']-.397)<1e-9 and abs(geometry['normal_below_horizontal_deg']-30.)<1e-9
    slots=scene['starting_grid']['slots'];track=boxes(world);rows=[]
    lamps=[dict(name=l['name'],xyz=l['xyz_m'],radius=l['radius_m']) for l in manifest['lamps']]
    for camera_name,mount in CAMERAS.items():
        camera=dict(mount,width_px=1280,height_px=720,horizontal_fov_deg=75)
        vertical_half=math.degrees(math.atan(math.tan(math.radians(37.5))*720/1280))
        for height in PEER_HEIGHTS:
            for slot in slots:
                projection=project(lamps,slot,camera);origin=np.array(projection['camera_world_xyz_m'])
                obstacles=track+peer_boxes(slots,slot['index'],height)
                inside=height is not None and any(
                    np.all(np.abs(rotation(0.,0.,s['yaw_rad']).T@(origin-np.array([s['x'],s['y'],height/2])))<=[.10,.075,height/2])
                    for s in slots if s['index']==slot['index'])
                blocked=[];facing=[]
                for lamp in manifest['lamps']:
                    xyz=np.asarray(lamp['xyz_m']);rad=lamp['radius_m'];right=np.cross(lamp['face_up'],n)
                    facing.append(float(np.dot(n,(origin-xyz)/np.linalg.norm(origin-xyz))))
                    front=xyz+.0011*n
                    targets=[front]+[front+.8*rad*(math.cos(a)*right+math.sin(a)*np.asarray(lamp['face_up']))
                                     for a in np.linspace(0,2*math.pi,8,endpoint=False)]
                    found=[hits(origin,t,obstacles) for t in targets]
                    blocked.append(dict(name=lamp['name'],blocked_samples=sum(bool(v) for v in found),
                                        blocking=sorted({x for h in found for x in h})))
                order=all(all(b>a for a,b in zip(c,c[1:])) for c in
                          ([projection['lamps'][2*k+row]['center_uv'][0] if projection['lamps'][2*k+row]['center_uv'] else -1e9
                            for k in range(5)] for row in range(2)))
                delta=center-origin
                rows.append(dict(camera=camera_name,peer_height_m=height,grid=slot['index'],
                    camera_world_xyz_m=origin.tolist(),array_elevation_deg=math.degrees(math.atan2(delta[2],math.hypot(*delta[:2]))),
                    array_horizontal_distance_m=float(math.hypot(*delta[:2])),vertical_half_fov_deg=vertical_half,
                    all_inside_image=projection['all_inside_image'],all_inside_detector_region=projection['all_inside_detector_region'],
                    left_to_right_each_row=order,min_facing_cos=min(facing),
                    camera_inside_own_body_box=bool(inside),
                    lamps_fully_unblocked=sum(b['blocked_samples']==0 for b in blocked),
                    lamps_partly_blocked=sum(0<b['blocked_samples']<9 for b in blocked),
                    lamps_fully_blocked=sum(b['blocked_samples']==9 for b in blocked),
                    blocking=sorted({x for b in blocked for x in b['blocking']}),
                    lamp_pixel_diameter=[r['diameter_approx_px'] for r in projection['lamps']],lamps=blocked))
    # 제작측 설계 가정(가장 앞 카메라·높이100 mm·시야60°)을 같은 위치에서 계산한 참고값. 팀 장착 제안 아님.
    lead=next(s for s in slots if s['index']==manifest['lead_grid_index'])
    reference=[]
    for s in slots:
        cam=np.array([s['x'],s['y'],0.])+rotation(0.,0.,s['yaw_rad'])@np.array([.10,0.,.10]);d=center-cam
        reference.append(dict(grid=s['index'],elevation_deg=math.degrees(math.atan2(d[2],math.hypot(*d[:2]))),
                              horizontal_distance_m=float(math.hypot(*d[:2]))))
    def view(r):return dict(camera=r['camera'],peer=r['peer_height_m'],grid=r['grid'],roi=r['all_inside_detector_region'],
        image=r['all_inside_image'],elev=round(r['array_elevation_deg'],1),free=r['lamps_fully_unblocked'],
        part=r['lamps_partly_blocked'],full=r['lamps_fully_blocked'],by=r['blocking'],inside_own=r['camera_inside_own_body_box'])
    result=dict(integrity_passed=True,scope='static pinhole + 9 face samples per lamp vs box visuals and nominal 20x15 cm peer boxes; '
                'not continuous visible area, lens distortion, rendering, detection or driving',
                geometry=geometry,lead_grid=lead['index'],organizer_camera_reference=reference,rows=rows,
                source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                               [Path(__file__),BASE/'prepare.py',*fixture.rglob('*')] if p.is_file()})
    with output.open('w',encoding='utf-8',newline='\n') as stream:stream.write(json.dumps(result,ensure_ascii=False,indent=2))
    with (BASE/'preflight_summary.json').open('w',encoding='utf-8',newline='\n') as stream:
        stream.write(json.dumps(dict(geometry=geometry,reference=reference,views=[view(r) for r in rows]),ensure_ascii=False,indent=1))
    for r in rows:print(json.dumps(view(r),ensure_ascii=False))
    return 0


if __name__=='__main__':raise SystemExit(main())
