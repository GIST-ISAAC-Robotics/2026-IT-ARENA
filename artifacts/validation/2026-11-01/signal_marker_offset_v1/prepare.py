"""10/29 제작측 신호 시험본에서 신호를 그대로 두고 ID45만 옮긴다."""
from pathlib import Path
import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET

import numpy as np

BASE=Path(__file__).resolve().parent
PARENT=BASE.parent.parent/'2026-10-29'/'signal_spec_1029_v1'
SHIFT=.20  # 팀 시험 가정: 진행 방향의 오른쪽. 실제 설치 위치가 아니다.


def lamp_blocks(origin, lamps, face, board, yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    axes=np.array([[c,s,0.],[-s,c,0.],[0.,0.,1.]])
    half=np.array([.0025,.05,.05])
    a=axes@(origin-board)
    result=[]
    for lamp in lamps:
        up=np.asarray(lamp['face_up']);right=np.cross(up,face)
        center=np.asarray(lamp['xyz_m'])+.0011*face
        targets=[center]+[center+.8*lamp['radius_m']*(math.cos(k*math.pi/4)*right+
                        math.sin(k*math.pi/4)*up) for k in range(8)]
        hits=0
        for target in targets:
            d=axes@(target-origin)
            parallel=np.abs(d)<1e-12
            if np.any(parallel&(np.abs(a)>half)):
                continue
            d=np.where(parallel,1.,d)
            t1,t2=(-half-a)/d,(half-a)/d
            lo=np.max(np.where(parallel,-np.inf,np.minimum(t1,t2)))
            hi=np.min(np.where(parallel,np.inf,np.maximum(t1,t2)))
            hits+=int(max(lo,0.)<=min(hi,1.) and hi>1e-6 and lo<1.-1e-6)
        result.append(hits)
    return result


def build():
    source=PARENT/'fixture'
    scene=json.loads((source/'scene.json').read_text(encoding='utf-8'))
    manifest=json.loads((source/'manifest.json').read_text(encoding='utf-8'))
    old=json.loads((PARENT/'preflight.json').read_text(encoding='utf-8'))
    tree=ET.parse(source/'world.sdf')
    board=tree.find("./world/model[@name='it_arena_track_static']/link[@name='aruco_45']")
    p=board.find('pose'); x,y,z,roll,pitch,yaw=map(float,p.text.split())
    assert board.findtext('visual/geometry/box/size')==board.findtext('collision/geometry/box/size')=='0.0050 0.1000 0.1000'
    slot=next(k for k in scene['starting_grid']['slots'] if k['index']==manifest['lead_grid_index'])
    dx,dy=SHIFT*math.sin(slot['yaw_rad']),-SHIFT*math.cos(slot['yaw_rad'])
    before=np.array([x,y,z]); after=before+np.array([dx,dy,0.])
    results=[]
    for row in old['rows']:
        if row['peer_height_m'] is not None:
            continue
        origin=np.asarray(row['camera_world_xyz_m'])
        a,b=[lamp_blocks(origin,manifest['lamps'],np.asarray(manifest['face_normal']),v,yaw)
             for v in (before,after)]
        # 기존 기록은 다른 지지물까지 검사한다. 여기서는 ID45의 가림만 대조한다.
        if set(row['blocking'])<= {'aruco_45'}:
            assert a==[v['blocked_samples'] for v in row['lamps']]
        results.append(dict(camera=row['camera'],grid=row['grid'],in_image=row['all_inside_image'],
                            before_blocked_face_samples=sum(a),after_blocked_face_samples=sum(b)))
    assert max(r['after_blocked_face_samples'] for r in results)==0
    original_world=(source/'world.sdf').read_bytes()
    # 원본 SDF를 재직렬화하지 않고 유일한 링크 자세만 교체한다.
    old_text=p.text
    new_text=' '.join([f'{x+dx:.8f}',f'{y+dy:.8f}',*old_text.split()[2:]])
    anchor=f'<link name="aruco_45">\n        <pose>{old_text}</pose>'.encode()
    assert original_world.count(anchor)==1
    world=original_world.replace(anchor,anchor.replace(old_text.encode(),new_text.encode()))
    marker=next(v for v in scene['aruco_markers']['markers'] if v['id']==45)
    assert (marker['pose']['x'],marker['pose']['y'])==(x,y)
    marker['pose'].update(x=x+dx,y=y+dy)
    marker['note']='팀 시험본: 신호 가림을 줄이기 위해 ID45만 진행 오른쪽 0.20 m로 옮겼다. 높이·방향·이미지는 유지.'
    scene['aruco_markers']['team_exception']='ID45만 옆으로 이동. 원본 중심선 위 시설 검사 카메라 위치는 유지.'
    out=BASE/'fixture'
    out.mkdir(exist_ok=False)
    for name,digest in manifest['assets_sha256'].items():
        f=source/name
        assert hashlib.sha256(f.read_bytes()).hexdigest()==digest
        (out/name).parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(f,out/name)
    shutil.copyfile(source/'centerline.csv',out/'centerline.csv')
    (out/'world.sdf').write_bytes(world)
    (out/'scene.json').write_text(json.dumps(scene,ensure_ascii=False,indent=2),encoding='utf-8')
    report=dict(shift_right_m=SHIFT,old_center=before.tolist(),new_center=after.tolist(),
        unchanged_except_one_sdf_pose=world.replace(new_text.encode(),old_text.encode(),1)==original_world,
        parent_sha256=hashlib.sha256(original_world).hexdigest(),output_sha256=hashlib.sha256(world).hexdigest(),
        scope='only ID45 ray intersections; same camera/FOV and lamp arrangement. No hardware or peer clearance claim.',
        rows=results)
    assert report['unchanged_except_one_sdf_pose']
    (BASE/'geometry.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    build()
