"""자산·설정·실제 시선 가림을 함께 검사한다. 주행이나 전체 픽셀 검출 증거는 아니다."""
import hashlib
import importlib.util
import itertools
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from audit_start_signal_geometry import pose, rotation, project


def boxes(world):
    rows=[]
    for model in world.findall('model'):
        m,mr=pose(model)
        for link in model.findall('link'):
            l,lr=pose(link)
            for visual in link.findall('visual'):
                box=visual.find('geometry/box/size')
                if box is None:
                    # 이 입력의 비상자 형상은 광원 구체·지면·낮은 방지턱이다. 연속 면적 증명은 아니다.
                    continue
                v,vr=pose(visual)
                rows.append((link.get('name'),m+mr@(l+lr@v),mr@lr@vr,np.array(list(map(float,box.text.split())))/2))
    return rows


def hits(origin,target,obstacles):
    centers=np.array([r[1] for r in obstacles]);rs=np.array([r[2] for r in obstacles]);half=np.array([r[3] for r in obstacles])
    a=np.einsum('nji,nj->ni',rs,origin-centers)
    d=np.einsum('nji,nj->ni',rs,target-origin+np.zeros_like(centers))
    parallel=np.abs(d)<1e-12
    outside=np.any(parallel & (np.abs(a)>half),axis=1)
    nonzero=np.where(parallel,1.,d)
    t1,t2=(-half-a)/nonzero,(half-a)/nonzero
    lower=np.where(parallel,-np.inf,np.minimum(t1,t2)).max(axis=1)
    upper=np.where(parallel,np.inf,np.maximum(t1,t2)).min(axis=1)
    mask=(~outside)&(np.maximum(lower,0.)<=np.minimum(upper,1.))&(upper>1e-6)&(lower<1.-1e-6)
    return [obstacles[i][0] for i in np.flatnonzero(mask)]


def signature(e):return (e.tag,sorted(e.attrib.items()),(e.text or '').strip(),[signature(c) for c in e])


def main():
    output=BASE/'preflight.json'
    if output.exists():raise FileExistsError(output)
    fixture=BASE/'fixture';scene=json.loads((fixture/'scene.json').read_text(encoding='utf-8'))
    manifest=json.loads((fixture/'manifest.json').read_text(encoding='utf-8'))
    world=ET.parse(fixture/'world.sdf').find('world');model=world.find("model[@name='it_arena_track_static']")
    source=ET.parse(ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf').find('world')
    old=source.find("model[@name='it_arena_track_static']")
    for link in old.findall('link'):
        if link.get('name') not in manifest['removed_links']:
            assert signature(link)==signature(model.find(f"link[@name='{link.get('name')}']"))
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in manifest['sources'].items())
    assert all(hashlib.sha256((fixture/p).read_bytes()).hexdigest()==h for p,h in manifest['assets_sha256'].items())
    assert 'lamp_poses' not in scene['traffic_light']
    spec=importlib.util.spec_from_file_location('trial',BASE/'run.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    launch,params,args=module.wrapper_launch(BASE/'run1');compile(launch,'trial launch','exec')
    assert params['start_signal_mode']=='official_4red_candidate' and args['actuation_mode']=='virtual_mcu'
    assert args['traffic_light']=='false' and params['partial_observation_policy']=='bounded_local_v4'
    camera=dict(xyz_m=manifest['camera_xyz_m'],rpy_rad=manifest['camera_rpy_rad'],width_px=1280,height_px=720,horizontal_fov_deg=75)
    lamps=[dict(name=l['name'],xyz=l['xyz_m'],radius=l['radius_m']) for l in manifest['lamps']]
    obstacles=boxes(world);rows=[]
    for slot in scene['starting_grid']['slots']:
        projection=project(lamps,slot,camera);origin=np.array(projection['camera_world_xyz_m'])
        cr=rotation(0.,0.,slot['yaw_rad']);samples=[]
        for lamp in lamps:
            # 광원 구체의 카메라 쪽 중심과8점. 연속 가시면적의 증명은 아니다.
            xyz=np.asarray(lamp['xyz']);rad=lamp['radius'];toward=(origin-xyz)/np.linalg.norm(origin-xyz)
            targets=[xyz+toward*rad]
            targets += [xyz+toward*rad*math.sqrt(1-.8**2)+.8*rad*(math.cos(a)*cr[:,1]+math.sin(a)*cr[:,2])
                        for a in np.linspace(0,2*math.pi,8,endpoint=False)]
            blocked=[hits(origin,t,obstacles) for t in targets]
            samples.append(dict(name=lamp['name'],blocked_samples=sum(bool(v) for v in blocked),blocking_links=sorted({x for h in blocked for x in h}),samples=len(blocked)))
        for row in range(2):
            columns=[projection['lamps'][2*c+row]['center_uv'][0] for c in range(5)]
            assert all(b>a for a,b in zip(columns,columns[1:]))
        rows.append(dict(grid=slot['index'],projection=projection,occlusion=samples,
                         unobstructed=all(v['blocked_samples']==0 for v in samples)))
    # 원래 실패 배치에서 ID45 가림을 다시 잡는지 음성 대조한다.
    old_world=ET.parse(BASE.parent/'official_departure_v1/fixture/world.sdf').find('world')
    old_model=old_world.find("model[@name='it_arena_track_static']")
    old_obstacles=boxes(old_world);origin=np.asarray(next(r for r in rows if r['grid']==3)['projection']['camera_world_xyz_m'])
    blocked_old={}
    for lamp in lamps:
        target=np.array(list(map(float,old_model.findtext(f"link[@name='{lamp['name']}']/pose").split()))[:3])
        blocked_old[lamp['name']]=hits(origin,target,old_obstacles)
    assert all('aruco_45' in blocked_old[f'trial_lamp_{c}_{r}'] for c,r in itertools.product(range(4),range(2)))
    passed=all(r['unobstructed'] and r['projection']['all_inside_detector_region'] for r in rows)
    result=dict(passed=passed,scope='six static grid pinhole/9-ray samples, no continuous visible-area, peer or driving proof',
                same_non_signal_geometry=True,assets_verified=manifest['assets_sha256'],parameters=params,launch_arguments=args,
                grids=rows,previous_failure_detected=blocked_old,box_count=len(obstacles),
                source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                              [Path(__file__),BASE/'prepare.py',BASE/'run.py',BASE/'signal_driver.py',*fixture.rglob('*')] if p.is_file()})
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(passed=passed,grids=[dict(grid=r['grid'],in_roi=r['projection']['all_inside_detector_region'],
        blocked=[v for v in r['occlusion'] if v['blocked_samples']]) for r in rows]),ensure_ascii=False))
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
