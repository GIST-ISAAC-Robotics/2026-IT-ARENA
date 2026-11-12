"""앞차 상자를 가리지 않고 넘겨보는 렌즈 높이 계산. 제조 설계 승인 아님."""
import importlib.util
from pathlib import Path

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('audit',HERE/'audit.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
rows=[]
for slot in a.SLOTS:
    others=[s for s in a.SLOTS if s['index']!=slot['index']]
    targets=a.targets(-.009)
    for h in (.224,.230,.250,.280,.350):
        result={}
        for kind in ('lamp','aruco'):
            points=[p for t in targets if t['name'].startswith(kind) for p in t['samples']]
            low,high=.05,.40
            for _ in range(32):
                mid=(low+high)/2;origin,_=a.camera_pose(slot,dz=mid-a.MOUNT['xyz_m'][2])
                clear=all(not any(a.ray_box(origin,p,s,h) for s in others) for p in points)
                if clear:high=mid
                else:low=mid
            result[kind+'_min_camera_axis_above_road_m']=high
        rows.append(dict(slot=slot['index'],other_height_m=h,**result))
a.save(HERE/'mount_margin.json',dict(assumptions='nominal 6 aligned slots; opaque 200x150 mm actors; lower head 9 mm; exact sampled rays without clearance allowance',rows=rows))
for h in (.224,.230,.250,.280,.350):
    chosen=[r for r in rows if r['other_height_m']==h]
    print(dict(other_height=h,lamps=max(r['lamp_min_camera_axis_above_road_m'] for r in chosen),markers=max(r['aruco_min_camera_axis_above_road_m'] for r in chosen)))
