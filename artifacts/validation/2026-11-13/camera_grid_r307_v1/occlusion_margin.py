"""높이 경계의 표본 광선 계산. 정답 기하를 제어기에 제공하지 않는 평가 전용."""
import importlib.util
import itertools
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('audit',HERE/'audit.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)


def threshold(origin,points,obstacles):
    def clear(height):
        return all(not any(a.ray_box(origin,p,s,height) for s in obstacles) for p in points)
    low,high=0.,.5
    if clear(high):return None
    for _ in range(30):
        mid=(low+high)/2
        if clear(mid):low=mid
        else:high=mid
    return low


rows=[]
for shift in (0.,-.009):
    targets=a.targets(shift)
    for slot in a.SLOTS:
        other=[s for s in a.SLOTS if s['index']!=slot['index']]
        origin,_=a.camera_pose(slot)
        lamps=[p for t in targets if t['name'].startswith('lamp') for p in t['samples']]
        markers=[p for t in targets if t['name'].startswith('aruco') for p in t['samples']]
        low,_=a.camera_pose(slot,dz=-.01,dx=.02)
        per_height={}
        for h in (.224,.23,.24,.25,.26,.28,.35):
            per_height[str(h)]={t['name']:sum(not any(a.ray_box(origin,p,s,h) for s in other) for p in t['samples'])/len(t['samples']) for t in targets}
        rows.append(dict(slot=slot['index'],head_shift_m=shift,
                         first_lamp_sample_occlusion_height_m=threshold(origin,lamps,other),
                         lower_camera_10mm_forward_20mm_height_m=threshold(low,lamps,other),
                         first_marker_sample_occlusion_height_m=threshold(origin,markers,other),
                         visible_sample_fraction=per_height))
a.save(HERE/'occlusion_margin.json',dict(scope='sample ray occlusion; not detector success or calibrated hardware',rows=rows))
for row in rows:
    print({k:v for k,v in row.items() if k!='visible_sample_fraction'},flush=True)
