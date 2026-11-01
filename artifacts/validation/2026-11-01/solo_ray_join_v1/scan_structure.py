"""시뮬레이션 LiDAR의 인접 광선 거리 반복을 센다. 원인 판단 보조이며 실물 C1 특성 측정이 아니다."""
from collections import Counter
from hashlib import sha256
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig

source=HERE.parent/'solo_corridor_arc_v1/run1/follow_decisions.jsonl'
seen=set();equal=Counter();total=Counter();samples=[]
with source.open(encoding='utf-8') as stream:
    for line in stream:
        e=json.loads(line);d=e['data']
        if e['kind']=='header':motion=MotionConfig(**d['motion_config'])
        if e['kind']!='call':continue
        inp,scan,_=restore_call(d['input'],motion)
        key=(scan.header.stamp.sec,scan.header.stamp.nanosec)
        if key in seen:continue
        seen.add(key)
        ranges=np.asarray(scan.ranges,dtype=float)
        angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
        valid=np.isfinite(ranges[:-1])&np.isfinite(ranges[1:])
        same=valid&(ranges[:-1]==ranges[1:])
        sector=((np.degrees(np.abs(angles[:-1]))//15)*15).astype(int)
        for value in np.unique(sector):
            total[int(value)]+=int(np.sum(valid&(sector==value)))
            equal[int(value)]+=int(np.sum(same&(sector==value)))
        if len(samples)<1 and np.sum(same)>0:
            i=int(np.flatnonzero(same&(np.abs(angles[:-1])<.4))[0]) if np.any(same&(np.abs(angles[:-1])<.4)) else 0
            samples.append(dict(index=i,ranges=ranges[i-2:i+4].tolist()))
result=dict(scope='저장 스캔의 인접 광선 거리 완전 동일 비율. 시뮬레이터 표본화 영향의 진단이다.',scans=len(seen),
    by_abs_angle_deg={str(k):dict(pairs=total[k],equal=equal[k],share=equal[k]/max(1,total[k])) for k in sorted(total)},sample=samples,
    input_sha256=sha256(source.read_bytes()).hexdigest())
(HERE/'scan_structure.json').write_text(json.dumps(result,ensure_ascii=False,indent=1),encoding='utf-8',newline='\n')
print(json.dumps(result,ensure_ascii=False))
