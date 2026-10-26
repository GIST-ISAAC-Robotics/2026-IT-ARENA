"""저장 장면의 후방 마스크 경계 광선이 후면 평면(x=-0.10 m)을 지나는 횡위치로 꼬리 띠 폭(|y|-0.075 m)을 잰다.

10판단마다 1장면 표본. 운동 보정 원점은 실제 판단 입력과 같은 motion_scene으로 만든다. 판정·제어에 쓰지 않는다.
"""
import sys, json, math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
for p in ('src/arena_autonomy','src/arena_vehicle_interface','scripts','tests'): sys.path.insert(0,str(ROOT/p))
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig
out={}
for name in ['run17_observed_release_closed_loop','run21_right_encounter','run19_stop_restart','run35_committed_surface_corner']:
    path=ROOT/'artifacts/validation/2026-10-25/m3_execution_review_v1'/name/'run/follow_decisions.jsonl'
    widths={'left':[], 'right':[]}; speeds=[]
    with open(path) as f:
        head=json.loads(f.readline())['data']; motion=MotionConfig(**head['motion_config'])
        k=0
        for line in f:
            e=json.loads(line)
            if e['kind']!='call': continue
            k+=1
            if k%10: continue
            data,scan,hist=restore_call(e['data']['input'],motion)
            try:
                scene=motion_scene(scan,hist,data['reference_s'],timing_verified=data['timing_verified'],epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
            except Exception as ex:
                continue
            speeds.append(e['data']['input']['speed_mps'])
            for side,sgn in (('left',1),('right',-1)):
                # 마스크 경계 광선: 마스크 광선 중 후방 쐐기의 해당 측 가장자리(방향 y 부호가 sgn이며 |각|이 가장 작은 것)
                best=None
                for r in scene.rays:
                    if r.status!='masked': continue
                    dx,dy=r.direction_xy
                    if sgn*dy<=0: continue
                    ang=abs(math.atan2(dy,dx))
                    if best is None or ang<best[0]: best=(ang,r)
                if best is None: continue
                r=best[1]; ox,oy=r.ray_origin_xy_m; dx,dy=r.direction_xy
                t=(-0.10-ox)/dx
                y=oy+t*dy
                widths[side].append(sgn*y-0.075)
    out[name]={s:{'n':len(v),'min_m':float(np.min(v)),'median_m':float(np.median(v)),'max_m':float(np.max(v))} for s,v in widths.items()}
    out[name]['speed_median']=float(np.median(speeds))
    print(name, json.dumps(out[name]))
Path(sys.argv[1]).write_text(json.dumps({'scope':'saved-scene geometry sample (every 10th decision); not a hardware mask measurement','results':out},indent=1)+'\n',encoding='utf-8',newline='\n')
