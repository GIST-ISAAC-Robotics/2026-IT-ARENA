"""옛 위치의 명시 반환 대조. 평가 정답은 별도 진단에만 사용한다."""
import json,hashlib,math
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import transport_points,motion_scene
from probe_anchor_clearance import fresh_clearance

BASE=Path(__file__).resolve().parent
path=BASE/'target_frames.json';before=hashlib.sha256(path.read_bytes()).hexdigest()
saved=json.loads(path.read_text(encoding='utf-8'));motion=MotionConfig(**saved['header']['motion_config'])
frames=saved['frames'];latch=next(r for r in frames if abs(r['data']['input']['reference_s']-129.88)<1e-8)
e=latch['data']['output'][2]['partial_observation']['latch_evidence'];ref=e['previous_reference_s']
old=np.array(e['correspondence_context']['previous_points_at_reference_xy'])
data,raw,h=restore_call(latch['data']['input'],motion)
carried=transport_points(old,h,ref,data['reference_s'])
i=int(np.argmin(np.linalg.norm(carried-e['witnesses'][0]['previous_point_now_xy'],axis=1)))
rows=[];last=None;consecutive=0
for row in frames:
    data,raw,h=restore_call(row['data']['input'],motion);t=data['reference_s']
    source=raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9
    if t<129.88-1e-9 or source==last:continue
    last=source
    current,ids,_=h.project(raw.ranges,raw.angle_min,raw.angle_increment,raw.range_min,raw.range_max,
        source,raw.time_increment,t,mode='both',timing_verified=True)
    scene=motion_scene(raw,h,t,timing_verified=True)
    result=fresh_clearance(old[[i]],old,ref,scene,h,current,ids)
    consecutive=consecutive+1 if result['accepted'] else 0
    rows.append(dict(t_s=t,source_s=source,consecutive_fresh=consecutive,result=result))
# 독립 평가: 원래 점을 평가 자세로 월드에 옮겨 actor 경계까지의 거리만 비교한다.
# 이 값은 위 함수/차량 제어의 입력이 아니다. 실제 물체 식별의 증명도 아니다.
run=BASE.parent/'traffic_speed_contract_v1/run1_course'
def near_rows(name,t):
    best=None
    for line in (run/name).open(encoding='utf-8'):
        r=json.loads(line)
        if best is None or abs(r[0]-t)<abs(best[0]-t):best=r
        if r[0]>t+.01:break
    return best
ego=near_rows('ego_poses.jsonl',ref);actor=near_rows('actor_poses.jsonl',ref)
def world(p,pose):
    c,s=math.cos(pose[3]),math.sin(pose[3]);return np.array([pose[1]+c*p[0]-s*p[1],pose[2]+s*p[0]+c*p[1]])
wp=world(old[i],ego)
def actor_local(p,pose):
    c,s=math.cos(pose[3]),math.sin(pose[3]);d=p-np.array(pose[1:3]);return [c*d[0]+s*d[1],-s*d[0]+c*d[1]]
additional=json.loads((run/'additional_actors.json').read_text(encoding='utf-8'))['poses']
side={name:min(values,key=lambda r:abs(r[0]-ref)) for name,values in additional.items()}
truth=dict(scope='evaluation only, never supplied to clearance candidate',reference_s=ref,
    ego=ego,old_point_world_xy=wp.tolist(),main_actor=actor,main_actor_local_point=actor_local(wp,actor),
    other_actor_local_points={name:actor_local(wp,pose) for name,pose in side.items()})
report=dict(input_sha256=before,input_unchanged=before==hashlib.sha256(path.read_bytes()).hexdigest(),
    rows=rows,evaluation_only=truth)
with (BASE/'clearance_comparison.json').open('x',encoding='utf-8') as f:json.dump(freeze(report),f,ensure_ascii=False,indent=2)
print(json.dumps(dict(rows=[dict(t_s=r['t_s'],reason=r['result']['reason'],n=r['consecutive_fresh'],
    proofs=r['result'].get('proofs')) for r in rows],truth=truth)))
