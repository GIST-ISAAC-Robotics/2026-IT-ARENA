"""평가 전용 차체 이동과 요청/보호/가상 MCU의 구간 대조. 제어 입력 아님."""
import bisect,collections,json,math,statistics
from pathlib import Path
A=Path(__file__).resolve().parent;D=A/'two_actor_run1';R=D/'run1'
def lines(p):return [json.loads(x) for x in p.read_text().splitlines() if x.strip()]
pose=lines(R/'ego_poses.jsonl');mcu=json.loads((R/'actuation_trace.json').read_bytes())
status=lines(D/'full_status.jsonl');safety=lines(D/'full_safety.jsonl')
def med(v):return statistics.median(v) if v else None
rows=[]
for lo,hi in ((8,15),(15,20),(20,30),(30,40),(40,50),(50,60),(60,70),(70,80),(80,88)):
    p=[v for v in pose if lo<=v[0]<hi];m=[v for v in mcu if lo<=v['now_us']/1e6<hi]
    s=[v for v in status if lo<=v['sim_time_s']<hi];g=[v for v in safety if lo<=v['reference_s']<hi]
    distance=sum(math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in zip(p,p[1:]))
    rows.append(dict(from_s=lo,to_s=hi,body_path_m=distance,first_pose=p[0] if p else None,last_pose=p[-1] if p else None,
        commanded_median=med([v['speed_command_mps'] for v in s]),safe_median=med([v['safe_speed_mps'] for v in g]),
        mcu_target_median=med([v['target_speed_mps'] for v in m]),motor_mean_median=med([v['mean_wheel_mps'] for v in m]),
        guard_reasons=dict(collections.Counter(v['reason'] for v in g)),mcu_reasons=dict(collections.Counter(v['reason'] for v in m))))
result=dict(rows=rows,first_actor_contact=next(iter(lines(R/'actor_contacts.jsonl')),None),
    scope='offline evaluation poses only; body path is traveled length, not legal course progress or controller input')
(A/'motion_chain_audit.json').write_text(json.dumps(result,indent=2)+'\n')
for row in rows:print(json.dumps(row))
