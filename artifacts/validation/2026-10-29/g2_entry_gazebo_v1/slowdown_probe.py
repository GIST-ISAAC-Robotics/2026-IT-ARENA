"""run1의 13.2–13.5 s 차체 감속·모터축 측정 급상승 구간의 바퀴 지면/자세 진단(평가 전용)."""
import json,sys
from bisect import bisect_right
from pathlib import Path
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from surface_contact_eval import SurfaceMap,evaluate_pose,load_steering
run=BASE/'run1'
surface=SurfaceMap.from_world(run/'runtime_world.sdf')
ego=[json.loads(l) for l in (run/'ego_poses.jsonl').read_text().splitlines() if l.strip()]
steer=load_steering(run);index=[s[0] for s in steer]
dyn=[json.loads(l) for l in (run/'braking_diagnostics.jsonl').read_text().splitlines() if l.strip()]
print('diag keys',sorted(dyn[0].keys())[:40])
rows=[]
for t,x,y,yaw in ego:
    if 12.6<=t<=13.9 and (not rows or t-rows[-1]['t']>=.049):
        i=bisect_right(index,t)-1;st=steer[i][1] if i>=0 else 0.
        e=evaluate_pose(surface,x,y,yaw,st)
        wheels={k:max(v['fractions'],key=v['fractions'].get) if isinstance(v,dict) and 'fractions' in v else v for k,v in e.get('wheels',{}).items()}
        rows.append(dict(t=round(t,3),x=round(x,4),y=round(y,4),yaw=round(yaw,4),steer=round(st,3),wheels=wheels,off=e.get('off_wheels')))
for r in rows:print(r)
d=[r for r in dyn if 12.9<=r.get('sim_time_s',0)<=13.9]
d=[r for r in dyn if 12.6<=r.get('sim_time_s',0)<=13.9]
for r in d[::3]:print({k:(round(v,4) if isinstance(v,float) else v) for k,v in r.items() if k in ('sim_time_s','truth_longitudinal_mps','reference_speed_mps','applied_target_mps','motor_speed_rad_s','motor_torque_nm','raw_demand_nm','limited_demand_nm','wheel_surface_speed_mps','left_speed_rad_s','right_speed_rad_s','truth_pitch_rad','truth_roll_rad','truth_z_m','left_steering_rad','pi_integral_after_nm','watchdog_stop')})
