"""옛 위치 회복 직전 상태에서 센서 반례를 분기한다. 원시 입력은 불변이다."""
from collections import Counter
import copy,hashlib,json,math,sys
from pathlib import Path
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path.insert(0,str(BASE.parent/'traffic_residual_v1'))
from safety_shadow import ShadowSafety
from arena_autonomy.follow_trace import thaw,freeze,replay_call
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy

trace=BASE.parent/'traffic_speed_contract_v1/run1_course/follow_decisions.jsonl'
def digest(path):
    with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
before=digest(trace);prefix=0;window=[];gate=None
with trace.open(encoding='utf-8') as stream:
    header=json.loads(next(stream))['data'];cfg=FollowConfig(**header['follow_config'])
    follow=LocalFollow(cfg);motion=MotionConfig(**header['motion_config'])
    for line in stream:
        row=json.loads(line)
        if row['kind']=='reset':
            if window:raise RuntimeError('unexpected reset in branch window')
            follow.reset();continue
        if row['kind']!='call':continue
        data=row['data']['input'];t=data['reference_s']
        if t>=130.20:
            if t>131.0:break
            window.append(data);continue
        command=replay_call(follow,data,motion)
        if gate is None:gate=ShadowSafety(RecoveryPolicy(True,True,True),cfg,data['rear_half_angle_deg'])
        gate.step(data,motion,command[:2]);prefix+=1

def change(data,kind):
    d=thaw(data);raw=d['scan']
    if kind in ('near_front','front_unknown'):
        for i in range(len(raw['ranges'])):
            angle=(raw['angle_min']+i*raw['angle_increment']+motion.lidar_yaw_rad+math.pi)%math.tau-math.pi
            if abs(angle)<=math.radians(15):raw['ranges'][i]=.1 if kind=='near_front' else math.inf
    if kind=='gyro_gap':
        gyro=d['history']['gyro'];kept=[(t,v) for t,v in zip(gyro['times'],gyro['values']) if t<d['reference_s']-.1]
        gyro['times']=[t for t,v in kept];gyro['values']=[v for t,v in kept]
    return freeze(d)

results={}
for kind in ('unchanged','near_front','front_unknown','gyro_gap','explicit_zero'):
    candidate=copy.deepcopy(follow)
    safety=ShadowSafety(copy.deepcopy(gate.node.partial_policy),cfg,window[0]['rear_half_angle_deg'])
    safety.node.previous_safe_steering=gate.node.previous_safe_steering
    safety.node.last_clock=gate.node.last_clock
    rows=[];reasons=Counter();safe_reasons=Counter()
    for original in window:
        data=change(original,kind);out=replay_call(candidate,data,motion)
        command=(0.,out[1]) if kind=='explicit_zero' else out[:2]
        safe,status=safety.step(data,motion,command)
        rows.append(dict(t_s=data['reference_s'],command=command,safe=safe))
        reasons[out[2].get('reason')]+=1;safe_reasons[status.get('reason')]+=1
    results[kind]=dict(calls=len(rows),positive_commands=sum(r['command'][0]>0 for r in rows),
        positive_safe=sum(r['safe'][0]>0 for r in rows),max_safe=max(r['safe'][0] for r in rows),
        controller_reasons=dict(reasons),safety_reasons=dict(safe_reasons),rows=rows)
checks=dict(baseline_progress=results['unchanged']['positive_safe']>0,
    near_front_stops=results['near_front']['positive_safe']==0,
    front_unknown_stops=results['front_unknown']['positive_safe']==0,
    gyro_gap_stops=results['gyro_gap']['positive_safe']==0,
    explicit_zero_stays_zero=results['explicit_zero']['positive_safe']==0,
    input_unchanged=before==digest(trace))
report=dict(scope='saved sensor counterfactual; independent guard function, not ROS or new drive',
    input_sha256=before,prefix_calls=prefix,checks=checks,passed=all(checks.values()),results=results)
with (BASE/'counterfactual_boundaries.json').open('x',encoding='utf-8') as f:json.dump(freeze(report),f,ensure_ascii=False,indent=2)
print(json.dumps(dict(passed=report['passed'],checks=checks,summary={k:{n:v for n,v in r.items() if n!='rows'} for k,r in results.items()})))
raise SystemExit(not report['passed'])
