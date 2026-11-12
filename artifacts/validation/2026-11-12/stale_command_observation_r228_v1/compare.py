"""실제 센서와 별개인 명령 갱신 공백의 노드 경계 합성 반례."""
from pathlib import Path
from types import SimpleNamespace
import hashlib,json,time
import numpy as np
from builtin_interfaces.msg import Time
from arena_autonomy.lidar_safety import LidarSafety
from arena_autonomy.partial_observation import PartialObservationPolicy
from test_lidar_safety import node_double
from test_partial_observation import sample
from test_local_follow import history
from candidate import control

A=Path(__file__).resolve().parent
def stamp(t):return Time(sec=int(t),nanosec=round((t-int(t))*1e9))
def run(fn,kind):
    node,commands,statuses=node_double()
    node.partial_policy=PartialObservationPolicy(allow_narrow_forward=True)
    node.motion.history=history()
    points=np.full((60,2),20.)
    node.motion.project=lambda raw:(points.copy(),len(points))
    node.command.drive.speed=.2
    results=[]
    for now in np.arange(.5,1.61,.1):
        now=round(float(now),6)
        node.get_clock=lambda t=now:SimpleNamespace(now=lambda:SimpleNamespace(
            nanoseconds=round(t*1e9),to_msg=lambda:stamp(t)))
        node.command.header.stamp=stamp(.5 if now<1.6 else now)
        node.command_wall=time.monotonic()-(4. if kind=='command_transport' and .8<=now<1.6 else 0.)
        node.scan_wall=time.monotonic()
        node.scan=sample(now-.1,[] if now==.5 else [250])
        points[:]=20.
        if kind=='known_disappearance' and now<=.7:points[0]=[.4,0.]
        if kind=='scan_missing' and .8<=now<1.6:node.scan=sample(.4,[250])
        fn(node)
        report=json.loads(statuses[-1].data)
        p=report.get('partial_observation') or {}
        results.append(dict(now=now,speed=commands[-1].drive.speed,reason=report['reason'],
            partial=p.get('reason'),latch=p.get('latch_reason'),
            previous_source=node.partial_policy.previous[2] if node.partial_policy.previous else None))
    return results
results={kind:{label:run(fn,kind) for label,fn in [('baseline',LidarSafety.control),('candidate',control)]}
         for kind in ('command_age','command_transport','scan_missing','known_disappearance')}
checks={}
for kind,pair in results.items():
    b,c=pair['baseline'],pair['candidate']
    checks[kind+'_expired_output_zero']=all(r['speed']==0. for r in c if .8<=r['now']<1.6)
    if kind in ('command_age','command_transport'):
        checks[kind+'_baseline_history_expired']=b[-1]['latch']=='partial_history_expired' and b[-1]['speed']==0.
        checks[kind+'_fresh_command_recovers']=c[-1]['speed']==.2 and c[-1]['latch'] is None
    elif kind=='scan_missing':checks['missing_scan_stays_latched']=c[-1]['latch']=='partial_history_expired' and c[-1]['speed']==0.
    else:checks['known_disappearance_stays_latched']=c[-1]['latch']=='previous_point_in_missing_sweep' and c[-1]['speed']==0.
report=dict(passed=all(checks.values()),checks=checks,rows=results,
    scope='synthetic node boundary with fresh sensor input; not replay of r225 transport or hardware',
    candidate_sha256=hashlib.sha256((A/'candidate.py').read_bytes()).hexdigest())
with (A/'report.json').open('x') as f:json.dump(report,f,indent=2);f.write('\n')
print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
raise SystemExit(0 if report['passed'] else 1)
