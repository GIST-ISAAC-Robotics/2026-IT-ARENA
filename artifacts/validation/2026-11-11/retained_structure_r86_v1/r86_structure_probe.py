import json
from pathlib import Path
import sys
import numpy as np
root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root/p) for p in ('src/arena_autonomy','src/arena_vehicle_interface','scripts')]
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.known_structure import structure_policy, classify
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
A=root/'artifacts/validation/2026-11-11/guard_masked_speed_r84_v1'
fixture=json.loads((A/'latch_sensor_fixture.json').read_bytes())
trace=next((A/'two_actor_run1/run1').rglob('follow_decisions.jsonl'))
with trace.open(encoding='utf8') as f: header=json.loads(next(f))['data']
data,scan,history=restore_call(fixture['input'],MotionConfig(**header['motion_config']))
policy=structure_policy(RecoveryPolicy,header['follow_config']['known_structure_sectors_json'])()
valid,excused,rows=classify(scan,policy.structure_sectors)
shadows=policy._shadow(scan,rows,history,data['reference_s'],scan.acquired_at_s[0])
points=np.unique(np.array([w['previous_point_now_xy'] for w in fixture['witnesses']]),axis=0)
print(json.dumps(dict(rows=[dict(center=r['center_deg'],explained=r['explained'],first=int(r['rays'][0]),last=int(r['rays'][-1])) for r in rows],
 points=[dict(xy=p.tolist(),inside=[s['center_deg'] for s in shadows if policy._inside([p],[s])[0]]) for p in points]),indent=2))
