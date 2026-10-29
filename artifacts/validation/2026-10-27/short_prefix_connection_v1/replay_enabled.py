"""실제 선택형 설정의 저장 입력 비교. 명령 발행·새 물리 주행은 하지 않는다."""
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT=Path(__file__).resolve().parents[4]
for directory in ('scripts','src/arena_autonomy','src/arena_vehicle_interface'):
    sys.path.insert(0,str(ROOT/directory))
from arena_autonomy.follow_trace import replay_call,freeze
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.lidar_motion import MotionConfig

base=Path(__file__).resolve().parent
trace=ROOT/'artifacts/validation/2026-10-27/g1_traffic_lap_v1/run1_course/follow_decisions.jsonl'
shadow=json.loads((ROOT/'artifacts/validation/2026-10-27/g1_traffic_lap_v1/prefix_replay_v1/changes.json').read_text())
reference={row['t_s']:row['candidate'] for row in shadow}
input_sha=hashlib.sha256(trace.read_bytes()).hexdigest()
output=base/'enabled_replay_v1.json'
if output.exists():raise FileExistsError(output)
counts=Counter();changes=[];differences=[];costs=[]
with trace.open() as file:
    header=json.loads(next(file))['data']
    cfg=replace(FollowConfig(**header['follow_config']),short_boundary_prefix_enabled=True)
    ctl=LocalFollow(cfg);motion=MotionConfig(**header['motion_config'])
    for line in file:
        row=json.loads(line)
        if row['kind']=='reset':ctl.reset();counts['resets']+=1;continue
        if row['kind']!='call':continue
        data=row['data'];stamp=data['input']['reference_s']
        if stamp>120.5:break
        begin=time.perf_counter();actual=replay_call(ctl,data['input'],motion)
        costs.append((time.perf_counter()-begin)*1000);counts['calls']+=1
        expected=reference.get(stamp,data['output'][:2])
        if not np.allclose(actual[:2],expected,rtol=0,atol=1e-10):
            differences.append(dict(t_s=stamp,expected=expected,actual=actual[:2]))
        if not np.allclose(actual[:2],data['output'][:2],rtol=0,atol=1e-10):
            counts['command_changed']+=1
            changes.append(dict(t_s=stamp,command=actual[:2],detail=actual[2]))
report=dict(scope='saved-input open-loop runtime-option comparison, no ROS or physics',counts=dict(counts),
    matches_reviewed_shadow=not differences,shadow_command_differences=differences,
    changed_commands=changes,wall_call_ms={'p50':float(np.percentile(costs,50)),
    'p95':float(np.percentile(costs,95)),'max':max(costs)},
    input_sha256=input_sha,input_unchanged=hashlib.sha256(trace.read_bytes()).hexdigest()==input_sha)
output.write_text(json.dumps(freeze(report),ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='changed_commands'},ensure_ascii=False))
if differences or not report['input_unchanged']:raise SystemExit(1)
