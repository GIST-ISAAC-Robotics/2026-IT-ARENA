"""교집합 후보 뒤에도 명령이 같았던 이유와 점별 취득 시각 한계만 회수한다."""
import hashlib
import json
from pathlib import Path
import sys
from unittest.mock import patch

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
for directory in ('scripts','src/arena_autonomy','src/arena_vehicle_interface'):
    sys.path.insert(0,str(ROOT/directory))
from arena_autonomy.follow_trace import replay_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.partial_observation import PartialObservationPolicy
from probe_joint_missing_replay import evaluate_joint

OUT=BASE/'joint_remaining_v1.json'
if OUT.exists():raise FileExistsError(OUT)
trace=BASE.parent/'short_prefix_connection_v1/run1_course/follow_decisions.jsonl'
samples=[]
with trace.open() as stream:
    header=json.loads(next(stream))['data']
    follow=LocalFollow(FollowConfig(**header['follow_config']));motion=MotionConfig(**header['motion_config'])
    with patch.object(PartialObservationPolicy,'evaluate',evaluate_joint):
        for line in stream:
            row=json.loads(line)
            if row['kind']=='reset':follow.reset();continue
            if row['kind']!='call':continue
            data=row['data'];t=data['input']['reference_s']
            if t>131.:break
            output=replay_call(follow,data['input'],motion)
            if any(abs(t-goal)<1e-5 for goal in (129.32,129.48,129.6,129.8,130.,130.2,131.)):
                detail=output[2]
                samples.append(dict(t_s=t,command=output[:2],detail={k:v for k,v in detail.items()
                    if k not in ('path_points','boundary','selector_evidence')},
                    sensor_input_reference='unchanged original trace'))
diagnosis=json.loads((BASE/'diagnosis_v1.json').read_text(encoding='utf-8'))
joint=json.loads((BASE/'joint_first_v1.json').read_text(encoding='utf-8'))
rows=[]
now=diagnosis['first_controller']['latch_time_s']
for witness,distance in zip(diagnosis['witnesses'],joint['results']['controller']['minimum_joint_distances_m']):
    raw_age=now-witness['previous_acquired_s'];radius=.03+2*raw_age
    rows.append(dict(source_index=witness['previous_source_index'],raw_age_s=raw_age,
        raw_age_radius_m=radius,joint_distance_m=distance,remainder_m=distance-radius))
result=dict(scope='open-loop diagnostic, no new driving; raw age sensitivity does not cover target motion after a missing ray',
    samples=samples,raw_age_sensitivity=rows,
    source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (Path(__file__),trace,ROOT/'scripts/probe_joint_missing_replay.py',ROOT/'scripts/probe_joint_missing_sweep.py')})
OUT.write_text(json.dumps(freeze(result),ensure_ascii=False,indent=2),encoding='utf-8')
for r in samples:
    d=r['detail'];g=d.get('selector_guard_candidate') or {}
    print(json.dumps(freeze(dict(t_s=r['t_s'],command=r['command'],reason=d.get('reason'),
        baseline=d.get('baseline_reason'),selector=d.get('selector_reason'),guard=g))))
print(json.dumps(rows))
