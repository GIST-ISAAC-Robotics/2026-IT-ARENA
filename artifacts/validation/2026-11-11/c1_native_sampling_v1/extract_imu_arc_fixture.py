"""기존 편향 실패에서 호길이 비교가 달랐던 실제 후보/기억 경로만 추출한다."""
import inspect
import json
from pathlib import Path
import textwrap
import types

from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call

ROOT=Path(__file__).resolve().parents[4]
TRACE=ROOT/'artifacts/validation/2026-11-11/imu_observed_sensitivity_v1/solo_run1/follow_decisions.jsonl'
OUTPUT=ROOT/'artifacts/validation/2026-11-11/c1_native_sampling_v1/imu_arc_fixture.json'
captured=[]
def capture(candidate, reference, info, config, scene):
    captured.append(dict(candidate=candidate.tolist(),reference=reference.tolist(),
        info={k:v for k,v in info.items() if k in ('wall_model','boundary_observed_ahead_m','path_anchor_error_m','wall_distance_m')},
        time_s=scene.t_ref_s,config=dict(config.__dict__)))

old=inspect.getsource(LocalFollow._baseline_command)
needle='eligible = eligible and path_agrees(candidate, carried, cfg.boundary_agreement_m)'
assert old.count(needle)==1
new=old.replace(needle,"""matches=eligible and path_agrees(candidate, carried, cfg.boundary_agreement_m)
                    if (not matches and eligible and cfg.piecewise_wall_trial
                            and info.get('wall_model')=='piecewise_corner'):
                        matches=path_agrees_arc(candidate,carried,cfg.boundary_agreement_m)
                        if matches:capture(candidate,carried,info,cfg,scene)
                    eligible = eligible and matches""")
namespace=dict(LocalFollow._baseline_command.__globals__,capture=capture)
exec(compile(textwrap.dedent(new),'<fixture-extraction>','exec'),namespace)
for line in TRACE.open(encoding='utf-8'):
    event=json.loads(line);data=event['data']
    if event['kind']=='header':
        motion=MotionConfig(**data['motion_config']);follow=LocalFollow(FollowConfig(**data['follow_config']))
        follow._baseline_command=types.MethodType(namespace['_baseline_command'],follow)
    elif event['kind']=='reset':follow.reset()
    elif event['kind']=='call':replay_call(follow,data['input'],motion)
if OUTPUT.exists():raise FileExistsError(OUTPUT)
OUTPUT.write_text(json.dumps(dict(scope='saved_simulation_corner_geometry_not_hardware',captures=captured),indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(captures=len(captured),times=[r['time_s'] for r in captured])))
