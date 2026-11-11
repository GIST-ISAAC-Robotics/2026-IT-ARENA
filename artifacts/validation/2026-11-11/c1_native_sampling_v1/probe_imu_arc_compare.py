"""운영 파일을 바꾸지 않는 저장 입력 비교. 변경 뒤 궤적의 주행 성공 증명이 아니다."""
import hashlib
import inspect
import json
from pathlib import Path
import textwrap
import types

from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,comparable_decision,equal_output

ROOT=Path(__file__).resolve().parents[4]
TRACE=ROOT/'artifacts/validation/2026-11-11/imu_observed_sensitivity_v1/solo_run1/follow_decisions.jsonl'
OUTPUT=ROOT/'build/c1_native_sampling_2026_11_11/imu_arc_compare.json'
old=inspect.getsource(LocalFollow._baseline_command)
needle='eligible = eligible and path_agrees(candidate, carried, cfg.boundary_agreement_m)'
assert old.count(needle)==1
new=old.replace(needle,"""matches=eligible and path_agrees(candidate, carried, cfg.boundary_agreement_m)
                    if (not matches and eligible and cfg.piecewise_wall_trial
                            and info.get('wall_model')=='piecewise_corner'):
                        matches=path_agrees_arc(candidate,carried,cfg.boundary_agreement_m)
                        if matches: detail['boundary_history_agreement']='current_corner_arc'
                    eligible = eligible and matches""")
namespace=dict(LocalFollow._baseline_command.__globals__)
exec(compile(textwrap.dedent(new),'<saved-input-corner-candidate>','exec'),namespace)
candidate_method=namespace['_baseline_command']
rows=[];baseline_mismatches=0;first_divergence=None
with TRACE.open() as stream:
    for line in stream:
        event=json.loads(line);data=event['data']
        if event['kind']=='header':
            motion=MotionConfig(**data['motion_config']);cfg=FollowConfig(**data['follow_config'])
            baseline=LocalFollow(cfg);candidate=LocalFollow(cfg)
            candidate._baseline_command=types.MethodType(candidate_method,candidate)
        elif event['kind']=='reset':baseline.reset();candidate.reset()
        elif event['kind']=='call':
            outputs=[replay_call(control,data['input'],motion) for control in (baseline,candidate)]
            baseline_mismatches+=not equal_output(comparable_decision(freeze(outputs[0])),comparable_decision(data['output']))
            changed=not equal_output(comparable_decision(freeze(outputs[0])),comparable_decision(freeze(outputs[1])))
            if changed and first_divergence is None:first_divergence=event['seq']
            rows.append(dict(seq=event['seq'],time_s=data['input']['reference_s'],changed=changed,
                baseline_speed=outputs[0][0],candidate_speed=outputs[1][0],
                baseline_reason=outputs[0][2]['reason'],candidate_reason=outputs[1][2]['reason'],
                candidate_arc=outputs[1][2].get('boundary_history_agreement')))
summary=dict(scope='saved_input_counterfactual_not_closed_loop',runtime_modified=False,
    trace_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),
    method_before_sha256=hashlib.sha256(old.encode()).hexdigest(),
    method_candidate_sha256=hashlib.sha256(new.encode()).hexdigest(),
    calls=len(rows),baseline_mismatches=baseline_mismatches,first_divergence=first_divergence,
    candidate_arc_uses=sum(r['candidate_arc'] is not None for r in rows),
    original_zero_to_positive=sum(r['baseline_speed']==0 and r['candidate_speed']>0 for r in rows),
    positive_to_zero=sum(r['baseline_speed']>0 and r['candidate_speed']==0 for r in rows))
OUTPUT.parent.mkdir(parents=True,exist_ok=True)
if OUTPUT.exists():raise FileExistsError(OUTPUT)
OUTPUT.write_text(json.dumps(dict(summary=summary,rows=rows),indent=2))
print(json.dumps(summary))
