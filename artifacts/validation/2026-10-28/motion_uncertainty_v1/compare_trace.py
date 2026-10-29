"""잔여 센서 오차의 전체 저장 입력 비교. 변경 명령은 기록된 차량을 움직이지 않는다."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time
import zipfile

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),
             str(BASE.parent/'onroad_candidate_v1'),str(BASE.parent/'traffic_residual_v1')]
from compare_saved import preflight,sha
from safety_shadow import ShadowSafety
from arena_autonomy.follow_trace import freeze,thaw,replay_call
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from perturb import CASES,transform


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--trace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--cases',nargs='+',choices=[c.name for c in CASES],default=[c.name for c in CASES])
    args=parser.parse_args()
    if args.cases[0]!='baseline' or len(set(args.cases))!=len(args.cases):
        raise ValueError('unique cases beginning with baseline required')
    cases=[next(c for c in CASES if c.name==name) for name in args.cases]
    trace=args.trace.resolve(); output=args.output.resolve()
    header,expected=preflight(trace)
    if header['schema']!=1: raise ValueError('G1 input required')
    config=FollowConfig(**header['follow_config']); motion=MotionConfig(**header['motion_config'])
    assert config.traffic_path_recovery_trial and config.short_boundary_prefix_enabled
    output.mkdir(exist_ok=False)
    before=sha(trace)
    files=[p for folder in ('src/arena_autonomy','src/arena_vehicle_interface')
           for p in (ROOT/folder).rglob('*.py') if '__pycache__' not in p.parts]
    files += list(BASE.glob('*.py'))+[BASE.parent/'traffic_residual_v1/safety_shadow.py',
                                   BASE.parent/'onroad_candidate_v1/compare_saved.py']
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in files}
    with zipfile.ZipFile(output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in files: archive.write(p,p.relative_to(ROOT).as_posix())
    (output/'parameters.json').write_text(json.dumps(dict(header=header,cases=[asdict(c) for c in cases],
        trace=str(trace),trace_sha256=before,source_sha256=hashes),ensure_ascii=False,indent=2),encoding='utf-8')
    controllers={c.name:LocalFollow(config) for c in cases}
    guards={}; counts={c.name:Counter() for c in cases}; reasons={c.name:Counter() for c in cases}
    active_stop={c.name:None for c in cases}; longest={c.name:0. for c in cases}
    maximum_steering={c.name:0. for c in cases}; examples={c.name:[] for c in cases}
    calls=0; began=time.monotonic()
    with trace.open(encoding='utf-8') as stream,(output/'decisions.jsonl').open('x',encoding='utf-8') as sink:
        for line in stream:
            event=json.loads(line)
            if event['kind']=='reset':
                for controller in controllers.values(): controller.reset()
                guards.clear()
                active_stop={c.name:None for c in cases}
                continue
            if event['kind']!='call': continue
            original=thaw(event['data']['input']); t=original['reference_s']; calls+=1
            turning=bool(original['history']['gyro']['values'] and abs(original['history']['gyro']['values'][-1])>.2)
            outcomes={}
            for case in cases:
                data=transform(original,case)
                result=replay_call(controllers[case.name],data,motion)
                if case.name not in guards:
                    guards[case.name]=ShadowSafety(RecoveryPolicy(True,True,True),config,data['rear_half_angle_deg'])
                    assert guards[case.name].node.partial_policy is not controllers[case.name].partial_policy
                safe,status=guards[case.name].step(data,motion,result[:2])
                assert 0<=safe[0]<=result[0]+1e-12
                if result[0]==0: assert safe[0]==0
                row=dict(command=list(result[:2]),safe=list(safe),reason=result[2].get('reason'),
                         selector=result[2].get('selector_reason'),safety_reason=status.get('reason'),
                         input_error=result[2].get('input_error'))
                outcomes[case.name]=row; stat=counts[case.name]
                if case.name=='baseline':
                    recorded=thaw(event['data']['output'])
                    stat['recorded_command_differences']+=int(any(abs(a-b)>1e-10 for a,b in zip(result[:2],recorded[:2])))
                stat.update(calls=1,turn_calls=int(turning),positive=int(result[0]>0),safe_positive=int(safe[0]>0),
                            turn_safe_positive=int(turning and safe[0]>0))
                reasons[case.name][f"{row['reason']} / {row['safety_reason']}"]+=1
                base=outcomes['baseline']
                changed=any(abs(a-b)>1e-10 for a,b in zip(row['safe'],base['safe']))
                stop=base['safe'][0]>0 and safe[0]==0
                stat.update(changed_safe=int(changed),extra_stop=int(stop),
                            restored_positive=int(base['safe'][0]==0 and safe[0]>0),
                            turn_extra_stop=int(turning and stop))
                maximum_steering[case.name]=max(maximum_steering[case.name],abs(safe[1]-base['safe'][1]))
                if stop:
                    if active_stop[case.name] is None: active_stop[case.name]=t
                    longest[case.name]=max(longest[case.name],t-active_stop[case.name])
                else: active_stop[case.name]=None
                if (stop or changed) and len(examples[case.name])<8:
                    examples[case.name].append(dict(seq=event['seq'],t=t,baseline=base,perturbed=row,turning=turning))
            sink.write(json.dumps(freeze(dict(seq=event['seq'],t=t,turning=turning,results=outcomes)),allow_nan=False)+'\n')
    assert calls==expected and sha(trace)==before
    assert all(sha(ROOT/p)==h for p,h in hashes.items())
    report=dict(scope='post-acceptance residual error open-loop comparison with independent guard function; no ROS transport, physics, or collision proof',
        input_unchanged=True,source_unchanged=True,source_sha256=hashes,input_sha256=before,calls=calls,
        elapsed_wall_s=time.monotonic()-began,
        results={c.name:dict(parameters=asdict(c),counts=dict(counts[c.name]),reasons=dict(reasons[c.name]),
                            longest_extra_stop_sample_span_s=longest[c.name],
                            max_safe_steering_difference_rad=maximum_steering[c.name],examples=examples[c.name]) for c in cases})
    (output/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({c:dict(counts[c]) for c in counts}),flush=True)
    if counts['baseline']['recorded_command_differences']:
        raise RuntimeError('baseline differs from captured command; preserve result but investigate before attributing effects to error injection')


if __name__=='__main__': main()
