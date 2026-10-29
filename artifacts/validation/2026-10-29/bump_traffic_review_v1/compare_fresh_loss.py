"""기록 전체 상태의 원래 판단/미연결 후보와 각각 독립 보호를 비교한다."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time
import zipfile
import numpy as np

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),
    str(ROOT/'artifacts/validation/2026-10-28/onroad_candidate_v1'),str(ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1')]
from compare_saved import preflight, sha
from safety_shadow import ShadowSafety
from fresh_loss_candidate import FreshLossCandidate
from arena_autonomy.follow_trace import replay_call, thaw, freeze, comparable_decision, equal_output
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    header,expected=preflight(args.trace)
    config=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
    assert config.traffic_recovery_trial and config.partial_observation_policy=='bounded_local_v4'
    args.output.mkdir(exist_ok=False)
    paths=[p for name in ('src/arena_autonomy','src/arena_vehicle_interface')
        for p in (ROOT/name).rglob('*.py') if '__pycache__' not in p.parts]
    paths += [Path(__file__),BASE/'fresh_loss_candidate.py',
        ROOT/'artifacts/validation/2026-10-28/onroad_candidate_v1/compare_saved.py',ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1/safety_shadow.py']
    sources={p.relative_to(ROOT).as_posix():sha(p) for p in paths}
    before=sha(args.trace)
    with zipfile.ZipFile(args.output/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in paths:archive.write(p,p.relative_to(ROOT).as_posix())
    (args.output/'parameters.json').write_text(json.dumps(dict(input_path=str(args.trace),
        input_sha256=before,source_sha256=sources),indent=2),encoding='utf-8')
    controllers={name:LocalFollow(config) for name in ('baseline','candidate')}
    controllers['candidate'].partial_policy=FreshLossCandidate(True,True,True)
    guards={};counts=Counter();releases=[];calls=0;cost=Counter();samples={name:[] for name in controllers}
    with args.trace.open(encoding='utf-8') as stream, (args.output/'decisions.jsonl').open('x',encoding='utf-8') as sink:
        for line in stream:
            event=json.loads(line)
            if event['kind']=='reset':
                for ctl in controllers.values():ctl.reset()
                guards={}
            if event['kind']!='call':continue
            inputs=thaw(event['data']['input']);outputs={}
            for name,ctl in controllers.items():
                if name not in guards:
                    policy=(RecoveryPolicy if name=='baseline' else FreshLossCandidate)(True,True,True)
                    guards[name]=ShadowSafety(policy,config,inputs['rear_half_angle_deg'])
                started=time.perf_counter()
                result=replay_call(ctl,inputs,motion)
                safe,status=guards[name].step(inputs,motion,result[:2])
                elapsed=time.perf_counter()-started
                cost[name]+=elapsed; samples[name].append(elapsed*1000)
                if name=='baseline':
                    assert equal_output(comparable_decision(freeze(result)),
                        comparable_decision(event['data']['output'])),event['seq']
                assert 0<=safe[0]<=result[0]+1e-12
                outputs[name]=dict(command=list(result[:2]),safe=list(safe),
                    reason=result[2].get('reason'),safety_reason=status.get('reason'))
                if name=='candidate':
                    for side,detail in (('planner',result[2].get('partial_observation') or {}),
                                        ('guard',status.get('partial_observation') or {})):
                        clearance=detail.get('clearance_diagnostic') or {}
                        if detail.get('reason')=='same_scan_missing_wait':
                            releases.append(dict(seq=event['seq'],t=inputs['reference_s'],side=side,
                                reason=detail.get('reason'),same_scan_wait=detail.get('same_scan_wait')))
            a,b=outputs['baseline'],outputs['candidate']
            counts['new_positive_safe']+=a['safe'][0]==0 and b['safe'][0]>0
            counts['new_zero_safe']+=a['safe'][0]>0 and b['safe'][0]==0
            counts['baseline_positive_safe']+=a['safe'][0]>0
            counts['candidate_positive_safe']+=b['safe'][0]>0
            counts['changed_commands']+=not equal_output(a['command'],b['command'])
            sink.write(json.dumps(dict(seq=event['seq'],t=inputs['reference_s'],outputs=outputs),allow_nan=False)+'\n')
            calls+=1
    assert calls==expected and sha(args.trace)==before
    assert all(sha(ROOT/p)==digest for p,digest in sources.items())
    result=dict(scope='full saved-input counterfactual; two independent controller/guard states; no new driving',
        calls=calls,baseline_record_matches=calls,input_unchanged=True,source_unchanged=True,
        counts=dict(counts),fresh_events=releases,serialized_compute_s=dict(cost),
        total_compute_ms={name:dict(p95=float(np.percentile(values,95)),maximum=max(values),
            above_20ms=sum(v>20 for v in values)) for name,values in samples.items()},
        safety_scope='same recorded input reference assumed, not original independent ROS receipt sequence')
    (args.output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='fresh_events'},ensure_ascii=True))


if __name__=='__main__':main()
