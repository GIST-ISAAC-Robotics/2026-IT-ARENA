"""고정 입력 전체의 상태/독립 보호 대조. 후보는 이 프로세스에서만 주입한다."""
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
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),
    str(BASE.parent/'onroad_candidate_v1'),str(BASE.parent/'traffic_residual_v1')]
from compare_saved import preflight,sha
from safety_shadow import ShadowSafety
from arena_autonomy.follow_trace import replay_call,thaw,freeze,comparable_decision,equal_output
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
import arena_autonomy.surface_anchor_trial as anchor
import arena_autonomy.surface_reobservation as surface
import arena_autonomy.corridor_observer as observer
from finite_segment_candidate import surface_translation as candidate_fit


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('trace',type=Path);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--old-nominal-source',action='store_true');args=parser.parse_args()
    header,expected=preflight(args.trace)
    config=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
    assert config.short_boundary_prefix_enabled and config.traffic_recovery_trial
    args.output.mkdir(exist_ok=False)
    paths=[p for directory in ('src/arena_autonomy','src/arena_vehicle_interface')
        for p in (ROOT/directory).rglob('*.py') if '__pycache__' not in p.parts]
    paths += [Path(__file__),BASE/'finite_segment_candidate.py',
        BASE.parent/'traffic_residual_v1/safety_shadow.py',BASE.parent/'onroad_candidate_v1/compare_saved.py']
    sources={p.relative_to(ROOT).as_posix():sha(p) for p in paths};before=sha(args.trace)
    with zipfile.ZipFile(args.output/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for path in paths:archive.write(path,path.relative_to(ROOT).as_posix())
    (args.output/'parameters.json').write_text(json.dumps(dict(trace=str(args.trace),header=header,
        input_sha256=before,source_sha256=sources,old_nominal_source=args.old_nominal_source,
        candidate='small_finite_support_consistent; process-only injection'),ensure_ascii=False,indent=2),encoding='utf-8')
    controllers={name:LocalFollow(config) for name in ('recorded','baseline','candidate')}
    guards={};counts=Counter();cost={name:[] for name in ('baseline','candidate')}
    original_fit=anchor.surface_translation;original_surface=surface.supported_surface
    original_inner=observer.short_inner_prefix_evidence
    try:
        with args.trace.open(encoding='utf-8') as stream,(args.output/'decisions.jsonl').open('x',encoding='utf-8') as sink:
            for line in stream:
                event=json.loads(line)
                if event['kind']=='reset':
                    for controller in controllers.values():controller.reset()
                    guards.clear()
                if event['kind']!='call':continue
                data=thaw(event['data']['input']);outputs={};counts['calls']+=1
                for name,controller in controllers.items():
                    anchor.surface_translation=candidate_fit if name=='candidate' else original_fit
                    legacy=name=='recorded' and args.old_nominal_source
                    surface.supported_surface=surface.short_observed_surface if legacy else original_surface
                    observer.short_inner_prefix_evidence=(lambda geometry,_:geometry) if legacy else original_inner
                    started=time.perf_counter();result=replay_call(controller,data,motion)
                    if name=='recorded':
                        assert equal_output(comparable_decision(freeze(result)),comparable_decision(thaw(event['data']['output']))),(event['seq'],data['reference_s'])
                        counts['recorded_matches']+=1
                        continue
                    if name not in guards:
                        guards[name]=ShadowSafety(RecoveryPolicy(True,True,True),config,data['rear_half_angle_deg'])
                        assert guards[name].node.partial_policy is not controller.partial_policy
                    safe,status=guards[name].step(data,motion,result[:2])
                    cost[name].append((time.perf_counter()-started)*1000)
                    assert 0<=safe[0]<=result[0]+1e-12
                    outputs[name]=dict(command=list(result[:2]),safe=list(safe),
                        reason=result[2].get('reason'),safety_reason=status.get('reason'))
                old,new=outputs['baseline']['safe'],outputs['candidate']['safe']
                counts['changed_commands']+=int(any(abs(a-b)>1e-10 for a,b in zip(old,new)))
                counts['restored_positive']+=int(old[0]==0 and new[0]>0)
                counts['suppressed_positive']+=int(old[0]>0 and new[0]==0)
                counts['candidate_positive']+=int(new[0]>0)
                sink.write(json.dumps(dict(seq=event['seq'],t=data['reference_s'],outputs=outputs),allow_nan=False)+'\n')
    finally:
        anchor.surface_translation=original_fit;surface.supported_surface=original_surface
        observer.short_inner_prefix_evidence=original_inner
    assert counts['calls']==expected and sha(args.trace)==before
    assert all(sha(ROOT/path)==value for path,value in sources.items())
    summary=dict(scope='fixed-input stateful replay and independent safety function; not new driving or ROS timing',
        counts=dict(counts),input_unchanged=True,source_unchanged=True,input_sha256=before,
        controller_and_safety_ms={key:dict(zip(('median','p95','p99','max'),map(float,np.percentile(values,[50,95,99,100]))))
            for key,values in cost.items()})
    (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
