"""실제 연결 코드와 사전 비교안의 모든 명령/독립 보호 결과를 대조한다."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time
import zipfile

import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),
    str(BASE.parent/'traffic_residual_v1'),str(BASE.parent/'onroad_candidate_v1')]
from compare_saved import preflight,sha
from safety_shadow import ShadowSafety
from arena_autonomy.follow_trace import replay_call,thaw
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
import arena_autonomy.surface_reobservation as surface


def distribution(values):
    return None if not values else dict(zip(('median','p95','p99','max'),map(float,np.percentile(values,[50,95,99,100]))))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('trace',type=Path);parser.add_argument('expected',type=Path)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    header,expected_calls=preflight(args.trace);args.output.mkdir(exist_ok=False)
    inputs={str(p):sha(p) for p in (args.trace,args.expected)}
    paths=[p for directory in ('src/arena_autonomy','src/arena_vehicle_interface')
        for p in (ROOT/directory).rglob('*.py') if '__pycache__' not in p.parts]
    paths += [Path(__file__),BASE.parent/'traffic_residual_v1/safety_shadow.py',BASE.parent/'onroad_candidate_v1/compare_saved.py']
    sources={p.relative_to(ROOT).as_posix():sha(p) for p in paths}
    with zipfile.ZipFile(args.output/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in paths:archive.write(p,p.relative_to(ROOT).as_posix())
    (args.output/'parameters.json').write_text(json.dumps(dict(input_sha256=inputs,source_sha256=sources),indent=2),encoding='utf-8')
    config=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
    controller=LocalFollow(config);guard=None;calls=0;all_cost=[];fit_cost=[];fit_reasons=Counter();buckets={}
    original=surface.small_surface_translation
    def timed(*args):
        started=time.perf_counter();result=original(*args);elapsed=(time.perf_counter()-started)*1000
        fit_cost.append(elapsed);fit_reasons[result.get('reason')]+=1
        return result
    surface.small_surface_translation=timed
    try:
        with args.trace.open(encoding='utf-8') as stream,args.expected.open(encoding='utf-8') as reference:
            for line in stream:
                event=json.loads(line)
                if event['kind']=='reset':controller.reset();guard=None
                if event['kind']!='call':continue
                data=thaw(event['data']['input']);row=json.loads(next(reference))
                assert row['seq']==event['seq'] and row['t']==data['reference_s']
                if guard is None:guard=ShadowSafety(RecoveryPolicy(True,True,True),config,data['rear_half_angle_deg'])
                start=time.perf_counter();before_fit=len(fit_cost);result=replay_call(controller,data,motion)
                safe,status=guard.step(data,motion,result[:2]);elapsed=(time.perf_counter()-start)*1000
                expected=row['outputs']['candidate'];actual=dict(command=list(result[:2]),safe=list(safe),
                    reason=result[2].get('reason'),safety_reason=status.get('reason'))
                for key in ('command','safe'):np.testing.assert_allclose(actual[key],expected[key],rtol=0,atol=1e-10)
                assert actual['reason']==expected['reason'] and actual['safety_reason']==expected['safety_reason']
                all_cost.append(elapsed);calls+=1
                key='alignment_called' if len(fit_cost)>before_fit else 'no_alignment'
                buckets.setdefault(key,[]).append(elapsed)
            assert next(reference,None) is None
    finally:surface.small_surface_translation=original
    assert calls==expected_calls
    assert all(sha(Path(p))==value for p,value in inputs.items())
    assert all(sha(ROOT/p)==value for p,value in sources.items())
    summary=dict(scope='connected candidate equals previously compared commands and safety; fixed inputs, no new driving',
        calls=calls,matches=calls,input_unchanged=True,source_unchanged=True,
        total_controller_and_safety_ms=distribution(all_cost),alignment_calls=len(fit_cost),
        alignment_ms=distribution(fit_cost),alignment_reasons=dict(fit_reasons),
        cost_buckets={name:dict(calls=len(values),ms=distribution(values)) for name,values in buckets.items()})
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
