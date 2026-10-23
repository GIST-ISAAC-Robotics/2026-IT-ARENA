#!/usr/bin/env python3
"""저장 6호출의 외곽 질의 후보 AB/BA. 운영 함수를 바꾸지 않는 평가 전용 실행."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import time
import zipfile

import numpy as np
from arena_autonomy import observation_support as current
from arena_autonomy.follow_trace import equal_output,freeze,restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_runtime import digest

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    candidate_path=ROOT/'artifacts/validation/2026-10-21/g1_query_batch_candidate_v1/candidate.py'
    spec=importlib.util.spec_from_file_location('query_batch_benchmark',candidate_path)
    candidate=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(candidate)
    capture=ROOT/'artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl'
    before=digest(capture)
    sources={str(candidate_path.relative_to(ROOT)):digest(candidate_path)}
    for module in tuple(sys.modules.values()):
        name=getattr(module,'__file__',None)
        if name:
            p=Path(name).resolve()
            if p.is_relative_to(ROOT) and p.suffix=='.py':
                sources[p.relative_to(ROOT).as_posix()]=digest(p)
    sources['tests/test_observation_query_batch.py']=digest(ROOT/'tests/test_observation_query_batch.py')
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(ROOT/name,name)
    reference=current.sweep_queries
    cases=[];matched=0
    with capture.open() as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']=='header':
                motion=MotionConfig(**event['data']['motion_config'])
                follow=LocalFollow(FollowConfig(**event['data']['follow_config']))
            elif event['kind']=='reset':
                follow.reset()
            elif event['kind']=='call':
                if event['seq']>3156:
                    break
                data,scan,history=restore_call(event['data']['input'],motion)
                scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                                   epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
                def capture_query(*values):
                    cases.append((event['seq'],values))
                    return reference(*values)
                if event['seq']>=3151:
                    current.sweep_queries=capture_query
                try:
                    output=follow.command(scene,history,data['speed_mps'],data['side'],
                                          data['previous_steering_rad'],data['dt_s'])
                finally:
                    current.sweep_queries=reference
                if not equal_output(freeze(output),event['data']['output']):
                    raise ValueError('original replay mismatch')
                matched+=1
    if not cases:
        raise ValueError('no target calls')
    samples=[];geometry=[]
    for index,(seq,values) in enumerate(cases):
        expected,thin=reference(*values)
        actual,other=candidate.sweep_queries(*values)
        np.testing.assert_array_equal(actual,expected)
        if thin!=other:
            raise ValueError('thin count mismatch')
        geometry.append(dict(index=index,seq=seq,queries=len(expected),thin=thin))
        functions=[('reference',reference),('batch',candidate.sweep_queries)]
        for repeat in range(20):
            for name,function in functions if repeat%2==0 else functions[::-1]:
                begin=time.perf_counter_ns()
                result,n=function(*values)
                elapsed=(time.perf_counter_ns()-begin)/1e6
                np.testing.assert_array_equal(result,expected)
                if n!=thin:
                    raise ValueError('repeated thin count mismatch')
                samples.append(dict(index=index,seq=seq,repeat=repeat,version=name,elapsed_ms=elapsed))
    summary={}
    for name in ('reference','batch'):
        values=[r['elapsed_ms'] for r in samples if r['version']==name]
        summary[name]=dict(count=len(values),p50=float(np.percentile(values,50)),
            p95=float(np.percentile(values,95)),maximum=max(values))
    report=dict(scope='same_saved_query_input_AB_BA_not_ROS_deadline',matched=matched,
        cases=geometry,samples=samples,summary_ms=summary,all_outputs_equal=True,
        runtime_modified=False,input_sha256=before,input_unchanged=digest(capture)==before,
        source_sha256=sources,source_unchanged=all(digest(ROOT/n)==h for n,h in sources.items()))
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('samples','source_sha256')}))
    return 0 if report['input_unchanged'] and report['source_unchanged'] else 2


if __name__=='__main__':
    raise SystemExit(main())
