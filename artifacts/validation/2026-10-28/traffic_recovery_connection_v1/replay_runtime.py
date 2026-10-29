"""실행용 후보의 OFF 원본 동일성·ON 진단 사본 대조. 실제 주행을 대신하지 않는다."""
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import time
import zipfile
import numpy as np
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1')]
from arena_autonomy.follow_trace import freeze,replay_call,equal_output,comparable_decision
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from safety_shadow import ShadowSafety

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def stats(v):return dict(median=float(np.median(v)),p99=float(np.percentile(v,99)),maximum=max(v))
def main():
    out=BASE/'runtime_replay_v1';out.mkdir(exist_ok=False)
    trace=ROOT/'artifacts/validation/2026-10-27/short_prefix_connection_v1/run1_course/follow_decisions.jsonl'
    old=ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1/safety_replay_v1/target_calls.json'
    expected={round(r['t_s'],8):r for r in json.loads(old.read_text())}
    sources=[Path(__file__),*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    hashes={p.relative_to(ROOT).as_posix():digest(p) for p in sources};input_hash=digest(trace)
    with zipfile.ZipFile(out/'source.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in sources:z.write(p,p.relative_to(ROOT).as_posix())
    counts=Counter();mismatches=[];costs={name:[] for name in ('planner','safety','target_planner','target_safety')};samples=[];gate=None
    with trace.open() as f:
        header=json.loads(next(f))['data'];config=FollowConfig(**header['follow_config'])
        off=LocalFollow(config);on=LocalFollow(replace(config,traffic_recovery_trial=True));motion=MotionConfig(**header['motion_config'])
        for line in f:
            row=json.loads(line)
            if row['kind']=='reset':off.reset();on.reset();counts['resets']+=1;continue
            if row['kind']!='call':continue
            data=row['data'];stamp=data['input']['reference_s']
            actual=replay_call(off,data['input'],motion);counts['off_calls']+=1
            if not equal_output(comparable_decision(freeze(actual)),comparable_decision(data['output'])):
                counts['off_mismatch']+=1
                if len(mismatches)<10:mismatches.append(dict(kind='off',t_s=stamp,expected=data['output'],actual=actual))
            if stamp>132:continue
            t=time.perf_counter();result=replay_call(on,data['input'],motion);ms=(time.perf_counter()-t)*1000
            costs['planner'].append(ms);counts['on_calls']+=1
            if not np.allclose(result[:2],data['output'][:2],rtol=0,atol=1e-10):counts['changed']+=1
            if gate is None:gate=ShadowSafety(RecoveryPolicy(True,True,True),on.config,data['input']['rear_half_angle_deg'])
            assert gate.node.partial_policy is not on.partial_policy
            t=time.perf_counter();safe,status=gate.step(data['input'],motion,result[:2]);sms=(time.perf_counter()-t)*1000;costs['safety'].append(sms)
            if not 0<=safe[0]<=result[0]+1e-12:raise AssertionError('command amplified/reversed')
            if result[0]==0:assert safe[0]==0
            if stamp>=129.3:
                counts['target_calls']+=1;costs['target_planner'].append(ms);costs['target_safety'].append(sms)
                previous=expected[round(stamp,8)]
                if not (np.allclose(result[:2],previous['candidate'],rtol=0,atol=1e-10) and
                        np.allclose(safe,previous['safety']['recovery']['output'],rtol=0,atol=1e-10)):
                    counts['target_mismatch']+=1
                    if len(mismatches)<10:mismatches.append(dict(kind='target',t_s=stamp,old=previous,candidate=result,safety=safe))
                if safe[0]>0:counts['target_positive']+=1
                if (status.get('partial_observation') or {}).get('anchor_diagnostic',{}).get('released'):counts['anchor_release']+=1
                samples.append(dict(t_s=stamp,command=result[:2],safety=safe,detail=result[2],status=status))
    unchanged=all(digest(ROOT/p)==h for p,h in hashes.items()) and digest(trace)==input_hash
    passed=unchanged and not counts['off_mismatch'] and not counts['target_mismatch'] and counts['target_positive']==96 and counts['anchor_release']==1
    report=dict(passed=passed,counts=dict(counts),cost_ms={k:stats(v) for k,v in costs.items()},source_unchanged=unchanged,source_sha256=hashes,input_sha256=input_hash,scope='recorded inputs only; no ROS loop/physics; static background assumption remains')
    for name,value in [('report',report),('mismatches',mismatches),('target_calls',samples)]:
        (out/(name+'.json')).write_text(json.dumps(freeze(value),ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('passed','counts','cost_ms','source_unchanged')}))
    return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())
