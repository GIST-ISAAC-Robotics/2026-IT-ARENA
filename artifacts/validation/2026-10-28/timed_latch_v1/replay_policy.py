"""132초까지 저장 입력만 비교한다. 운영 소스나 출력은 변경하지 않는다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch
import zipfile
import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy import partial_observation as policy
from arena_autonomy.follow_trace import replay_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from probe_timed_policy import policy_class


def main():
    out=BASE/'policy_replay_v1';out.mkdir(exist_ok=False)
    cls,source,changed=policy_class()
    trace=ROOT/'artifacts/validation/2026-10-27/short_prefix_connection_v1/run1_course/follow_decisions.jsonl'
    files=[Path(__file__),ROOT/'scripts/probe_timed_policy.py',ROOT/'scripts/probe_timed_missing_sweep.py',
        ROOT/'scripts/probe_joint_missing_sweep.py',*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    trace_hash=hashlib.sha256(trace.read_bytes()).hexdigest()
    with zipfile.ZipFile(out/'source.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in files:z.write(p,p.relative_to(ROOT).as_posix())
        z.writestr('effective_partial.py',changed)
    counts=Counter();reasons=Counter();changes=[];latches=[];cost=[]
    with patch.object(policy,'PartialObservationPolicy',cls),trace.open() as stream:
        header=json.loads(next(stream))['data']
        follow=LocalFollow(FollowConfig(**header['follow_config']));motion=MotionConfig(**header['motion_config'])
        for line in stream:
            row=json.loads(line)
            if row['kind']=='reset':follow.reset();counts['resets']+=1;continue
            if row['kind']!='call':continue
            d=row['data'];stamp=d['input']['reference_s']
            if stamp>132.:break
            start=time.perf_counter();result=replay_call(follow,d['input'],motion);cost.append((time.perf_counter()-start)*1000)
            counts['calls']+=1;detail=result[2];reasons[detail.get('selector_reason',detail['reason'])]+=1
            partial=detail.get('partial_observation')or{}
            if partial.get('latch_reason'):
                counts['latched']+=1
                if not latches or partial.get('latch_time_s')!=latches[-1]['latch_time_s']:
                    latches.append(dict(t_s=stamp,**partial))
            if not np.allclose(result[:2],d['output'][:2],rtol=0,atol=1e-10):
                counts['changed']+=1
                if len(changes)<80:changes.append(dict(t_s=stamp,original=d['output'][:2],candidate=result[:2],detail=detail))
    unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    unchanged=unchanged and hashlib.sha256(trace.read_bytes()).hexdigest()==trace_hash
    report=dict(scope='open-loop timed geometry diagnostic; no ROS/safety execution or new driving',
        counts=dict(counts),reasons=dict(reasons),first_latch=latches[0] if latches else None,
        cost_ms=dict(median=float(np.median(cost)),maximum=float(max(cost))),
        source_sha256=hashes,trace_sha256=trace_hash,source_unchanged=unchanged)
    for name,data in [('report',report),('changes',changes),('latches',latches)]:
        (out/f'{name}.json').write_text(json.dumps(freeze(data),ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('counts','cost_ms','source_unchanged')}))
    return 0 if unchanged else 1


if __name__=='__main__':raise SystemExit(main())
