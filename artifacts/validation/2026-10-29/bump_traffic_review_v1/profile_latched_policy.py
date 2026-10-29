"""ROS 고장과 같은 반환 순서의 함수 비용 분해. 실제 ROS 수신/실시간 성능의 재현은 아니다."""
import argparse
import cProfile
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import pstats
import sys
import time

import numpy as np

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'tests')]
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from test_local_follow import history,scan


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(exist_ok=False)
    policy=RecoveryPolicy(True,True,True)
    h=history(end=5.)
    rows=[]
    profiler=cProfile.Profile()
    for frame in range(120):
        now=.5+frame*.02
        first=round(.4+(frame//5)*.1,6)
        if frame<20:kind,value='finite',2.
        elif frame<45:kind,value='near',.08
        elif frame<95:kind,value='disappeared',math.inf
        else:kind,value='recovered',2.
        values=[2.]*500;values[375]=value
        raw=scan(first,values)
        points,_,_=h.project(values,raw.angle_min,raw.angle_increment,raw.range_min,raw.range_max,
            first,raw.time_increment,now,mode='both',timing_verified=True)
        before=time.perf_counter()
        # cProfile 비용은 포함된다. 목표 주기 통과/실패 판정에 이 수치를 쓰지 않는다.
        if kind=='disappeared':profiler.enable()
        result=policy.evaluate(raw,points,h,now,1.,0.,0.,.135,.2,.15)
        profiler.disable()
        rows.append(dict(t=now,source_s=first,fresh=frame%5==0,case=kind,
            duration_ms=(time.perf_counter()-before)*1000,allowed=result['allowed'],
            reason=result['reason'],anchor_reason=(result.get('anchor_diagnostic') or {}).get('reason'),
            clearance_reason=(result.get('clearance_diagnostic') or {}).get('reason')))
    profiler.dump_stats(str(args.output/'disappeared.cprofile'))
    stats=pstats.Stats(profiler)
    entries=[]
    for (file,line,function),(primitive,calls,self_s,total_s,_) in stats.stats.items():
        if 'arena_autonomy' in file:
            entries.append(dict(file=file,line=line,function=function,primitive=primitive,calls=calls,
                self_ms=self_s*1000,cumulative_ms=total_s*1000))
    entries.sort(key=lambda x:x['cumulative_ms'],reverse=True)
    phases={}
    for name in ('finite','near','disappeared','recovered'):
        selected=[r for r in rows if r['case']==name]
        phases[name]=dict(reasons=dict(Counter(r['reason'] for r in selected)),
            calls=len(selected),allowed=sum(r['allowed'] for r in selected),
            cost_ms={str(fresh):float(np.median([r['duration_ms'] for r in selected if r['fresh']==fresh]))
                     for fresh in (True,False)})
    result=dict(scope='synthetic policy sequence with profiling overhead; not actual ROS scheduling or timing proof',
        phases=phases,profile_functions=entries,rows=rows,
        source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (Path(__file__),ROOT/'src/arena_autonomy/arena_autonomy/traffic_recovery_trial.py')})
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(phases=phases,top_functions=entries[:8])))


if __name__=='__main__':main()
