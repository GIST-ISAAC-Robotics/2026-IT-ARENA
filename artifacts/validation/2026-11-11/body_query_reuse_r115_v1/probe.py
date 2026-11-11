"""장면·자세·조향·설정이 같은 차체 질의만 재사용하는 미연결 비용 후보."""
from collections import OrderedDict
import copy
import hashlib
import json
from pathlib import Path
import statistics
import time

import numpy as np
from arena_autonomy import m3_execution_trial as mx
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import comparable_decision, equal_output, freeze, replay_call

A=Path(__file__).resolve().parent
SOURCE=A.parent/'forward_gap_scope_r109_v1/two_actor_run1/run1/follow_decisions.jsonl'


class QueryReuse:
    """한 장면 최대8항목. 입력이 달라지면 재계산하고 반환 배열은 복사한다."""
    def __init__(self):
        self.scene=None;self.cache=OrderedDict();self.hits=self.misses=0
        self.key_ms=0.;self.copy_ms=0.;self.work_ms=0.;self.peak_bytes=0
    def __enter__(self):
        self.original=mx.body_queries;mx.body_queries=self.query;return self
    def __exit__(self,*args):mx.body_queries=self.original
    def query(self,scene,poses,steering,cfg):
        if self.scene is not scene:
            self.scene=scene;self.cache.clear()
        # 정확히 동일한 표현만 재사용한다. 반올림/자세 합치기/결측 제거 없음.
        started=time.perf_counter()
        p=np.asarray(poses);s=np.asarray(steering)
        if p.dtype!=np.float64 or s.dtype!=np.float64 or not np.all(np.isfinite(p)) or not np.all(np.isfinite(s)):
            return self.original(scene,poses,steering,cfg)
        key=(p.shape,p.dtype.str,p.tobytes(),s.shape,s.dtype.str,s.tobytes(),cfg)
        self.key_ms+=(time.perf_counter()-started)*1000.
        if key in self.cache:
            self.hits+=1;result=self.cache.pop(key);self.cache[key]=result
        else:
            self.misses+=1;started=time.perf_counter()
            result=self.original(scene,poses,steering,cfg)
            self.work_ms+=(time.perf_counter()-started)*1000.
            stored=(result[0].copy(),result[1]);self.cache[key]=stored
            if len(self.cache)>8:self.cache.popitem(last=False)
            self.peak_bytes=max(self.peak_bytes,sum(v[0].nbytes+len(k[2])+len(k[5]) for k,v in self.cache.items()))
            return result
        started=time.perf_counter();out=(result[0].copy(),result[1])
        self.copy_ms+=(time.perf_counter()-started)*1000.
        return out
    def summary(self):
        return {k:getattr(self,k) for k in ('hits','misses','key_ms','copy_ms','work_ms','peak_bytes')}


def main():
    rows=[json.loads(s) for s in SOURCE.open(encoding='utf8')];digest=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    baseline=[];timings={};stats={}
    from contextlib import nullcontext
    for candidate in (False,True):
        follow,motion=make_controller(rows[0]['data']);cost=[];differences=[];record_mismatch=[]
        hook=QueryReuse()
        with hook if candidate else nullcontext():
            i=0
            for row in rows[1:]:
                if row['kind']=='reset':follow.reset()
                elif row['kind']=='call':
                    start=time.perf_counter();out=freeze(replay_call(follow,row['data']['input'],motion))
                    cost.append((time.perf_counter()-start)*1000.)
                    if not candidate:
                        baseline.append(out)
                        if not equal_output(comparable_decision(out),comparable_decision(row['data']['output'])):record_mismatch.append(row['seq'])
                    elif not equal_output(comparable_decision(out),comparable_decision(baseline[i])):differences.append(row['seq'])
                    i+=1
        ordered=sorted(cost)
        timings[str(candidate)]=dict(p50=statistics.median(cost),p95=ordered[int(.95*len(ordered))],maximum=max(cost),sum_ms=sum(cost))
        stats[str(candidate)]=dict(calls=i,record_mismatch=record_mismatch,candidate_differences=differences,cache=hook.summary())
    report=dict(source_sha256=digest,source_unchanged=hashlib.sha256(SOURCE.read_bytes()).hexdigest()==digest,
        timing_ms=timings,results=stats,scope='same saved input; PC sequential cost only, no runtime mutation outside this process or Jetson speed proof')
    with (A/'comparison.json').open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps(report))


if __name__=='__main__':main()
