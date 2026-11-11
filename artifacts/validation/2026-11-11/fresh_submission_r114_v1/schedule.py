"""10Hz 스캔·50Hz 수신·이전 Jetson 시간열의 계산 일정 대조. 차체 물리 없음."""
import bisect
import collections
import hashlib
import json
from pathlib import Path
import statistics

from arena_autonomy.decision_delivery import DecisionDelivery
from fresh_delivery import FreshStartDelivery

A=Path(__file__).resolve().parent
PROFILE=A.parent/'decision_delay_r110_v1/delivery_profile.json'


class Worker:
    def __init__(self,profile,clock):
        self.clock=clock;self.pending=self.inflight=self.fault=None
        self.generation=0;self.origin=None
        first=profile[0]['offset_s'];self.times=[v['offset_s']-first for v in profile];self.delays=[v['delay_s'] for v in profile]
    def offer(self,data):self.pending=data
    def invalidate(self):self.generation+=1;self.pending=None
    def pump(self):
        now=self.clock()
        if self.inflight is not None:
            data,release,generation=self.inflight
            if now+1e-9<release:return None
            self.inflight=None
            if generation==self.generation:return dict(output=[1.,0.,{}])
        if self.pending is not None:
            data=self.pending;self.pending=None
            if self.origin is None:self.origin=data['reference_s']
            index=max(0,bisect.bisect_right(self.times,data['reference_s']-self.origin)-1)
            self.inflight=(data,data['reference_s']+self.delays[index],self.generation)
        return None


def run(profile,gate_type):
    now=0.;worker=Worker(profile,lambda:now);gate=gate_type(worker,align_first_source=True)
    last_ref=None;applied=[];discarded=[];stale=0.;submitted=[]
    for tick in range(6,3007):
        now=tick*.02
        first=round((tick-1)//5*.1-.098,9)
        data=dict(reference_s=now,epoch=0,previous_steering_rad=0.,side='left',dt_s=.1,
            scan=dict(representation='native_measurements_v1',measurements=dict(source_epoch=0,
                acquired_at_s=[first,first+.0996])))
        item=gate.poll(now,previous_steering_rad=0.,side='left',motion_epoch=0,sensor_epoch=0)
        if item:
            if item['kind']=='candidate':
                last_ref=item['reference_s'];applied.append((now,last_ref,item['source_age_s']))
            elif item['kind']=='discarded':discarded.append((now,item['reason']))
        if gate.offer(data):submitted.append((now,now-first))
        if last_ref is not None and now-last_ref>=.2:stale+=.02
    interval=[b[0]-a[0] for a,b in zip(applied,applied[1:])]
    def stats(x):
        x=sorted(x)
        return dict(p50=statistics.median(x),p95=x[int(.95*len(x))],maximum=max(x)) if x else {}
    return dict(submitted=len(submitted),applied=len(applied),discarded=len(discarded),
        discard_reasons=dict(collections.Counter(v[1] for v in discarded)),
        source_age_at_submit_s=stats([v[1] for v in submitted]),source_age_at_apply_s=stats([v[2] for v in applied]),
        positive_interval_s=stats(interval),command_expired_duration_s=stale,
        all_applied_source_age_under_300ms=all(a[2]<.3 for a in applied),
        all_applied_command_age_under_200ms=all(a[0]-a[1]<.2 for a in applied),
        positive_interval_at_least_100ms=all(t>=.1-1e-9 for t in interval))


def main():
    profile=json.loads(PROFILE.read_bytes())['samples']
    result=dict(scope='synthetic scheduling only; constant positive output, no stateful controller or driving proof',
        profile_sha256=hashlib.sha256(PROFILE.read_bytes()).hexdigest(),
        baseline=run(profile,DecisionDelivery),fresh=run(profile,FreshStartDelivery))
    with (A/'schedule.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result))


if __name__=='__main__':main()
