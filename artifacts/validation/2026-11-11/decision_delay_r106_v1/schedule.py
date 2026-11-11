"""Jetson 저장 비용과 실제 배송 계약의 가상 시계 대조. 차량/ROS 주행 아님."""
import hashlib,json,math
from pathlib import Path
import sys
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path.insert(0,str(ROOT/'src/arena_autonomy'))
from arena_autonomy.decision_delivery import DecisionDelivery


class TimedWorker:
    def __init__(self):
        self.inflight=self.pending=self.fault=None
        self.now=0.;self.delay=0.;self.completed=math.inf;self.sequence=0;self.generation=0
    def offer(self,data):
        assert self.inflight is None
        self.sequence+=1;self.inflight=data
        self.completed=self.now+self.delay
    def pump(self):
        if self.inflight is not None and self.now+1e-9>=self.completed:
            self.inflight=None
            return dict(output=[1.,0.,{'reason':'schedule_only'}])
        return None
    def invalidate(self):
        self.generation+=1


def main():
    cost_path=A.parent/'query_set_r103_v1/jetson_stages.json'
    trace_path=A.parent/'control_10hz_r91_v1/two_actor_run4/run1/follow_decisions.jsonl'
    report_path=trace_path.parent/'report.json'
    sources=[cost_path,trace_path,report_path]
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    profile=json.loads(cost_path.read_bytes());window=json.loads(report_path.read_bytes())['case_window']
    times={r['seq']:r['data']['input']['reference_s'] for r in map(json.loads,trace_path.read_text('utf8').splitlines()) if r['kind']=='call'}
    costs=[dict(offset_s=times[r['seq']]-window['green_sim_s'],delay_s=sum(s['wall_ms'] for s in r['stages'].values())/1000.,seq=r['seq'])
           for r in profile['per_call_stages'] if window['green_sim_s']<=times[r['seq']]<=window['end_sim_s']]
    assert costs and all(b['offset_s']>a['offset_s'] for a,b in zip(costs,costs[1:]))
    scenarios=[]
    for name,constant in [('20ms_reference',.02),('measured_jetson_timeline',None)]:
        w=TimedWorker();gate=DecisionDelivery(w);applied=[];discarded=[];sources_seen=set();index=0;expired_ticks=0;last_reference=None
        for tick in range(3000):
            elapsed=tick*.02;w.now=elapsed+1.
            while index+1<len(costs) and costs[index+1]['offset_s']<=elapsed:index+=1
            item=gate.poll(w.now,previous_steering_rad=0.,side='left',motion_epoch=0,sensor_epoch=0)
            if item:
                row=dict(apply_s=elapsed,reference_s=item['reference_s']-1.,source_age_s=item['source_age_s'])
                if item['kind']=='candidate':applied.append(row);last_reference=item['reference_s']
                else:discarded.append({**row,'reason':item['reason']})
            first=1.+math.floor((elapsed+1e-9)/.1)*.1-.108
            data=dict(scan=dict(representation='native_measurements_v1',measurements=dict(source_epoch=0,acquired_at_s=[first,first+.0998])),
                      reference_s=w.now,previous_steering_rad=0.,epoch=0,dt_s=.1,side='left')
            w.delay=constant if constant is not None else costs[index]['delay_s']
            if gate.offer(data):sources_seen.add(first)
            if last_reference is not None and w.now-last_reference>=.2:expired_ticks+=1
        gaps=[b['apply_s']-a['apply_s'] for a,b in zip(applied,applied[1:])]
        scenarios.append(dict(name=name,applied=len(applied),discarded=len(discarded),submitted=len(sources_seen),
            max_apply_gap_s=max(gaps),time_with_last_request_expired_s=expired_ticks*.02,
            first_applied_s=applied[0]['apply_s'],discard_events=discarded,
            scope='positive placeholder output only; expiry exposure is not actual vehicle STOP'))
    result=dict(scope='synthetic scheduling uses real DecisionDelivery and case-window Jetson wall-cost timeline; not per-scene CPU prediction, no trajectory simulation',
        cost_samples=len(costs),cost_window_s=[costs[0]['offset_s'],costs[-1]['offset_s']],source_sha256=hashes,scenarios=scenarios)
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    with (A/'schedule.json').open('x',encoding='utf8') as f:json.dump(result,f,indent=2)
    print(json.dumps({**{k:v for k,v in result.items() if k not in ('scenarios','source_sha256')},'scenarios':[{k:v for k,v in s.items() if k!='discard_events'} for s in scenarios]}))

if __name__=='__main__':main()
