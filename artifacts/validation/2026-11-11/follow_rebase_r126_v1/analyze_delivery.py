"""배송 만료·판단 reset·평가용 실제 진행을 분리한다. 제어 입력으로 쓰지 않는다."""
from collections import Counter
import hashlib
import json
from pathlib import Path

A=Path(__file__).resolve().parent
D=A/'two_actor_run1'
RUN=D/'run1'


def read_rows(path):
    with path.open(encoding='utf8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def main():
    sources=[RUN/'follow_decisions.jsonl',RUN/'trajectory.jsonl',RUN/'report.json']
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    events=read_rows(sources[0]);trajectory=read_rows(sources[1])
    window=json.loads(sources[2].read_bytes())['case_window']
    begin,end=window['green_sim_s'],window['end_sim_s']
    calls=[e for e in events if e['kind']=='call' and begin<=e['data']['input']['reference_s']<=end]
    discarded=[e for e in calls if e['data'].get('delivery',{}).get('kind')=='discarded']
    groups=Counter(e['data']['delivery'].get('reason','unknown') for e in discarded)
    samples=[t for t in trajectory if begin<=t['t']<=end]
    safety=Counter(t.get('safety_reason','unknown') for t in samples)
    low=Counter(t.get('safety_reason','unknown') for t in samples if t.get('ground_speed_mps',0)<.1)
    windows=[]
    for event in discarded[:8]:
        data=event['data'];reference=data['input']['reference_s']
        delivery=data['delivery'];released=data['compute']['released_sim_s']
        later=[e['data'] for e in calls if e['seq']>event['seq']][:4]
        nearby=[t for t in samples if released-.2<=t['t']<=released+.8]
        windows.append(dict(seq=event['seq'],reference_s=reference,released_s=released,
            reason=delivery['reason'],computed_output=data['output'][:2],
            computed_mode=data['output'][2].get('handoff_state'),
            next_outputs=[dict(reference_s=c['input']['reference_s'],delivery=c['delivery']['kind'],
                output=c['output'][:2],reason=c['output'][2].get('reason'),
                normal_reason=c['output'][2].get('normal',{}).get('reason')) for c in later],
            evaluation_samples=[{k:t.get(k) for k in ('t','progress_m','ground_speed_mps',
                'drive_speed_mps','safe_speed_mps','safety_reason')} for t in nearby]))
    result=dict(scope='saved input/delivery and evaluation samples; correlation is not causal proof',
        source_sha256=hashes,window=window,calls=len(calls),discarded=len(discarded),
        discard_reasons=dict(groups),reset_reasons=dict(Counter(e['data'].get('reason','unknown')
            for e in events if e['kind']=='reset')),
        safety_reason_samples=dict(safety),low_speed_safety_reason_samples=dict(low),
        first_discard_windows=windows)
    assert all(hashlib.sha256(p.read_bytes()).hexdigest()==hashes[p.name] for p in sources)
    with (D/'delivery_analysis.json').open('x',encoding='utf8') as stream:
        json.dump(result,stream,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k not in ('first_discard_windows','source_sha256')}))


if __name__=='__main__':main()
