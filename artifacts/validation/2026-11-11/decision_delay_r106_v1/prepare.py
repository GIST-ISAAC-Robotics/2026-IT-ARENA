"""기존 Jetson 단계 비용을 보존된 시각 순서의 시험 프로필로 변환한다."""
import hashlib
import json
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]


def main():
    cost=A.parent/'query_set_r103_v1/jetson_stages.json'
    trace=A.parent/'control_10hz_r91_v1/two_actor_run4/run1/follow_decisions.jsonl'
    report=trace.parent/'report.json'
    window=json.loads(report.read_bytes())['case_window']
    times={r['seq']:r['data']['input']['reference_s'] for r in map(json.loads,trace.read_text('utf8').splitlines()) if r['kind']=='call'}
    samples=[dict(offset_s=times[row['seq']]-window['green_sim_s'],
                  delay_s=sum(s['wall_ms'] for s in row['stages'].values())/1000.,
                  source_seq=row['seq'])
             for row in json.loads(cost.read_bytes())['per_call_stages']
             if window['green_sim_s']<=times[row['seq']]<=window['end_sim_s']]
    record=dict(schema='r106_measured_delivery_v1',
        scope='prior Jetson per-call stage sum as simulation delivery floor; not current-scene CPU or hardware latency prediction',
        samples=samples,
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in (cost,trace,report)})
    with (A/'delivery_profile.json').open('x',encoding='utf8') as f:
        json.dump(record,f,indent=2);f.write('\n')
    print(json.dumps(dict(samples=len(samples),delay_s=[min(s['delay_s'] for s in samples),max(s['delay_s'] for s in samples)])))


if __name__=='__main__':main()
