"""r104/r105의 스캔 완료→제출 위상 차이와 최종 정지 이유를 보존한다."""
import collections
import hashlib
import json
from pathlib import Path
import statistics
A=Path(__file__).resolve().parent;ROOT=A.parents[3]


def main():
    result=[]
    for name in ('async_handoff_r104_v1','async_corner_r105_v1'):
        run=A.parent/name/'two_actor_run1/run1'
        paths=[run/f for f in ('report.json','follow_decisions.jsonl','acquisition.json','trace.json')]
        report=json.loads(paths[0].read_bytes())
        rows=[r['data'] for r in map(json.loads,paths[1].read_text('utf8').splitlines()) if r['kind']=='call']
        acquisition=json.loads(paths[2].read_bytes())
        window=report['case_window']
        calls=[r for r in rows if window['green_sim_s']<=r['input']['reference_s']<=window['end_sim_s']]
        by_first={round(a['first_ray_stamp_s'],6):a for a in acquisition}
        pairs=[(c,by_first[round(c['input']['scan']['measurements']['acquired_at_s'][0],6)]) for c in calls]
        delay=[c['input']['reference_s']-a['source_stamp_at_publish_s'] for c,a in pairs]
        trace=[t for t in json.loads(paths[3].read_bytes()) if window['green_sim_s']<=t['t']<=window['end_sim_s']]
        last_positive=next((c for c in reversed(calls) if c['output'][0]>.01),None)
        reasons=collections.Counter(c['output'][2].get('normal',{}).get('reason') for c in calls[-50:])
        result.append(dict(name=name,case_calls=len(calls),first_ray=acquisition[0]['first_ray_stamp_s'],
            initial_discard_counter=acquisition[0]['discarded_source_gaps'],
            source_publish_to_submit_s=dict(min=min(delay),p50=statistics.median(delay),max=max(delay)),
            all_results_applied=all(c['delivery']['kind']=='candidate' for c in rows),
            last_case_trace={k:trace[-1][k] for k in ('t','progress_m','ground_speed_mps','safety_reason')},
            last_positive_reference_s=last_positive['input']['reference_s'] if last_positive else None,
            last_50_normal_reasons=dict(reasons),
            source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}))
    with (A/'phase_audit.json').open('x',encoding='utf8') as f:
        json.dump(dict(scope='observed timing difference, not isolation of every trajectory difference',runs=result),f,indent=2);f.write('\n')
    print(json.dumps([{k:v for k,v in r.items() if k!='source_sha256'} for r in result]))


if __name__=='__main__':main()
