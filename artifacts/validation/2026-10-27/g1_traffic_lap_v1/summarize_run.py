"""완료된 교통 랩의 분기·장기 정지를 요약한다. 판정/입력/원시 결과는 수정하지 않는다."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def summarize(run):
    report=json.loads((run/'report.json').read_text())
    counters={k:Counter() for k in ('reason','selector_mode','selector_reason','partial_reason','guard_baseline_partial')}
    ranges={}
    calls=0
    for line in (run/'follow_decisions.jsonl').open():
        row=json.loads(line)
        if row['kind']!='call':continue
        d=row['data'];target,steer,meta=d['output'];calls+=1
        partial=meta.get('partial_observation',{})
        keys={'reason':meta.get('reason'),'selector_mode':meta.get('selector_mode'),
              'selector_reason':meta.get('selector_reason'),'partial_reason':partial.get('reason'),
              'guard_baseline_partial':(meta.get('selector_guard_candidate') or {}).get('baseline_partial_reason')}
        for key,value in keys.items():counters[key][str(value)]+=1
        t=d['input']['reference_s']
        key=(str(keys['selector_reason']),str(keys['partial_reason']))
        item=ranges.setdefault(key,dict(first_s=t,last_s=t,calls=0,positive_calls=0))
        item['last_s']=t;item['calls']+=1;item['positive_calls']+=int(target>0.)
    rows=[json.loads(line) for line in (run/'trajectory.jsonl').open()]
    green=report['green_observed_sim_s'];stop=report['stop_request_sim_s']
    active=[r for r in rows if green<=r['t']<=stop]
    stalls=[];block=[]
    def end_block():
        if block and block[-1]['t']-block[0]['t']>=2.5:
            stalls.append(dict(first_s=block[0]['t'],last_s=block[-1]['t'],
                duration_s=block[-1]['t']-block[0]['t'],start_progress_m=block[0]['progress_m'],
                end_progress_m=block[-1]['progress_m'],
                controller_reasons=dict(Counter(str(r['autonomy'].get('reason')) for r in block)),
                safety_reasons=dict(Counter(str(r.get('safety_reason')) for r in block)),
                first_pose=[block[0][k] for k in ('x','y','yaw')]))
        block.clear()
    for r in active:
        if block and r['t']-block[-1]['t']>.25:end_block()
        if r['ground_speed_mps']<.05:block.append(r)
        else:end_block()
    end_block()
    return dict(scope='post-run diagnostic only; no altered verdict',
        report_sha256=hashlib.sha256((run/'report.json').read_bytes()).hexdigest(),
        report_passed=report.get('passed'),run_valid=report.get('run_valid'),calls=calls,
        failed_gates=[k for k,v in report.get('gates',{}).items() if not v],
        stop_checks=report.get('stop_checks'),progress=report['evaluation'].get('full_course_progress'),
        wheel_centers=report['evaluation'].get('wheel_centers'),
        max_observed_ground_speed_mps=max((r['ground_speed_mps'] for r in active),default=None),
        counters={k:dict(v) for k,v in counters.items()},
        branch_windows=[dict(selector_reason=k[0],partial_reason=k[1],**v) for k,v in ranges.items()],
        stationary_spans_at_least_2p5s=stalls)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=summarize(args.run)
    with args.output.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps({k:result[k] for k in ('report_passed','run_valid','calls','failed_gates','progress','stationary_spans_at_least_2p5s')},ensure_ascii=False))
