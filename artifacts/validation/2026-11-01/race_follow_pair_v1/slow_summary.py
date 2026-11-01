"""0.35 m/s 등속 앞차를 만난 12초 동안 출발 대기와 도중 정체를 따로 집계한다."""
import argparse
import json
from pathlib import Path


def summarize(run):
    report=json.loads((run/'report.json').read_text(encoding='utf-8'))
    if report['case']!='slow_lead' or not report['run_valid']:
        raise ValueError('등속 앞차의 유효 주행만 집계한다')
    green=report['green_observed_sim_s']
    duration=report['case_window']['duration_s']
    physical=[]
    with (run/'trajectory.jsonl').open(encoding='utf-8') as f:
        for row in map(json.loads,f):
            t=row.get('elapsed_since_green_s')
            if t is not None:
                physical.append((t,row['ground_speed_mps'],row['sampled_gap_m']))
    started=next((t for t,v,_ in physical if 0<=t<=duration and v>=.05),None)
    quiet=longest=chunk=observed=0.
    for (a,v,_),(b,_,_) in zip(physical,physical[1:]):
        dt=min(b,duration)-max(a,0.)
        if dt<=0 or b-a>.25:
            chunk=0.;continue
        observed+=dt
        if started is not None and a>=started and v<.05:
            chunk+=dt;quiet+=dt;longest=max(longest,chunk)
        else:chunk=0.
    phases=[];calls=zeros=0;md=None
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as f:
        for e in map(json.loads,f):
            if e['kind']!='call':continue
            t=e['data']['input']['reference_s']-green
            if not 0<=t<=duration:continue
            v,_,detail=e['data']['output'];calls+=1;zeros+=v==0
            mode=detail['handoff_state']
            if mode!=md:
                md=mode;phases.append(dict(elapsed_s=t,mode=md))
    return dict(case=report['case'],duration_s=duration,covered_trace_s=observed,
                first_005_mps_elapsed_s=started,sub_005_s_after_start=quiet,
                longest_sub_005_s_after_start=longest,decision_calls=calls,zero_requests=zeros,
                transitions=phases,ever_bypass=any(p['mode']=='BYPASS' for p in phases),
                progress=report['evaluation']['progress'],gap=report['evaluation']['gap'],
                original_verdict=report['passed'],original_gates=report['gates'],
                stop_checks=report['stop_checks'],case_metrics=report['evaluation']['case_metrics'],
                scope='고정 12초 각본, 최대 0.25초 간격 실제 속도 표본. 미관측 구간 제외, 실차 추종 증명 아님.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args();result=summarize(args.run)
    with args.output.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
