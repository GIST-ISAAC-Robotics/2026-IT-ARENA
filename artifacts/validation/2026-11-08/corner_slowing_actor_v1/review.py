"""주행이 끝난 뒤 명령·코너 근거·원 판정을 간략히 회수한다."""
from collections import Counter
from pathlib import Path
import hashlib
import json

A=Path(__file__).resolve().parent;RUN=A/'run1'
assert (A/'run1.exit').exists(), 'review completed trial only'
read=lambda p:json.loads(p.read_bytes())
report=read(RUN/'report.json');begin=report['case_window']['green_sim_s']
reason=Counter();stops=Counter();support=[];far=[];motion=[];groups=[];current=None
for line in (RUN/'follow_decisions.jsonl').open():
    e=json.loads(line)
    if e['kind']!='call':continue
    d=e['data'];t=d['input']['reference_s']
    if not begin <= t <= report['case_window']['end_sim_s']:continue
    out=d['output'];n=out[2].get('normal',out[2]);key=(n.get('reason'),n.get('baseline_reason'),n.get('selector_reason'))
    reason[str(key)]+=1
    if out[0]<=0:stops[str(key)]+=1
    proof=n.get('current_corner_support') or {}
    if proof.get('accepted'):support.append(dict(t=t,source=n.get('path_source_s'),speed=out[0],evidence=proof))
    if proof.get('far_occlusion_support'):far.append(dict(t=t,speed=out[0],evidence=proof))
    sig=(out[0]<=0,*key)
    if current is None or sig!=current['key']:
        current=dict(key=sig,start=t,end=t,count=1,detail={k:n.get(k) for k in
            ('boundary_selection','boundary_age_s','observed_path_ahead_m','selector_guard_candidate','partial_observation')})
        groups.append(current)
    else:current['end']=t;current['count']+=1
assessment=read(A/'assessment_run1.json')
summary=dict(green_sim_s=begin,assessment=assessment,reasons=reason,zero_reasons=stops,
    corner_support_calls=len(support),far_support_calls=len(far),far_support=far,
    long_command_groups=[g for g in groups if g['end']-g['start']>=.3],
    source=read(A/'postflight_integrity_run1.json'))
(A/'decision_window_review.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in summary.items() if k not in ('far_support','long_command_groups','source')},ensure_ascii=False))
