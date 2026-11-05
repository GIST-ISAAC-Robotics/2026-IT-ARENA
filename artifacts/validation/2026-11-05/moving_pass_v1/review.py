"""직선 이동차 추월 평가 전용. 정답 자세는 종료 후 이 파일에서만 읽는다."""
from bisect import bisect_left
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[3]
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def lines(path):
    with path.open(encoding='utf-8') as f:
        yield from (json.loads(row) for row in f if row.strip())
def at(rows,times,t):
    i=min(max(bisect_left(times,t),1),len(times)-1);a,b=rows[i-1:i+1]
    if not a[0]<=t<=b[0] or not 0<b[0]-a[0]<=.12:raise ValueError('incomplete pose window')
    w=(t-a[0])/(b[0]-a[0]);return (a[1]+w*(b[1]-a[1]),a[2]+w*(b[2]-a[2]))

def main(name):
    folder=HERE/name;r=folder/'run1';reg=read(folder/'registered.json');report=read(r/'report.json')
    offset=reg.get('actor_offset_m',0.)
    follow=reg.get('variant')=='follow'
    tg,te=(report['case_window'][k] for k in ('green_sim_s','end_sim_s'))
    ego=list(lines(r/'ego_poses.jsonl'));actor=list(lines(r/'actor_poses.jsonl'))
    et=[v[0] for v in ego];act=[v[0] for v in actor]
    a0=at(actor,act,tg);a1=at(actor,act,te);dx=a1[0]-a0[0];dy=a1[1]-a0[1];length=math.hypot(dx,dy)
    c,s=dx/length,dy/length
    segments=[math.dist(at(actor,act,tg+k),at(actor,act,tg+k+1)) for k in range(round(te-tg))]
    timeline=[];marks={}; rejoin_start=None;continuous_after=0.
    for i in range(int((te-tg)/.05)+1):
        t=tg+i*.05;e=at(ego,et,t);a=at(actor,act,t);u,v=e[0]-a[0],e[1]-a[1]
        along=c*u+s*v;cross=-s*u+c*v;progress=c*(e[0]-a0[0])+s*(e[1]-a0[1])
        if not marks and along<=-.20:marks['behind']=t
        elif 'behind' in marks and 'beside' not in marks and abs(along)<=.10 and abs(cross)>=.15:marks['beside']=t
        elif 'beside' in marks and 'ahead' not in marks and along>=.20:marks['ahead']=t
        if 'ahead' in marks and along>=.20 and abs(cross+offset)<=.06:
            if rejoin_start is None:rejoin_start=(t,progress)
            continuous_after=max(continuous_after,progress-rejoin_start[1])
            if progress-rejoin_start[1]>=.20 and 'rejoined' not in marks:marks['rejoined']=t
        else:rejoin_start=None
        timeline.append((t,along,cross))
    trace=lines(r/'follow_decisions.jsonl');head=next(trace)['data'];phases=Counter();reasons=Counter();completed=[];entries=[];rear=[]
    active=False;withdrawals=[]
    for call in trace:
        if call['kind']=='reset':active=False
        if call['kind']!='call':continue
        d=call['data'];t=d['input']['reference_s']
        if not tg<=t<=te:continue
        detail=d['output'][2];m=detail.get('moving_pass',{});phases[m.get('phase','none')]+=1
        reasons[(m.get('reason') or detail.get('reason','unknown'))]+=1
        if m.get('completed'):completed.append(t)
        if m.get('reason')=='aborted_to_observed_follow':withdrawals.append(t)
        now_active=detail.get('handoff_state')=='MOVING_PASS'
        if now_active and not active and detail.get('reason')=='moving_pass_entry':entries.append(t)
        active=now_active
        if m.get('target',{}).get('rear_clear'):rear.append(t)
    gap=report['evaluation']['gap'];surf=read(folder/'surface.json')
    base=(report if follow else read(HERE/'closed_loop_side_follow_1/run1/report.json') if offset
          else read(HERE.parent/'moving_follow_continuity_v1/closed_loop_v2/run1/report.json'))
    old_progress=base['evaluation']['progress']['at_window_end_m'];progress=report['evaluation']['progress']['at_window_end_m']
    withdrawal=None
    if withdrawals:
        wt=withdrawals[0];p=at(ego,et,wt);last=at(ego,et,te)
        further=c*(last[0]-p[0])+s*(last[1]-p[1])
        later=[(t,x,y) for t,x,y in timeline if t>=wt]
        withdrawal=dict(t=wt,t_after_green=wt-tg,progress_after_m=further,
            stayed_behind=bool(later and all(x<0 for _,x,_ in later)),
            returned_to_lane=any(abs(y+offset)<=.06 for _,_,y in later),distinct_withdrawals=len(withdrawals))
    checks=dict(valid=report['run_valid'] and report['gates']['data_coverage'] and all(report['stop_checks'].values()),
        unchanged=all(hashlib.sha256((ROOT/k).read_bytes()).hexdigest()==v for k,v in reg['sources'].items()),
        activated=head['execution_config'].get('moving_pass_trial') is True and report['handoff_execution']['passed']
            and 'evaluation_moving_pass_trial:=true' in report['handoff_launch_arguments'],
        actor_advancing=min(segments)>=.25, ordered_pass='rejoined' in marks,
        planning_completed=bool(completed and entries and rear and min(entries)<min(rear)<min(completed)),
        no_contact=gap['actor_contact_messages_with_ego']==0 and gap['sampled_overlap_samples']==0
            and gap['sampled_finite_complete'] and gap['swept_missing_actor_intervals']==0 and gap['min_swept_lower_bound_m']>0,
        wheel_surface=surf['interpretations']['any']['max_off_wheels']<=2 and surf['wall_wheel_samples']==0,
        progress_gain=progress>old_progress)
    summary=dict(scope='evaluation_only_single_straight_actor_not_real_vehicle',accepted=all(checks.values()),
        checks=checks,old_follow_verdict=report['passed'],old_follow_gates=report['gates'],stop=report['stop_checks'],
        sampled_gap_m=gap['min_sampled_gap_m'],swept_lower_bound_m=gap['min_swept_lower_bound_m'],
        contact_events=gap['actor_contact_messages_with_ego'],max_off_wheels=surf['interpretations']['any']['max_off_wheels'],
        body_outside=surf['body_outside_samples_over_1e-6_m2'],base_progress_m=old_progress,progress_m=progress,
        min_actor_1s_mps=min(segments),max_actor_1s_mps=max(segments),ordered_times=marks,
        post_rejoin_progress_m=continuous_after,maximum_ahead_m=max(v[1] for v in timeline),
        maximum_absolute_lateral_m=max(abs(v[2]) for v in timeline),entry_times=entries,
        rear_first_s=min(rear) if rear else None,completed_times=completed,phases=phases,reasons=reasons)
    summary['withdrawal']=withdrawal
    summary['withdrawal_only_passed']=bool(withdrawal and withdrawal['progress_after_m']>=.50
        and withdrawal['stayed_behind'] and withdrawal['returned_to_lane'] and withdrawal['distinct_withdrawals']==1
        and all(checks[k] for k in ('valid','unchanged','activated','actor_advancing','no_contact','wheel_surface')))
    if follow:
        base_checks={k:checks[k] for k in ('valid','unchanged','actor_advancing','no_contact','wheel_surface')}
        base_checks['follow_only']=head['execution_config'].get('moving_pass_trial') is False and not entries
        summary.update(variant='follow_comparison_only',accepted=all(base_checks.values()),
                       baseline_checks=base_checks,base_progress_m=None)
    with (folder/'review.json').open('x',encoding='utf-8') as f:json.dump(summary,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps(summary,ensure_ascii=False));return 0 if summary['accepted'] else 1
if __name__=='__main__':raise SystemExit(main(sys.argv[1]))
