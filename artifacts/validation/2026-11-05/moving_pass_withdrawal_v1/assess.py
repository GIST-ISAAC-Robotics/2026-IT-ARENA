"""경로를 원래 선으로 되돌린 것과 그냥 속도가 줄거나 상대가 사라진 것을 구분한다."""
from bisect import bisect_left
import math
from collections import Counter
import sys
from prepare import HERE,ROOT,read,save,sha

def rows(path):
    import json
    with path.open(encoding='utf-8') as f:
        yield from (json.loads(line) for line in f if line.strip())

def interpolate(data,times,t):
    i=bisect_left(times,t);i=max(1,min(i,len(times)-1));a,b=data[i-1:i+1]
    if not a[0]<=t<=b[0] or not 0<b[0]-a[0]<=.12:raise ValueError('missing pose coverage')
    k=(t-a[0])/(b[0]-a[0]);return [a[j]+k*(b[j]-a[j]) for j in (1,2)]

def main(name):
    folder=HERE/name;r=folder/'run1';rep=read(r/'report.json');reg=read(folder/'registered.json')
    tg,te=(rep['case_window'][k] for k in ('green_sim_s','end_sim_s'))
    ego=list(rows(r/'ego_poses.jsonl'));actor=list(rows(r/'actor_poses.jsonl'))
    et=[e[0] for e in ego];act=[a[0] for a in actor]
    def pos(seq,times,t):return interpolate(seq,times,tg+t)
    a0=pos(actor,act,0);a1=pos(actor,act,te-tg);dx,dy=(a1[j]-a0[j] for j in range(2))
    d=math.hypot(dx,dy);c,s=dx/d,dy/d;offset=read(HERE/'course.json')['cases'][0]['actor_lateral_offset_m']
    def coordinate(t):
        e=pos(ego,et,t);a=pos(actor,act,t);x=e[0]-a[0];y=e[1]-a[1]
        return (t,c*x+s*y,-s*x+c*y+offset,c*(e[0]-a0[0])+s*(e[1]-a0[1]))
    trajectory=[coordinate(i*.05) for i in range(round((te-tg)*20)+1)]
    rates=[math.dist(pos(actor,act,k),pos(actor,act,k+1)) for k in range(round(te-tg))]
    trace=rows(r/'follow_decisions.jsonl');header=next(trace)['data']
    complete=[];withdraw=[];entry=None;reasons=Counter()
    for event in trace:
        if event['kind']!='call':continue
        q=event['data'];t=q['input']['reference_s']-tg
        if not 0<=t<=te-tg:continue
        detail=q['output'][2];m=detail.get('moving_pass',{});why=m.get('reason') or detail['reason']
        reasons[why]+=1
        if detail['reason']=='moving_pass_entry' and entry is None:entry=t
        if m.get('reason')=='aborted_to_observed_follow':withdraw.append(t)
        if m.get('completed'):complete.append(t)
    post=0.;first=None
    for t,x,y,p in trajectory:
        if withdraw and t>=withdraw[0] and x<=-.20 and abs(y)<=.06:
            if first is None:first=p
            post=max(post,p-first)
        else:first=None
    after=[p for p in trajectory if withdraw and p[0]>=withdraw[0]]
    gap=rep['evaluation']['gap'];surf=read(folder/'surface.json')
    gates=dict(valid=bool(rep['run_valid'] and rep['gates']['data_coverage'] and all(rep['stop_checks'].values())),
        sources=all(sha(ROOT/p)==h for p,h in reg['sources'].items()),
        activated=header['execution_config'].get('moving_pass_trial') is True and rep['handoff_execution']['passed'],
        actor_followed_schedule=all(.25<=v<=.35 for v in rates[:4]) and all(.53<=v<=.63 for v in rates[5:]),
        entered_before_acceleration=entry is not None and entry<4. and coordinate(4.)[1]<=-.20,
        withdrew_and_followed=bool(len(withdraw)==1 and post>=.50 and after and all(p[1]<0 for p in after)),
        no_contact=gap['actor_contact_messages_with_ego']==0 and gap['sampled_overlap_samples']==0
            and gap['sampled_finite_complete'] and gap['swept_missing_actor_intervals']==0 and gap['min_swept_lower_bound_m']>0,
        wheel_surface=surf['interpretations']['any']['max_off_wheels']<=2 and surf['wall_wheel_samples']==0)
    result=dict(scope='single scripted speed change on straight, evaluation only',accepted=all(gates.values()),
        gates=gates,actor_1s_mps=rates,entry_after_green_s=entry,relative_at_4s=coordinate(4.),withdrawal_times_s=withdraw,
        pass_completed_times_s=complete,post_return_progress_m=post,final_position=trajectory[-1],
        progress_m=rep['evaluation']['progress']['at_window_end_m'],swept_lower_bound_m=gap['min_swept_lower_bound_m'],
        contact_messages=gap['actor_contact_messages_with_ego'],max_off_wheels=surf['interpretations']['any']['max_off_wheels'],
        reasons=dict(reasons),old_follow_verdict=rep['passed'],old_gates=rep['gates'],stop=rep['stop_checks'])
    save(folder/'assessment.json',result);print(result)
    return 0 if result['accepted'] else 1
if __name__=='__main__':sys.exit(main(sys.argv[1]))
