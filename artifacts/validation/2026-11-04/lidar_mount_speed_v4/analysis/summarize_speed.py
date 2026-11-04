"""Completed-run comparisons only; ground truth is evaluation input, never control."""
import json,math,sys
from collections import Counter
from pathlib import Path
HERE=Path(__file__).resolve().parent.parent
def read(p):return json.loads(p.read_text(encoding='utf8'))
def rows(p):
    with p.open(encoding='utf8') as f:
        for l in f:
            if l.strip():yield json.loads(l)
def intervals(ts,vs,predicate,start,end):
    result=[]; active=None;last=None
    for a,b,v in zip(ts,ts[1:]+[end],vs):
        if b<=start or a>=end:continue
        a,b=max(a,start),min(b,end)
        if predicate(v):
            if active is None:active=a
            last=b
        elif active is not None:
            result.append([active,last]);active=None
    if active is not None:result.append([active,last])
    return result
def compact_episodes(iv,start,end,min_s):
    q=[(a,b) for a,b in iv if b-a>=min_s-1e-9]
    return dict(count=len(q),total_s=round(sum(b-a for a,b in q),3),
                longest_s=round(max((b-a for a,b in q),default=0),3),
                episodes_s=[[round(a-start,3),round(b-start,3)] for a,b in q])
def summarize(name):
    run=HERE/name
    report=read(run/'report.json'); ev=read(HERE/(name+'_eval.json'))
    metrics=read(HERE/(name+'_metrics.json'));surf=read(HERE/(name+'_surface.json'))
    receipt=(HERE/(name+'.exit')).read_text().strip()
    trace=[r for r in rows(run/'trajectory.jsonl') if r.get('light')=='green']
    start=trace[0]['time'];end=start+ev['lap_s'] if ev.get('lap_s') else trace[-1]['time']
    trace=[r for r in trace if start<=r['time']<=end]
    ts=[r['time'] for r in trace]
    speeds=[math.hypot(r['dynamics'].get('truth_longitudinal_mps',0),r['dynamics'].get('truth_lateral_mps',0)) for r in trace]
    initial_move=next((t for t,v in zip(ts,speeds) if v>=.1),end)
    moving_gt=[(t,v) for t,v in zip(ts,speeds) if t>=initial_move]
    real=intervals([a for a,b in moving_gt],[b for a,b in moving_gt],lambda v:v<.05,initial_move,end)
    low=intervals([a for a,b in moving_gt],[b for a,b in moving_gt],lambda v:v<.2,initial_move,end)
    dt=[min(b,end)-a for a,b in zip(ts,ts[1:]+[end])]
    high={str(x):round(sum(d for d,v in zip(dt,speeds) if v>x),3) for x in [1.2,2.]}
    elapsed_by_progress={};speed_by_progress={}
    for metre in (8,12,15,16,24,32,40,44):
        pair=next(((a,b) for a,b in zip(trace,trace[1:])
                   if a['progress_m']<metre<=b['progress_m']),None)
        if pair:
            a,b=pair; f=(metre-a['progress_m'])/(b['progress_m']-a['progress_m'])
            elapsed_by_progress[metre]=round((a['time']-start)+f*(b['time']-a['time']),3)
            va,vb=[math.hypot(z['dynamics']['truth_longitudinal_mps'],z['dynamics']['truth_lateral_mps']) for z in pair]
            speed_by_progress[metre]=round(va+f*(vb-va),3)
    calls=Counter(); reasons=Counter(); decisions=[]
    with (run/'follow_decisions.jsonl').open(encoding='utf8') as f:
        for l in f:
            r=json.loads(l)
            if r['kind']!='call':continue
            t=r['data']['input']['reference_s']
            if not start<=t<=end:continue
            speed,steer,d=r['data']['output'];decisions.append((t,speed,d))
            calls['total']+=1
            if 'open_route_speed_enabled' in d:
                calls['gate_evaluated']+=1; calls['gate_enabled']+=bool(d['open_route_speed_enabled'])
            calls['candidate_requires_structure']+=bool(d.get('open_route_requires_structure_check'))
            calls['final_command_over_1_2']+=speed>1.2
            if speed<=.001:reasons[d.get('reason')]+=1
    cts=[t for t,v,d in decisions];cvs=[v for t,v,d in decisions]
    stall_context=None
    if not ev['completed_laps'] and decisions:
        lp=next(((t,v,d) for t,v,d in reversed(decisions) if v>.001),None)
        last=decisions[-1][-1]
        part=last.get('partial_observation') or {};structure=part.get('known_structure') or {}
        stall_context=dict(last_reason=last.get('reason'),corridor=(last.get('corridor_candidate') or {}).get('reason'),
            partial_allowed=part.get('allowed'),partial_speed_cap_mps=part.get('speed_cap_mps'),
            known_structure_cap_mps=structure.get('speed_cap_mps'))
        if lp:
            t,v,d=lp;near=min(trace,key=lambda r:abs(r['time']-t))
            stall_context.update(last_positive_command_mps=v,last_positive_progress_m=near['progress_m'],
                last_positive_path_from_memory=d.get('boundary_from_memory'),last_positive_path_source=d.get('reason'))
    cmd=intervals(cts,cvs,lambda v:v<=.001,start,end)
    cmd_after=intervals(cts,cvs,lambda v:v<=.001,initial_move,end)
    safe_after=intervals(ts,[r['safe_drive_speed_mps'] for r in trace],lambda v:v is not None and v<=.001,initial_move,end)
    recoveries=[]
    for a,b in real:
        if b-a<.1-1e-9:continue
        rec=next((t for t,v in zip(ts,speeds) if t>=b and v>=.1),None)
        recoveries.append(dict(stopped_at_s=round(a-start,3),duration_s=round(b-a,3),
                               to_motion_0_1_s=None if rec is None else round(rec-a,3)))
    return dict(run=name,receipt=receipt,passed=ev['passed'],laps=ev['completed_laps'],progress_m=ev['progress_m'],
                lap_s=ev['lap_s'],peak_mps=ev['peak_ground_speed_mps'],first_motion_after_green_s=round(initial_move-start,3),
                measured_above_speed_s=high,mean_speed_green_mps=sum(d*v for d,v in zip(dt,speeds))/(end-start),
                elapsed_green_s_at_first_progress_m=elapsed_by_progress,
                ground_speed_mps_at_first_progress_m=speed_by_progress,
                measured_stop_after_first_motion=compact_episodes(real,start,end,.1),
                measured_slow_after_first_motion=compact_episodes(low,start,end,.1),stop_recoveries=recoveries,
                zero_command_all=compact_episodes(cmd,start,end,0.),
                zero_command_after_motion=compact_episodes(cmd_after,start,end,0.),
                safe_zero_after_motion=compact_episodes(safe_after,start,end,0.),
                zero_command_ge_0_1s=compact_episodes(cmd,start,end,.1),zero_reason_calls=reasons,
                calls=calls,collision_samples=ev['collision_samples'],stable_stop=ev['stop_stable'],
                stall_context=stall_context,failure_stop_ack=metrics.get('failure_cleanup_stop_ack'),
                failure_dense_stop_confirmed=metrics.get('failure_cleanup_dense_stop'),
                shutdown_clean=report['shutdown_clean'],max_trace_gap_s=max((b-a for a,b in zip(ts,ts[1:])),default=0.),
                body_offroad_samples=surf['body_outside_samples_over_1cm2'],wall_wheel_samples=surf['wall_wheel_samples'],
                steering_footprint_outside_samples=surf['steering_footprint_outside_samples_over_1e-6_m2'],
                safety_reason_s=ev.get('safety_reason_s'),metrics=metrics)
def main():
    out={n:summarize(n) for n in sys.argv[1:]}
    source=[read(HERE/(n+'_registered.json'))['source_sha256'] for n in sys.argv[1:]]
    shared=set.intersection(*(set(r) for r in source))
    different={p:[d[p] for d in source] for p in shared if len({d[p] for d in source})>1}
    runtime_diff={p:v for p,v in different.items() if p.startswith(('src/','install/','scripts/'))}
    assert not runtime_diff,runtime_diff
    result=dict(scope='same final code; per-run traces, truth evaluation only; stops <0.05m/s for >=0.1s after first motion, before lap end; command=0 counted separately',
                common_source_entries=len(shared),common_runtime_source_hashes_match=True,
                nonruntime_registration_differences=different,runs=out)
    (HERE/'comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    for n,r in out.items():
        print(n,r['lap_s'],r['peak_mps'],r['measured_stop_after_first_motion'],r['zero_command_ge_0_1s'])
if __name__=='__main__':main()
