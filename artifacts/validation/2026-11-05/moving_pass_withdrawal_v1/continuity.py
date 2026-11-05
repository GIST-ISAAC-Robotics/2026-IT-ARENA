"""r14 양보의 속도·전진 기록. 정답은 사후 차체 이동률 평가에만 쓴다."""
from prepare import HERE,read,save
from assess import rows

folder=HERE/'candidate_r14';run=folder/'run1'
window=read(run/'report.json')['case_window'];g=window['green_sim_s']
decisions=[]
for event in rows(run/'follow_decisions.jsonl'):
    if event['kind']!='call':continue
    d=event['data'];t=d['input']['reference_s']-g
    if 2.5<=t<=window['duration_s']:
        v,a,why=d['output'];m=why.get('moving_pass',{})
        decisions.append(dict(t=t,v=v,a=a,reason=why['reason'],
            cap=m.get('withdrawal_speed_cap_mps')))
poses=list(rows(run/'ego_poses.jsonl'))
speed=[]
for a,b in zip(poses,poses[1:]):
    dt=b[0]-a[0];t=(a[0]+b[0])/2-g
    if 2.5<=t<=window['duration_s'] and 0<dt<=.12:
        speed.append(dict(t=t,v=((b[1]-a[1])**2+(b[2]-a[2])**2)**.5/dt,dt=dt))
yielding=[r for r in decisions if r['reason'] in ('moving_pass_yield_for_return','moving_pass_return_behind')]
slopes=[(b['v']-a['v'])/(b['t']-a['t']) for a,b in zip(yielding,yielding[1:]) if 0<b['t']-a['t']<=.1]
result=dict(scope='post-entry 2.5~10s; pose speed evaluation only',
    post_entry_zero_commands=sum(r['v']<=1e-8 for r in decisions),
    yield_command_min=min(r['v'] for r in yielding),yield_command_max=max(r['v'] for r in yielding),
    yield_largest_planned_deceleration_mps2=-min(slopes),
    post_entry_ground_min_mps=min(r['v'] for r in speed),
    post_entry_ground_below_005_s=sum(r['dt'] for r in speed if r['v']<.05),
    phases=[r for i,r in enumerate(decisions) if i==0 or r['reason']!=decisions[i-1]['reason']],
    note='target changes and ground speed are separate; not a real braking calibration')
save(folder/'continuity.json',result)
print(result)
