"""공용 런타임 변경 없이 복귀 동안의 G1 준비를 저장 입력에서 비교한다."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from collections import Counter

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('arena_autonomy.moving_pass',HERE/'candidate_moving_pass.py')
module=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=module;spec.loader.exec_module(module)
from arena_autonomy.follow_trace import replay_events

def read(p):return json.loads(p.read_text(encoding='utf-8'))
def replay(folder):
    run=folder/'run1';report=read(run/'report.json');g=report['case_window']['green_sim_s']
    original={};at=None
    def events():
        nonlocal at
        with (run/'follow_decisions.jsonl').open(encoding='utf-8') as f:
            for line in f:
                e=json.loads(line)
                if e['kind']=='call':
                    d=e['data'];original[e['seq']]=d['output']
                    m=d['output'][2].get('moving_pass',{})
                    if m.get('completed') or m.get('reason')=='aborted_to_observed_follow':
                        at=d['input']['reference_s']-g
                yield e
    result=replay_events(events());changed=[];before=Counter();after=Counter();samples=[]
    for row in result['outcomes']:
        t=row['reference_s']-g;o=original[row['seq']];v=row['speed_mps'];a=row['steering_rad']
        if abs(v-o[0])>1e-9 or abs(a-o[1])>1e-9:changed.append(t)
        if at is not None and at-.01<=t<at+.50:
            before[str(o[2].get('normal',{}).get('reason'))]+=1
            after[str(row['details'].get('normal',{}).get('reason'))]+=1
            samples.append(dict(t=t,old_speed=o[0],speed=v,old_steer=o[1],steer=a,
                                reason=row['details'].get('reason'),normal=row['details'].get('normal',{}).get('reason')))
    return dict(calls=result['calls'],handoff_s=at,changed_motion_times_s=changed,
        changed_before_handoff=sum(t<at-1e-8 for t in changed),
        old_zero_in_05_s=sum(x['old_speed']<=1e-8 for x in samples),
        new_zero_in_05_s=sum(x['speed']<=1e-8 for x in samples),old_reasons=dict(before),new_reasons=dict(after),samples=samples)

out=dict(source_sha256=hashlib.sha256((HERE/'candidate_moving_pass.py').read_bytes()).hexdigest(),
    pass_case=replay(HERE.parent/'moving_pass_v1/closed_loop_side_pass_2'),
    withdrawal=replay(HERE.parent/'moving_pass_withdrawal_v1/candidate_r14'))
with (HERE/(sys.argv[1]+'.json')).open('x',encoding='utf-8') as f:json.dump(out,f,indent=2)
for k,v in out.items():
    if isinstance(v,dict):print(k,{x:y for x,y in v.items() if x not in ('samples','changed_motion_times_s')})
assert all(out[k]['changed_before_handoff']==0 for k in ('pass_case','withdrawal'))
assert out['withdrawal']['new_zero_in_05_s']<out['withdrawal']['old_zero_in_05_s']
