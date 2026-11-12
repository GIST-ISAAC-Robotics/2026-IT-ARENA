"""실제 r279 궤적에 고정한 후보 OFF/ON 대조. 다른 폐루프 결과로 확대하지 않는다."""
import json,sys
from pathlib import Path
from collections import Counter
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,replay_reset,freeze,equal_output
from arena_autonomy.numeric_kernels import configure
from worker_candidate import load_candidate

with (A/'two_actor_run1/run1/follow_decisions.jsonl').open() as f:
    header=json.loads(next(f))['data']
    calls=[e['data'] for line in f if (e:=json.loads(line))['kind']=='call']
calls.sort(key=lambda c:(c['compute']['generation'],c['compute']['sequence']))
configure('native_v1');result={}
for enabled in (False,True):
    patch=load_candidate()
    if enabled:patch.install()
    follow,motion=make_controller(header);generation=None;differences=[];selected=[];reasons=Counter()
    for c in calls:
        if generation!=c['compute']['generation']:
            replay_reset(follow,{'reason':None});generation=c['compute']['generation']
        out=freeze(replay_call(follow,c['input'],motion));normal=out[2].get('normal',{})
        reasons[out[2].get('reason')]+=1
        if not equal_output(out,c['output']):differences.append(c['input']['reference_s'])
        if normal.get('single_wall_reacquire'):
            selected.append(dict(reference_s=c['input']['reference_s'],speed_mps=out[0],
                                 selection=normal['single_wall_reacquire']))
    if enabled:patch.uninstall()
    result['on' if enabled else 'off']=dict(calls=len(calls),different=len(differences),
        first_difference=differences[0] if differences else None,reasons=dict(reasons),
        selected_count=len(selected),first_selected=selected[:6])
assert result['on']['different']==0
result['scope']='same recorded inputs and generation order; not an OFF closed-loop trial'
(A/'causal_compare.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
