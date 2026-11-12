"""ROS 발행 순서가 아닌 실제 planner generation/sequence 순서로 수치 재생한다."""
import json,sys
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface'),str(A.parent/'retained_route_execution_r266_v1')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,replay_reset,freeze,equal_output
from arena_autonomy.numeric_kernels import configure
from overlap import evaluate_current,plan_reference
from worker_candidate import load_candidate
load_candidate().install()

def events():
    with (A/'two_actor_run1/run1/follow_decisions.jsonl').open() as f:
        for line in f:yield json.loads(line)

header=next(events())['data'];configure('native_v1')
follow,motion=make_controller(header)
calls=[e['data'] for e in events() if e['kind']=='call']
ordered=sorted(calls,key=lambda c:(c['compute']['generation'],c['compute']['sequence']))
identities=[(c['compute']['generation'],c['compute']['sequence']) for c in ordered]
assert len(set(identities))==len(ordered)
generation=None;different=[];current_different=[];checked=0
for c in ordered:
    if generation!=c['compute']['generation']:
        replay_reset(follow,{'reason':None});generation=c['compute']['generation']
    output=freeze(replay_call(follow,c['input'],motion))
    if not equal_output(output,c['output']):
        different.append(dict(generation=generation,sequence=c['compute']['sequence'],reference_s=c['input']['reference_s']))
    current=c.get('current_rebase')
    if current:
        checked+=1
        result=evaluate_current(c['output'],plan_reference(c['input']),current['input'],motion,follow.config)
        if not equal_output(result,current['output']):
            current_different.append(dict(sequence=c['compute']['sequence'],reference_s=c['input']['reference_s']))
result=dict(plans=len(ordered),unique_plans=len(identities),plan_differences=different,
            current_checks=checked,current_differences=current_different,
            scope='original computational order using worker generation; ROS timing not replayed; generic call/reset order is not this parallel worker order')
(A/'parallel_replay.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
raise SystemExit(bool(different or current_different))
