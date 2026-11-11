"""폭 분리 후에도 명령이 0인 실제 저장 입력의 후속 검사만 회수한다."""
import json
import sys
from pathlib import Path

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze

source=A.parent/'scan_start_phase_r107_v1/two_actor_run1/run1/follow_decisions.jsonl'
rows=[json.loads(s) for s in source.open(encoding='utf8')]
controller,motion=make_controller(rows[0]['data'])
out=[]
for row in rows[1:]:
    if row['kind']=='reset':controller.reset()
    elif row['kind']=='call':
        result=freeze(replay_call(controller,row['data']['input'],motion))
        if row['seq'] in (919,929,939,959,969):
            out.append(dict(seq=row['seq'],output=result))
with (A/'gate_trace.json').open('x',encoding='utf8') as f:json.dump(out,f,indent=2)
for row in out:
    print(row['seq'],row['output'][:2])
    def show(value,path=''):
        if isinstance(value,dict):
            for key,child in value.items():
                if key in ('reason','selector_reason','allowed','forward_gap_deg','same_path_gap'):
                    print(path+key,json.dumps(child))
                elif isinstance(child,dict):show(child,path+key+'.')
    show(row['output'][2])
