"""원 실패의 동일성 확인 이후 후보를 같은 저장 입력에서 대조한다. 새 주행 아님."""
import hashlib,json
from pathlib import Path
import sys
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision


def compare(path):
    path=path.resolve()
    raw=path.read_bytes();rows=[json.loads(line) for line in raw.splitlines()]
    follow,motion=make_controller(rows[0]['data']);changes=[];mismatches=[];calls=0
    for row in rows[1:]:
        if row['kind']=='reset':follow.reset()
        elif row['kind']=='call':
            data=row['data'];actual=freeze(replay_call(follow,data['input'],motion));calls+=1
            if not equal_output(comparable_decision(actual),comparable_decision(data['output'])):mismatches.append(row['seq'])
            if not equal_output(actual[:2],data['output'][:2]):
                normal=actual[2].get('normal',actual[2])
                changes.append(dict(seq=row['seq'],reference_s=data['input']['reference_s'],
                    old=data['output'][:2],new=actual[:2],reason=normal.get('reason'),boundary=normal.get('boundary_selection')))
    assert path.read_bytes()==raw
    return dict(source=str(path.relative_to(ROOT)),sha256=hashlib.sha256(raw).hexdigest(),calls=calls,
        full_output_mismatches=len(mismatches),command_changes=changes,
        stop_to_motion=sum(c['old'][0]==0 and c['new'][0]>0 for c in changes),
        motion_to_stop=sum(c['old'][0]>0 and c['new'][0]==0 for c in changes))


if __name__=='__main__':
    paths=[A.parent/'async_handoff_r104_v1/two_actor_run1/run1/follow_decisions.jsonl',Path(sys.argv[1])]
    result=[compare(p) for p in paths]
    with (A/'comparison.json').open('x',encoding='utf8') as f:json.dump(result,f,indent=2)
    print(json.dumps([{k:v if k!='command_changes' else len(v) for k,v in r.items()} for r in result]))
