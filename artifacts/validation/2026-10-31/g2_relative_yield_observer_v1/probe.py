"""저장 입력에서 제안이 언제 생기는지만 비교. 제어기/주행/원본 출력 불변."""
import argparse,hashlib,json
from pathlib import Path
from observer import RelativeYieldObserver,observation
BASE=Path(__file__).resolve().parent


def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def compare(case,end):
    path=BASE.parent/case/'run1/follow_decisions.jsonl';digest=sha(path)
    observer=RelativeYieldObserver();calls=active=actionable=0;events=[]
    for line in path.open():
        event=json.loads(line)
        if event['kind']=='reset':observer.reset()
        if event['kind']!='call':continue
        call=event['data'];t=call['input']['reference_s']
        if t>end:break
        detail=observer.update(observation(call));calls+=1
        if detail['suggest_yield']:
            active+=1;actionable+=call['output'][0]>.2
            if not events or t-events[-1]['last_s']>.041:events.append(dict(first_s=t,last_s=t,first_evidence=detail,source_speed=call['output'][0]))
            else:events[-1]['last_s']=t
    return dict(case=case,calls=calls,active_calls=active,would_reduce_positive_command=actionable,
        windows=events,input_sha256=digest,input_unchanged=sha(path)==digest,actual_commands_changed=False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=dict(scope='read-only saved decision observer; no new path, permission or driving outcome',
        results=[compare(c,40.) for c in ('g1_forward_same_path_gazebo_v1','g2_motion_pause_gazebo_v1','g2_motion_pause_level_v1')],
        source_sha256={p.name:sha(p) for p in (BASE/'observer.py',Path(__file__))})
    with a.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps(result))
