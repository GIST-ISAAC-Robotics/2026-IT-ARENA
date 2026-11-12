"""동일 계산 순서 저장 입력의 반대 벽 재획득 대조. 궤적 되먹임 없음."""
import json,sys
from pathlib import Path
from collections import Counter
A=Path(__file__).resolve().parent;R=A.parents[3];B=A.parent/'parallel_ros_runtime_r274_v1'
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,replay_reset,freeze,equal_output
from arena_autonomy.numeric_kernels import configure
import candidate

with (B/'two_actor_run1/run1/follow_decisions.jsonl').open() as f:
    header=json.loads(next(f))['data']
    calls=[e['data'] for line in f if (e:=json.loads(line))['kind']=='call']
calls.sort(key=lambda c:(c['compute']['generation'],c['compute']['sequence']))
configure('native_v1');assert header['follow_config']['standing_wall_reacquire']
summary={}
for mode in ('off','on'):
    if mode=='on':candidate.install()
    follow,motion=make_controller(header);generation=None;differences=[];selected=[];late=Counter()
    for c in calls:
        if generation!=c['compute']['generation']:
            replay_reset(follow,{'reason':None});generation=c['compute']['generation']
        out=freeze(replay_call(follow,c['input'],motion));normal=out[2].get('normal',{})
        if not equal_output(out,c['output']):differences.append(c['input']['reference_s'])
        if c['input']['reference_s']>=54.:
            late['positive' if out[0]>0 else 'zero']+=1
            late[normal.get('reason',out[2].get('reason'))]+=1
        if normal.get('single_wall_reacquire'):
            selected.append(dict(reference_s=c['input']['reference_s'],speed_mps=out[0],
                selection=normal['single_wall_reacquire'],reason=normal.get('reason')))
    summary[mode]=dict(calls=len(calls),different=len(differences),first_difference=differences[0] if differences else None,
                       late=dict(late),selected=selected)
    candidate.uninstall()
assert summary['off']['different']==0
summary['scope']='fixed recorded sensor trajectories; candidate plans only; no drive, no closed-loop or safety proof'
(A/'comparison.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary))
