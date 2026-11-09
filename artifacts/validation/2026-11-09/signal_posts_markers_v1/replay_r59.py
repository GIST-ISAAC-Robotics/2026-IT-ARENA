"""연결 r59와 사본 후보의 명령 일치, 같은 복귀 상태의 현재 장애물/미관측 반례."""
import copy,hashlib,json,math
from pathlib import Path
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
A=Path(__file__).resolve().parent;path=A/'c_stationary_posts/run4/follow_decisions.jsonl'
expected=[json.loads(x) for x in (A/'c4_extent_probe/rows.jsonl').open()]
out=A/'r59_connected_replay';out.mkdir(exist_ok=False)
digest=hashlib.sha256(path.read_bytes()).hexdigest();n=bad=0;faults={};confirmed=False
for event in read_events(path):
    d=event['data']
    if event['kind']=='header':c,mc=build_controller(d)
    elif event['kind']=='reset':c.reset()
    elif event['kind']=='call':
        t=d['input']['reference_s'];want=expected[n]['geometry_forward']
        if not confirmed and t>=24.52 and want['command'][0]>.1:
            for kind in ('normal','front_obstacle','front_unknown','outer_wall_unknown'):
                changed=copy.deepcopy(d['input']);scan=changed['scan']
                affected=[]
                for i in range(len(scan['ranges'])):
                    angle=scan['angle_min']+i*scan['angle_increment']
                    chosen=(abs(angle)<math.radians(35) if kind.startswith('front_') else angle>0)
                    if kind=='normal' or not chosen:continue
                    scan['ranges'][i]=.07 if kind=='front_obstacle' else {'nonfinite':'nan'}
                    affected.append(i)
                answer=freeze(replay_call(copy.deepcopy(c),changed,mc))
                faults[kind]=dict(t=t,command=answer[:2],behavior=answer[2].get('behavior'),
                    reason=answer[2].get('bypass',{}).get('reason'),changed_rays=len(affected))
            confirmed=True
        answer=freeze(replay_call(c,d['input'],mc))
        bad+=not equal_output(answer[:2],want['command']) or answer[2].get('bypass',{}).get('reason')!=want['reason']
        n+=1
summary=dict(calls=n,candidate_command_reason_mismatches=bad,faults=faults,
    source_unchanged=hashlib.sha256(path.read_bytes()).hexdigest()==digest,
    counterexamples_passed=bool(faults) and faults['normal']['command'][0]>.1 and
        all(faults[k]['command'][0]<=1e-6 for k in ('front_obstacle','front_unknown','outer_wall_unknown')))
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary),flush=True)
