"""실제 정체 접두부 재현 및 같은 경로 정면 결측 후보의 저장 입력 비교."""
from pathlib import Path
from dataclasses import replace
from unittest.mock import patch
import json,hashlib,zipfile
from candidate import bounded_same_path_gap
import arena_autonomy.same_path_gap_trial as runtime
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
class CandidateFollow(BypassHandoffFollow):
    def command(self,*args,**kwargs):
        self.forward_use=[]
        def scoped(*values):
            result=bounded_same_path_gap(*values)
            if values[-2].get('reason')=='bounded_forward_missing':self.forward_use.append(result)
            return result
        with patch.object(runtime,'bounded_same_path_gap',scoped):
            return super().command(*args,**kwargs)
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def compare(path,out,*,prefix=None,expected=None):
    digest=sha(path);calls=matched=changed=zeros=increases=uses=0;footer=None;first=None;changes=[]
    with (out/(path.parent.parent.name+'_changes.jsonl')).open('x') as log:
        for seq,line in enumerate(path.open()):
            e=json.loads(line);assert e['seq']==seq and footer is None
            kind,data=e['kind'],e['data']
            if seq==0:
                assert kind=='header' and data['schema']==5
                cfg=FollowConfig(**data['follow_config']);ex=HandoffExecutionConfig.from_mapping(data['execution_config']);motion=MotionConfig(**data['motion_config'])
                if expected is not None:ex=replace(ex,initial_reference_policy='ready_directional_pair_v2')
                actual=BypassHandoffFollow(cfg,ex,allow_simulation_trial=True) if expected is None else None
                candidate=CandidateFollow(cfg,ex,allow_simulation_trial=True)
            elif kind=='reset':
                if actual:actual.reset()
                candidate.reset()
            elif kind=='call':
                t=data['input']['reference_s']
                if prefix is not None and t>prefix:continue
                if actual:
                    baseline=freeze(replay_call(actual,data['input'],motion))
                    matched+=equal_output(comparable_decision(baseline),comparable_decision(data['output']))
                    old=baseline[:2]
                else:
                    old=expected.get(t,data['output'][:2]);matched+=1
                value=freeze(replay_call(candidate,data['input'],motion));calls+=1
                diff=not equal_output(old,value[:2]);changed+=diff
                zeros+=value[0]==0<old[0];increases+=value[0]>old[0]+1e-9
                accepted=any(v['accepted'] for v in candidate.forward_use);uses+=accepted
                if diff or accepted:
                    row=dict(t=t,old=old,new=value[:2],state=value[2].get('handoff_state'),
                        forward=candidate.forward_use,normal=value[2].get('normal'),reason=value[2].get('reason'))
                    log.write(json.dumps(row)+'\n')
                    if diff:
                        slim={k:row[k] for k in ('t','old','new','state','forward','reason')};changes.append(slim)
                        if first is None:first=slim
            elif kind=='footer':footer=data
            else:raise ValueError('unexpected event')
    assert footer and footer['accepted']==footer['written']==seq and not footer['worker_alive']
    assert all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending'))
    return dict(input=path.relative_to(ROOT).as_posix(),input_sha256=digest,input_unchanged=sha(path)==digest,
        calls=calls,baseline_reproduced=matched if actual else None,cached_command_baseline=actual is None,
        command_changes=changed,new_zero_calls=zeros,increased_calls=increases,forward_accepted_calls=uses,
        first_change=first,changes=changes,footer_checked=True,passed=matched==calls and sha(path)==digest and zeros==0)
if __name__=='__main__':
    out=BASE/'run1';out.mkdir(exist_ok=False)
    cache=BASE.parent/'g2_directional_reference_connection_v1/run1/summary.json'
    v1=BASE.parent/'g2_ready_reference_v1/run1/g2_course_trial_v1_changes.jsonl'
    normal=json.loads(cache.read_text())['results'][1]
    expected={row['time_s']:row['candidate_command'] for row in map(json.loads,v1.open())}
    expected.update({row['time_s']:row['actual'] for row in normal['changes']})
    normal_path=ROOT/normal['input'];assert sha(normal_path)==normal['input_sha256']
    files=[*BASE.glob('*.py'),*BASE.glob('*.sh'),BASE/'PLAN.md',cache,v1]
    files.extend(p for folder in ('src/arena_autonomy/arena_autonomy','src/arena_vehicle_interface/arena_vehicle_interface') for p in (ROOT/folder).glob('*.py'))
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in files}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in hashes:z.write(ROOT/p,p)
    results=[compare(BASE.parent/'g2_directional_reference_gazebo_v1/run1/follow_decisions.jsonl',out,prefix=80.),
        compare(normal_path,out,expected=expected)]
    unchanged=all(sha(ROOT/p)==v for p,v in hashes.items())
    result=dict(passed=unchanged and all(r['passed'] for r in results),source_unchanged=unchanged,source_sha256=hashes,
        results=results,scope='saved-input counterfactual; not new drive; zero new stop commands required')
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],source_unchanged=unchanged,results=[{k:v for k,v in r.items() if k!='changes'} for r in results])))
    raise SystemExit(0 if result['passed'] else 1)
