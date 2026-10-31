"""초기 진입 관측의 성공/실패 차이를 과거 선택기까지 복원해 조사한다."""
import json,hashlib,zipfile,types
from pathlib import Path
from candidate import DirectionalPairReference
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy.traffic_path_recovery_trial import SuspendedPassageSelector
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
SNAP=BASE.parent/'g1_same_scan_history_connection_v1/before_connection.zip'
with zipfile.ZipFile(SNAP) as z:old_source=z.read('src/arena_autonomy/arena_autonomy/traffic_path_recovery_trial.py')
legacy=types.ModuleType('preserved_pre_same_scan_selector');exec(compile(old_source,str(SNAP),'exec'),legacy.__dict__)
CASES=[('g2_ready_reference_gazebo_v1/run2/follow_decisions.jsonl',True),('g2_ready_reference_course_v1/run2/follow_decisions.jsonl',True),('g1_same_scan_history_gazebo_v1/run1/follow_decisions.jsonl',False)]
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def slim(value):
    if isinstance(value,dict):return {k:slim(v) for k,v in value.items() if k!='source_ray_indices'}
    if isinstance(value,(tuple,list)):return [slim(v) for v in value]
    return freeze(value)
def observe(ctl):
    h=ctl._initial_reference
    return dict(original=slim(h.original),proposed=slim(h.proposed),key=h.memory.key,anchors={k:dict(time=v['time'],line=freeze(v['line'])) for k,v in h.memory.anchors.items()},motion_evidence=ctl.monitor.motion_evidence())
def compare(relative,was_old,out):
    p=BASE.parent/relative;digest=sha(p);calls=matched=count=0;rows=[];changes=[];footer=None
    for seq,line in enumerate(p.open()):
        e=json.loads(line);assert e['seq']==seq and footer is None
        kind,data=e['kind'],e['data']
        if seq==0:
            assert kind=='header' and data['schema']==5
            cfg=FollowConfig(**data['follow_config']);ex=HandoffExecutionConfig.from_mapping(data['execution_config']);motion=MotionConfig(**data['motion_config'])
            old=BypassHandoffFollow(cfg,ex,allow_simulation_trial=True);old.normal.pass_selector=legacy.SuspendedPassageSelector(cfg)
            new=BypassHandoffFollow(cfg,ex,allow_simulation_trial=True)
            new._initial_reference.memory=DirectionalPairReference()
            if was_old:new.normal.pass_selector=legacy.SuspendedPassageSelector(cfg)
            else:old.normal.pass_selector=SuspendedPassageSelector(cfg)
        elif kind=='reset':old.reset();new.reset()
        elif kind=='call':
            count+=1;t=data['input']['reference_s']
            if t>16.:continue
            before_old=observe(old);before_new=observe(new)
            a=freeze(replay_call(old,data['input'],motion));v=freeze(replay_call(new,data['input'],motion));calls+=1
            recorded=a;matched+=equal_output(comparable_decision(recorded),comparable_decision(data['output']))
            same=equal_output(comparable_decision(a),comparable_decision(v))
            if not same:changes.append(dict(time_s=t,old_command=a[:2],new_command=v[:2],old_state=a[2].get('handoff_state'),new_state=v[2].get('handoff_state'),normal_same=equal_output(comparable_decision([0,0,a[2].get('normal')]),comparable_decision([0,0,v[2].get('normal')])),old_reason=(a[2].get('normal')or{}).get('reason'),new_reason=(v[2].get('normal')or{}).get('reason')))
            if 11.5<=t<=14.5:
                def row(ctl,value,before):
                    d=value[2]
                    return dict(command=value[:2],reason=d.get('reason'),state=d.get('handoff_state'),normal_reason=(d.get('normal')or{}).get('reason'),persistence=d.get('persistence'),preparation=d.get('preparation'),reserve=d.get('follow_entry_space'),bypass_reason=(d.get('bypass')or{}).get('reason'),before=before,after=observe(ctl))
                rows.append(dict(t=t,speed=data['input']['speed_mps'],old=row(old,a,before_old),new=row(new,v,before_new)))
        elif kind=='footer':footer=data
        else:raise ValueError('unexpected event')
    assert footer and footer['accepted']==footer['written']==seq and not footer['worker_alive']
    assert all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending'))
    name=p.parent.parent.name
    (out/(name+'_observations.jsonl')).write_text(''.join(json.dumps(row,ensure_ascii=False)+'\n' for row in rows))
    return dict(input=p.relative_to(ROOT).as_posix(),input_sha256=digest,input_unchanged=sha(p)==digest,recorded_old_selector=was_old,all_calls=count,prefix_calls=calls,prefix_end_s=16.,recorded_output_same=matched,old_new_changes=changes,command_changes=sum(c['old_command']!=c['new_command'] for c in changes),new_zero_calls=sum(c['new_command'][0]==0<c['old_command'][0] for c in changes),normal_changes=sum(not c['normal_same'] for c in changes),observations_file=name+'_observations.jsonl',passed=calls==matched and sha(p)==digest)
if __name__=='__main__':
    out=BASE/'run1';out.mkdir(exist_ok=False)
    sources=[*BASE.glob('*.py'),*BASE.glob('*.sh'),BASE/'PLAN.md',SNAP]
    sources.extend(p for directory in ('src/arena_autonomy/arena_autonomy','src/arena_vehicle_interface/arena_vehicle_interface') for p in (ROOT/directory).glob('*.py'))
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in sources}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in hashes:z.write(ROOT/p,p)
    results=[compare(*c,out) for c in CASES]
    unchanged=all(sha(ROOT/p)==h for p,h in hashes.items())
    result=dict(passed=unchanged and all(r['passed'] for r in results),source_unchanged=unchanged,source_sha256=hashes,results=results,scope='recorded initial 16s only; directional outer-wall reference candidate, not a new drive')
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],source_unchanged=unchanged,results=[{k:v for k,v in r.items() if k!='old_new_changes'} for r in results])))
    raise SystemExit(0 if result['passed'] else 1)
