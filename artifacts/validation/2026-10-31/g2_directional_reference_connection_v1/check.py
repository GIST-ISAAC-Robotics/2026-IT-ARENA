"""등록 명령 보존과 새 연결의 기존 오프라인 후보 일치를 검사한다."""
from dataclasses import replace
import hashlib,json,zipfile,time
from pathlib import Path
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def run(path,expected,*,prefix=None,expected_sha=None):
    digest=sha(path)
    if expected_sha:assert digest==expected_sha
    calls=matched=0;footer=None;changes=[];cost=[];last_state=None;transitions=[]
    for seq,line in enumerate(path.open()):
        e=json.loads(line);assert e['seq']==seq and footer is None
        kind,data=e['kind'],e['data']
        if seq==0:
            assert kind=='header' and data['schema']==5
            ex=replace(HandoffExecutionConfig.from_mapping(data['execution_config']),initial_reference_policy='ready_directional_pair_v2')
            ctl=BypassHandoffFollow(FollowConfig(**data['follow_config']),ex,allow_simulation_trial=True)
            motion=MotionConfig(**data['motion_config'])
        elif kind=='reset':ctl.reset()
        elif kind=='call':
            t=data['input']['reference_s']
            if prefix is not None and t>prefix:continue
            start=time.perf_counter();value=freeze(replay_call(ctl,data['input'],motion));cost.append((time.perf_counter()-start)*1000)
            state=value[2].get('handoff_state')
            if state!=last_state:transitions.append(dict(time_s=t,state=state));last_state=state
            row=expected.get(t)
            if prefix is not None and row is None:continue
            target=row['candidate_command'] if row else data['output'][:2]
            same=equal_output(value[:2],target)
            if row and 'state' in row:same=same and state==row['state']
            calls+=1;matched+=same
            if not same:changes.append(dict(time_s=t,expected=target,actual=value[:2],state=state,reason=value[2].get('reason')))
        elif kind=='footer':footer=data
        else:raise ValueError('unexpected event')
    assert footer and footer['accepted']==footer['written']==seq and not footer['worker_alive']
    assert all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending'))
    ordered=sorted(cost)
    return dict(input=path.relative_to(ROOT).as_posix(),input_sha256=digest,input_unchanged=sha(path)==digest,
        compared_calls=calls,commands_same=matched,changes=changes,transitions=transitions,
        processing_ms={k:ordered[int((len(ordered)-1)*q)] for k,q in [('p50',.5),('p95',.95),('max',1.)]},
        footer_checked=True,passed=calls==matched and sha(path)==digest)
if __name__=='__main__':
    out=BASE/'run1';out.mkdir(exist_ok=False)
    prior=BASE.parent/'g2_ready_reference_v1/run1'
    case=next(r for r in json.loads((prior/'summary.json').read_text())['results'] if 'g2_course_trial_v1' in r['input'])
    expected_path=prior/'g2_course_trial_v1_changes.jsonl'
    expected={r['time_s']:r for r in map(json.loads,expected_path.open())}
    prefix_path=BASE.parent/'g2_directional_reference_v1/run1/g1_same_scan_history_gazebo_v1_observations.jsonl'
    prefix_expected={r['t']:dict(candidate_command=r['new']['command'],state=r['new']['state']) for r in map(json.loads,prefix_path.open())}
    files=[*BASE.glob('*.py'),*BASE.glob('*.sh'),BASE/'PLAN.md',expected_path,prefix_path,prior/'summary.json']
    files.extend(p for directory in ('src/arena_autonomy/arena_autonomy','src/arena_vehicle_interface/arena_vehicle_interface') for p in (ROOT/directory).glob('*.py'))
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in files}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in hashes:z.write(ROOT/p,p)
    results=[run(BASE.parent/'g1_same_scan_history_gazebo_v1/run1/follow_decisions.jsonl',prefix_expected,prefix=14.5),
        run(ROOT/case['input'],expected,expected_sha=case['sha256'])]
    unchanged=all(sha(ROOT/p)==h for p,h in hashes.items())
    passed=unchanged and all(r['passed'] for r in results) and results[1]['compared_calls']==case['calls']
    result=dict(passed=passed,source_unchanged=unchanged,source_sha256=hashes,results=results,
        scope='saved inputs; cached prior verified commands; not a new trajectory')
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=passed,source_unchanged=unchanged,results=results)))
    raise SystemExit(0 if passed else 1)
