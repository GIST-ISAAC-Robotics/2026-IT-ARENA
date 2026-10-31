"""기본 OFF의 기존 판단과 ON의 보존된 오프라인 제안을 대조한다."""
from dataclasses import replace
import hashlib,json,sys,zipfile
from pathlib import Path
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision


def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def compare(case):
    path=ROOT/case['input'];digest=sha(path);assert digest==case['sha256']
    expected_path=BASE.parent/'g2_ready_reference_v1/run1'/(path.parent.parent.name+'_changes.jsonl')
    expected={r['time_s']:r for r in map(json.loads,expected_path.open())}
    before_expected=sha(expected_path)
    calls=off_same=command_same=normal_same=state_same=details_same=details_available=0;errors=[];footer=None
    for seq,line in enumerate(path.open()):
        event=json.loads(line);assert event['seq']==seq and footer is None
        kind,data=event['kind'],event['data']
        if seq==0:
            assert kind=='header' and data['schema']==5
            cfg=FollowConfig(**data['follow_config']);execution=HandoffExecutionConfig.from_mapping(data['execution_config'])
            off=BypassHandoffFollow(cfg,execution,allow_simulation_trial=True)
            on=BypassHandoffFollow(cfg,replace(execution,initial_reference_policy='ready_current_pair_v1'),allow_simulation_trial=True)
            motion=MotionConfig(**data['motion_config'])
        elif kind=='reset':off.reset();on.reset()
        elif kind=='call':
            actual=freeze(replay_call(off,data['input'],motion));candidate=freeze(replay_call(on,data['input'],motion));old=data['output']
            now=data['input']['reference_s'];row=expected.get(now);calls+=1
            a=equal_output(comparable_decision(actual),comparable_decision(old));off_same+=a
            b=equal_output(candidate[:2],row['candidate_command'] if row else old[:2]);command_same+=b
            c=equal_output(comparable_decision([0,0,candidate[2].get('normal')]),comparable_decision([0,0,old[2].get('normal')]));normal_same+=c
            state=next(r['state'] for r in reversed(case['transitions']) if r['time_s']<=now)
            d=candidate[2].get('handoff_state')==state;state_same+=d
            e=True
            if row:
                details_available+=1
                e=equal_output(comparable_decision([0,0,candidate[2]]),comparable_decision([0,0,row['candidate_detail']]))
                details_same+=e
            if not all((a,b,c,d,e)) and len(errors)<5:errors.append(dict(time_s=now,off=a,command=b,normal=c,state=d,detail=e))
        elif kind=='footer':footer=data
        else:raise ValueError('unexpected event')
    assert footer and footer['accepted']==footer['written']==seq and footer['worker_alive'] is False
    assert all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending'))
    return dict(input=case['input'],calls=calls,off_same=off_same,on_command_same=command_same,on_normal_same=normal_same,
        on_state_same=state_same,on_details_same=details_same,on_details_available=details_available,first_errors=errors,
        input_unchanged=sha(path)==digest,expected_unchanged=sha(expected_path)==before_expected,
        passed=calls==case['calls']==off_same==command_same==normal_same==state_same and details_same==details_available)


if __name__=='__main__':
    output=BASE/'run2';output.mkdir(exist_ok=False)
    inputs=json.loads((BASE.parent/'g2_ready_reference_v1/run1/summary.json').read_text())['results']
    sources=[*BASE.glob('*.py'),*BASE.glob('*.sh'),BASE/'PLAN.md']
    sources.extend(p for directory in ('src/arena_autonomy/arena_autonomy','src/arena_vehicle_interface/arena_vehicle_interface') for p in (ROOT/directory).glob('*.py'))
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in sources}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in hashes:archive.write(ROOT/p,p)
    results=[compare(case) for case in inputs]
    unchanged=all(sha(ROOT/p)==h for p,h in hashes.items())
    passed=unchanged and all(r['passed'] and r['input_unchanged'] and r['expected_unchanged'] for r in results)
    summary=dict(passed=passed,results=results,source_unchanged=unchanged,source_sha256=hashes)
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(dict(passed=passed,results=results,source_unchanged=unchanged)))
    raise SystemExit(0 if passed else 1)
