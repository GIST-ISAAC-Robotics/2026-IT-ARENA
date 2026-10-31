"""현재 실행본은 그대로 두고 선택기 후보만 저장 입력에 대조한다."""
import json,hashlib,zipfile
from pathlib import Path
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from candidate import SameScanHistoryCandidate
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
CASES=[('g2_ready_reference_course_v1/run2/follow_decisions.jsonl',74.5),('g2_course_trial_v1/run1/follow_decisions.jsonl',None)]
def sha(path):
    with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def compare(relative,end):
    path=BASE.parent/relative;digest=sha(path);calls=replayed=matched=same=up=down=0;changes=[];footer=None
    for seq,line in enumerate(path.open()):
        event=json.loads(line);assert event['seq']==seq and footer is None
        kind,data=event['kind'],event['data']
        if seq==0:
            assert kind=='header' and data['schema']==5
            cfg=FollowConfig(**data['follow_config']);ex=HandoffExecutionConfig.from_mapping(data['execution_config'])
            old=BypassHandoffFollow(cfg,ex,allow_simulation_trial=True);new=BypassHandoffFollow(cfg,ex,allow_simulation_trial=True)
            assert type(new.normal.pass_selector).__name__=='SuspendedPassageSelector'
            new.normal.pass_selector=SameScanHistoryCandidate(cfg);motion=MotionConfig(**data['motion_config'])
        elif kind=='reset':old.reset();new.reset()
        elif kind=='call':
            calls+=1;now=data['input']['reference_s']
            if end is not None and now>end:continue
            a=freeze(replay_call(old,data['input'],motion));v=freeze(replay_call(new,data['input'],motion));replayed+=1
            matched+=equal_output(comparable_decision(a),comparable_decision(data['output']))
            equal=equal_output(comparable_decision(a),comparable_decision(v));same+=equal
            up+=v[0]>a[0]+1e-9;down+=v[0]<a[0]-1e-9
            if not equal:
                detail=v[2].get('normal',{});before=a[2].get('normal',{})
                changes.append(dict(time_s=now,original_command=a[:2],candidate_command=v[:2],original_state=a[2].get('handoff_state'),candidate_state=v[2].get('handoff_state'),original_reason=before.get('reason'),candidate_reason=detail.get('reason'),selector_reason=detail.get('selector_reason'),suspended=detail.get('suspended_passage_history'),restored=detail.get('restored_passage_history')))
        elif kind=='footer':footer=data
        else:raise ValueError('unexpected event')
    assert footer and footer['accepted']==footer['written']==seq and not footer['worker_alive']
    assert all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending'))
    return dict(input=path.relative_to(ROOT).as_posix(),input_sha256=digest,input_unchanged=sha(path)==digest,calls=calls,replayed_calls=replayed,replay_end_s=end,baseline_same=matched,candidate_full_same=same,speed_increases=up,speed_decreases=down,changes=changes,passed=matched==replayed and sha(path)==digest)

if __name__=='__main__':
    out=BASE/'run1';out.mkdir(exist_ok=False)
    sources=[*BASE.glob('*.py'),*BASE.glob('*.sh'),BASE/'PLAN.md',BASE/'parent.json']
    sources.extend(p for directory in ('src/arena_autonomy/arena_autonomy','src/arena_vehicle_interface/arena_vehicle_interface') for p in (ROOT/directory).glob('*.py'))
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in sources}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in hashes:z.write(ROOT/p,p)
    results=[compare(*case) for case in CASES]
    unchanged=all(sha(ROOT/p)==h for p,h in hashes.items())
    summary=dict(passed=unchanged and all(r['passed'] for r in results),results=results,source_unchanged=unchanged,source_sha256=hashes,scope='saved sensor inputs; candidate commands are not new trajectory/safety evidence')
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=summary['passed'],source_unchanged=unchanged,results=[{k:v for k,v in r.items() if k!='changes'} for r in results])))
    raise SystemExit(0 if summary['passed'] else 1)
