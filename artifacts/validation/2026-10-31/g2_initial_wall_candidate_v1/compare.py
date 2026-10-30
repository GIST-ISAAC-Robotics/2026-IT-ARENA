"""현재 반환 재적합의 저장 입력 비교. 실제 대체 궤적의 증명은 아니다."""
import hashlib
import json
from pathlib import Path
import sys
import time
import zipfile
from collections import Counter
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from candidate import CurrentPairFollow
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision


def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def compare(path,output):
    digest=sha(path);calls=baseline_same=command_same=normal_same=0;first=None;footer=None
    states=Counter();reasons=Counter();costs=[];last_state=None;transitions=[];first_changes=[]
    with path.open() as stream,(output/(path.parent.parent.name+'_changes.jsonl')).open('x') as changes:
        for seq,line in enumerate(stream):
            event=json.loads(line);assert event['seq']==seq and footer is None
            kind,data=event['kind'],event['data']
            if seq==0:
                assert kind=='header' and data['schema']==5
                config=FollowConfig(**data['follow_config']);execution=HandoffExecutionConfig.from_mapping(data['execution_config'])
                baseline=BypassHandoffFollow(config,execution,allow_simulation_trial=True)
                candidate=CurrentPairFollow(config,execution,allow_simulation_trial=True)
                motion=MotionConfig(**data['motion_config'])
            elif kind=='reset':baseline.reset();candidate.reset()
            elif kind=='call':
                old=data['output'];actual=freeze(replay_call(baseline,data['input'],motion));calls+=1
                baseline_same+=equal_output(comparable_decision(old),comparable_decision(actual))
                start=time.perf_counter();proposed=freeze(replay_call(candidate,data['input'],motion));costs.append((time.perf_counter()-start)*1000)
                matched=equal_output(old[:2],proposed[:2]);command_same+=matched
                normal_same+=equal_output(comparable_decision([0,0,old[2].get('normal')]),comparable_decision([0,0,proposed[2].get('normal')]))
                state=proposed[2].get('handoff_state');states.update([state])
                ref=freeze(candidate.last_reference);why=ref[1]['reason'] if ref else 'reset';reasons.update([why])
                if state!=last_state:transitions.append(dict(time_s=data['input']['reference_s'],state=state));last_state=state
                if not matched or why=='current_tracked_pair_candidate':
                    row=dict(time_s=data['input']['reference_s'],original_command=old[:2],candidate_command=proposed[:2],
                        reference=ref,candidate_detail=proposed[2],original_state=old[2].get('handoff_state'))
                    changes.write(json.dumps(row)+'\n')
                    if not matched and first is None:first=dict(time_s=row['time_s'],original=old[:2],candidate=proposed[:2])
                    if len(first_changes)<5 and not matched:first_changes.append(row)
            elif kind=='footer':footer=data
            else:raise ValueError('unexpected event')
    assert footer and footer['accepted']==footer['written']==seq and footer['worker_alive'] is False
    assert all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending'))
    ordered=sorted(costs)
    return dict(input=str(path.relative_to(ROOT)),sha256=digest,input_unchanged=sha(path)==digest,calls=calls,
        baseline_same=baseline_same,command_same=command_same,normal_detail_same=normal_same,first_command_change=first,
        states=dict(states),reference_reasons=dict(reasons),transitions=transitions,
        processing_ms={name:ordered[int((len(ordered)-1)*q)] for name,q in [('p50',.5),('p95',.95),('max',1.)]},
        footer_checked=True,scope='recorded input only; changed trajectory is not reproduced')


if __name__=='__main__':
    output=BASE/'run1';output.mkdir(exist_ok=False)
    files=[*BASE.glob('*.py'),*BASE.glob('*.sh'),BASE/'PLAN.md']
    files.extend(p for folder in ('src/arena_autonomy/arena_autonomy','src/arena_vehicle_interface/arena_vehicle_interface') for p in (ROOT/folder).glob('*.py'))
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in files}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name in hashes:archive.write(ROOT/name,name)
    results=[compare(BASE.parent/case/'run1/follow_decisions.jsonl',output)
        for case in ('g2_preparation_hold_gazebo_v1','g2_course_trial_v1')]
    result=dict(results=results,source_sha256=hashes,source_unchanged=all(sha(ROOT/p)==h for p,h in hashes.items()))
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(results=results,source_unchanged=result['source_unchanged'])))
