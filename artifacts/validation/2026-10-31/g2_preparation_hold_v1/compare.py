"""준비 대기 후보의 고정 입력 비교. 다른 출력 이후의 실제 궤적을 예측하지 않는다."""
import hashlib
import json
from pathlib import Path
import sys
import time
import zipfile
from collections import Counter
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from candidate import PreparationHoldFollow
from arena_autonomy.bypass_handoff import HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision


def sha(path):
    with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def compare(path,end,output):
    before=sha(path);count=same=normal=increased=newzero=retained=0;first=None;footer=None;last=None
    costs=[];held_sources=set();states=Counter()
    with path.open() as stream,(output/(path.parent.parent.name+'_changes.jsonl')).open('x') as changes:
        for index,line in enumerate(stream):
            event=json.loads(line);assert event['seq']==index and footer is None
            kind,data=event['kind'],event['data']
            if index==0:
                assert kind=='header' and data['schema']==5
                obj=PreparationHoldFollow(FollowConfig(**data['follow_config']),HandoffExecutionConfig.from_mapping(data['execution_config']),allow_simulation_trial=True)
                motion=MotionConfig(**data['motion_config'])
            elif kind=='reset':obj.reset()
            elif kind=='call':
                last=data['input']['reference_s']
                if end is not None and last>end:break
                start=time.perf_counter()
                actual=freeze(replay_call(obj,data['input'],motion));costs.append((time.perf_counter()-start)*1000)
                old=data['output'];count+=1;states.update([actual[2].get('handoff_state')])
                commands_same=equal_output(actual[:2],old[:2]);same+=commands_same
                normal+=equal_output(comparable_decision([0,0,actual[2].get('normal')]),comparable_decision([0,0,old[2].get('normal')]))
                increased+=actual[0]>old[0]+1e-10;newzero+=actual[0]==0<old[0]
                detail=actual[2].get('follow_entry_space') or {};held=detail.get('reason')=='bounded_preparation_hold_candidate';retained+=held
                if held:
                    scan=data['input']['scan'];held_sources.add((scan['sec'],scan['nanosec']))
                if not commands_same or held:
                    row=dict(seq=index,reference_s=last,original_command=old[:2],candidate_command=actual[:2],reserve=detail,
                        original_state=old[2].get('handoff_state'),candidate_state=actual[2].get('handoff_state'))
                    changes.write(json.dumps(row)+'\n')
                    if first is None and not commands_same:first=row
            elif kind=='footer':footer=data
            else:raise ValueError('unexpected trace event')
    if end is None:
        assert footer and all(footer.get(k)==0 for k in ('capture_errors','failed','dropped','pending'))
        assert footer.get('worker_alive') is False and footer['written']==index and footer['accepted']==index
    ordered=sorted(costs)
    timing={name:ordered[min(len(ordered)-1,int((len(ordered)-1)*quantile))] for name,quantile in
            [('p50',.5),('p95',.95),('maximum',1.)]} if ordered else {}
    return dict(input=str(path.relative_to(ROOT)),sha256=before,input_unchanged=sha(path)==before,prefix_end_s=end,
        calls=count,command_same=same,normal_detail_same=normal,increased_speed_calls=increased,new_zero_calls=newzero,
        retained_calls=retained,held_distinct_scans=len(held_sources),states=dict(states),processing_ms=timing,
        first_command_change=first,footer_checked=end is None,
        scope='recorded motion/input comparison; not changed closed-loop trajectory or passing proof')


if __name__=='__main__':
    output=BASE/'run1';output.mkdir(exist_ok=False)
    inputs=[(ROOT/'artifacts/validation/2026-10-31/g2_blocked_course_v3/run1/follow_decisions.jsonl',35.),
            (ROOT/'artifacts/validation/2026-10-31/g2_course_trial_v1/run1/follow_decisions.jsonl',None)]
    assert all((p.parent/'report.json').is_file() for p,_ in inputs),'wait for owned simulation to finish first'
    files=[*BASE.glob('*.py'),*BASE.glob('*.sh'),BASE/'PLAN.md']
    files.extend(p for directory in ('src/arena_autonomy/arena_autonomy','src/arena_vehicle_interface/arena_vehicle_interface') for p in (ROOT/directory).glob('*.py'))
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in files}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name in hashes:z.write(ROOT/name,name)
    results=[compare(p,end,output) for p,end in inputs]
    result=dict(results=results,source_sha256=hashes,source_unchanged=all(sha(ROOT/p)==h for p,h in hashes.items()))
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(results=results,source_unchanged=result['source_unchanged'])))
