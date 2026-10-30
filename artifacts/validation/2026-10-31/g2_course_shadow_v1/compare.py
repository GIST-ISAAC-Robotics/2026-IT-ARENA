"""완주1랩 저장 입력의 현재 G1 재현과 명시 G1/G2 후보 비교. 새 궤적 예측이 아니다."""
import hashlib
import json
import sys
import time
import zipfile
from collections import Counter
from pathlib import Path

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
from arena_autonomy.bypass_entry_follow import BypassExecutionConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig

INPUT=ROOT/'artifacts/validation/2026-10-28/bump_traffic_v1/run4/follow_decisions.jsonl'


def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as stream:
        for block in iter(lambda:stream.read(2**20),b''):h.update(block)
    return h.hexdigest()


def main():
    output=BASE/'run1';output.mkdir(exist_ok=False)
    files=[*BASE.glob('*.py'),*BASE.glob('*.sh'),BASE/'PLAN.md']
    files += [p for directory in ('src/arena_autonomy/arena_autonomy','src/arena_vehicle_interface/arena_vehicle_interface')
              for p in (ROOT/directory).glob('*.py')]
    hashes={str(p.relative_to(ROOT)):sha(p) for p in files};input_hash=sha(INPUT)
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in hashes:archive.write(ROOT/p,p)
    policy=HandoffExecutionConfig(entry_policy='corridor_local_v1',preparation_policy='observed_entry_space_v1',
        persistence_window_policy='trim_before_compare_v1',follow_entry_space_enabled=True,reacquire_policy='known_hold_v1',
        bypass=BypassExecutionConfig(corridor_policy='corridor_v2_signed_hold'))
    calls=matched=normal_same=changed=0;states=Counter();reasons=Counter();prep=Counter();phases=Counter()
    transitions=[];first_change=None;footer=None;cost=[];mismatch_examples=[];normal_examples=[]
    with INPUT.open() as source,(output/'rows.jsonl').open('x') as rows:
        for index,line in enumerate(source):
            event=json.loads(line);assert event['seq']==index and footer is None
            kind,data=event['kind'],event['data']
            if index==0:
                assert kind=='header' and data['schema']==1
                header=data;motion=MotionConfig(**data['motion_config']);config=FollowConfig(**data['follow_config'])
                original=LocalFollow(config);candidate=BypassHandoffFollow(config,policy,allow_simulation_trial=True)
            elif kind=='reset':original.reset();candidate.reset()
            elif kind=='call':
                before=time.perf_counter();old=freeze(replay_call(original,data['input'],motion))
                old_time=time.perf_counter()-before
                before=time.perf_counter();new=freeze(replay_call(candidate,data['input'],motion))
                new_time=time.perf_counter()-before;cost.append([old_time*1000,new_time*1000])
                same=equal_output(comparable_decision(old),comparable_decision(data['output']))
                matched+=same;calls+=1
                if not same and len(mismatch_examples)<5:mismatch_examples.append(dict(seq=index,reference_s=data['input']['reference_s']))
                detail=new[2];same_normal=equal_output(comparable_decision([0.,0.,old[2]]),comparable_decision([0.,0.,detail.get('normal',{})]))
                normal_same+=same_normal
                if not same_normal and len(normal_examples)<5:normal_examples.append(dict(seq=index,reference_s=data['input']['reference_s'],reason=detail.get('reason')))
                different=not equal_output(new[:2],old[:2]);changed+=different
                row=dict(seq=index,reference_s=data['input']['reference_s'],scan_first_s=data['input']['scan']['sec']+data['input']['scan']['nanosec']*1e-9,
                    baseline_command=old[:2],candidate_command=new[:2],measured_speed=data['input']['speed_mps'],state=detail.get('handoff_state'),reason=detail.get('reason'),
                    preparation=detail.get('preparation'),reserve=detail.get('follow_entry_space'),persistence=detail.get('persistence'),
                    bypass_reason=(detail.get('bypass') or {}).get('reason'),entry_phase=(detail.get('bypass') or {}).get('entry_phase'),
                    normal_reason=(detail.get('normal') or {}).get('reason'),baseline_matches=same,normal_same=same_normal,
                    changed=different,processing_ms=dict(baseline=old_time*1000,candidate=new_time*1000))
                if different and first_change is None:first_change=row
                if not transitions or transitions[-1]['state']!=row['state']:transitions.append({k:row[k] for k in ('seq','reference_s','state','reason')})
                states.update([row['state']]);reasons.update([row['reason']]);phases.update([row['entry_phase']])
                prep.update([(row['preparation'] or {}).get('reason')])
                rows.write(json.dumps(row)+'\n')
            elif kind=='footer':footer=data
            else:raise ValueError('unexpected event')
    assert footer is not None and calls>0
    assert all(footer.get(k)==0 for k in ('capture_errors','failed','dropped','pending'))
    assert footer.get('worker_alive') is False and footer['written']==index and footer['accepted']==index
    def percentile(values,q):
        values=sorted(values);return values[round((len(values)-1)*q)]
    result=dict(scope='saved observed trajectory, not a new drive or closed-loop prediction after command divergence',
        input=str(INPUT.relative_to(ROOT)),input_sha256=input_hash,input_unchanged=sha(INPUT)==input_hash,
        source_sha256=hashes,source_unchanged=all(sha(ROOT/p)==h for p,h in hashes.items()),calls=calls,baseline_matched=matched,
        normal_detail_same=normal_same,baseline_mismatch_examples=mismatch_examples,normal_mismatch_examples=normal_examples,
        command_changed=changed,first_command_change=first_change,states=dict(states),reasons=dict(reasons),phases=dict(phases),
        preparation_reasons=dict(prep),transitions=transitions,
        cost_ms={name:dict(p50=percentile([c[i] for c in cost],.5),p95=percentile([c[i] for c in cost],.95),maximum=max(c[i] for c in cost))
                 for i,name in enumerate(('baseline','candidate'))},trace_footer=footer)
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('source_sha256','first_command_change','trace_footer')},ensure_ascii=False),flush=True)
    return 0 if result['input_unchanged'] and result['source_unchanged'] else 1


if __name__=='__main__':raise SystemExit(main())
