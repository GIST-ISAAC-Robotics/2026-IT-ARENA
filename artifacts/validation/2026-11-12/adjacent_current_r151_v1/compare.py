"""r149 변경 명령의 다음 저장 입력 재검사. 실제 도착/진행 증명이 아니다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

import current
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call, freeze, equal_output
from arena_autonomy.scene_motion import motion_scene

A=Path(__file__).resolve().parent
OLD=current.load('r151_old_current',A.parent/'guard_current_r142_v1/candidate.py')
TRACE=A.parent/'guard_current_r143_v1/two_actor_run1/run1/follow_decisions.jsonl'


def main():
    digest=hashlib.sha256(TRACE.read_bytes()).hexdigest()
    events=[json.loads(line) for line in TRACE.open()]
    candidates={r['seq']:r['output'] for r in map(json.loads,(A.parent/'adjacent_cost_r150_v1/candidate_expected.jsonl').open())}
    owner,motion=make_controller(events[0]['data'])
    calls=[e for e in events if e['kind']=='call']
    rows=[];reasons=Counter();fixtures=[]
    for i,event in enumerate(calls[:-1]):
        output=candidates[event['seq']]
        if not current.supported(output):continue
        later=calls[i+1]
        if any(e['kind']=='reset' for e in events[event['seq']+1:later['seq']]):continue
        inputs,scan,history=restore_call(later['data']['input'],motion)
        scene=motion_scene(scan,history,inputs['reference_s'],timing_verified=inputs['timing_verified'],
            epoch=inputs['epoch'],rear_half_angle_deg=inputs['rear_half_angle_deg'])
        before=OLD.rebase(output,event['data']['input'],inputs,scan,scene,history,owner.config)
        began=time.perf_counter()
        after=current.rebase(output,event['data']['input'],inputs,scan,scene,history,owner.config)
        elapsed=1000*(time.perf_counter()-began)
        assert after is not None and after['speed_mps']<=output[0]
        assert after['reference_s']==inputs['reference_s']
        old_policy=(output[2]['normal']['selector_guard_candidate'].get('same_path_gap') or {}).get('policy')
        if old_policy!='guard_adjacent_structure_gap_r149':assert equal_output(freeze(before),freeze(after))
        if after['speed_mps']>0:
            assert after['region']['masked_front_samples']==0 and after['region']['masked_tail_extent_m']<=.01
            assert after['independent_guard_required'] and not after['runtime_authorized']
        changed=before['speed_mps']<=0<after['speed_mps']
        if changed and len(fixtures)<2:
            fixtures.append(dict(header=events[0]['data'],seq=event['seq'],original=dict(output=output,input=event['data']['input']),
                current=later['data']['input'],output=freeze(after)))
        reasons[after['reason']]+=1
        rows.append(dict(seq=event['seq'],next_seq=later['seq'],original_speed=output[0],policy=old_policy,
            before=freeze(before),after=freeze(after),cost_ms=elapsed,zero_to_positive=changed))
    summary=dict(calls=len(rows),reasons=dict(reasons),zero_to_positive=sum(r['zero_to_positive'] for r in rows),
        positive_to_zero=sum(r['before']['speed_mps']>0>=r['after']['speed_mps'] for r in rows),
        current_positive=sum(r['after']['speed_mps']>0 for r in rows),
        trace_sha256=digest,trace_unchanged=hashlib.sha256(TRACE.read_bytes()).hexdigest()==digest,
        scope='Next saved submission input. Candidate commands do not modify later sensors. No actual delivery/control.')
    assert summary['trace_unchanged']
    with (A/'comparison.json').open('x') as f:json.dump(summary,f,indent=2)
    with (A/'details.jsonl').open('x') as f:
        for row in rows:f.write(json.dumps(row)+'\n')
    with (A/'fixtures.json').open('x') as f:json.dump(fixtures,f,indent=2)
    print(json.dumps(summary))


if __name__=='__main__':main()
