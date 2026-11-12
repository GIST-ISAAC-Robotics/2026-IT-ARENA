"""동일한 저장 숫자에 시각 출처/선택만 바꾼 계약 대조. 실물 시각 오차 시험 아님."""
import copy
import hashlib
import json
from pathlib import Path
import time

from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, replay_reset, equal_output, freeze
from arena_autonomy.recheck_process import evaluate_recheck
from arena_autonomy.numeric_kernels import configure

A=Path(__file__).resolve().parent;ROOT=A.parents[3]


def estimated(data):
    result=copy.deepcopy(data)
    assert result['scan']['representation']=='native_measurements_v1'
    result['scan']['measurements']['time_basis']='unverified_estimate'
    result['timing_verified']=False
    result['timing_policy']='host_estimated_v1'
    return result


def main():
    trace=ROOT/'artifacts/validation/2026-11-12/traffic_solo_gap_runtime_r225_v1/two_actor_run1/run1/follow_decisions.jsonl'
    digest=hashlib.sha256(trace.read_bytes()).hexdigest()
    events=[json.loads(x) for x in trace.open()]
    header=events[0]['data'];configure('native_v1')
    original,motion=make_controller(header);candidate,_=make_controller(header)
    differences=[];plans=currents=positive=0;began=time.monotonic()
    for event in events[1:]:
        d=event['data']
        if event['kind']=='reset':
            replay_reset(original,d);replay_reset(candidate,d)
        elif event['kind']=='call':
            changed=estimated(d['input'])
            a=freeze(replay_call(original,d['input'],motion))
            b=freeze(replay_call(candidate,changed,motion));plans+=1;positive+=b[0]>0
            if not equal_output(a,b):differences.append(['plan',event['seq']])
            if d.get('current_rebase'):
                current=d['current_rebase']['input']
                a2=evaluate_recheck(a,d['input'],current,motion,original.config)
                b2=evaluate_recheck(b,changed,estimated(current),motion,candidate.config)
                currents+=1
                if not equal_output(a2,b2):differences.append(['current',event['seq']])
    assert hashlib.sha256(trace.read_bytes()).hexdigest()==digest
    result=dict(trace_sha256=digest,plans=plans,current_checks=currents,positive_candidates=positive,
        differences=differences,elapsed_s=time.monotonic()-began,
        numeric_source_values_unchanged=True,hardware_timing_verified=False,
        scope='same stored numeric values, explicit estimated provenance only; no physical error or closed-loop proof')
    (A/'replay_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    assert not differences


if __name__=='__main__':main()
