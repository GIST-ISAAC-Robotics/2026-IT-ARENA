"""고정 입력에서 원 실행 재현과 한 번 순회 후보의 판단/비용 비교."""
from pathlib import Path
import collections,copy,hashlib,json,sys,time
import numpy as np
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(A),str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,replay_reset,freeze,equal_output,comparable_decision
import candidate

summary={}
for label,relative,has_straight in [('r308','2026-11-13/straight_fragment_drive_r308_v1',True),('r304','2026-11-13/facility_geometry_r304_v1',False),('r294','2026-11-12/cad_facility_lap_r294_v1',False)]:
    trace=R/'artifacts/validation'/relative/'run1/follow_decisions.jsonl'
    with trace.open() as f:
        header=json.loads(next(f))['data'];events=[json.loads(l) for l in f]
    summary[label]={'trace_sha256':hashlib.sha256(trace.read_bytes()).hexdigest()}
    for mode in ('recorded_baseline','one_pass'):
        candidate.uninstall()
        if mode=='one_pass':candidate.install()
        elif has_straight:candidate.straight.install()
        controller,motion=make_controller(copy.deepcopy(header))
        counts=collections.Counter();times=[];changed=[];rows=[]
        for e in events:
            if e['kind']=='reset':replay_reset(controller,e['data'])
            if e['kind']!='call':continue
            call=e['data'];start=time.perf_counter()
            result=freeze(replay_call(controller,call['input'],motion));times.append((time.perf_counter()-start)*1000)
            different=not equal_output(comparable_decision(result),comparable_decision(call['output']))
            if different:changed.append(call['input']['reference_s'])
            counts['positive' if result[0]>0 else 'zero']+=1;counts[result[2].get('reason')]+=1
            rows.append(dict(time=call['input']['reference_s'],speed=result[0],steering=result[1],reason=result[2].get('reason'),different=different))
        if mode=='recorded_baseline':assert not changed,(label,changed[:5])
        entry=dict(calls=len(times),different=len(changed),first_difference=changed[0] if changed else None,
                   counts=dict(counts),decision_ms=dict(median=float(np.median(times)),p95=float(np.percentile(times,95)),max=max(times)))
        summary[label][mode]=entry
        (A/(label+'_'+mode+'_rows.json')).write_text(json.dumps(rows,separators=(',',':'))+'\n')
        (A/'replay_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps(dict(source=label,mode=mode,**entry)),flush=True)
candidate.uninstall()
