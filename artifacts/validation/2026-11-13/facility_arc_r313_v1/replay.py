"""r310 실행 그대로의 입력. 후보 조향 뒤 새 차체 궤적을 예측한 시험이 아니다."""
from pathlib import Path
import collections,copy,hashlib,json,sys,time
import numpy as np
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(A),str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,replay_reset,freeze,equal_output,comparable_decision
import candidate

p=R/'artifacts/validation/2026-11-13/occluder_pass_drive_r310_v1/run1/follow_decisions.jsonl'
with p.open() as f:
    header=json.loads(next(f))['data'];events=[json.loads(l) for l in f]
summary=dict(source_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),scope='fixed input including recorded previous steering; no new vehicle trajectory')
for mode in ('baseline','nearby'):
    candidate.uninstall()
    if mode=='baseline':candidate.prior.install()
    else:candidate.install()
    controller,motion=make_controller(copy.deepcopy(header));counts=collections.Counter();times=[];rows=[];different=commands=0;ever_selected=False
    for e in events:
        if e['kind']=='reset':replay_reset(controller,e['data'])
        if e['kind']!='call':continue
        c=e['data'];start=time.perf_counter();out=freeze(replay_call(controller,c['input'],motion));times.append((time.perf_counter()-start)*1000)
        different+=not equal_output(comparable_decision(out),comparable_decision(c['output']))
        commands+=not equal_output(out[:2],c['output'][:2])
        evidence=out[2].get('nearby_steering_candidate') or {}
        ever_selected=ever_selected or evidence.get('applied',False)
        if mode=='nearby' and not ever_selected:
            without_diagnostic=[out[0],out[1],{k:v for k,v in out[2].items() if k!='nearby_steering_candidate'}]
            assert equal_output(comparable_decision(without_diagnostic),comparable_decision(c['output'])),c['input']['reference_s']
        counts[out[2]['reason']]+=1;counts['positive' if out[0]>0 else 'zero']+=1
        if evidence.get('applied'):counts['nearby_selected']+=1
        rows.append(dict(time=c['input']['reference_s'],speed=out[0],angle=out[1],reason=out[2]['reason'],evidence=evidence))
    if mode=='baseline':assert different==0,different
    summary[mode]=dict(calls=len(times),output_differences=different,command_differences=commands,counts=dict(counts),cost_ms=dict(median=float(np.median(times)),p95=float(np.percentile(times,95)),maximum=max(times)))
    (A/(mode+'_rows.json')).write_text(json.dumps(rows,separators=(',',':'))+'\n')
    (A/'replay_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(mode,json.dumps(summary[mode]),flush=True)
candidate.uninstall()
