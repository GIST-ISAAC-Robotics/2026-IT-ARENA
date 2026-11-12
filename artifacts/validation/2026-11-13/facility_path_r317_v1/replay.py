"""원 궤적 고정 재생. 후보 경로의 새 주행/시간 여유가 실측됐다고 해석하지 않는다."""
from pathlib import Path
import collections,copy,hashlib,json,sys,time
import numpy as np
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(A),str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,replay_reset,freeze,equal_output,comparable_decision
import candidate

p=A.parent/'occluder_pass_drive_r310_v1/run1/follow_decisions.jsonl'
with p.open() as f:
    header=json.loads(next(f))['data'];events=[json.loads(l) for l in f]
summary=dict(source_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
             scope='fixed recorded previous steering and motion; no new trajectory')
baseline=[]
for mode in ('baseline','offset'):
    candidate.uninstall();candidate.prior.uninstall()
    if mode=='baseline':candidate.prior.install()
    else:candidate.install()
    controller,motion=make_controller(copy.deepcopy(header));counts=collections.Counter();times=[];rows=[];different=commands=0;ever_selected=False;prefix=0
    for e in events:
        if e['kind']=='reset':replay_reset(controller,e['data'])
        if e['kind']!='call':continue
        c=e['data'];start=time.perf_counter();out=freeze(replay_call(controller,c['input'],motion));times.append((time.perf_counter()-start)*1000)
        different+=not equal_output(comparable_decision(out),comparable_decision(c['output']))
        commands+=not equal_output(out[:2],c['output'][:2])
        evidence=out[2].get('facility_path_offset') or {}
        ever_selected=ever_selected or evidence.get('applied',False)
        if mode=='offset' and not ever_selected:
            without=[out[0],out[1],{k:v for k,v in out[2].items() if k!='facility_path_offset'}]
            assert equal_output(comparable_decision(without),comparable_decision(c['output'])),c['input']['reference_s']
            prefix+=1
        counts[out[2]['reason']]+=1;counts['positive' if out[0]>0 else 'zero']+=1
        if evidence.get('applied'):counts['path_shift_selected']+=1
        rows.append(dict(time=c['input']['reference_s'],speed=out[0],angle=out[1],reason=out[2]['reason'],
                         boundary_reason=out[2].get('boundary_candidate_reason'),evidence=evidence))
    if mode=='baseline':assert different==0,different
    summary[mode]=dict(calls=len(times),output_differences=different,command_differences=commands,
                      identical_prefix=prefix,counts=dict(counts),
                      cost_ms=dict(median=float(np.median(times)),p95=float(np.percentile(times,95)),maximum=max(times)))
    (A/(mode+'_rows.json')).write_text(json.dumps(rows,separators=(',',':'))+'\n')
    (A/'replay_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(mode,json.dumps(summary[mode]),flush=True)
candidate.uninstall();candidate.prior.uninstall()
