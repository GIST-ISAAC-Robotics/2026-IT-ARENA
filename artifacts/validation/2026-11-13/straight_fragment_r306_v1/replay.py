"""r294/r304 고정 입력 대조. 새 궤적/실시간 성능 증명이 아니다."""
from pathlib import Path
import json,sys,collections,copy
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(A),str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,replay_reset,freeze,equal_output,comparable_decision
import candidate
summary={};details={}
for label,source in [('r294','2026-11-12/cad_facility_lap_r294_v1'),('r304','2026-11-13/facility_geometry_r304_v1')]:
    with (R/'artifacts/validation'/source/'run1/follow_decisions.jsonl').open() as f:
        header=json.loads(next(f))['data'];events=[json.loads(l) for l in f]
    summary[label]={}
    for mode in ('common_baseline','straight_fragment'):
        if mode=='straight_fragment':candidate.install()
        else:candidate.uninstall()
        follow,motion=make_controller(copy.deepcopy(header));changed=[];counts=collections.Counter();rows=[];n=0
        for e in events:
            if e['kind']=='reset':replay_reset(follow,e['data'])
            if e['kind']!='call':continue
            c=e['data'];actual=freeze(replay_call(follow,c['input'],motion));n+=1
            if not equal_output(comparable_decision(actual),comparable_decision(c['output'])):
                changed.append(c['input']['reference_s'])
                rows.append(dict(reference_s=c['input']['reference_s'],before=c['output'],after=actual))
            counts['positive' if actual[0]>0 else 'zero']+=1;counts[actual[2].get('reason')]+=1
            shape=actual[2].get('fragment_hypothesis',{}).get('shape')
            if shape:counts['shape_'+shape]+=1
        if mode=='common_baseline':assert not changed
        summary[label][mode]=dict(calls=n,different=len(changed),first_difference=changed[0] if changed else None,counts=dict(counts))
        details[label+'/'+mode]=rows
        print(json.dumps(dict(source=label,mode=mode,**summary[label][mode])),flush=True)
(A/'replay_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(A/'replay_changes.json').write_text(json.dumps(details,separators=(',',':'))+'\n')
