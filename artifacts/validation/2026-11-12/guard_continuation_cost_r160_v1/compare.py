"""r149 원판과 r160 순서 후보를 동일 저장 입력 순서로 대조한다."""
import hashlib,importlib.util,json,time
from pathlib import Path
import numpy as np
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,comparable_decision,equal_output
A=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('r160_trial',A/'candidate.py');c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)
source=A.parent/'guard_current_r143_v1/two_actor_run1/run1/follow_decisions.jsonl'
expected={r['seq']:r['output'] for r in map(json.loads,(A.parent/'adjacent_cost_r150_v1/candidate_expected.jsonl').open())}
events=[json.loads(line) for line in source.open()]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest={p.as_posix():sha(p) for d in ('src/arena_autonomy','src/arena_vehicle_interface') for p in Path(d).rglob('*.py') if '__pycache__' not in p.parts}
manifest.update({str(A/'candidate.py'):sha(A/'candidate.py'),str(source):sha(source)})
outputs={};costs={}
for mode in ('baseline','candidate'):
    c.uninstall();c.prior.install()
    if mode=='candidate':c.install()
    owner,motion=make_controller(events[0]['data']);rows=[];timings=[]
    for event in events[1:-1]:
        if event['kind']=='reset':owner.reset()
        if event['kind']!='call':continue
        started=time.perf_counter_ns();out=replay_call(owner,event['data']['input'],motion);ms=(time.perf_counter_ns()-started)/1e6
        out=comparable_decision(freeze(out));rows.append({'seq':event['seq'],'output':out});timings.append(ms)
        if mode=='baseline':assert equal_output(out,expected[event['seq']]),event['seq']
    outputs[mode]=rows;costs[mode]={'count':len(rows),'p50_ms':float(np.median(timings)),'p95_ms':float(np.percentile(timings,95))}
    with (A/(mode+'_outputs.jsonl')).open('x') as f:
        for row in rows:f.write(json.dumps(row)+'\n')
c.uninstall()
differences=[];counts={'used':0,'stop_to_go':0,'go_to_stop':0,'positive_speed_changed':0}
for b,n in zip(outputs['baseline'],outputs['candidate']):
    assert b['seq']==n['seq'];bo,no=b['output'],n['output']
    normal=no[2].get('normal',no[2]);counts['used']+=bool(normal.get('selector_guard_continuation',{}).get('applied'))
    counts['stop_to_go']+=bo[0]<=0<no[0];counts['go_to_stop']+=no[0]<=0<bo[0]
    counts['positive_speed_changed']+=bo[0]>0 and no[0]>0 and abs(bo[0]-no[0])>1e-10
    if not equal_output(bo[:2],no[:2]):differences.append({'seq':b['seq'],'before':bo[:2],'after':no[:2]})
result={'calls':len(outputs['baseline']),'baseline_all_expected_equal':True,'commands':counts,'differences':differences,'cost':costs,'sources_unchanged':all(sha(Path(p))==h for p,h in manifest.items()),'scope':'saved inputs; full current guard each continuation; new closed loop not run'}
(A/'source_manifest.json').write_text(json.dumps(manifest,indent=2))
(A/'comparison.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
assert result['sources_unchanged']
