import hashlib,importlib.util,json,sys
from pathlib import Path
root=Path(sys.argv[1]);A=root/'artifacts/validation/2026-11-12/guard_continuation_cost_r160_v1';old=A.parent/'adjacent_cost_r150_v1'
spec=importlib.util.spec_from_file_location('measure_r150',old/'measure.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
spec=importlib.util.spec_from_file_location('candidate_r160',A/'candidate.py');c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)
manifest=json.loads((old/'bundle_manifest.json').read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
unchanged=lambda:all(sha(root/name)==row['sha256'] for name,row in manifest['files'].items())
assert unchanged()
expected={r['seq']:r['output'] for r in map(json.loads,(old/'jetson_audited_candidate_outputs.jsonl').open())}
events=[json.loads(line) for line in (A.parent/'guard_current_r143_v1/two_actor_run1/run1/follow_decisions.jsonl').open()]
c.install();owner,motion=m.make_controller(events[0]['data']);rows=[];bad=[];choices=[];used=0
with (A/'jetson_outputs.jsonl').open('x') as output:
    for event in events[1:-1]:
        if event['kind']=='reset':owner.reset()
        if event['kind']!='call':continue
        out,cost=m.timed(owner,event['data']['input'],motion);out=m.comparable_decision(m.freeze(out));ref=expected[event['seq']]
        if not m.equal_output(out[:2],ref[:2]):bad.append(event['seq'])
        n=out[2].get('normal',out[2]);b=ref[2].get('normal',ref[2]);applied=bool((n.get('selector_guard_continuation') or {}).get('applied'));used+=applied
        keys=('reason','behavior','selector_choice','selector_reason')
        if any(n.get(k)!=b.get(k) for k in keys):choices.append(event['seq'])
        output.write(json.dumps({'seq':event['seq'],'output':out})+'\n')
        rows.append(dict(seq=event['seq'],continuation=applied,positive=out[0]>0,**cost))
result={'calls':len(rows),'used':used,'command_mismatches':bad,'choice_reason_mismatches':choices,'source_unchanged':unchanged(),'candidate_sha256':sha(A/'candidate.py'),'groups':{name:{k:m.dist([r[k] for r in subset]) for k in ('restore_ms','projection_ms','decision_ms','total_ms')} for name,subset in [('all',rows),('continuation',[r for r in rows if r['continuation']]),('positive',[r for r in rows if r['positive']])]},'scope':'One CPU run; compare against prior r158 same-source outputs; diagnostics search trace intentionally differs; no ROS/live drive'}
with (A/'jetson_costs.jsonl').open('x') as f:
    for row in rows:f.write(json.dumps(row)+'\n')
(A/'jetson_summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));assert not bad and not choices and result['source_unchanged']
