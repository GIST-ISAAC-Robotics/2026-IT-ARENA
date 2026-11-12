"""r150 동결 입력의 첫 플랫폼 불일치 진단. 출력/판정 허용치를 바꾸지 않는다."""
import importlib.util,json,platform,sys,hashlib
from pathlib import Path
root=Path(sys.argv[1]); a=root/'artifacts/validation/2026-11-12/adjacent_cost_r150_v1'
spec=importlib.util.spec_from_file_location('r150_measure',a/'measure.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
trace=a.parent/'guard_current_r143_v1/two_actor_run1/run1/follow_decisions.jsonl'
events=[json.loads(line) for line in trace.open()]
m.candidate.prior.prior.install();owner,motion=m.make_controller(events[0]['data'])
def differences(a,b,path=''):
    if m.equal_output(a,b): return []
    if isinstance(a,dict) and isinstance(b,dict):
        rows=[]
        for key in sorted(a.keys()|b.keys()):
            if key not in a or key not in b: rows.append({'path':path+'/'+key,'a':a.get(key),'b':b.get(key)})
            else: rows.extend(differences(a[key],b[key],path+'/'+key))
        return rows
    if isinstance(a,list) and isinstance(b,list) and len(a)==len(b):
        return [r for i,(x,y) in enumerate(zip(a,b)) for r in differences(x,y,path+'/'+str(i))]
    return [{'path':path,'actual':a,'expected':b}]
rows=[];calls=0
for event in events[1:-1]:
    if event['kind']=='reset':owner.reset()
    if event['kind']!='call':continue
    calls+=1;out,cost=m.timed(owner,event['data']['input'],motion)
    out=m.comparable_decision(m.freeze(out));ref=m.comparable_decision(event['data']['output'])
    if not m.equal_output(out,ref):
        rows.append({'seq':event['seq'],'commands_equal':m.equal_output(out[:2],ref[:2]),'differences':differences(out,ref),'cost':cost})
        if len(rows)>=3:break
result={'calls':calls,'mismatches':rows,'numpy':m.np.__version__,'python':platform.python_version(),'platform':platform.platform(),'trace_sha256':hashlib.sha256(trace.read_bytes()).hexdigest(),'scope':'prefix diagnostic; not full equivalence or timing pass'}
with (a/'jetson_alignment_prefix.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({'calls':calls,'first_seq':rows[0]['seq'] if rows else None,'mismatches':len(rows),'command_changes':sum(not r['commands_equal'] for r in rows)}))
