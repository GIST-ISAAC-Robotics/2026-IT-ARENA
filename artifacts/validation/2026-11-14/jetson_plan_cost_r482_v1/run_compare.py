"""ARM 동일 상태 8행, 원본/후보 각각 3회. 첫 profile 1행은 warmup으로만 사용."""
from pathlib import Path
import cProfile,hashlib,json,pstats,resource,sys,time
import state_codec
A=Path(__file__).resolve().parent
version,round_text=sys.argv[1:];round_id=int(round_text)
S=A/version/'src'
sys.path[:0]=[str(S/'arena_autonomy'),str(S/'arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy import _numeric_kernels
data=json.loads((A/'profile_inputs.json').read_bytes())
expected={tuple(x['key']):x['output'] for x in data['expected_arm']}
out=A/'compare_results';out.mkdir(exist_ok=True)
plan=Evaluator('plan',data['header']);rows=[];warmup=None
def restore(key):
    plan.follow=state_codec.loads((A/'states_v2'/f'{key[0]}_{key[1]}.pickle').read_bytes())
try:
    if round_id==0:
        item=next(x for x in data['selected'] if x['key'][1]==229);key=tuple(item['key']);restore(key)
        profile=cProfile.Profile();profile.enable();value=plan(item['input']);profile.disable()
        stats=pstats.Stats(profile);entries=[]
        for (file,line,fn),(primitive,calls,total,cumulative,_) in stats.stats.items():
            entries.append(dict(file=file,line=line,function=fn,calls=calls,self_ms=total*1000,cumulative_ms=cumulative*1000))
        entries.sort(key=lambda x:x['cumulative_ms'],reverse=True)
        (out/f'{version}_warmup_profile.json').write_text(json.dumps(entries,indent=2)+'\n')
        warmup=dict(key=list(key),raw_exact=value==expected[key],included_in_timing=False)
        assert warmup['raw_exact']
    for item in data['selected']:
        key=tuple(item['key']);restore(key)
        start=time.perf_counter_ns();value=plan(item['input']);elapsed=(time.perf_counter_ns()-start)/1e6
        rows.append(dict(key=list(key),wall_ms=elapsed,raw_exact=value==expected[key],output=value))
finally:plan.close()
result=dict(version=version,round=round_id,rows=rows,warmup=warmup,
    native_path=_numeric_kernels.__file__,native_sha256=hashlib.sha256(Path(_numeric_kernels.__file__).read_bytes()).hexdigest(),
    peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
(out/f'{version}_{round_id}.json').write_text(json.dumps(result,allow_nan=False)+'\n')
print(json.dumps(dict(version=version,round=round_id,ms=[r['wall_ms'] for r in rows],exact=all(r['raw_exact'] for r in rows))),flush=True)
assert all(r['raw_exact'] for r in rows)
