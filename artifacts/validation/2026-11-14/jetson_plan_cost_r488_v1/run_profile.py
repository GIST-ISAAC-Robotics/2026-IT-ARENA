"""최신 공통의 동일 저장 상태4개 profile/비계측 각1회. 실제 장치 입력 없음."""
from pathlib import Path
import cProfile,hashlib,json,pstats,resource,sys,time
import state_codec
A=Path(__file__).resolve().parent;S=A/'base/src'
sys.path[:0]=[str(S/'arena_autonomy'),str(S/'arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
data=json.loads((A/'inputs.json').read_bytes());out=A/'profile_results';out.mkdir()
expected={tuple(x['key']):x['output'] for x in data['expected_r482']};rows=[]
sha=lambda x:hashlib.sha256(json.dumps(x,sort_keys=True,allow_nan=False).encode()).hexdigest()
plan=Evaluator('plan',data['header'])
def restore(key):plan.follow=state_codec.loads((A/'states'/f'{key[0]}_{key[1]}.pickle').read_bytes())
try:
    for item in data['selected']:
        key=tuple(item['key']);before=sha(item['input']);restore(key)
        profiler=cProfile.Profile();profiler.enable();profile_output=plan(item['input']);profiler.disable()
        stats=pstats.Stats(profiler);entries=[]
        for (file,line,function),(primitive,calls,total,cumulative,_) in stats.stats.items():
            entries.append(dict(file=file,line=line,function=function,calls=calls,self_ms=1000*total,cumulative_ms=1000*cumulative))
        entries.sort(key=lambda r:r['cumulative_ms'],reverse=True)
        (out/f'{key[0]}_{key[1]}_profile.json').write_text(json.dumps(entries,indent=2)+'\n')
        restore(key);start=time.perf_counter_ns();value=plan(item['input']);elapsed=(time.perf_counter_ns()-start)/1e6
        normal=value[2].get('normal') or {}
        rows.append(dict(key=list(key),wall_ms=elapsed,profile_total_ms=stats.total_tt*1000,
            profile_wall_exact=profile_output==value,r482_raw_exact=value==expected[key],input_unchanged=sha(item['input'])==before,
            output=value,old_r482_command=expected[key][:2],command=value[:2],selector_reason=normal.get('selector_reason'),
            reason=normal.get('reason'),partial_reason=(normal.get('partial_observation') or {}).get('reason')))
        print(json.dumps({k:rows[-1][k] for k in ('key','wall_ms','profile_wall_exact','r482_raw_exact','command','selector_reason')}),flush=True)
finally:plan.close()
result=dict(rows=rows,profile_calls=4,wall_calls=4,prefix_replayed=False,source='same saved r476 pre-call state used by r482; newest common code',
    peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
    passed=all(r['profile_wall_exact'] and r['input_unchanged'] for r in rows))
(out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
raise SystemExit(0 if result['passed'] else 1)
