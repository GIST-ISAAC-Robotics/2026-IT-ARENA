"""r476 상태 prefix98회 한 번, 선택8행 profile/상태복원. 장치 접근 없음."""
from pathlib import Path
import cProfile, hashlib, json, pickle, pstats, resource, sys, time
A=Path(__file__).resolve().parent; S=A/'base/src'
sys.path[:0]=[str(S/'arena_autonomy'),str(S/'arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.follow_trace import equal_output
def digest(value):return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()
def main():
    data=json.loads((A/'profile_inputs.json').read_bytes());before=digest(data)
    selected={tuple(x['key']):x for x in data['selected']};expected={tuple(x['key']):x['output'] for x in data['expected_arm']}
    out=A/'profile_results';out.mkdir();states=A/'states';states.mkdir()
    summaries=[];generation=None;profile_outputs={};plan=Evaluator('plan',data['header'])
    try:
        for index,item in enumerate(data['prefix']):
            key=item['generation'],item['sequence']
            if key[0]!=generation:plan.reset(item['reset_reason']);generation=key[0]
            if key in selected:
                (states/f'{key[0]}_{key[1]}.pickle').write_bytes(pickle.dumps(plan.follow,protocol=5))
                profiler=cProfile.Profile();profiler.enable();output=plan(item['input']);profiler.disable()
                stats=pstats.Stats(profiler);entries=[]
                for (filename,line,function),(primitive,calls,total,cumulative,callers) in stats.stats.items():
                    entries.append(dict(file=filename,line=line,function=function,primitive_calls=primitive,calls=calls,
                        self_ms=1000*total,cumulative_ms=1000*cumulative))
                entries.sort(key=lambda r:r['cumulative_ms'],reverse=True)
                (out/f'{key[0]}_{key[1]}_profile.json').write_text(json.dumps(entries,indent=2)+'\n')
                profiler.dump_stats(str(out/f'{key[0]}_{key[1]}.pstats'));profile_outputs[key]=output
                normal=output[2].get('normal') or {}
                summaries.append(dict(key=list(key),prefix_index=index,reference_s=item['input']['reference_s'],
                    selector_reason=normal.get('selector_reason'),command=output[:2],
                    existing_arm_raw_exact=output==expected[key],existing_arm_comparator=equal_output(output,expected[key]),
                    profile_total_ms=1000*stats.total_tt))
            else:plan(item['input'])
    finally:plan.close()
    plan=Evaluator('plan',data['header']);outputs=[]
    try:
        for row,item in zip(summaries,data['selected']):
            key=tuple(item['key']);plan.follow=pickle.loads((states/f'{key[0]}_{key[1]}.pickle').read_bytes())
            start=time.perf_counter_ns();output=plan(item['input']);ms=(time.perf_counter_ns()-start)/1e6
            row.update(restored_unprofiled_ms=ms,profile_restored_raw_exact=output==profile_outputs[key],
                profile_restored_comparator=equal_output(output,profile_outputs[key]))
            outputs.append(dict(key=list(key),output=output))
    finally:plan.close()
    assert digest(data)==before
    result=dict(rows=summaries,prefix_calls=len(data['prefix']),profiled_calls=len(selected),
        state_restore_check_calls=len(selected),full_sequence_replayed=False,input_unchanged=True,
        state_source='same ARM original sequential prefix; private local generated pickle state only',
        peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        all_profile_restore_raw_exact=all(r['profile_restored_raw_exact'] for r in summaries))
    (out/'reference_outputs.json').write_text(json.dumps(outputs,allow_nan=False)+'\n')
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    assert result['all_profile_restore_raw_exact']
if __name__=='__main__':main()
