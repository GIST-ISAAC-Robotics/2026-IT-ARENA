"""r493 순차 plan 508개 중 고정한 3호출만 cProfile. 새 처리율 시험 아님."""
import cProfile
import hashlib
import json
import pstats
import sys
import time
from pathlib import Path

A = Path(__file__).resolve().parent
SOURCE = Path('/home/q/it-arena-r493-a')
S = SOURCE / 'src'
sys.path[:0] = [str(S/'arena_autonomy'), str(S/'arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.follow_trace import equal_output


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def main():
    selection = json.loads((A/'selection.json').read_bytes())
    targets = {tuple(x['key']): x for x in selection['targets']}
    raw = (SOURCE/'inputs.json').read_bytes()
    data = json.loads(raw)
    expected = {tuple(x['key']): x['output'] for x in
        (json.loads(line) for line in (SOURCE/'results/outputs.jsonl').open())}
    assert len(data['plans']) == 508
    evaluator = Evaluator('plan', data['header'])
    generation = None
    rows, profiles, resets = [], [], []
    try:
        for index, item in enumerate(data['plans']):
            key = item['generation'], item['sequence']
            if key[0] != generation:
                evaluator.reset(item['reset_reason'])
                resets.append(dict(index=index, generation=key[0], reason=item['reset_reason']))
                generation = key[0]
            before = json.dumps(item['input'], sort_keys=True, allow_nan=False)
            profile = cProfile.Profile() if key in targets else None
            if profile is not None:
                started = time.perf_counter_ns()
                profile.enable()
            try:
                value = evaluator(item['input'])
            finally:
                if profile is not None:
                    profile.disable()
                    profiled_wall_ms = (time.perf_counter_ns()-started)/1e6
            assert before == json.dumps(item['input'], sort_keys=True, allow_nan=False)
            old = expected[key]
            row = dict(index=index, key=key, reference_s=item['input']['reference_s'],
                profiled=profile is not None, command_equal_r493=equal_output(value[:2], old[:2]),
                output_equal_r493=equal_output(value, old), strict_output_equal_r493=value == old)
            rows.append(row)
            if profile is not None:
                name = f'profile_{key[0]}_{key[1]}'
                profile.dump_stats(str(A/'results'/f'{name}.pstats'))
                stats = pstats.Stats(profile)
                functions = []
                for (file, line, function), (primitive, calls, own, cumulative, callers) in stats.stats.items():
                    functions.append(dict(file=file, line=line, function=function,
                        primitive_calls=primitive, calls=calls, own_ms=own*1000,
                        cumulative_ms=cumulative*1000,
                        callers=[dict(file=f, line=n, function=fn, cost=cost)
                            for (f,n,fn),cost in callers.items()]))
                result = dict(**row, original_unprofiled_r493_ms=targets[key]['ms'],
                    profiled_wall_ms_not_throughput=profiled_wall_ms,
                    profiler_total_ms=stats.total_tt*1000, total_calls=stats.total_calls,
                    output=value, top_cumulative=sorted(functions,key=lambda x:x['cumulative_ms'],reverse=True)[:30],
                    top_own=sorted(functions,key=lambda x:x['own_ms'],reverse=True)[:30], functions=functions,
                    scope='Profiler overhead included; function attribution only, not new processing-rate evidence.')
                dump(A/'results'/f'{name}.json', result)
                profiles.append({k:v for k,v in result.items() if k not in ('output','top_cumulative','top_own','functions')})
            elif not row['output_equal_r493']:
                dump(A/'results'/f'mismatch_{key[0]}_{key[1]}.json', dict(**row, output=value, r493_output=old))
            if (index+1)%100 == 0:
                print(json.dumps(dict(completed=index+1)), flush=True)
    finally:
        evaluator.close()
    assert len(profiles) == len(targets) == 3
    assert hashlib.sha256((SOURCE/'inputs.json').read_bytes()).digest() == hashlib.sha256(raw).digest()
    imports = {n:str(Path(m.__file__).resolve()) for n,m in sys.modules.items()
        if n.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(m,'__file__',None)}
    assert all(Path(p).is_relative_to(S) for p in imports.values())
    result = dict(plan_calls=len(rows), selected_profile_calls=len(profiles), resets=resets,
        command_equal_r493=sum(r['command_equal_r493'] for r in rows),
        output_equal_r493=sum(r['output_equal_r493'] for r in rows),
        strict_output_equal_r493=sum(r['strict_output_equal_r493'] for r in rows),
        profiles=profiles, rows=rows, imports=imports, input_unchanged=True,
        state_injected_per_row=False, warmup_calls=0, repeat_count=0,
        throughput_claim=False, scope='One r493 frozen plan sequence; no current/guard/ROS/sensors/actuation.')
    dump(A/'results/summary.json', result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','imports','profiles')}), flush=True)


if __name__ == '__main__':
    main()
