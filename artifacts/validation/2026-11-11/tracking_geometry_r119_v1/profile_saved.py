"""r117 현재 관측 후보의 기하 비용을 같은 실패 입력에서 한 번 분해한다."""
import cProfile
import collections
import json
from pathlib import Path
import pstats
import sys
import time

A = Path(__file__).resolve().parent
PREVIOUS = A.parent / 'current_tracking_r117_v1'
sys.path.insert(0, str(PREVIOUS))
import probe
from arena_autonomy import pass_release_trial as pr
from arena_autonomy import m3_execution_trial as mx
from arena_autonomy import observed_lateral_trial as ol
from arena_autonomy.follow_trace import equal_output


def main():
    costs = collections.defaultdict(list)
    sizes = collections.defaultdict(list)
    def measure(module, name):
        original = getattr(module, name)
        def timed(*args, **kwargs):
            start = time.perf_counter()
            result = original(*args, **kwargs)
            costs[name].append(1000*(time.perf_counter()-start))
            if name in ('depth_support', 'masked_only'):
                sizes[name].append(len(args[1]))
            elif name == 'body_queries':
                sizes[name].append(len(result[0]))
            return result
        setattr(module, name, timed)
    for name in ('motion_queries', 'depth_support'):
        measure(pr, name)
    for name in ('body_queries', 'masked_only', 'path_observation_queries'):
        measure(mx, name)
    for name in ('clip_polygons', 'exterior_queries'):
        measure(ol, name)
    probe.A = A
    profiler = cProfile.Profile()
    profiler.runcall(probe.main)
    profiler.dump_stats(str(A/'saved.prof'))
    stats = pstats.Stats(profiler)
    top = sorted(stats.stats.items(), key=lambda p: p[1][3], reverse=True)[:55]
    expected = [json.loads(s) for s in (PREVIOUS/'details.jsonl').open()]
    actual = [json.loads(s) for s in (A/'details.jsonl').open()]
    assert len(actual) == len(expected)
    differences = [i for i,(a,b) in enumerate(zip(expected, actual)) if not equal_output(a,b)]
    summary = {name:dict(calls=len(v),sum_ms=sum(v),p50_ms=sorted(v)[len(v)//2],
        p95_ms=sorted(v)[int(.95*len(v))],maximum_ms=max(v)) for name,v in costs.items()}
    output = dict(stages=summary,query_counts={k:dict(minimum=min(v),maximum=max(v),total=sum(v))
        for k,v in sizes.items()},full_output_difference_indices=differences,
        profile=[dict(file=k[0],line=k[1],name=k[2],calls=v[1],self_s=v[2],cumulative_s=v[3])
                 for k,v in top],scope='Profile overhead included; no runtime or live actuation')
    with (A/'profile.json').open('x') as f:
        json.dump(output,f,indent=2)
    print(json.dumps({k:v for k,v in output.items() if k!='profile'}))


if __name__ == '__main__':
    main()
