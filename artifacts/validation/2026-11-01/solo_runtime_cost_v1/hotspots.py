"""20 ms를 넘긴 판단의 비용 원인을 찾는다. profiler 오버헤드 시간은 성능 결과로 쓰지 않는다."""
import cProfile
from hashlib import sha256
import json
from pathlib import Path
import pstats
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
import profile_follow_trace as profiler


if __name__ == '__main__':
    output = HERE/'hotspots'
    output.mkdir(exist_ok=False)
    trace = HERE.parent/'solo_route_v1/run1/follow_decisions.jsonl'
    prior = json.loads((HERE/'run1/report.json').read_text(encoding='utf-8'))
    def sources_match():
        return all(sha256((ROOT/path).read_bytes()).hexdigest()==digest
                   for path,digest in prior['source_sha256'].items())
    if not sources_match() or sha256(trace.read_bytes()).hexdigest()!=prior['input_sha256']:
        raise ValueError('비용 기준 측정 이후 입력 또는 소스가 달라짐')
    source_before = sha256(Path(__file__).read_bytes()).hexdigest()
    calls = cProfile.Profile()
    original = profiler.timed_call

    def measured(*args, **kwargs):
        calls.enable()
        try:
            return original(*args, **kwargs)
        finally:
            calls.disable()

    profiler.timed_call = measured
    result = profiler.profile_source(lambda: profiler.read_events(trace), repeats=1, warmups=0)
    calls.dump_stats(str(output/'calls.prof'))
    with (output/'functions.txt').open('x',encoding='utf-8') as f:
        stats = pstats.Stats(calls,stream=f)
        stats.sort_stats('cumulative').print_stats(50)
        stats.sort_stats('tottime').print_stats(35)
        stats.sort_stats('cumulative').print_callers('swept_limit|_partial_command|boundary_support|audit|query|corridor')
    summary = dict(passed=result['passed'], baseline_matched=result.get('baseline_matched'),
                   timed_calls=result.get('timed_calls'), source_sha256=source_before,
                   source_unchanged=sources_match() and source_before==sha256(Path(__file__).read_bytes()).hexdigest(),
                   trace_sha256=sha256(trace.read_bytes()).hexdigest(),
                   scope='cProfile 함수 비용 분해. 오버헤드가 있으므로 수치는 실제 20 ms 마감 판정에 쓰지 않는다.')
    with (output/'summary.json').open('x',encoding='utf-8') as f:
        json.dump(summary,f,ensure_ascii=False,indent=2)
    print(json.dumps(summary,ensure_ascii=False))
    raise SystemExit(0 if summary['passed'] and summary['source_unchanged'] else 1)
