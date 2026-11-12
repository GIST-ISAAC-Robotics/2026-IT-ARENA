"""저장 입력을 순서대로 재생하며 대표 계획의 함수별 비용만 진단한다. ROS/구동 없음."""
import cProfile
import hashlib
import json
from pathlib import Path
import platform
import pstats
import sys
import time

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,replay_reset,freeze,equal_output,comparable_decision


def main():
    output=A/'result';output.mkdir(exist_ok=False)
    manifest=json.loads((A/'bundle_manifest.json').read_bytes())
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    assert sha(ROOT/manifest['trace'])==manifest['trace_sha256']
    for name,row in manifest['files'].items():assert sha(ROOT/name)==row['sha256'],name
    costs=json.loads((A/'selection.json').read_bytes())
    selected=set(costs['selected_sequences'])
    # 이벤트는 스트리밍하여 이전 preload 진단의 큰 gen2 객체 집합을 만들지 않는다.
    trace=(ROOT/manifest['trace']).open(encoding='utf8')
    header=json.loads(next(trace))['data'];follow,motion=make_controller(header)
    profiler=cProfile.Profile();rows=[];differences=[];full_differences=[]
    started=time.monotonic()
    for line in trace:
        event=json.loads(line);kind=event['kind'];data=event['data']
        if kind=='reset':replay_reset(follow,data)
        elif kind=='call':
            enabled=event['seq'] in selected
            began=time.perf_counter()
            if enabled:profiler.enable()
            try:answer=replay_call(follow,data['input'],motion)
            finally:
                if enabled:profiler.disable()
            elapsed=1000*(time.perf_counter()-began)
            actual=freeze(answer)
            if not equal_output(comparable_decision(actual),comparable_decision(data['output'])):
                differences.append(event['seq'])
            if not equal_output(actual,data['output']):full_differences.append(event['seq'])
            detail=answer[2]
            rows.append(dict(seq=event['seq'],profiled=enabled,elapsed_ms=elapsed,
                state=detail.get('handoff_state'),reason=detail.get('reason'),speed_mps=answer[0]))
    trace.close()
    profiler.dump_stats(str(output/'plan.prof'))
    stats=pstats.Stats(profiler)
    functions=[]
    for (filename,line,name),(primitive,total,self_time,cumulative,callers) in stats.stats.items():
        functions.append(dict(file=filename,line=line,name=name,primitive_calls=primitive,calls=total,
            self_s=self_time,cumulative_s=cumulative))
    functions.sort(key=lambda r:r['cumulative_s'],reverse=True)
    for path,data in [('functions.json',functions),('durations.json',rows)]:
        (output/path).write_text(json.dumps(data,indent=2)+'\n',encoding='utf8')
    report=dict(calls=len(rows),profiled=sum(r['profiled'] for r in rows),
        selected=len(selected),comparable_differences=differences,full_differences=full_differences,
        source_unchanged=all(sha(ROOT/n)==r['sha256'] for n,r in manifest['files'].items()),
        trace_unchanged=sha(ROOT/manifest['trace'])==manifest['trace_sha256'],
        elapsed_s=time.monotonic()-started,python=sys.version,platform=platform.platform(),
        scope='offline serial planner function-cost attribution; cProfile overhead; no ROS or actuator; not realtime benchmark')
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
    print(json.dumps(report),flush=True)
    return 0 if not differences and report['source_unchanged'] and report['trace_unchanged'] else 1


if __name__=='__main__':raise SystemExit(main())
