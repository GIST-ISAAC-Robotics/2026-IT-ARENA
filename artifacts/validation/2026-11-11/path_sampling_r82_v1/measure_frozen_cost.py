"""r82 동결 Jetson 판단 비용 관찰. 전체 출력 실패와 명령 동일성은 별도 보고."""
import argparse,collections,hashlib,json,platform,time
from pathlib import Path
import numpy as np
from arena_autonomy import follow_trace as ft

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--trace',type=Path,required=True);p.add_argument('--manifest',type=Path,required=True)
p.add_argument('--out',type=Path,required=True);a=p.parse_args();assert not a.out.exists()
root=Path(ft.__file__).resolve().parents[3]
manifest=json.loads(a.manifest.read_bytes());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert all(sha(root/n)==v['sha256'] for n,v in manifest['files'].items())
before=sha(a.trace);original=ft.replay_call;current=None;rows=[];command_diffs=[]
def call(*args,**kwargs):
    cpu=time.thread_time_ns();start=time.perf_counter_ns()
    output=original(*args,**kwargs)
    elapsed=(time.perf_counter_ns()-start)/1e6;cpu_elapsed=(time.thread_time_ns()-cpu)/1e6
    # 시간 창에는 순수 입력 복원·운동 투영·판단만 포함. JSON 읽기/비교는 밖이다.
    frozen=ft.comparable_decision(ft.freeze(output));expected=ft.comparable_decision(current['data']['output'])
    if not ft.equal_output(frozen[:2],expected[:2]):command_diffs.append(current['seq'])
    normal=output[2].get('normal',output[2]);reason=normal.get('selector_reason') or normal.get('reason')
    rows.append(dict(seq=current['seq'],wall_ms=elapsed,cpu_ms=cpu_elapsed,reason=reason))
    return output
def events():
    global current
    for line in a.trace.open(encoding='utf8'):
        current=json.loads(line);yield current
def stats(values):
    return dict(count=len(values),p50=float(np.percentile(values,50)),p95=float(np.percentile(values,95)),
        p99=float(np.percentile(values,99)),maximum=max(values),over_20ms=sum(v>20 for v in values))
ft.replay_call=call;started=time.monotonic()
try:result=ft.replay_events(events())
finally:ft.replay_call=original
groups=collections.defaultdict(list)
for r in rows:groups[str(r['reason'])].append(r['wall_ms'])
report=dict(scope='frozen r82 traffic offline cost observation only; no strict whole-output pass, ROS/vision/safety/actuation deadline or hardware timing claim',
    calls=result['calls'],full_outputs_matched=result['matched'],all_outputs_matched=result['passed'],
    commands_matched=not command_diffs,command_difference_seqs=command_diffs,
    input_sha256=before,input_unchanged=before==sha(a.trace),
    source_unchanged=all(sha(root/n)==v['sha256'] for n,v in manifest['files'].items()),
    wall_ms=stats([r['wall_ms'] for r in rows]),thread_cpu_ms=stats([r['cpu_ms'] for r in rows]),
    by_reason={k:stats(v) for k,v in groups.items()},elapsed_s=time.monotonic()-started,
    python=platform.python_version(),numpy=np.__version__,machine=platform.machine())
a.out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='by_reason'}))
