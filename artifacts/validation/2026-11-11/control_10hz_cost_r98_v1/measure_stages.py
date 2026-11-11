"""One stateful stream pass with bounded output retention; never publishes commands."""
import argparse
import collections
import gc
import gzip
import hashlib
import json
from pathlib import Path
import platform
import resource
import time
import numpy as np
from arena_autonomy import follow_trace as ft
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--trace',type=Path,required=True)
p.add_argument('--manifest',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--decisions',type=Path,required=True)
a=p.parse_args()
assert not a.output.exists() and not a.decisions.exists()
root=Path(ft.__file__).resolve().parents[3]
manifest=json.loads(a.manifest.read_bytes())
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
assert all(sha(root/n)==v['sha256'] for n,v in manifest['files'].items())
trace_sha=sha(a.trace)
rows=[];differences=[];command_diffs=[];rss=[]
stage_rows=[]; stage_now={}
def timed_stage(name, fn):
    def wrapped(*args, **kwargs):
        cpu=time.thread_time_ns(); wall=time.perf_counter_ns()
        try:return fn(*args, **kwargs)
        finally:
            previous=stage_now.setdefault(name, dict(wall_ms=0.,cpu_ms=0.,calls=0))
            previous['wall_ms']+=(time.perf_counter_ns()-wall)/1e6
            previous['cpu_ms']+=(time.thread_time_ns()-cpu)/1e6
            previous['calls']+=1
    return wrapped
ft.restore_call=timed_stage('restore_input',ft.restore_call)
ft.motion_scene=timed_stage('motion_scene',ft.motion_scene)
gc_count=0;inside=False
def observe_gc(phase,info):
    global gc_count
    if phase=='stop' and inside:gc_count+=1
gc.callbacks.append(observe_gc)
started=time.monotonic();footer=None;follow=None;motion=None;count=0
try:
    with a.trace.open(encoding='utf8') as stream,gzip.open(a.decisions,'xt',encoding='utf8',compresslevel=1) as sink:
        for event in (json.loads(line) for line in stream):
            assert footer is None and event['seq']==count
            count+=1;kind,data=event['kind'],event['data']
            if count==1:
                assert kind=='header' and data['schema']==5
                assert data['scope']=='g1_g2_handoff_decision_stage_only'
                assert data['controller_mode']=='g1_g2_handoff_trial'
                follow=BypassHandoffFollow(ft.FollowConfig(**data['follow_config']),
                    HandoffExecutionConfig.from_mapping(data['execution_config']),allow_simulation_trial=True)
                motion=ft.MotionConfig(**data['motion_config'])
                follow.command=timed_stage('follow_command',follow.command)
            elif kind=='reset':
                follow.reset()
            elif kind=='call':
                stage_now.clear()
                before_gc=gc_count;inside=True
                cpu=time.thread_time_ns();wall=time.perf_counter_ns()
                output=ft.replay_call(follow,data['input'],motion)
                wall_ms=(time.perf_counter_ns()-wall)/1e6
                cpu_ms=(time.thread_time_ns()-cpu)/1e6;inside=False
                stage_rows.append(dict(seq=event['seq'],stages={k:dict(v) for k,v in stage_now.items()}))
                actual=ft.comparable_decision(ft.freeze(output))
                expected=ft.comparable_decision(data['output'])
                if not ft.equal_output(actual,expected):differences.append(event['seq'])
                if not ft.equal_output(actual[:2],expected[:2]):command_diffs.append(event['seq'])
                normal=output[2].get('normal',output[2])
                reason=normal.get('selector_reason') or normal.get('reason')
                rows.append(dict(seq=event['seq'],wall_ms=wall_ms,cpu_ms=cpu_ms,
                    gc_collections=gc_count-before_gc,reason=reason))
                sink.write(json.dumps(dict(seq=event['seq'],output=actual),allow_nan=False,separators=(',',':'))+'\n')
                if len(rows)%250==0:rss.append(dict(calls=len(rows),maximum_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss))
                # No retained point cloud/paths from earlier calls. Native GC remains enabled.
                del output,actual,expected,normal
            elif kind=='footer':
                footer=data
            else:raise ValueError('unexpected trace event')
finally:
    gc.callbacks.remove(observe_gc)
assert footer is not None and rows
assert all(footer.get(k)==0 for k in ('capture_errors','failed','dropped','pending'))
assert footer.get('worker_alive') is False
assert footer.get('written')==count-1 and footer.get('accepted')==count-1
def stats(values):
    return dict(count=len(values),p50=float(np.percentile(values,50)),p95=float(np.percentile(values,95)),
        p99=float(np.percentile(values,99)),maximum=max(values),over_20ms=sum(v>20 for v in values),over_100ms=sum(v>100 for v in values))
groups=collections.defaultdict(list)
for row in rows:groups[str(row['reason'])].append(row['wall_ms'])
report=dict(scope='r91 run4 actual 10Hz trace; immutable replay, stage CPU cost only; no ROS or actuation deadline claim',
    calls=len(rows),full_outputs_matched=len(rows)-len(differences),full_mismatch_seqs=differences,
    commands_matched=not command_diffs,command_difference_seqs=command_diffs,
    trace_sha256=trace_sha,input_unchanged=trace_sha==sha(a.trace),
    source_unchanged=all(sha(root/n)==v['sha256'] for n,v in manifest['files'].items()),
    stages={name:{metric:stats([r['stages'].get(name,{}).get(metric,0.) for r in stage_rows]) for metric in ('wall_ms','cpu_ms')} for name in ('restore_input','motion_scene','follow_command')},
    stage_invocations={name:sum(r['stages'].get(name,{}).get('calls',0) for r in stage_rows) for name in ('restore_input','motion_scene','follow_command')},
    wall_ms=stats([r['wall_ms'] for r in rows]),thread_cpu_ms=stats([r['cpu_ms'] for r in rows]),
    by_reason={k:stats(v) for k,v in groups.items()},
    gc=dict(enabled=gc.isenabled(),collections_inside=gc_count,calls_with_collection=sum(r['gc_collections']>0 for r in rows)),
    maximum_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,rss_samples=rss,
    elapsed_s=time.monotonic()-started,python=platform.python_version(),numpy=np.__version__,machine=platform.machine(),
    decisions_sha256=sha(a.decisions),per_call_stages=stage_rows)
a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in report.items() if k not in ('by_reason','rss_samples','full_mismatch_seqs','per_call_stages')}))
