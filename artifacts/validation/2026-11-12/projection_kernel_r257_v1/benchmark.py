"""같은 저장 호출을 각각 별도 프로세스로 재생한다. 장치·ROS 없음."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,replay_reset,freeze,equal_output
from arena_autonomy.numeric_kernels import configure
from arena_autonomy.recheck_process import evaluate_recheck

def main():
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=('baseline','candidate'));args=parser.parse_args()
    target=A/args.mode;target.mkdir(exist_ok=False)
    manifest=json.loads((ROOT/'artifacts/validation/2026-11-12/numeric_ros_cost_r248_v1/bundle_manifest.json').read_bytes())
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    assert all(sha(ROOT/n)==r['sha256'] for n,r in manifest['files'].items())
    trace=ROOT/manifest['trace'];assert sha(trace)==manifest['trace_sha256']
    configure('native_v1')
    if args.mode=='candidate':
        import candidate
        import arena_autonomy.native_geometry as original
        original._geometry=candidate.geometry
    answers=[];costs=[];start=time.monotonic()
    with trace.open() as f:
        header=json.loads(next(f))['data'];follow,motion=make_controller(header)
        for line in f:
            e=json.loads(line);d=e['data']
            if e['kind']=='reset':replay_reset(follow,d)
            if e['kind']!='call':continue
            began=time.perf_counter();out=freeze(replay_call(follow,d['input'],motion));plan_ms=1000*(time.perf_counter()-began)
            check=d.get('current_rebase');current=None;current_ms=None
            if check:
                began=time.perf_counter();current=evaluate_recheck(out,d['input'],check['input'],motion,follow.config)
                current_ms=1000*(time.perf_counter()-began)
            answers.append([e['seq'],out,current]);costs.append(dict(seq=e['seq'],plan_ms=plan_ms,current_ms=current_ms))
    (target/'answers.json').write_text(json.dumps(answers)+'\n');(target/'durations.json').write_text(json.dumps(costs,indent=2)+'\n')
    stats={}
    for key in ('plan_ms','current_ms'):
        vals=[r[key] for r in costs if r[key] is not None]
        stats[key]=dict(count=len(vals),p50=float(np.percentile(vals,50)),p95=float(np.percentile(vals,95)),max=max(vals))
    references={'r248_native':ROOT/'artifacts/validation/2026-11-12/numeric_ros_cost_r248_v1/native_run/answers.json'}
    if args.mode=='candidate':references['paired_baseline']=A/'baseline/answers.json'
    comparison={}
    for label,p in references.items():
        expected=json.loads(p.read_bytes());assert len(expected)==len(answers)
        comparison[label]=[a[0] for a,b in zip(answers,expected) if not equal_output(a,b)]
    report=dict(mode=args.mode,calls=len(answers),costs=stats,full_differences=comparison,
        source_unchanged=all(sha(ROOT/n)==r['sha256'] for n,r in manifest['files'].items()),
        trace_unchanged=sha(trace)==manifest['trace_sha256'],elapsed_s=time.monotonic()-start,
        scope='paired serial full-output replay; no ROS load, IPC or runtime validity/drive proof')
    (target/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
    return int(any(comparison.values()) or not report['source_unchanged'] or not report['trace_unchanged'])

if __name__=='__main__':raise SystemExit(main())
