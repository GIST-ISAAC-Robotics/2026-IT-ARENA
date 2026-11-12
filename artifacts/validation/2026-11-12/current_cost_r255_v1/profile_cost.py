"""공통 native 계획/현재 재검사의 잔여 비용. 저장 입력만, 장치/ROS/출력 없음."""
import cProfile
import hashlib
import json
from pathlib import Path
import pstats
import sys
import time

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,replay_reset,freeze,equal_output,comparable_decision
from arena_autonomy.numeric_kernels import configure
from arena_autonomy.recheck_process import evaluate_recheck

def main():
    output=A/'result';output.mkdir(exist_ok=False)
    manifest=json.loads((ROOT/'artifacts/validation/2026-11-12/numeric_ros_cost_r248_v1/bundle_manifest.json').read_bytes())
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    assert all(sha(ROOT/n)==r['sha256'] for n,r in manifest['files'].items())
    trace=ROOT/manifest['trace'];assert sha(trace)==manifest['trace_sha256']
    selected=set(json.loads((A/'selection.json').read_bytes())['selected_sequences'])
    configure('native_v1')
    planners=cProfile.Profile();current=cProfile.Profile()
    rows=[];differences=[];began=time.monotonic()
    with trace.open() as f:
        header=json.loads(next(f))['data'];follow,motion=make_controller(header)
        for line in f:
            event=json.loads(line);data=event['data'];seq=event['seq']
            if event['kind']=='reset':replay_reset(follow,data)
            if event['kind']!='call':continue
            enabled=seq in selected
            if enabled:planners.enable()
            start=time.perf_counter()
            try:answer=freeze(replay_call(follow,data['input'],motion))
            finally:
                elapsed=1000*(time.perf_counter()-start)
                if enabled:planners.disable()
            entry=dict(seq=seq,profiled=enabled,plan_ms=elapsed)
            if not equal_output(comparable_decision(answer),comparable_decision(data['output'])):
                differences.append(dict(seq=seq,stage='plan'))
            recheck=data.get('current_rebase')
            if recheck:
                if enabled:current.enable()
                start=time.perf_counter()
                try:checked=evaluate_recheck(answer,data['input'],recheck['input'],motion,follow.config)
                finally:
                    elapsed=1000*(time.perf_counter()-start)
                    if enabled:current.disable()
                entry['current_ms']=elapsed
                # 원 PC 진단 float 차이를 지우지 않고 별도 계수로 보존한다.
                entry['current_full_equal']=equal_output(checked,recheck['output'])
                if not equal_output([checked['speed_mps'],checked['steering_rad']],
                                    [recheck['output']['speed_mps'],recheck['output']['steering_rad']]):
                    differences.append(dict(seq=seq,stage='current'))
            rows.append(entry)
    for name,profiler in [('plan',planners),('current',current)]:
        profiler.dump_stats(str(output/(name+'.prof')))
        funcs=[]
        for (filename,line,name),(primitive,total,self_s,cumulative_s,_) in pstats.Stats(profiler).stats.items():
            funcs.append(dict(file=filename,line=line,name=name,primitive_calls=primitive,calls=total,
                              self_s=self_s,cumulative_s=cumulative_s))
        funcs.sort(key=lambda r:r['cumulative_s'],reverse=True)
        (output/(('plan' if profiler is planners else 'current')+'_functions.json')).write_text(json.dumps(funcs,indent=2)+'\n')
    (output/'durations.json').write_text(json.dumps(rows,indent=2)+'\n')
    report=dict(calls=len(rows),profiled=sum(r['profiled'] for r in rows),
        current_calls=sum('current_ms' in r for r in rows),command_differences=differences,
        current_full_differences=[r['seq'] for r in rows if r.get('current_full_equal') is False],
        source_unchanged=all(sha(ROOT/n)==r['sha256'] for n,r in manifest['files'].items()),
        trace_unchanged=sha(trace)==manifest['trace_sha256'],elapsed_s=time.monotonic()-began,
        scope='offline native cost attribution; cProfile overhead; no runtime scheduling or hardware proof')
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
    return int(bool(differences) or not report['source_unchanged'] or not report['trace_unchanged'])

if __name__=='__main__':raise SystemExit(main())
