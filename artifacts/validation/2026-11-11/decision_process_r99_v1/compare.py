"""실제 spawn 판단 동일성과 부모 응답 비용. ROS·명령 발행은 하지 않는다."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[4]
A=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import DecisionProcess
from arena_autonomy.follow_trace import comparable_decision, equal_output

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def stats(values):
    import numpy as np
    return dict(count=len(values),p50=float(np.percentile(values,50)),
        p95=float(np.percentile(values,95)),maximum=max(values)) if values else dict(count=0)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix',required=True)
    args=parser.parse_args()
    assert args.prefix.isidentifier()
    target=A/(args.prefix+'_process.json');rows=A/(args.prefix+'_decisions.jsonl.gz')
    assert not target.exists() and not rows.exists()
    base=ROOT/'artifacts/validation/2026-11-11/control_10hz_cost_r98_v1'
    manifest=json.loads((base/'bundle_manifest.json').read_bytes())
    unchanged=lambda:all(sha(ROOT/n)==v['sha256'] for n,v in manifest['files'].items())
    assert unchanged()
    hashes={n:sha(ROOT/n) for n in ['src/arena_autonomy/arena_autonomy/decision_process.py',
                                   'artifacts/validation/2026-11-11/decision_process_r99_v1/compare.py']}
    trace=ROOT/manifest['trace'];before=sha(trace)
    total=calls=0;footer=cleanup=worker=None
    mismatch=[];command_diff=[]
    submit=[];poll=[];roundtrip=[];work=[];wait=[];late=[]
    missed=0;start=time.monotonic();next_tick=start+.005
    def heartbeat():
        nonlocal next_tick,missed
        now=time.monotonic()
        if now>=next_tick:
            delay=now-next_tick;late.append(delay*1000.)
            skipped=int(delay/.005);missed+=skipped
            next_tick+=(skipped+1)*.005
    try:
        with trace.open(encoding='utf8') as stream,gzip.open(rows,'xt',encoding='utf8') as sink:
            for line in stream:
                event=json.loads(line)
                assert footer is None and event['seq']==total
                total+=1;kind,data=event['kind'],event['data']
                if kind=='header':
                    assert total==1
                    worker=DecisionProcess(data);next_tick=time.monotonic()+.005
                elif kind=='reset':worker.invalidate()
                elif kind=='call':
                    heartbeat();began=time.monotonic()
                    worker.offer(data['input']);submit.append((time.monotonic()-began)*1000.)
                    result=None;deadline=time.monotonic()+10.
                    while result is None:
                        heartbeat();tick=time.monotonic();result=worker.pump()
                        poll.append((time.monotonic()-tick)*1000.)
                        if worker.fault or time.monotonic()>=deadline:
                            raise RuntimeError(worker.fault or 'outer diagnostic deadline')
                        if result is None:time.sleep(.001)
                    heartbeat();roundtrip.append((time.monotonic()-began)*1000.)
                    work.append((result['completed_wall_s']-result['began_wall_s'])*1000.)
                    wait.append((result['received_wall_s']-result['completed_wall_s'])*1000.)
                    actual=comparable_decision(result['output']);expected=comparable_decision(data['output'])
                    calls+=1
                    if not equal_output(actual,expected):mismatch.append(event['seq'])
                    if not equal_output(actual[:2],expected[:2]):command_diff.append(event['seq'])
                    sink.write(json.dumps(dict(seq=event['seq'],output=actual),allow_nan=False)+'\n')
                elif kind=='footer':footer=data
                else:raise ValueError('unsupported trace event')
    finally:
        if worker is not None:cleanup=worker.close()
    assert footer and all(footer.get(k)==0 for k in ('capture_errors','failed','dropped','pending'))
    assert footer.get('worker_alive') is False and footer['written']==footer['accepted']==total-1
    report=dict(scope='stateful process and synthetic parent heartbeat; no ROS/application-geometry/actuation proof',
        calls=calls,full_outputs_matched=calls-len(mismatch),full_mismatch_seqs=mismatch,
        command_difference_seqs=command_diff,input_unchanged=before==sha(trace),
        source_unchanged=unchanged() and all(sha(ROOT/n)==h for n,h in hashes.items()),
        trace_sha256=before,source_hashes=hashes,cleanup=cleanup,process_alive=worker.process.is_alive(),
        worker_stats=worker.stats,submit_ms=stats(submit),pump_ms=stats(poll),roundtrip_ms=stats(roundtrip),
        worker_ms=stats(work),completion_wait_ms=stats(wait),synthetic_parent_5ms_lateness_ms=stats(late),
        heartbeat_missed=missed,elapsed_s=time.monotonic()-start)
    target.write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('full_mismatch_seqs','source_hashes')}))

if __name__=='__main__':main()
