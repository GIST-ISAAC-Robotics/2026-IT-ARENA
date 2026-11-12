"""실제 두 프로세스의 저장 입력 비교. ROS/장치/명령 발행은 실행하지 않는다."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from arena_autonomy.follow_trace import equal_output
from overlap import OverlapProcess

A=Path(__file__).resolve().parent;ROOT=A.parents[3]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('mode',choices=('serial','overlap'))
    parser.add_argument('--out',required=True)
    parser.add_argument('--reference')
    args=parser.parse_args()
    target=Path(args.out);target.mkdir(exist_ok=False)
    trace=ROOT/'artifacts/validation/2026-11-12/traffic_solo_gap_runtime_r225_v1/two_actor_run1/run1/follow_decisions.jsonl'
    reference=ROOT/'artifacts/validation/2026-11-12/projection_kernel_r257_v1/jetson/baseline/answers.json'
    if args.reference:reference=Path(args.reference)
    events=[json.loads(line) for line in trace.open()]
    header=events[0]['data'];header['numeric_execution']='native_v1'
    expected={row[0]:row[1:] for row in json.loads(reference.read_bytes())}
    work=[event for event in events[1:] if event['kind'] in ('call','reset')]
    worker=OverlapProcess(header,work_timeout=5.)
    index=0;owner={};plans={};current_rows=[];plan_rows=[];differences=[];seen=set();answers={}
    started=time.monotonic();deadline=started+300
    try:
        while index<len(work) or worker.plan.inflight or worker.current.inflight or worker.saved_plan:
            for result in worker.collect():
                if result['stage']=='plan':
                    event=owner.pop(result['sequence']);seq=event['seq'];d=event['data']
                    plans[result['sequence']]=event
                    result['trace_sequence']=seq
                    plan_rows.append(result)
                    answers[seq]=[seq,result['output'],None]
                    if not equal_output(result['output'],expected[seq][0]):differences.append(['plan',seq])
                else:
                    plan_seq=result['plan']['sequence'];event=plans.pop(plan_seq);seq=event['seq']
                    result['trace_sequence']=seq
                    current_rows.append(result)
                    answers[seq][2]=result['output']
                    if not equal_output(result['output'],expected[seq][1]):differences.append(['current',seq])
                    seen.add(seq)
            if worker.fault:raise RuntimeError(worker.fault)
            saved=worker.saved_plan
            if saved is not None:
                event=plans[saved['sequence']];d=event['data'];token=(saved['generation'],saved['sequence'])
                if not d.get('current_rebase'):
                    worker.discard_plan(token);plans.pop(saved['sequence']);seen.add(event['seq'])
                elif worker.current.inflight is None:
                    worker.offer_current(token,d['current_rebase']['input'])
            if index<len(work):
                event=work[index]
                if event['kind']=='reset':
                    if not(worker.plan.inflight or worker.current.inflight or worker.saved_plan):
                        worker.invalidate(event['data'].get('reason'));index+=1
                elif args.mode=='overlap' or worker.current.inflight is None:
                    if worker.offer_plan(event['data']['input']):
                        owner[worker.sequence]=event;index+=1
            if time.monotonic()>deadline:raise TimeoutError('bounded replay deadline')
            time.sleep(.0005)
        elapsed=time.monotonic()-started
    finally:
        cleanup=worker.close()
    def stats(rows):
        values=[1000*(r['completed_wall_s']-r['began_wall_s']) for r in rows]
        return dict(count=len(values),p50_ms=float(np.percentile(values,50)),
                    p95_ms=float(np.percentile(values,95)),max_ms=max(values))
    overlap_seconds=0.
    for p in plan_rows:
        for c in current_rows:
            overlap_seconds+=max(0.,min(p['completed_wall_s'],c['completed_wall_s'])-
                                max(p['began_wall_s'],c['began_wall_s']))
    report=dict(mode=args.mode,elapsed_s=elapsed,completed_calls=len(seen),
        plan=stats(plan_rows),current=stats(current_rows),full_output_differences=differences,
        measured_stage_overlap_s=overlap_seconds,cleanup=cleanup,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        scope='bounded actual process replay; no ROS, publication cadence, TTL application or closed-loop proof')
    compact=[{k:v for k,v in row.items() if k not in ('input','output','plan')}
             for row in plan_rows+current_rows]
    (target/'timings.json').write_text(json.dumps(compact,indent=2)+'\n')
    (target/'answers.json').write_text(json.dumps(list(answers.values()))+'\n')
    (target/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)
    assert not differences and all(not c['forced'] and c['exitcode']==0 for c in cleanup)
    assert len(seen)==len(expected)

if __name__=='__main__':main()
