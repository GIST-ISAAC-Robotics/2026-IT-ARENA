"""동일 Jetson·동일 입력 순서에서 원래/후보 계획을 번갈아 먼저 계산한다."""
from pathlib import Path
import hashlib
import json
import statistics
import sys
import time

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(A),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,replay_reset,freeze,equal_output,comparable_decision
from arena_autonomy import observation_support
import candidate

original=observation_support.depth_support


def select(fn):
    for name,module in tuple(sys.modules.items()):
        if name.startswith('arena_autonomy.') and getattr(module,'depth_support',None) in (original,candidate.depth_support):
            module.depth_support=fn


def distribution(values):
    q=sorted(values)
    return dict(count=len(q),p50=statistics.median(q),p95=q[min(len(q)-1,int(.95*len(q)))],maximum=max(q),mean=statistics.mean(q))


def main():
    manifest=json.loads((A/'bundle_manifest.json').read_bytes());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    assert sha(ROOT/manifest['trace'])==manifest['trace_sha256']
    for name,row in manifest['files'].items():assert sha(ROOT/name)==row['sha256'],name
    output=A/'result';output.mkdir(exist_ok=False)
    rows=[];mismatches=[];commands=[];historical=[]
    with (ROOT/manifest['trace']).open(encoding='utf8') as stream:
        header=json.loads(next(stream))['data']
        baseline,motion=make_controller(header);trial,_=make_controller(header)
        for line in stream:
            event=json.loads(line);data=event['data']
            if event['kind']=='reset':
                replay_reset(baseline,data);replay_reset(trial,data)
            elif event['kind']=='call':
                outcomes={};costs={}
                order=['baseline','candidate'] if len(rows)%2==0 else ['candidate','baseline']
                for mode in order:
                    select(original if mode=='baseline' else candidate.depth_support)
                    began=time.perf_counter()
                    result=replay_call(baseline if mode=='baseline' else trial,data['input'],motion)
                    costs[mode]=1000*(time.perf_counter()-began)
                    outcomes[mode]=freeze(result)
                if not equal_output(comparable_decision(outcomes['baseline']),comparable_decision(outcomes['candidate'])):
                    mismatches.append(event['seq'])
                    if len(mismatches)<=3:
                        (output/f"difference_{event['seq']}.json").write_text(json.dumps(outcomes,indent=2)+'\n')
                if not equal_output(outcomes['baseline'][:2],outcomes['candidate'][:2]):commands.append(event['seq'])
                if not equal_output(outcomes['baseline'],data['output']):historical.append(event['seq'])
                rows.append(dict(seq=event['seq'],first=order[0],**costs))
    select(original)
    (output/'durations.json').write_text(json.dumps(rows,indent=2)+'\n')
    report=dict(calls=len(rows),output_differences=mismatches,command_differences=commands,
        historical_baseline_differences=historical,
        baseline_ms=distribution([r['baseline'] for r in rows]),
        candidate_ms=distribution([r['candidate'] for r in rows]),
        budget100ms_exceed=dict(baseline=sum(r['baseline']>100 for r in rows),candidate=sum(r['candidate']>100 for r in rows)),
        source_unchanged=all(sha(ROOT/n)==r['sha256'] for n,r in manifest['files'].items()),
        scope='same Jetson alternating pair offline replay; explicit depth kernel only; no ROS or hardware output')
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)
    return 0 if not mismatches and report['source_unchanged'] else 1


if __name__=='__main__':raise SystemExit(main())
