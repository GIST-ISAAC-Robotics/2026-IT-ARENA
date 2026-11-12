"""동결 Jetson 파생 입력의 일부 판단에서 함수 비용만 조사한다. 실제 지연 계측 아님."""
import argparse
import cProfile
import hashlib
import json
from pathlib import Path
import pstats
import time
import arena_autonomy.follow_trace as trace

parser=argparse.ArgumentParser();parser.add_argument('input',type=Path);parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
before=hashlib.sha256(args.input.read_bytes()).hexdigest()
events=[json.loads(line) for line in args.input.open()]
prof=cProfile.Profile();call_index=0;sampled=0;original=trace.replay_call
def measured(*a,**kw):
    global call_index,sampled
    take=call_index%20 in (0,1);call_index+=1
    if take:prof.enable();sampled+=1
    try:return original(*a,**kw)
    finally:
        if take:prof.disable()
started=time.monotonic();trace.replay_call=measured
try:result=trace.replay_events(events)
finally:trace.replay_call=original
prof.dump_stats(str(args.output/'calls.prof'));stats=pstats.Stats(prof);rows=[]
for (file,line,name),(primitive,total,own,cumulative,callers) in stats.stats.items():
    rows.append(dict(file=file,line=line,name=name,primitive_calls=primitive,total_calls=total,
                     own_s=own,cumulative_s=cumulative))
summary=dict(scope='44_sampled_calls_cprofile_not_uninstrumented_latency_or_ros',
    calls=result['calls'],sampled_calls=sampled,matched=result['matched'],mismatches=[r['seq'] for r in result['mismatches']],
    sample_rule='zero_based_call_index_modulo20_in_0_1',input_sha256=before,
    input_unchanged=before==hashlib.sha256(args.input.read_bytes()).hexdigest(),
    elapsed_s=time.monotonic()-started,
    by_cumulative=sorted(rows,key=lambda r:r['cumulative_s'],reverse=True)[:50],
    by_self=sorted(rows,key=lambda r:r['own_s'],reverse=True)[:50])
summary['passed']=result['passed'] and summary['input_unchanged']
(args.output/'report.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k not in ('by_cumulative','by_self')}))
raise SystemExit(0 if summary['passed'] else 1)
