"""r98에서180ms 초과한 판단만 프로파일링한다. 전체 상태 재생은1회."""
import argparse
import cProfile
import hashlib
import json
from pathlib import Path
import pstats
import sys

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--root',type=Path,required=True)
parser.add_argument('--cost',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
a=parser.parse_args()
sys.path[:0]=[str(a.root/'src/arena_autonomy'),str(a.root/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import replay_call,freeze,comparable_decision,equal_output,FollowConfig,MotionConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
manifest=json.loads((a.root/'artifacts/validation/2026-11-11/control_10hz_cost_r98_v1/bundle_manifest.json').read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
valid=lambda:all(sha(a.root/p)==v['sha256'] for p,v in manifest['files'].items())
assert valid() and not a.output.exists()
cost=json.loads(a.cost.read_bytes())
tail={x['seq'] for x in cost['per_call_stages'] if sum(s['wall_ms'] for s in x['stages'].values())>180.}
trace=a.root/manifest['trace']; before=sha(trace);cost_hash=sha(a.cost)
profile=cProfile.Profile();calls=0;count=0;footer=None;diffs=[]
with trace.open(encoding='utf8') as stream:
    for line in stream:
        e=json.loads(line);assert footer is None and e['seq']==count;count+=1
        k=e['kind'];d=e['data']
        if k=='header':
            assert d['schema']==5
            follow=BypassHandoffFollow(FollowConfig(**d['follow_config']),HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
            motion=MotionConfig(**d['motion_config'])
        elif k=='reset':follow.reset()
        elif k=='call':
            if e['seq'] in tail:profile.enable()
            try:output=replay_call(follow,d['input'],motion)
            finally:profile.disable()
            if not equal_output(comparable_decision(freeze(output)),comparable_decision(d['output'])):diffs.append(e['seq'])
            calls+=1
        elif k=='footer':footer=d
        else:raise ValueError('unexpected event')
assert footer and calls==610 and all(footer[k]==0 for k in ('capture_errors','dropped','failed','pending'))
assert footer['worker_alive'] is False and footer['accepted']==footer['written']==count-1
stats=pstats.Stats(profile)
rows=[]
for (filename,line,func),(primitive,total,own,cumulative,callers) in stats.stats.items():
    filename=str(filename)
    if str(a.root) in filename:filename=str(Path(filename).relative_to(a.root))
    rows.append(dict(file=filename,line=line,function=func,primitive_calls=primitive,calls=total,
        own_s=own,cumulative_s=cumulative,
        callers=[dict(file=str(c[0]),line=c[1],function=c[2],metrics=list(v) if isinstance(v,tuple) else v)
                 for c,v in sorted(callers.items(),key=lambda x: x[1][-1] if isinstance(x[1],tuple) else x[1],reverse=True)[:4]]))
report=dict(scope='PC cProfile only selected180ms-tail from prior Jetson run; costs not comparable to unprofiled runtime',
    calls=calls,tail_seqs=sorted(tail),full_difference_seqs=diffs,
    input_unchanged=before==sha(trace) and cost_hash==sha(a.cost),source_unchanged=valid(),
    trace_sha256=before,cost_sha256=cost_hash,
    functions=sorted(rows,key=lambda r:r['cumulative_s'],reverse=True)[:100],
    own_hotspots=sorted(rows,key=lambda r:r['own_s'],reverse=True)[:40])
with a.output.open('x',encoding='utf8') as stream:json.dump(report,stream,indent=2);stream.write('\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('functions','own_hotspots')}))
