"""고정 소스/순차 입력에서 일부 호출의 함수 비용만 조사. deadline/새 주행 시험 아님."""
import argparse
import cProfile
import hashlib
import json
from pathlib import Path
import pstats
import time

from arena_autonomy.follow_trace import replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('trace',type=Path);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    original=digest(args.trace);prof=cProfile.Profile();calls=sampled=0;mismatches=[]
    start=time.monotonic()
    for line in args.trace.open(encoding='utf-8'):
        e=json.loads(line);d=e['data']
        if e['kind']=='header':
            motion=MotionConfig(**d['motion_config']);f=LocalFollow(FollowConfig(**d['follow_config']))
        elif e['kind']=='reset':f.reset()
        elif e['kind']=='call':
            selected=e['seq']%25 in (0,1,2)
            calls+=1
            if selected:prof.enable();sampled+=1
            try:out=replay_call(f,d['input'],motion)
            finally:
                if selected:prof.disable()
            if selected and not equal_output(comparable_decision(freeze(out)),comparable_decision(d['output'])):
                mismatches.append(e['seq'])
    prof.dump_stats(str(args.output/'calls.prof'))
    stats=pstats.Stats(prof)
    rows=[]
    for (file,line,name),(primitive,total,own,cumulative,callers) in stats.stats.items():
        rows.append(dict(file=file,line=line,name=name,primitive_calls=primitive,total_calls=total,
                         own_s=own,cumulative_s=cumulative))
    result=dict(scope='sampled_cprofile_function_cost_not_uninstrumented_latency_or_ros',
        source_version='fixed_r67_bundle',calls=calls,sampled_calls=sampled,
        sample_rule='seq modulo25 in0,1,2; full preceding state replayed',
        sampled_output_mismatches=mismatches,input_sha256=original,input_unchanged=digest(args.trace)==original,
        wall_elapsed_s=time.monotonic()-start,
        by_cumulative=sorted(rows,key=lambda x:x['cumulative_s'],reverse=True)[:60],
        by_self=sorted(rows,key=lambda x:x['own_s'],reverse=True)[:60])
    result['passed']=not mismatches and result['input_unchanged']
    (args.output/'report.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('by_cumulative','by_self')}))
    return int(not result['passed'])


if __name__=='__main__':raise SystemExit(main())
