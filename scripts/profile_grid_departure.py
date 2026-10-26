"""저장 M5 판단의 함수별 비용 진단. 프로파일러 계측 시간은 실시간 성능 수치가 아니다."""
import argparse
import cProfile
import hashlib
import json
from pathlib import Path
import pstats
import sys
import time
import zipfile
from collections import OrderedDict

import yaml
from arena_autonomy.follow_trace import freeze, replay_call
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.m3_execution_trial import M3ExecutionFollow, ExecutionConfig
from arena_autonomy import m3_execution_trial as m3
from probe_grid_departure_observation import motion_history
from probe_grid_follow import ROOT, CAMERA, VEHICLE


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--query-key-audit',action='store_true')
    args=p.parse_args();out=args.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts/validation'):p.error('new validation directory required')
    out.mkdir(parents=True,exist_ok=False)
    path=ROOT/'artifacts/validation/2026-10-26/m5_start_constraints_v1/run31_grid_coverage/data/grid5_ego4/report.json'
    raw=path.read_bytes();report=json.loads(raw)
    sensor=yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    wb=yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']['drivetrain']['wheelbase_m']
    cfg=FollowConfig(wheelbase_m=wb,occupancy_timing_mode='per_ray_candidate',
        boundary_evidence_mode='corner_distance_candidate',corridor_fallback_enabled=True)
    ex=ExecutionConfig(command_fallback=True,release_to_baseline=True,small_center_straight=True,
        straight_surface_commitment=True,rounded_occupancy=True,straight_with_threat=True)
    follow=M3ExecutionFollow(cfg,ex,allow_simulation_trial=True);motion=motion_history(wb,sensor).config
    sources={Path(__file__),CAMERA,VEHICLE}|set((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    for module in tuple(sys.modules.values()):
        f=getattr(module,'__file__',None)
        if f:
            q=Path(f).resolve()
            if q.is_relative_to(ROOT) and q.suffix=='.py':sources.add(q)
    frozen={q.relative_to(ROOT).as_posix():q.read_bytes() for q in sources}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for n,b in frozen.items():archive.writestr(n,b)
    profiler=cProfile.Profile();differences=[];count=0
    caches={limit:OrderedDict() for limit in (8,32,128)}
    audit={limit:dict(calls=0,hits=0,repeat_compute_s=0.,total_compute_s=0.,key_s=0.) for limit in caches}
    original=m3.path_observation_queries
    if args.query_key_audit:
        def audited_queries(scene,poses,config):
            began=time.perf_counter()
            key=(config.length_m,config.width_m,tuple(tuple(p) for p in poses),
                 tuple((r.ray_origin_xy_m,r.direction_xy) for r in scene.rays if r.status!='return'))
            key_s=time.perf_counter()-began
            began=time.perf_counter();value=original(scene,poses,config);cost=time.perf_counter()-began
            for limit,cache in caches.items():
                a=audit[limit];a['calls']+=1;a['key_s']+=key_s;a['total_compute_s']+=cost
                if key in cache:
                    a['hits']+=1;a['repeat_compute_s']+=cost;cache.move_to_end(key)
                else:
                    cache[key]=None
                    if len(cache)>limit:cache.popitem(last=False)
            return value
        m3.path_observation_queries=audited_queries
    try:
        for i,call in enumerate(report['calls']):
            if call['explicit_stop']:break
            actual=profiler.runcall(replay_call,follow,call['captured_input'],motion)
            count+=1
            if freeze(actual)!=[call['request_mps'],call['request_steering'],call['metadata']]:differences.append(i)
    finally:
        m3.path_observation_queries=original
    stats=pstats.Stats(profiler)
    rows=[]
    for (file,line,name),(primitive,total,self_s,cumulative_s,callers) in stats.stats.items():
        q=Path(file)
        if q.is_absolute() and q.is_relative_to(ROOT):file=q.relative_to(ROOT).as_posix()
        rows.append(dict(file=file,line=line,name=name,primitive_calls=primitive,total_calls=total,
                         self_s=self_s,cumulative_s=cumulative_s))
    result=dict(input_sha256=hashlib.sha256(raw).hexdigest(),input_unchanged=path.read_bytes()==raw,
        source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},
        sources_unchanged=all((ROOT/n).read_bytes()==b for n,b in frozen.items()),
        checked_calls=count,output_differences=differences,
        query_key_audit=audit if args.query_key_audit else None,
        top_cumulative=sorted(rows,key=lambda r:r['cumulative_s'],reverse=True)[:55],
        top_self=sorted(rows,key=lambda r:r['self_s'],reverse=True)[:40],
        scope='instrumented fixed-input replay for hotspot identification; not real-time latency or new driving proof')
    result['integrity_pass']=count==250 and not differences and result['input_unchanged'] and result['sources_unchanged']
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('source_sha256','top_self','top_cumulative')}))
    return 0 if result['integrity_pass'] else 1


if __name__=='__main__':raise SystemExit(main())
