#!/usr/bin/env python3
"""저장 입력의 현재/짧은 이전 광선 깊이 지지 비교. 제어/공간 채우기 미연결."""
import argparse
from collections import deque, Counter
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np
from shapely.geometry import Point

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_runtime import digest
from audit_corridor_visibility import bracket_depth
from probe_observation_envelope import swept_region, unknown_sector
from audit_corridor_sweep import required_stopping_distance

ROOT=Path(__file__).resolve().parents[1]


def region_queries(scene, steering, distance, increment, config):
    _,_,region=swept_region(steering,distance,config,step=.01)
    x0,y0,x1,y1=region.bounds
    grid=np.array([(x,y) for x in np.arange(x0,x1+.005,.01) for y in np.arange(y0,y1+.005,.01)
        if region.covers(Point(x,y))])
    queries=list(grid)
    # 얇은 unknown 쐐기는 1 cm 격자에 안 잡힐 수 있으므로 별도 교차 대표점을 추가한다.
    thin=0
    for ray in scene.rays:
        if ray.status=='return': continue
        hit=region.intersection(unknown_sector(ray,increment/2,region))
        if not hit.is_empty:
            p=hit.representative_point(); queries.append([p.x,p.y]); thin+=1
    return np.unique(np.asarray(queries).reshape(-1,2),axis=0),thin


def temporal_support(scenes,queries,config):
    queries=np.asarray(queries,dtype=float).reshape(-1,2)
    supported=np.zeros(len(queries),dtype=bool)
    first=np.full(len(queries),-1,dtype=int)
    coverage=[]
    for i,scene in enumerate(scenes):
        padding=np.array([.025+config.measurement_padding_m+config.object_motion_bound_mps*(scene.t_ref_s-r.acquired_at_s)
                          for r in scene.rays])
        results=bracket_depth(scene,queries,padding)
        current=np.array([r['supported'] for r in results])
        first[current&~supported]=i
        supported|=current
        coverage.append(dict(source_first_s=scene.source_first_s,age_s=scene.t_ref_s-scene.source_first_s,
            supported=int(current.sum()),unsupported_reasons=dict(Counter(r['reason'] for r in results if not r['supported']))))
    return dict(queries=len(queries),sources=coverage,supported=int(supported.sum()),all_supported=bool(supported.all()),
        new_support_from_earlier=int(np.sum(first>0)),unsupported_examples=queries[~supported][:6].tolist(),
        between_queries_verified=False,dynamic_cut_in_verified=False,free_area_certified=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path); parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); args.output.mkdir(parents=True,exist_ok=False)
    before=digest(args.capture)
    sources={}
    for module in tuple(sys.modules.values()):
        name=getattr(module,'__file__',None)
        if name:
            p=Path(name).resolve()
            if p.is_relative_to(ROOT) and p.suffix=='.py': sources[p.relative_to(ROOT).as_posix()]=digest(p)
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name in sources: archive.write(ROOT/name,name)
    scans=deque(maxlen=4); last_key=None; rows=[]
    with args.capture.open() as stream:
        for line in stream:
            e=json.loads(line)
            if e['kind']=='header':
                motion=MotionConfig(**e['data']['motion_config']); config=FollowConfig(**e['data']['follow_config'])
            if e['kind']!='call': continue
            data,scan,history=restore_call(e['data']['input'],motion)
            key=(data['epoch'],scan.header.stamp.sec,scan.header.stamp.nanosec)
            fresh=key!=last_key
            if not fresh: continue
            last_key=key; scans.append((scan,data['epoch'],data['timing_verified'],data['rear_half_angle_deg']))
            if e['data']['output'][2]['reason']!='follow_front_unobserved': continue
            scenes=[]; rejected=[]
            for prior,epoch,verified,rear in reversed(scans):
                if epoch!=data['epoch']: continue
                try:
                    scenes.append(motion_scene(prior,history,data['reference_s'],timing_verified=verified,
                        epoch=epoch,rear_half_angle_deg=rear))
                except MotionUnavailable as error: rejected.append(str(error))
            scene=scenes[0]
            distance=required_stopping_distance(max(0.,data['speed_mps']),.2,scene.t_ref_s-scene.source_first_s,config)
            # 실제 첫 호출의 직전각과 고정 0.1754 rad는 각각 실제/민감도 조건으로 구분한다.
            cases=[]
            for name,steer in [('recorded_previous',data['previous_steering_rad']),('left_turn_sensitivity',.175417)]:
                queries,thin=region_queries(scene,steer,distance,scan.angle_increment,config)
                cases.append(dict(name=name,steering_rad=steer,distance_m=distance,
                    thin_unknown_queries=thin,coverage=temporal_support(scenes,queries,config)))
            rows.append(dict(seq=e['seq'],time_s=data['reference_s'],measured_speed_mps=data['speed_mps'],
                cases=cases,unavailable_sources=rejected))
            if len(rows)>=8: break
    report=dict(scope='first_eight_unique_unknown_scans_temporal_depth_support_only',rows=rows,
        input_sha256=before,source_sha256=sources,input_unchanged=before==digest(args.capture),
        source_unchanged=all(digest(ROOT/p)==h for p,h in sources.items()),runtime_changed=False,
        current_unknown_modified=False,actual_braking_verified=False)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(rows=len(rows),counts=[dict(seq=r['seq'],cases=[dict(name=c['name'],
        supported=c['coverage']['supported'],queries=c['coverage']['queries'],earlier_added=c['coverage']['new_support_from_earlier'])
        for c in r['cases']]) for r in rows],input_unchanged=report['input_unchanged'],source_unchanged=report['source_unchanged'])))


if __name__=='__main__': main()
