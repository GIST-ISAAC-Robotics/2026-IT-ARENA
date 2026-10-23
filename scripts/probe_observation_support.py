#!/usr/bin/env python3
"""관측 이력 모듈의 저장 입력 비교 및 단일 호출 비용. 폐루프 주행 증거 아님."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time
import zipfile
import types

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.observation_support import RecentScenes,sweep_support
from arena_autonomy import observation_lateral  # 선택 후보도 실행 전 snapshot에 포함한다.
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_runtime import digest

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--compare-source',type=Path,help='직전 snapshot의 동일 모듈과 판단/메타데이터 대조')
    p.add_argument('--protected-scope',action='store_true',help='센서 전방 ±100도에 측방 시간 불확실성 추가')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    before=digest(args.capture);sources={}
    for module in tuple(sys.modules.values()):
        name=getattr(module,'__file__',None)
        if name:
            path=Path(name).resolve()
            if path.is_relative_to(ROOT) and path.suffix=='.py':sources[path.relative_to(ROOT).as_posix()]=digest(path)
    for name in ('tests/test_observation_support.py',):sources[name]=digest(ROOT/name)
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name in sources:z.write(ROOT/name,name)
    reference=None
    if args.compare_source:
        with zipfile.ZipFile(args.compare_source) as z:
            code=z.read('src/arena_autonomy/arena_autonomy/observation_support.py').decode()
        reference=types.ModuleType('frozen_observation_support')
        exec(compile(code,'frozen_observation_support','exec'),reference.__dict__)
    cache=RecentScenes();last=None;rows=[];mismatches=0
    with args.capture.open() as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']=='header':
                motion=MotionConfig(**event['data']['motion_config']);config=FollowConfig(**event['data']['follow_config'])
            if event['kind']!='call':continue
            data,scan,h=restore_call(event['data']['input'],motion)
            key=(data['epoch'],scan.header.stamp.sec,scan.header.stamp.nanosec)
            if key==last:continue
            last=key
            scene=motion_scene(scan,h,data['reference_s'],timing_verified=data['timing_verified'],
                epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
            sources_now,reason=cache.observe(scene,h)
            if event['data']['output'][2]['reason']!='follow_front_unobserved':continue
            cases=[]
            for label,steer in [('recorded_previous',data['previous_steering_rad']),('left_sensitivity',.175417)]:
                start=time.perf_counter()
                kwargs={}
                if args.protected_scope:
                    kwargs['protected_sensor_pose']=(motion.lidar_x_m,motion.lidar_y_m,motion.lidar_yaw_rad)
                result=sweep_support(sources_now,data['speed_mps'],.2,steer,data['previous_steering_rad'],config,**kwargs)
                elapsed=time.perf_counter()-start
                same=None
                if reference:
                    previous=reference.sweep_support(sources_now,data['speed_mps'],.2,steer,data['previous_steering_rad'],config,**kwargs)
                    same=previous==result;mismatches+=not same
                cases.append(dict(label=label,result=result,elapsed_ms=elapsed*1000,reference_equal=same))
            rows.append(dict(seq=event['seq'],time_s=data['reference_s'],cache_reason=reason,cases=cases))
            if len(rows)>=8:break
    report=dict(scope='first_eight_unknowns_sampled_module_no_runtime',protected_scope=args.protected_scope,rows=rows,input_sha256=before,
        source_sha256=sources,input_unchanged=before==digest(args.capture),
        source_unchanged=all(digest(ROOT/name)==h for name,h in sources.items()),runtime_changed=False,
        compared_source_sha256=digest(args.compare_source) if args.compare_source else None,
        reference_compared=bool(reference),
        reference_mismatches=mismatches if reference else None,
        previous_trace_success_changed=False)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(rows=len(rows),reasons=dict(Counter(c['result']['reason'] for r in rows for c in r['cases'])),
        maximum_case_ms=max(c['elapsed_ms'] for r in rows for c in r['cases']),
        reference_compared=report['reference_compared'],reference_mismatches=report['reference_mismatches'],
        input_unchanged=report['input_unchanged'],source_unchanged=report['source_unchanged'])))


if __name__=='__main__':main()
