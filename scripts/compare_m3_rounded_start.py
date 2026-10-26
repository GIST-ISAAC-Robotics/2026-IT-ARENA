#!/usr/bin/env python3
"""M5 저장 순차 입력에서 기본 M3와 선택형 원판 점유를 비교. 개루프, 명령 미발행."""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np
import yaml

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.m3_execution_trial import ExecutionConfig,M3ExecutionFollow
from arena_autonomy import rounded_occupancy_candidate
from analyze_grid_start_constraints import decode_ranges
from probe_grid_departure_observation import motion_history,append_stationary_measurements
from probe_grid_follow import ROOT,CAMERA,VEHICLE,json_value

INPUT=ROOT/'artifacts/validation/2026-10-26/m5_start_constraints_v1/run7/data/report.json'


def replay(case,sensor,lidar,cfg,rounded):
    execution=ExecutionConfig(command_fallback=True,release_to_baseline=True,
        small_center_straight=True,straight_surface_commitment=True,rounded_occupancy=rounded)
    controller=M3ExecutionFollow(cfg,execution,allow_simulation_trial=True)
    h=motion_history(cfg.wheelbase_m,sensor)
    rows=[]
    for stored in case['calls']:
        scan=case['scans'][stored['frame']]
        ranges=decode_ranges(scan['ranges']); stamps=scan['first_s']+np.arange(500)*.0002
        now=stored['t_s'];append_stationary_measurements(h,now)
        scene=scene_from_scan(ranges.tolist(),angle_min=-math.pi,angle_increment=math.tau/500,
            range_min=lidar['range_min_m'],range_max=lidar['range_max_m'],acquired_at_s=stamps.tolist(),
            t_ref_s=now,sensor_xy_m=tuple(sensor['xyz_m'][:2]),rear_half_angle_deg=sensor['masked_rear_half_angle_deg'])
        speed,steering,meta=controller.command(scene,h,0.,'left',0.,.02)
        rows.append(dict(t_s=now,frame=stored['frame'],fault=stored['fault'],speed_mps=speed,steering_rad=steering,meta=meta))
    return dict(rounded=rounded,calls=rows,positive_calls=sum(r['speed_mps']>0 for r in rows),
        fault_positive_calls=sum(r['speed_mps']>0 for r in rows if r['fault']),
        first_positive_s=next((r['t_s'] for r in rows if r['speed_mps']>0),None),
        reasons=dict(Counter(r['meta']['reason'] for r in rows)))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();output=args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/validation'):parser.error('new validation subdirectory required')
    output.mkdir(parents=True,exist_ok=False)
    vehicle=yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    sensor=yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    prior=json.loads(INPUT.read_text(encoding='utf-8'))
    cfg=FollowConfig(wheelbase_m=vehicle['drivetrain']['wheelbase_m'],occupancy_timing_mode='per_ray_candidate',
        boundary_evidence_mode='corner_distance_candidate',corridor_fallback_enabled=True)
    paths={Path(__file__),INPUT,CAMERA,VEHICLE,ROOT/'tests/test_m3_rounded_occupancy.py'}
    for module in tuple(sys.modules.values()):
        name=getattr(module,'__file__',None)
        if name:
            p=Path(name).resolve()
            if p.is_relative_to(ROOT) and p.suffix=='.py':paths.add(p)
    sources={p.relative_to(ROOT).as_posix():p.read_bytes() for p in sorted(paths)}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name,content in sources.items():z.writestr(name,content)
    results=[]
    for case in prior['cases']:
        if case['slot'] not in (2,5):continue
        results.append(dict(slot=case['slot'],bias=case['bias_m'],delay=case['additional_delay_s'],fault=case['fault_selected'],
            comparisons=[replay(case,sensor,vehicle['sensors']['lidar_2d'],cfg,r) for r in (False,True)]))
    report=dict(scope='M3 saved sequential scans, ego fixed, no command publication',cases=results,
        source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in sources.items()},
        sources_unchanged=all((ROOT/n).read_bytes()==b for n,b in sources.items()),driving_authorized=False)
    (output/'report.json').write_text(json.dumps(json_value(report),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    summary=[{**{k:v for k,v in case.items() if k!='comparisons'},'comparisons':[
        {k:v for k,v in c.items() if k!='calls'} for c in case['comparisons']]} for case in results]
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    fault_positive=sum(c['fault_positive_calls'] for r in results for c in r['comparisons'])
    print(json.dumps(dict(cases=len(results),sources_unchanged=report['sources_unchanged'],fault_positive_calls=fault_positive)))
    return 0 if report['sources_unchanged'] and not fault_positive else 1


if __name__=='__main__':raise SystemExit(main())
