#!/usr/bin/env python3
"""저장된 M5 장면/기본 판단에 정지 직진 관측기를 대조. 실제 명령 발행 없음."""
import argparse
from collections import Counter
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
from arena_autonomy.startup_straight_candidate import StartupStraightCandidate
from analyze_grid_start_constraints import decode_ranges
from probe_grid_follow import ROOT, CAMERA, VEHICLE, stationary_history, json_value

INPUT = ROOT/'artifacts/validation/2026-10-26/m5_start_constraints_v1/run2/data/report.json'


def compare(row,sensor,lidar,wheelbase,enabled,rounded=False):
    candidate=StartupStraightCandidate(FollowConfig(wheelbase_m=wheelbase,
        occupancy_timing_mode='per_ray_candidate',boundary_evidence_mode='corner_distance_candidate',
        corridor_fallback_enabled=True),enabled=enabled,rounded_occupancy=rounded)
    history=stationary_history(wheelbase,sensor)
    ranges=decode_ranges(row['ranges'])
    result=[]
    for call in row['diagnosis']['calls']:
        stamps=call['scan_first_s']+np.arange(len(ranges))/sensor['rate_hz']/len(ranges)
        scene=scene_from_scan(ranges.tolist(),angle_min=-math.pi,angle_increment=math.tau/len(ranges),
            range_min=lidar['range_min_m'],range_max=lidar['range_max_m'],acquired_at_s=stamps.tolist(),
            t_ref_s=call['t_s'],sensor_xy_m=tuple(sensor['xyz_m'][:2]),rear_half_angle_deg=sensor['masked_rear_half_angle_deg'])
        baseline=(call['requested_speed_mps'],call['requested_steering_rad'],call['meta'])
        result.append(dict(t_s=call['t_s'],baseline_reason=call['meta']['reason'],
                           candidate=candidate.evaluate(scene,history,baseline,0.)))
    return dict(enabled=enabled,rounded_occupancy=rounded,calls=result,selected_calls=sum(r['candidate']['selected'] for r in result),
                reasons=dict(Counter(r['candidate']['reason'] for r in result)),driving_authorized=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); output=args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/validation'):parser.error('new validation subdirectory required')
    output.mkdir(parents=True,exist_ok=False)
    vehicle=yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    sensor=yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    prior=json.loads(INPUT.read_text(encoding='utf-8'))
    # 지연 import 모듈도 실행 전 보존한다.
    from arena_autonomy import rounded_occupancy_candidate
    sources={Path(__file__),INPUT,CAMERA,VEHICLE,ROOT/'tests/test_startup_straight_candidate.py',
             ROOT/'tests/test_rounded_occupancy_candidate.py'}
    for module in tuple(sys.modules.values()):
        name=getattr(module,'__file__',None)
        if name:
            p=Path(name).resolve()
            if p.is_relative_to(ROOT) and p.suffix=='.py':sources.add(p)
    data={p.relative_to(ROOT).as_posix():p.read_bytes() for p in sorted(sources)}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as bundle:
        for name,content in data.items():bundle.writestr(name,content)
    results=[]
    for row in prior['scenarios']:
        if not row['placement']['nonoverlapping']:continue
        results.append(dict(slot=row['ego_grid_index'],selection=row['selection'],shift=row['peer_displacement_m'],
            comparison=[compare(row,sensor,vehicle['sensors']['lidar_2d'],vehicle['drivetrain']['wheelbase_m'],enabled,rounded)
                        for enabled,rounded in ((False,False),(True,False),(True,True))]))
    report=dict(scope='OFF/ON stationary counterfactual only; no actuation or motion',scenarios=results,
        sources_unchanged=all((ROOT/n).read_bytes()==b for n,b in data.items()),
        source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in data.items()},driving_authorized=False)
    (output/'report.json').write_text(json.dumps(json_value(report),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    summary=[dict(slot=r['slot'],selection=r['selection'],shift=r['shift'],
        off_selected=r['comparison'][0]['selected_calls'],on_selected=r['comparison'][1]['selected_calls'],
        rounded_selected=r['comparison'][2]['selected_calls'],
        reasons=r['comparison'][1]['reasons'],rounded_reasons=r['comparison'][2]['reasons']) for r in results]
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(scenarios=len(summary),sources_unchanged=report['sources_unchanged'],
        off_selected=sum(r['off_selected'] for r in summary),on_selected=sum(r['on_selected'] for r in summary),
        rounded_selected=sum(r['rounded_selected'] for r in summary))))
    return 0 if report['sources_unchanged'] and not any(r['off_selected'] for r in summary) else 1


if __name__=='__main__':raise SystemExit(main())
