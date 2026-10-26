"""저장 M5 거리 편향의 경로 투영과 공통 벽 법선을 대조한다. 제어 변경 없음."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np
import yaml

from arena_autonomy.lateral_intent import LateralIntentConfig,ego_state,surface_check
from arena_autonomy.m3_execution_trial import ExecutionFrame,straight_candidate,straight_surface_prefix
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from analyze_grid_start_constraints import decode_ranges
from probe_grid_follow import ROOT,CAMERA,VEHICLE,json_value
from probe_grid_departure_observation import motion_history,append_stationary_measurements
from paired_wall_interval_probe import binned_pair_probe
from parallel_wall_normal_probe import search_normal


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); output=args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/validation'):parser.error('validation directory required')
    output.mkdir(parents=True,exist_ok=False)
    base=ROOT/'artifacts/validation/2026-10-26/m5_start_constraints_v1'
    inputs=[base/'run7/data/report.json',base/'run8/data/report.json',CAMERA,VEHICLE]
    frozen={p.relative_to(ROOT).as_posix():p.read_bytes() for p in inputs}
    for module in tuple(sys.modules.values()):
        name=getattr(module,'__file__',None)
        if name:
            path=Path(name).resolve()
            if path.is_relative_to(ROOT) and path.suffix=='.py':frozen[path.relative_to(ROOT).as_posix()]=path.read_bytes()
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name,raw in frozen.items():
            if name.endswith('.py'):z.writestr(name,raw)
    scans,outputs=[json.loads(p.read_text(encoding='utf-8')) for p in inputs[:2]]
    sensor=yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    vehicle=yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    cfg=LateralIntentConfig(enabled=True,surface_policy='two_wheel_grass',wheelbase_m=vehicle['drivetrain']['wheelbase_m'])
    lidar=vehicle['sensors']['lidar_2d']; rows=[]
    for case in outputs['cases']:
        if case['slot']!=5 or case['delay'] or case['fault']:continue
        source=next(c for c in scans['cases'] if c['slot']==case['slot'] and c['bias_m']==case['bias']
                    and not c['additional_delay_s'] and not c['fault_selected'])
        calls=case['comparisons'][1]['calls']
        baseline=LocalFollow(FollowConfig(wheelbase_m=cfg.wheelbase_m,occupancy_timing_mode='per_ray_candidate',
            boundary_evidence_mode='corner_distance_candidate',corridor_fallback_enabled=True))
        history=motion_history(cfg.wheelbase_m,sensor)
        for index,row in enumerate(calls):
            meta=row['meta']; raw=source['scans'][row['frame']]; now=row['t_s']
            stamps=raw['first_s']+np.arange(500)*.0002
            scene=scene_from_scan(decode_ranges(raw['ranges']).tolist(),angle_min=-math.pi,angle_increment=math.tau/500,
                range_min=lidar['range_min_m'],range_max=lidar['range_max_m'],acquired_at_s=stamps.tolist(),t_ref_s=now,
                sensor_xy_m=tuple(sensor['xyz_m'][:2]),rear_half_angle_deg=sensor['masked_rear_half_angle_deg'])
            append_stationary_measurements(history,now)
            basic=baseline.command(scene,history,0.,'left',0.,.02)
            if index not in (49,len(calls)-1):continue
            if 'path_points' not in basic[2] or 'straight' not in meta.get('m3_lateral',{}).get('checks',{}):continue
            old=meta['m3_lateral']['checks']['straight']; frame=ExecutionFrame(basic[2]['path_points'])
            candidate=straight_surface_prefix(straight_candidate(frame,ego_state(frame,cfg),cfg),old['surface_scope']['end_m'])
            surface=surface_check(frame,candidate,cfg,current_pose=(0.,0.,0.)); need=surface['s_range_needing_edge']
            matches=basic[2]['reason']==meta['baseline_reason'] and bool(np.allclose(basic[:2],meta['baseline_output'],rtol=0.,atol=1e-12))
            if not need:
                rows.append(dict(bias_m=case['bias'],frame=row['frame'],t_s=now,
                    baseline_output_matches_saved=matches,normal_fits=None,
                    reason='current surface does not request edge support; no comparison interval'))
                continue
            lo,hi=need[0]-.05,need[1]+.05
            rays=[r for r in scene.rays if r.status=='return']
            p=np.array([r.hit_xy_m for r in rays]); d=np.array([r.direction_xy for r in rays])
            s,l=frame.project(p)
            signs=np.where(l>=0,1.,-1.)
            take=(s>=lo)&(s<=hi)&(np.abs(l-signs*cfg.wall_offset_m)<=2*cfg.budget.range_bias_m)
            theta=float(frame.heading(np.array([(lo+hi)/2]))[0])
            bins=binned_pair_probe(frame,p,lo,hi,cfg)
            inconsistencies=[]
            for section in bins['bins']:
                if section['status']!='inconsistent':continue
                mask=take&(s>=section['s_lo'])&(s<section['s_hi'])
                centers=l[mask]-signs[mask]*cfg.wall_offset_m
                inconsistencies.append(dict(s_lo=section['s_lo'],s_hi=section['s_hi'],
                    gap_m=float(np.max(centers-cfg.budget.range_bias_m)-np.min(centers+cfg.budget.range_bias_m))))
            fits={name:search_normal(p[take],d[take],signs[take],theta_lo=theta-.08,theta_hi=theta+.08,
                wall_half_m=cfg.wall_offset_m,error_m=cfg.budget.range_bias_m,radial=radial)
                for name,radial in [('euclidean',False),('radial',True)]}
            # 평가 전용: 이미 주입했다고 기록한 편향을 빼 모형/기하 잔차와 센서 오차를 분리한다.
            # 이 데이터는 제어기나 노면 판정으로 전달하지 않는다.
            de_biased=p[take]-case['bias']*d[take]
            exact_fit=search_normal(de_biased,d[take],signs[take],theta_lo=theta-.08,theta_hi=theta+.08,
                wall_half_m=cfg.wall_offset_m,error_m=0.)
            result=dict(bias_m=case['bias'],frame=row['frame'],t_s=now,
                baseline_output_matches_saved=matches,
                selected_points=int(sum(take)),selection='diagnostic doubled residual strip, not wall classification',
                path_heading_rad=theta,bin_inconsistencies=inconsistencies,normal_fits=fits,
                evaluation_known_bias_removed_fit=exact_fit,
                evaluation_selected_rays=[dict(source_index=rays[i].source_index,hit_xy_m=p[i].tolist(),
                    direction_xy=d[i].tolist(),side=float(signs[i])) for i in np.flatnonzero(take)],
                per_bin_compatible=bins['compatible_bins'],per_bin_total=len(bins['bins']))
            rows.append(result)
            print(json.dumps(json_value({k:v for k,v in result.items() if k!='evaluation_selected_rays'}),ensure_ascii=False),flush=True)
    unchanged=all((ROOT/n).read_bytes()==b for n,b in frozen.items())
    report=dict(cases=rows,sources_unchanged=unchanged,
        source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},driving_authorized=False,
        scope='fixed-pose range-only parallel-wall hypothesis; no controller/road permission change')
    (output/'summary.json').write_text(json.dumps(json_value(report),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    return 0 if unchanged and len(rows)==6 and all(r['baseline_output_matches_saved'] for r in rows) else 1


if __name__=='__main__':raise SystemExit(main())
