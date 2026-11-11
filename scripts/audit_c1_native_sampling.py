#!/usr/bin/env python3
"""비공개 SDK 기록의 원천 배열→점별 보정→관측 관문 감사. 시각은 합성 시험값이다."""
import argparse
from collections import Counter
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path

import numpy as np

from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable
from arena_autonomy.measurement_scan import MeasurementScan, sampling_gaps, angular_cell_widths
from arena_autonomy.observation_support import source_valid, transport_scene
from arena_autonomy.scene_motion import measurement_motion_scene


def audit(scans_path, points_path):
    measured = {int(r['scan']): int(r['count']) for r in csv.DictReader(scans_path.open(newline=''))
                if r['measured']=='1' and int(r['result'])==0 and int(r['count'])>0}
    h=MotionHistory(MotionConfig(lidar_x_m=.06,wheel_radius_m=.0325))
    for t in np.arange(.9,1.31,.005):
        h.add_mean_wheel(float(t),1./.0325)
        h.add_gyro(float(t),.8)
    seen=set();counts=[];gaps=[];spans=[];seams=[];statuses=Counter();rejected_estimates=0
    with points_path.open(newline='') as f:
        for sid, group in itertools.groupby(csv.DictReader(f),lambda r:int(r['scan'])):
            if sid not in measured: continue
            rows=list(group);n=len(rows)
            if sid in seen or n!=measured[sid] or [int(r['index']) for r in rows]!=list(range(n)):
                raise ValueError('source points lost/reordered/duplicated')
            angles=tuple(math.pi-int(r['angle_q14'])*math.pi/32768 for r in rows)
            distances=tuple(int(r['dist_mm_q2'])/4000 if int(r['quality']) else math.nan for r in rows)
            times=tuple(1.+i*.0002 for i in range(n))
            raw=MeasurementScan(distances,angles,times)
            try: measurement_motion_scene(raw,h,1.15,timing_verified=True)
            except MotionUnavailable as e:
                if str(e)!='scan_timing_unverified': raise
                rejected_estimates+=1
            else: raise AssertionError('unverified SDK time passed')
            # 시각은 API 검증용 합성값이며 실측 취득 시각이 아니다. 정답 자세 불사용.
            from dataclasses import replace
            sample=replace(raw,time_basis='simulation_acquisition')
            scene=measurement_motion_scene(sample,h,1.15,timing_verified=True,rear_half_angle_deg=0.)
            if not source_valid(scene): raise AssertionError('valid native order rejected')
            carried=transport_scene(scene,h,1.17)
            if not source_valid(carried) or carried.source_angles_rad!=angles: raise AssertionError('transport lost source')
            if tuple(r.acquired_at_s for r in scene.rays)!=times: raise AssertionError('time changed')
            if tuple(r.source_index for r in scene.rays)!=tuple(range(n)): raise AssertionError('index changed')
            widths=angular_cell_widths(scene,range(n))
            if not all(0<a<math.radians(1) and 0<b<math.radians(1) for a,b in widths):
                raise AssertionError('cell width invalid')
            for i,ray in enumerate(scene.rays):
                valid=math.isfinite(distances[i]) and .05<=distances[i]<=12.
                if (ray.status=='return')!=valid: raise AssertionError('invalid range became return')
                if valid and not math.isclose(math.dist(ray.ray_origin_xy_m,ray.hit_xy_m),distances[i],rel_tol=1e-12,abs_tol=1e-12):
                    raise AssertionError('range changed')
                statuses[ray.status]+=1
            ds=sampling_gaps(angles)
            gaps.extend(abs(v) for v in ds);spans.append(abs(sum(ds)))
            seams.append(math.tau-abs(sum(ds)));seen.add(sid);counts.append(n)
    if seen!=measured.keys(): raise ValueError('missing measured scans')
    return dict(scans=len(seen),measurements=sum(counts),counts_min_median_max=[min(counts),float(np.median(counts)),max(counts)],
        gap_deg_min_max=list(map(math.degrees,(min(gaps),max(gaps)))),
        span_deg_min_max=list(map(math.degrees,(min(spans),max(spans)))),
        seam_deg_min_max=list(map(math.degrees,(min(seams),max(seams)))),
        statuses=dict(statuses),source_preserved=True,source_gate_passed=True,
        unverified_estimate_rejected=rejected_estimates,timestamps='synthetic 200us, NOT hardware timing validation',
        synthetic_motion=dict(speed_mps=1.,yaw_rate_rad_s=.8),ros_runtime_connected=False,
        inputs_sha256={k:hashlib.sha256(p.read_bytes()).hexdigest() for k,p in [('scans',scans_path),('points',points_path)]})


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scans',type=Path,required=True);p.add_argument('--points',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():p.error('preserve previous output')
    result=audit(a.scans,a.points);a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(result))
