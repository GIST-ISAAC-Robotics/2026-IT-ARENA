"""G2 저장 궤적의 측정면·가림·현재/최근 관측 비교. 정답은 평가기에만 있다.

기존 M3 외곽 질의/관측 함수에 합성 순차 스캔을 넣는다. 경로·자세는 정답이며
센서 경로 생성·제동·Gazebo·우회 허가의 검증이 아니다. 숨은 차량 부재를 가정하지 않는다.
"""
import argparse
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np

from arena_autonomy.lateral_intent import LateralIntentConfig, committed_progress_m
from arena_autonomy.m3_execution_trial import (ExecutionConfig, body_queries, envelope_distance,
                                              support, masked_only, ray_padding)
from arena_autonomy.m3_observed_follow import command_arc
from arena_autonomy.observation_support import depth_support
from g2_visibility_probe import sequential_scene, local_pose, pose_at

ROOT = Path(__file__).resolve().parents[1]
STAGES = (.10, .40, 1., 1.50, 1.85, 2.60)


def validate_trace(rows):
    keys=('t_s','rear_x_m','rear_y_m','yaw_rad','applied_speed_mps','applied_steering_rad')
    if len(rows)<2 or any(any(not isinstance(r.get(k),(int,float)) or isinstance(r[k],bool)
                              or not math.isfinite(r[k]) for k in keys) for r in rows):
        raise ValueError('finite recorded trajectory required')
    if any(abs(r['t_s']-i*.02)>1e-9 for i,r in enumerate(rows)):
        raise ValueError('50 Hz trace starting at zero required')
    if any(not 0 <= r['applied_speed_mps'] <= .8 or abs(r['applied_steering_rad'])>.37 for r in rows):
        raise ValueError('registered speed/steering bounds exceeded')


def recent_frames(rows, now, height, rear_half, reference):
    """SceneHistory와 같이 가장 오래된 광선 취득 시각부터 .3초 미만만 쓴다."""
    frames=[]
    for delta in (.1,.2,.3):
        if now-delta < .0998: continue
        old,_,old_ref=sequential_scene(rows,now-delta,height,target_y=.15,rear_half_angle_deg=rear_half)
        if now-old.source_first_s >= .3:
            continue
        frames.append((old,local_pose(old_ref,reference)))
    return frames


def inspect_stage(rows, stage_x, *, height=.12, rear_half=30.):
    cfg=LateralIntentConfig(surface_policy='two_wheel_grass')
    execution=ExecutionConfig()
    index=min(range(len(rows)),key=lambda i:abs(rows[i]['rear_x_m']-stage_x))
    row=rows[index]
    now=row['t_s']
    scene,labels,reference=sequential_scene(rows,now,height,target_y=.15,rear_half_angle_deg=rear_half)
    frames=recent_frames(rows,now,height,rear_half,reference)
    age=scene.t_ref_s-scene.source_first_s
    speed=row['applied_speed_mps']
    distance=envelope_distance(speed,age,.02,execution)
    checks=[]
    steering=(row['applied_steering_rad'],rows[max(0,index-1)]['applied_steering_rad'])
    for angle in sorted(set(steering)):
        poses=command_arc(angle,distance,cfg)
        q,thin=body_queries(scene,poses,np.full(len(poses),angle),cfg)
        pad=ray_padding(scene,cfg,execution.history_motion_bound_mps)
        ok=depth_support(scene,q,pad)
        missing=q[~ok]
        checks.append(dict(kind='applied_or_previous_command_stop_arc',steering_rad=angle,
                           distance_m=distance,queries=len(q),unknown_intersections=thin,
                           current=support(scene,[],q,cfg,replace(execution,masked_history_support=False)),
                           recent=support(scene,frames,q,cfg,execution),
                           missing_xy=missing.tolist(),
                           missing_mask_only=int(masked_only(scene,missing,pad).sum()) if len(missing) else 0))
    # 저장 궤적의 미래 부분은 평가 입력이며 정지 원호/제어 입력을 대체하지 않는다.
    # 기록 속도의 확약 거리까지 선택하며 자세 사이 연속 자유 공간은 인증하지 않는다.
    committed=committed_progress_m(speed,age,cfg)
    future=[]
    angles=[]
    travelled=0.
    for j in range(index,len(rows)):
        future.append(local_pose(pose_at(rows,rows[j]['t_s']),reference))
        angles.append(rows[j]['applied_steering_rad'])
        if travelled>=committed: break
        travelled+=rows[j]['applied_speed_mps']*.02
    q,thin=body_queries(scene,np.asarray(future),np.asarray(angles),cfg)
    checks.append(dict(kind='stored_future_path',distance_m=committed,
                       trace_reaches_distance=travelled>=committed,queries=len(q),unknown_intersections=thin,
                       current=support(scene,[],q,cfg,replace(execution,masked_history_support=False)),
                       recent=support(scene,frames,q,cfg,execution)))
    return dict(stage_rear_x_m=stage_x,actual_rear_x_m=row['rear_x_m'],t_ref_s=now,
                height_m=height,rear_half_angle_deg=rear_half,
                sensor_pose_m=[.06,0.,.095],source_first_s=scene.source_first_s,
                source_last_s=scene.source_last_s,history_frames=len(frames),
                oldest_history_ray_age_s=max((now-s.source_first_s for s,_ in frames),default=None),
                target_returns=sum(l=='target' and r.status=='return' for l,r in zip(labels,scene.rays)),
                target_masked=sum(l=='target' and r.status=='masked' for l,r in zip(labels,scene.rays)),
                checks=checks,driving_authorized=False,truth_transport_evaluation_only=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    out=args.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts/validation'):
        parser.error('new validation directory required')
    trace=args.input.resolve(); raw_trace=trace.read_bytes(); rows=json.loads(raw_trace)
    validate_trace(rows)
    out.mkdir(parents=True,exist_ok=False)
    # 12 cm 표적·6 cm 낮은 표적·12 cm 표적의 가림 없는 대조. 실제 장착 변경은 아니다.
    sources=['scripts/g2_surface_visibility.py','scripts/g2_visibility_probe.py',
             'scripts/g2_tracking_probe.py','scripts/g2_geometry_probe.py','scripts/g0_geometry.py',
             'tests/test_g2_surface_visibility.py']
    sources+=['src/arena_autonomy/arena_autonomy/'+n+'.py' for n in
              ('local_scene','lateral_intent','m3_execution_trial','m3_observed_follow',
               'observation_support','observation_lateral','observed_lateral_trial','scan_topology','linear_observation')]
    raw={p:(ROOT/p).read_bytes() for p in sources}
    with zipfile.ZipFile(out/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p,b in raw.items(): z.writestr(p,b)
    result=[inspect_stage(rows,x,height=h,rear_half=mask)
            for h,mask in ((.12,30.),(.06,30.),(.12,0.)) for x in STAGES]
    unchanged=all((ROOT/p).read_bytes()==b for p,b in raw.items())
    input_unchanged=trace.read_bytes()==raw_trace
    report=dict(scope='stored trajectory synthetic sequential scan observation only',stages=result,
                source_unchanged=unchanged,input_unchanged=input_unchanged,
                source_sha256={p:hashlib.sha256(b).hexdigest() for p,b in raw.items()},
                input_path=str(trace),input_sha256=hashlib.sha256(raw_trace).hexdigest(),
                driving_authorized=False,continuous_sweep_certified=False,
                actual_odometry_verified=False,mask_zero_is_counterfactual_not_adopted=True,
                limits=['Exact poses and reference queries are evaluation truth, not controller input.',
                        'No sensor-derived path, noise, dynamics, emergency braking or runtime integration.',
                        'A low object may be invisible even when ray support passes.',
                        'Current and recent support are sampled requirements, not all-space safety.'])
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(dict(stages=len(result),source_unchanged=unchanged,input_unchanged=input_unchanged)))
    return 0 if unchanged and input_unchanged else 1


if __name__=='__main__':
    raise SystemExit(main())
