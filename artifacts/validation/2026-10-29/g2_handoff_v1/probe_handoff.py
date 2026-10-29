"""자동 선택 시제품의 센서 폐루프. 정답은 센서 생산과 평가에만 사용한다."""
from collections import Counter
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import zipfile
import numpy as np

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
import probe_bypass_entry_sequence as probe
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.follow_trace import snapshot_call,freeze
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from bypass_probe_sensors import ProbeSensors
from g0_geometry import TruthBox
from handoff_prototype import HandoffPrototype


def rollout(y, target_speed, profile, duration=12.):
    controller=HandoffPrototype()
    motion=MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06,history_s=5.)
    history=MotionHistory(motion)
    guard=probe.ShadowSafety(RecoveryPolicy(True,True,True),controller.normal.config,30.)
    sensors=ProbeSensors(profile,17)
    original_boxes=probe.evaluation_boxes
    def boxes(at,target_y=0.,restart=False):
        result=original_boxes(at,target_y,False)
        target=result[-1]
        result[-1]=TruthBox(target.x_m+target_speed*max(0.,at-.5),target.y_m,target.height_m)
        return result
    probe.evaluation_boxes=boxes
    records=[];physical=[];travel=0.
    pose=(-.0675,0.,0.);actual_speed=previous=0.
    for k in range(25):
        now=k*.02
        physical.append(dict(t=now,rear_pose=list(pose),speed=0.,steering=0.))
        mean,gyro,count=sensors.measure_motion(now,0.,0.,0.)
        history.add_mean_wheel(now,mean);history.add_gyro(now,gyro)
    try:
        for k in range(round((duration+.6)/.02)+1):
            now=round(.5+k*.02,9)
            physical.append(dict(t=now,rear_pose=list(pose),speed=actual_speed,steering=previous))
            mean,gyro,count=sensors.measure_motion(now,travel,actual_speed,previous)
            history.add_mean_wheel(now,mean);history.add_gyro(now,gyro)
            if k%5==0:raw=probe.render_scan(physical,now,y,False,sensors)
            scene=motion_scene(raw,history,now,timing_verified=True,rear_half_angle_deg=30.)
            measured=float(history.wheels.available(now)[1][-1])
            began=time.perf_counter()
            if now>=.5+duration:
                # 명시 STOP은 모드 판단보다 우선한다. 다음 재개에 기존 경로를 재사용하지 않는다.
                controller.reset();requested=(0.,previous);detail=dict(mode='STOP',reason='scheduled_stop')
            else:
                target,angle,detail=controller.command(scene,history,measured,'left',previous,.02,raw_scan=raw)
                requested=(target,angle)
            frozen=snapshot_call(raw,history,now,0,measured,'left',previous,.02,True,30.)
            safe,status=guard.step(frozen,motion,requested)
            cost=(time.perf_counter()-began)*1000
            if not 0<=safe[0]<=requested[0]+1e-12:raise ValueError('guard raised speed')
            evaluation=probe.evaluation_metrics(pose,previous,now,y,False)
            records.append(dict(t=now,requested=requested,safe=safe,detail=detail,
                safety_reason=status['reason'],evaluation=evaluation,measured_speed_mps=measured,
                actual_speed_mps=actual_speed,processing_ms=cost))
            if evaluation['contact_sample']:break
            next_speed=float(np.clip(safe[0],max(0.,actual_speed-2.*.02),actual_speed+1.*.02))
            next_steering=float(safe[1])
            physical[-1].update(speed=next_speed,steering=next_steering)
            travel+=next_speed*.02
            pose=probe.bicycle_step(*pose,next_speed,next_steering,.02,.135)
            actual_speed,previous=next_speed,next_steering
    finally:probe.evaluation_boxes=original_boxes
    transitions=[]
    for r in records:
        if not transitions or transitions[-1]['mode']!=r['detail']['mode']:
            transitions.append(dict(time_s=r['t'],mode=r['detail']['mode'],reason=r['detail']['reason'],
                                    center_pose=r['evaluation']['center_pose']))
    return dict(target_y=y,evaluation_target_speed_mps=target_speed,sensor_profile=sensors.description(),
        scope='synthetic sequential sensors, ideal bicycle, independent guard; no ROS/Gazebo/hardware validation',
        final_progress_m=records[-1]['evaluation']['center_pose'][0],
        final_lateral_m=records[-1]['evaluation']['center_pose'][1],
        contact_samples=sum(r['evaluation']['contact_sample'] for r in records),
        max_wheels_touching_grass=max(r['evaluation']['wheels_touching_grass'] for r in records),
        min_gap_m=min(r['evaluation']['minimum_body_gap_m'] for r in records),
        stopped=records[-1]['detail']['mode']=='STOP' and records[-1]['actual_speed_mps']<.001,
        mode_transitions=transitions,reason_counts=dict(Counter(r['detail']['reason'] for r in records)),
        median_processing_ms=float(np.median([r['processing_ms'] for r in records])),
        max_processing_ms=max(r['processing_ms'] for r in records),records=records)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',choices=('prototype1','prototype2'),default='prototype1')
    args=parser.parse_args()
    out=BASE/args.output;out.mkdir(exist_ok=False)
    paths=list((ROOT/'src').rglob('*.py'))+list(BASE.glob('*.py'))+list((ROOT/'scripts').glob('*bypass*.py'))
    paths += [ROOT/'scripts/g2_tracking_probe.py',ROOT/'scripts/g0_geometry.py',
              ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1/safety_shadow.py']
    sources={str(p.relative_to(ROOT)):p.read_bytes() for p in paths}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for n,v in sources.items():z.writestr(n,v)
    cases=[('right_ideal',-.15,0.,'ideal'),('left_combined',.15,0.,'combined'),
           ('right_combined',-.15,0.,'combined'),('moving_combined',-.15,.25,'combined')]
    reports=[]
    for name,y,speed,profile in cases:
        result=rollout(y,speed,profile)
        (out/(name+'.json')).write_text(json.dumps(freeze(result),indent=2,allow_nan=False)+'\n')
        small={k:v for k,v in result.items() if k!='records'}
        reports.append(dict(name=name,**small));print(json.dumps(small),flush=True)
    report=dict(cases=reports,source_sha256={n:hashlib.sha256(v).hexdigest() for n,v in sources.items()},
                source_unchanged=all((ROOT/n).read_bytes()==v for n,v in sources.items()))
    (out/'report.json').write_text(json.dumps(freeze(report),indent=2,allow_nan=False)+'\n')
    if not report['source_unchanged']:raise RuntimeError('source changed during trial')
