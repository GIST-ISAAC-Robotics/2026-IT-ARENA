"""이동→급정지→재출발 각본 한 개. 같은 초기 조건에서 두 정책의 실제 센서가 갈라지는 이상 기구 비교."""
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch
import json
import math
import numpy as np

import probe_bypass_entry_sequence as probe
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.follow_trace import snapshot_call
from arena_autonomy.lidar_motion import MotionHistory, MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from arena_vehicle_interface.drive_feedback import MotorEncoderSpec

HERE = Path(__file__).resolve().parent
PRIOR = HERE.parents[1]/'2026-10-31/g1_continuous_prefix_course_v1/run1/follow_decisions.jsonl'
DT = .02


def lead(at):
    # 1.00 m 중심 간격(차체 사이 0.80 m). 3초 후 즉시 정지하고 1.5초 뒤 재출발.
    elapsed=max(0., at-.5)
    return 1.+.35*(min(elapsed,3.)+max(0.,elapsed-4.5))


def evaluate(enabled):
    with PRIOR.open() as f:
        header=json.loads(next(f))['data']
    cfg=replace(FollowConfig(**header['follow_config']),signed_motion_policy='forward_drive')
    execution=replace(HandoffExecutionConfig.from_mapping(header['execution_config']),moving_follow_reserve=enabled)
    controller=BypassHandoffFollow(cfg,execution,allow_simulation_trial=True)
    guard=probe.ShadowSafety(RecoveryPolicy(True,True,True), cfg, 30.)
    motion=MotionConfig(**header['motion_config'])
    history=MotionHistory(motion)
    # 현재 구매 기준으로 양자화한 단일 모터 카운트·동일 gyro 오차 시드.
    sensors=probe.ProbeSensors('motion',17,motor_spec=MotorEncoderSpec(28,2.5*61/17,.0325))
    rows=[]; physical=[]; pose=(-.0675,0.,0.); travel=speed=angle=0.
    def feedback(t):
        mean,gyro,_=sensors.measure_motion(t,travel,speed,angle)
        history.add_mean_wheel(t,mean); history.add_gyro(t,gyro)
        return mean*.0325
    for k in range(25):
        t=round(k*DT,9); feedback(t)
        physical.append(dict(t=t,rear_pose=pose,speed=0.,steering=0.))
    original=probe.evaluation_boxes
    def boxes(at,*args):
        from g0_geometry import TruthBox
        return original(at)[:-1]+[TruthBox(lead(at),0.,.12)]
    with patch.object(probe,'evaluation_boxes',boxes):
        for k in range(401):
            t=round(.5+k*DT,9); measured=feedback(t)
            physical.append(dict(t=t,rear_pose=pose,speed=speed,steering=angle))
            if k%5==0: raw=probe.render_scan(physical,t,sensors=sensors)
            scene=motion_scene(raw,history,t,timing_verified=True,rear_half_angle_deg=30.)
            v,s,d=controller.command(scene,history,measured,'left',angle,DT,raw_scan=raw)
            f=snapshot_call(raw,history,t,0,measured,'left',angle,DT,True,30.)
            safe,_=guard.step(f,motion,(v,s))
            assert 0<=safe[0]<=v+1e-9
            e=probe.evaluation_metrics(pose,angle,t,0.,False)
            rows.append(dict(t=t,command=float(v),safe=safe[0],actual=speed,gap=e['target_body_gap_m'],
                             x=e['center_pose'][0],mode=controller.mode,reason=d.get('follow_entry_space',{}).get('reason'),
                             contact=e['contact_sample']))
            if e['contact_sample'] or k==400: break
            speed=float(np.clip(safe[0],max(0.,speed-2*DT),speed+DT)); angle=safe[1]
            physical[-1].update(speed=speed,steering=angle)
            pose=probe.bicycle_step(*pose,speed,angle,DT,.135); travel+=speed*DT
    moving=[r for r in rows if not 3.5<=r['t']<5.]
    return dict(option=enabled,rows=len(rows),duration_s=rows[-1]['t']-.5,progress_m=rows[-1]['x'],
        sampled_contact=any(r['contact'] for r in rows),min_gap_m=min(r['gap'] for r in rows),
        braking_phase_min_gap_m=min((r['gap'] for r in rows if 3.5<=r['t']<5.),default=None),
        lead_moving_zero_command_calls=sum(r['command']==0 for r in moving),
        lead_moving_low_speed_calls=sum(r['actual']<.05 for r in moving),
        moving_reserve_calls=sum(r['reason']=='advancing_patch_follow' for r in rows),
        modes=sorted(set(r['mode'] for r in rows)))


if __name__=='__main__':
    p=HERE/'short_traffic_result.json'
    if p.exists(): raise FileExistsError(p)
    result={'scope':'Two independent ideal bicycle rollouts; evaluation actor schedule not an input; no Gazebo/MCU/tire/real car.',
            'baseline':evaluate(False),'candidate':evaluate(True)}
    payload=json.dumps(result,indent=2)
    with p.open('x') as f: f.write(payload+'\n')
    print(payload)
