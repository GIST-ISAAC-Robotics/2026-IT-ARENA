"""전방 추종 중 진입 공간 보존 후보(기존 평가기 복사본, 기본/물성 불변).

g2_handoff_v1/probe_handoff.py의 같은 순차 LiDAR·모터축/gyro 생산·독립 보호
함수·이상 bicycle 평가기를 쓰되, 시제품 대신 연결 코드(BypassHandoffFollow)를
정책별로 실행한다. 정답 자세는 센서 생산과 평가에만 쓰며 제어기에 전달하지
않는다. ROS/Gazebo/실차 검증이 아니다.
"""
from collections import Counter
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import zipfile
import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'scripts')]
import probe_bypass_entry_sequence as probe
from scenario import Scenario
from arena_autonomy.bypass_entry_follow import BypassExecutionConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.follow_trace import snapshot_call, freeze, FollowTrace, replay_events
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from bypass_probe_sensors import ProbeSensors
from g0_geometry import TruthBox

CASES={
 'pullout_before':dict(kind='pullout',trigger_x=.95),
 'pullout_alongside':dict(kind='pullout',trigger_x=1.15),
 'slide_alongside':dict(kind='slide',trigger_x=1.15),
 'pullout_forward_blocked':dict(kind='pullout',trigger_x=1.15,blocked=True),
}

def rollout(name, policy, trace_path=None, duration=16., window='compare_then_trim_v0',reserve=False,reacquire='positive_command_v0'):
    target_y,start_x,target_speed,moving_s,profile=0.,1.2,0.,None,'combined'
    scenario=Scenario(**CASES[name])
    restart_after_s=None
    controller = BypassHandoffFollow(execution=HandoffExecutionConfig(preparation_policy=policy,
                                     persistence_window_policy=window,follow_entry_space_enabled=reserve,
                                     reacquire_policy=reacquire,entry_policy='corridor_local_v1',
                                     bypass=BypassExecutionConfig(corridor_policy='corridor_v2_signed_hold')),
                                     allow_simulation_trial=True)
    motion = MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=.06, history_s=5.)
    history = MotionHistory(motion)
    guard = probe.ShadowSafety(RecoveryPolicy(True, True, True), controller.normal.config, 30.)
    sensors = ProbeSensors(profile, 17)
    original_boxes = probe.evaluation_boxes
    trace = (FollowTrace(trace_path, motion, controller.config, mode='g1_g2_handoff_trial',
                         execution_config=controller.execution) if trace_path else None)

    def boxes(at,y=0.,restart=False):
        return scenario.boxes(at)
    probe.evaluation_boxes = boxes
    records = []; physical = []; travel = 0.
    pose = (-.0675, 0., 0.); actual_speed = previous = 0.
    for k in range(25):
        now = k*.02
        physical.append(dict(t=now, rear_pose=list(pose), speed=0., steering=0.))
        mean, gyro, count = sensors.measure_motion(now, 0., 0., 0.)
        history.add_mean_wheel(now, mean); history.add_gyro(now, gyro)
    try:
        for k in range(round((duration+.6)/.02)+1):
            now = round(.5+k*.02, 9)
            scenario.observe_ego(now,pose)
            physical.append(dict(t=now, rear_pose=list(pose), speed=actual_speed, steering=previous))
            mean, gyro, count = sensors.measure_motion(now, travel, actual_speed, previous)
            history.add_mean_wheel(now, mean); history.add_gyro(now, gyro)
            if k % 5 == 0:
                raw = probe.render_scan(physical, now, target_y, False, sensors)
            scene = motion_scene(raw, history, now, timing_verified=True, rear_half_angle_deg=30.)
            measured = float(history.wheels.available(now)[1][-1])
            began = time.perf_counter()
            if now >= .5+duration:
                # 명시 STOP은 모드 판단보다 우선한다. 다음 재개에 기존 경로를 재사용하지 않는다.
                controller.reset(); requested = (0., previous); detail = dict(handoff_state='STOP', reason='scheduled_stop')
                if trace:
                    trace.emit('reset', {'reason': 'scheduled_stop'})
            else:
                output = controller.command(scene, history, measured, 'left', previous, .02, raw_scan=raw)
                if trace:
                    trace.record_call(raw, history, now, 0, measured, 'left', previous, .02, True, 30., output)
                requested = output[:2]; detail = output[2]
            frozen = snapshot_call(raw, history, now, 0, measured, 'left', previous, .02, True, 30.)
            safe, status = guard.step(frozen, motion, requested)
            cost = (time.perf_counter()-began)*1000
            if not 0 <= safe[0] <= requested[0]+1e-12:
                raise ValueError('guard raised speed')
            evaluation = probe.evaluation_metrics(pose, previous, now, target_y, False)
            prep = detail.get('preparation') or {}
            reserved = detail.get('follow_entry_space') or {}
            normal = detail.get('normal') or {}
            records.append(dict(t=now, requested=list(requested), safe=list(safe), state=detail.get('handoff_state'),
                reason=detail.get('reason'), preparation=prep.get('reason'), prepared=prep.get('applied'),
                reserve=reserved,
                g1_reason=normal.get('reason'), safety_reason=status['reason'],
                ready=(detail.get('persistence') or {}).get('ready'),
                actor_pose=list(scenario.pose_at(now)),scenario_start=scenario.start,
                bypass_detail={k:(detail.get('bypass') or {}).get(k) for k in ('reason','entry_phase','corridor','slow_command_trial')},
                evaluation=evaluation, measured_speed_mps=measured, actual_speed_mps=actual_speed, processing_ms=cost))
            if evaluation['contact_sample']:
                break
            next_speed = float(np.clip(safe[0], max(0., actual_speed-2.*.02), actual_speed+1.*.02))
            next_steering = float(safe[1])
            physical[-1].update(speed=next_speed, steering=next_steering)
            travel += next_speed*.02
            pose = probe.bicycle_step(*pose, next_speed, next_steering, .02, .135)
            actual_speed, previous = next_speed, next_steering
    finally:
        probe.evaluation_boxes = original_boxes
        if trace:
            trace.close()
    transitions = []
    for r in records:
        if not transitions or transitions[-1]['state'] != r['state']:
            transitions.append(dict(time_s=r['t'], state=r['state'], reason=r['reason'],
                center_pose=r['evaluation']['center_pose'], target_gap_m=r['evaluation']['target_body_gap_m'],
                measured_speed_mps=r['measured_speed_mps'], actual_speed_mps=r['actual_speed_mps']))
    entry = next((r for r in records if r['reason'] == 'handoff_entry'), None)
    prepared = [r for r in records if r['prepared']]
    first_ready = next((r for r in records if r['ready']), None)
    summary = dict(case=name, policy=policy, persistence_window_policy=window,follow_entry_space_enabled=reserve,
        reacquire_policy=reacquire,duration_s=duration,
        reserve_applied_calls=sum(bool(r['reserve'].get('applied')) for r in records),
        reserve_zero_calls=sum(bool(r['reserve'].get('applied')) and r['requested'][0]==0 for r in records),
        target_y=target_y, target_start_x=start_x,
        evaluation_target_speed_mps=target_speed, target_moving_s=moving_s, sensor_profile=sensors.description(),
        scope='synthetic sequential sensors, ideal bicycle, independent guard function; no ROS/Gazebo/hardware validation',
        final_progress_m=records[-1]['evaluation']['center_pose'][0],
        final_lateral_m=records[-1]['evaluation']['center_pose'][1],
        contact_samples=sum(r['evaluation']['contact_sample'] for r in records),
        max_wheels_touching_grass=max(r['evaluation']['wheels_touching_grass'] for r in records),
        min_gap_m=min(r['evaluation']['minimum_body_gap_m'] for r in records),
        stopped=records[-1]['state'] == 'STOP' and records[-1]['actual_speed_mps'] < .001,
        first_prepared_s=prepared[0]['t'] if prepared else None, prepared_calls=len(prepared),
        first_ready_s=first_ready['t'] if first_ready else None,
        entry=None if entry is None else dict(time_s=entry['t'], center_pose=entry['evaluation']['center_pose'],
            target_gap_m=entry['evaluation']['target_body_gap_m'], measured_speed_mps=entry['measured_speed_mps'],
            actual_speed_mps=entry['actual_speed_mps']),
        last_follow_s=next((r['t'] for r in reversed(records) if r['state'] == 'FOLLOW'), None),
        returned_to_follow=any(r['reason'] == 'handoff_return' for r in records),
        mode_transitions=transitions, reason_counts=dict(Counter(r['reason'] for r in records)),
        preparation_reasons=dict(Counter(r['preparation'] for r in records if r['preparation'])),
        max_actual_speed_mps=max(r['actual_speed_mps'] for r in records),
        median_processing_ms=float(np.median([r['processing_ms'] for r in records])),
        max_processing_ms=max(r['processing_ms'] for r in records))
    summary['evaluation_scenario']=scenario.description()
    return summary, records

def main():
 out=BASE/'run1';out.mkdir(exist_ok=False)
 files=[*BASE.glob('*.py'),*BASE.glob('*.sh'),BASE/'PLAN.md']
 files += [p for folder in ('src/arena_autonomy/arena_autonomy','src/arena_vehicle_interface/arena_vehicle_interface','scripts') for p in (ROOT/folder).glob('*.py')]
 hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
 with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
  for name in hashes:z.write(ROOT/name,name)
 reports=[]
 for name in CASES:
  trace=(out/(name+'.jsonl')).resolve()
  summary,records=rollout(name,'observed_entry_space_v1',trace,duration=16.,window='trim_before_compare_v1',reserve=True,reacquire='known_hold_v1')
  with trace.open() as f:replay=replay_events(json.loads(line) for line in f)
  summary['replay']={k:v for k,v in replay.items() if k not in ('outcomes','mismatches')}
  summary['mismatches']=len(replay['mismatches'])
  summary['controller_read_actor_truth']=False
  summary['case_drive_passed']=None  # Characterization, not a newly declared race pass gate.
  (out/(name+'.records.json')).write_text(json.dumps(freeze(records))+'\n')
  reports.append(summary);print(json.dumps(summary),flush=True)
 result=dict(reports=reports,scope='sequential sensor synthetic; physical actor pulse or adversarial slide; no Gazebo/hardware',
  source_sha256=hashes,source_unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==v for p,v in hashes.items()))
 (out/'report.json').write_text(json.dumps(result,indent=2)+'\n')
 return 0 if result['source_unchanged'] and all(r['mismatches']==0 for r in reports) else 1
if __name__=='__main__':raise SystemExit(main())
