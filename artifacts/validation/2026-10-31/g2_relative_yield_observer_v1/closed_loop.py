"""미연결 양보 감속 후보. 10/29 합성 센서/독립 보호/이상 기구 평가기 재사용.
방지턱, 헛돎, 모터 토크 응답은 없다. ROS/물리/실차 검증이 아니다."""
from collections import Counter
from dataclasses import asdict
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
from arena_autonomy.bypass_handoff import HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from controller import RelativeYieldFollow
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.follow_trace import snapshot_call, freeze, FollowTrace, replay_events
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from bypass_probe_sensors import ProbeSensors
from g0_geometry import TruthBox


HEADER=json.loads((BASE/'baseline_header.json').read_text())['data']
CASES={
    'central_stopped':(0.,1.2,0.,None,'ideal'),
    'central_restart_4':(0.,1.2,.35,0.,'ideal',4.),
    'central_restart_7':(0.,1.2,.35,0.,'ideal',7.),
    'central_restart_4_combined':(0.,1.2,.35,0.,'combined',4.),
}
def rollout(name, policy, trace_path=None, duration=12., window='compare_then_trim_v0',reserve=False,reacquire='positive_command_v0'):
    if trace_path is not None:
        raise ValueError('prototype is not represented by production schema5; do not label its trace as stock replay')
    target_y, start_x, target_speed, moving_s, profile = CASES[name][:5]
    restart_after_s = CASES[name][5] if len(CASES[name]) > 5 else None
    controller = RelativeYieldFollow(config=FollowConfig(**HEADER['follow_config']),
        execution=HandoffExecutionConfig.from_mapping(HEADER['execution_config']),
        allow_simulation_trial=True, yield_enabled=policy=='yield')
    motion = MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=.06, history_s=5.)
    history = MotionHistory(motion)
    guard = probe.ShadowSafety(RecoveryPolicy(True, True, True), controller.normal.config, 30.)
    sensors = ProbeSensors(profile, 17)
    original_boxes = probe.evaluation_boxes
    trace = (FollowTrace(trace_path, motion, controller.config, mode='g1_g2_handoff_trial',
                         execution_config=controller.execution) if trace_path else None)

    def boxes(at, y=0., restart=False):
        result = original_boxes(at, y, False)
        target = result[-1]
        elapsed = max(0., at-.5) if moving_s is None else min(max(0., at-.5), moving_s)
        if restart_after_s is not None:
            elapsed += max(0., at-.5-moving_s-restart_after_s)
        result[-1] = TruthBox(start_x+target_speed*elapsed, target.y_m, target.height_m)
        return result
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
                reserve=reserved,yield_detail=controller.last_yield_detail,
                entry_phase=(detail.get('bypass') or {}).get('entry_phase'),
                g1_reason=normal.get('reason'), safety_reason=status['reason'],
                ready=(detail.get('persistence') or {}).get('ready'),
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
    summary = dict(yield_requested_calls=sum(bool(r['yield_detail'].get('cap_requested')) for r in records),
        case=name, policy=policy, persistence_window_policy=window,follow_entry_space_enabled=reserve,
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
    active=[r['t'] for r in records if r['yield_detail'].get('cap_requested')]
    summary.update(yield_enabled=controller.yield_enabled,execution_config=asdict(controller.execution),
        follow_config=asdict(controller.config),
        prototype_trace_replay_verified=False,
        controller_output_sha256=hashlib.sha256(json.dumps(freeze([
            (r['t'],r['requested'],r['safe'],r['state'],r['reason']) for r in records
        ]),allow_nan=False).encode()).hexdigest(),
        active_window_s=[active[0],active[-1]] if active else None)
    # 이전 함수의 인자 이름은 호출 호환용으로만 남긴다. 실제 설정은 저장 header 값이다.
    summary['persistence_window_policy']=controller.execution.persistence_window_policy
    summary['follow_entry_space_enabled']=controller.execution.follow_entry_space_enabled
    summary['reacquire_policy']=controller.execution.reacquire_policy
    return summary, records


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--cases',nargs='+',choices=list(CASES),default=list(CASES))
    parser.add_argument('--duration',type=float,default=18.)
    args=parser.parse_args();out=args.output.resolve();out.mkdir(exist_ok=False)
    paths=[p for p in (ROOT/'src').rglob('*.py') if '__pycache__' not in p.parts]
    paths+=list(BASE.glob('*.py'))+list(BASE.glob('*.sh'))+[BASE/'baseline_header.json',BASE/'README.md',
        ROOT/'scripts/probe_bypass_entry_sequence.py',ROOT/'scripts/bypass_probe_sensors.py',
        ROOT/'scripts/g0_geometry.py',ROOT/'scripts/g2_tracking_probe.py',
        ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1/safety_shadow.py',
        ROOT/'artifacts/validation/2026-10-29/g2_follow_reserve_v1/probe_closed_loop.py']
    sources={p.relative_to(ROOT).as_posix():p.read_bytes() for p in paths}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name,data in sources.items():archive.writestr(name,data)
    results=[];pairs=[]
    for case in args.cases:
        rows=[]
        for policy in ('baseline','yield'):
            summary,records=rollout(case,policy,duration=args.duration)
            with (out/(case+'__'+policy+'.records.jsonl')).open('x') as stream:
                for row in records:stream.write(json.dumps(freeze(row),allow_nan=False)+'\n')
            results.append(summary);rows.append(summary)
            (out/'partial.json').write_text(json.dumps(freeze(results),indent=2,allow_nan=False))
            print(json.dumps({k:summary[k] for k in ('case','yield_enabled','final_progress_m','min_gap_m',
                'contact_samples','max_wheels_touching_grass','returned_to_follow','stopped','yield_requested_calls')}),flush=True)
        pairs.append(dict(case=case,unchanged_commands=rows[0]['controller_output_sha256']==rows[1]['controller_output_sha256'],
            progress_difference_m=rows[1]['final_progress_m']-rows[0]['final_progress_m'],
            return_before=rows[0]['returned_to_follow'],return_after=rows[1]['returned_to_follow']))
    report=dict(scope='non-production synthetic sensor/independent guard/ideal bicycle; no bump or physical contact dynamics',
        cases=results,pairs=pairs,source_sha256={n:hashlib.sha256(v).hexdigest() for n,v in sources.items()},
        source_unchanged=all((ROOT/n).read_bytes()==v for n,v in sources.items()))
    (out/'report.json').write_text(json.dumps(freeze(report),indent=2,allow_nan=False))
    if not report['source_unchanged']:raise RuntimeError('source changed during trial')


if __name__=='__main__':main()

