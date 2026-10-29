"""준비 감속 정책별 G1/G2 연결 코드의 합성 센서 폐루프 비교.

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
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.follow_trace import snapshot_call, freeze, FollowTrace, replay_events
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from bypass_probe_sensors import ProbeSensors
from g0_geometry import TruthBox

# 이름, 표적 y, 초기 중심 x, 지면 속도, 이동 시간(None=계속), 센서 오차
CASES = {
    'right_ideal': (-.15, 1.2, 0., None, 'ideal'),
    'left_combined': (.15, 1.2, 0., None, 'combined'),
    'right_combined': (-.15, 1.2, 0., None, 'combined'),
    'moving_combined': (-.15, 1.2, .25, None, 'combined'),
    # 먼 거리에서 G1이 더 빠르게 접근한 뒤 정지 가장자리 표적을 만나는 경우
    'right_far_combined': (-.15, 2.4, 0., None, 'combined'),
    'left_far_combined': (.15, 2.4, 0., None, 'combined'),
    # 0.25 m/s로 움직이다 2초 뒤(0.5 m 이동) 가장자리에 멈추는 앞 표적
    'stopping_combined': (-.15, 1.2, .25, 2., 'combined'),
}


def rollout(name, policy, trace_path=None, duration=12.):
    target_y, start_x, target_speed, moving_s, profile = CASES[name]
    controller = BypassHandoffFollow(execution=HandoffExecutionConfig(preparation_policy=policy),
                                     allow_simulation_trial=True)
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
            normal = detail.get('normal') or {}
            records.append(dict(t=now, requested=list(requested), safe=list(safe), state=detail.get('handoff_state'),
                reason=detail.get('reason'), preparation=prep.get('reason'), prepared=prep.get('applied'),
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
    summary = dict(case=name, policy=policy, target_y=target_y, target_start_x=start_x,
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
    return summary, records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--cases', nargs='+', default=list(CASES))
    parser.add_argument('--policies', nargs='+', default=['front_patch_v0', 'observed_entry_relevance_v1'])
    parser.add_argument('--trace-policy', default='observed_entry_relevance_v1')
    args = parser.parse_args()
    out = BASE/args.output
    out.mkdir(exist_ok=False)
    paths = [p for p in (ROOT/'src').rglob('*.py') if '__pycache__' not in p.parts]+[Path(__file__)]
    paths += [ROOT/'scripts/probe_bypass_entry_sequence.py', ROOT/'scripts/bypass_probe_sensors.py',
              ROOT/'scripts/g0_geometry.py', ROOT/'scripts/g2_tracking_probe.py',
              ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1/safety_shadow.py']
    sources = {p.relative_to(ROOT).as_posix(): p.read_bytes() for p in paths}
    with zipfile.ZipFile(out/'sources.zip', 'x', zipfile.ZIP_DEFLATED) as z:
        for n, v in sources.items():
            z.writestr(n, v)
    reports = []
    for name in args.cases:
        for policy in args.policies:
            trace_path = (out/(name+'__'+policy+'.jsonl')).resolve() if policy == args.trace_policy else None
            summary, records = rollout(name, policy, trace_path)
            if trace_path:
                with trace_path.open(encoding='utf-8') as stream:
                    replay = replay_events(json.loads(line) for line in stream)
                summary['schema5_replay'] = {k: v for k, v in replay.items() if k not in ('outcomes', 'mismatches')}
                summary['schema5_replay']['mismatch_count'] = len(replay['mismatches'])
            (out/(name+'__'+policy+'.records.json')).write_text(
                json.dumps(freeze(records), allow_nan=False)+'\n', encoding='utf-8')
            reports.append(summary)
            small = {k: summary[k] for k in ('case', 'policy', 'final_progress_m', 'contact_samples',
                'max_wheels_touching_grass', 'first_prepared_s', 'prepared_calls', 'first_ready_s',
                'entry', 'returned_to_follow', 'last_follow_s', 'stopped')}
            small['replay'] = summary.get('schema5_replay', {}).get('passed')
            print(json.dumps(small), flush=True)
    report = dict(cases=reports, source_sha256={n: hashlib.sha256(v).hexdigest() for n, v in sources.items()},
                  source_unchanged=all((ROOT/n).read_bytes() == v for n, v in sources.items()))
    (out/'report.json').write_text(json.dumps(freeze(report), indent=2, allow_nan=False)+'\n', encoding='utf-8')
    if not report['source_unchanged']:
        raise RuntimeError('source changed during trial')
    if any(r.get('schema5_replay', {}).get('passed') is False for r in reports):
        raise SystemExit(2)


if __name__ == '__main__':
    main()
