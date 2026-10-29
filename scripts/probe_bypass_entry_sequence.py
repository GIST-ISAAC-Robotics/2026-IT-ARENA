"""G2 후보의 짧은 센서 폐루프 비교. 이상 bicycle, 가상 평균 엔코더/gyro.

정답 자세는 광선 합성과 접촉/바퀴 평가에만 사용한다. 실제 ROS 수신/MCU/동역학
시험은 아니며, 보호 함수는 별도 상태로 같은 센서 입력을 재계산한다.
"""
import argparse
from bisect import bisect_right
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import zipfile

import numpy as np
from shapely.geometry import Polygon

from arena_autonomy.bypass_entry_trial import compare_entries, EntryIntent, EntryGuide, EntryConfig
from arena_autonomy.follow_trace import snapshot_call
from arena_autonomy.lateral_intent import LateralIntentConfig, wheel_contact_corners
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import mask_scan
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from g0_geometry import TruthBox, Footprint, ray_box_distance
from g2_tracking_probe import bicycle_step
from bypass_probe_sensors import ProbeSensors

ROOT = Path(__file__).resolve().parents[1]
SHADOW = ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1'
sys.path.insert(0, str(SHADOW))
from safety_shadow import ShadowSafety


def physical_pose(rows, at):
    """가상 물리의 구간별 실제 적용 명령. 제어기에는 전달하지 않는다."""
    if not rows or at < rows[0]['t']-1e-10 or at > rows[-1]['t']+1e-10:
        raise ValueError('physical trace does not cover acquisition time')
    index = max(0, bisect_right([r['t'] for r in rows], at)-1)
    row = rows[index]
    return bicycle_step(*row['rear_pose'], row['speed'], row['steering'],
                        at-row['t'], .135) if at > row['t'] else tuple(row['rear_pose'])


def evaluation_boxes(at, target_y=0., restart=False):
    boxes = [TruthBox(2., sign*.45, .4, Footprint(8., .05), label='wall') for sign in (-1, 1)]
    boxes += [TruthBox(x, 0., .4, Footprint(.05, 2.), label='wall') for x in (-2.025, 6.025)]
    x = 1.2 + (.25*max(0., at-1.5) if restart else 0.)
    return boxes + [TruthBox(x, target_y, .12)]


def render_scan(rows, now, target_y=0., restart=False, sensors=None):
    # 마지막 광선은 발행보다 한 광선 간격 앞선다. 미래 시각을 잘라 맞추지 않는다.
    first_ns = round((now-.1)*1e9)
    first = first_ns*1e-9
    ranges = []
    for i in range(500):
        at = first+i*.0002
        rx, ry, yaw = physical_pose(rows, at)
        cx, cy = rx+.0675*math.cos(yaw), ry+.0675*math.sin(yaw)
        bearing = yaw-math.pi+i*math.tau/500+(sensors.actual_lidar_yaw if sensors else 0.)
        origin = (cx+.06*math.cos(yaw), cy+.06*math.sin(yaw), .095)
        direction = (math.cos(bearing), math.sin(bearing), 0.)
        hits = [distance for box in evaluation_boxes(at, target_y, restart)
                if (distance := ray_box_distance(origin, direction, box, 0., 12.)) is not None]
        distance = min(hits, default=math.inf)
        if sensors:distance=sensors.range_value(distance)
        ranges.append(distance if distance >= .05 else math.nan)
    raw = SimpleNamespace(header=SimpleNamespace(frame_id='laser_frame', stamp=SimpleNamespace(
        sec=first_ns//1_000_000_000, nanosec=first_ns%1_000_000_000)), ranges=ranges,
        angle_min=-math.pi, angle_increment=math.tau/500, time_increment=.0002,
        range_min=.05, range_max=12.)
    return mask_scan(raw, 30.)


def evaluation_metrics(rear, steering, now, target_y, restart):
    rx, ry, yaw = rear
    center = (rx+.0675*math.cos(yaw), ry+.0675*math.sin(yaw), yaw)
    footprint = Polygon(Footprint().polygon(*center))
    boxes = evaluation_boxes(now, target_y, restart)
    distances = [footprint.distance(Polygon(box.footprint.polygon(box.x_m, box.y_m, box.yaw_rad)))
                 for box in boxes]
    patches = wheel_contact_corners(np.array([center]), steering, LateralIntentConfig())
    outside = np.any(np.abs(patches[0, :, :, 1]) > .225+1e-10, axis=1)
    return dict(minimum_body_gap_m=min(distances), target_body_gap_m=distances[-1],
                contact_sample=any(d <= 1e-10 for d in distances),
                wheels_touching_grass=int(outside.sum()), center_pose=list(center))


def rollout(target_y, restart=False, duration=3., use_guide=False, stationary_outer_boundary=False, wide_entry=False,
            rejoin_enabled=False,sensor_profile='ideal',seed=17,wall_reanchor=False,tracked_wall_fit=False,earlier_transition=False):
    config = FollowConfig()
    motion = MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=.06, history_s=5.)
    history = MotionHistory(motion)
    guard = ShadowSafety(RecoveryPolicy(True, True, True), config, 30.)
    intent = EntryIntent()
    entry=EntryConfig(offsets_m=(0.,-.16,.16,-.20,.20,-.24,.24,-.27,.27),prefer_earlier_transition=earlier_transition) if wide_entry else EntryConfig(prefer_earlier_transition=earlier_transition)
    guide = EntryGuide(config,entry,stationary_outer_boundary=stationary_outer_boundary,rejoin_enabled=rejoin_enabled,wall_reanchor=wall_reanchor,tracked_wall_fit=tracked_wall_fit)
    sensors=ProbeSensors(sensor_profile,seed)
    travel=0.
    # 시작 전 정지 관측. 선택한 profile에서만 단일 축 양자화/gyro 오차를 주입한다.
    # 실제 MCU 전송 지연/슬립은 재현하지 않는다.
    physical = []
    for k in range(25):
        t = k*.02
        physical.append(dict(t=t, rear_pose=[-.0675, 0., 0.], speed=0., steering=0.))
        mean,gyro,count=sensors.measure_motion(t,0.,0.,0.)
        history.add_mean_wheel(t,mean);history.add_gyro(t,gyro)
    pose, actual_speed, previous = (-.0675, 0., 0.), 0., 0.
    records, raw, last_nonzero_sign, side_changes = [], None, None, 0
    for k in range(round((duration+.6)/.02)+1):
        now = round(.5+k*.02, 9)
        physical.append(dict(t=now, rear_pose=list(pose), speed=actual_speed, steering=previous))
        if sensors.motion_enabled:
            mean,gyro,count=sensors.measure_motion(now,travel,actual_speed,previous)
            history.add_mean_wheel(now,mean);history.add_gyro(now,gyro)
        if k % 5 == 0:
            raw = render_scan(physical, now, target_y, restart,sensors)
        # 정답 위치를 전달하지 않고 가상 측정 운동 이력으로 현재 장면을 구성한다.
        scene = motion_scene(raw, history, now, timing_verified=True, rear_half_angle_deg=30.)
        began = time.perf_counter()
        # 제어 입력은 가상 엔코더 생산자가 기록한 평균 속도다. 물리 속도는 평가 전용.
        _, measured_values = history.wheels.available(now)
        measured_speed = float(measured_values[-1])
        decision = (guide.evaluate(scene,history,measured_speed,previous,.02) if use_guide else
                    intent.select(scene, compare_entries(scene, now, measured_speed, previous, .02)))
        chosen = decision['selected']
        scheduled_stop = now >= .5+duration
        command = (0., previous) if scheduled_stop or chosen is None else (chosen['speed_mps'], chosen['steering_rad'])
        frozen = snapshot_call(raw, history, now, 0, measured_speed, 'left', previous, .02, True, 30.)
        safe, status = guard.step(frozen, motion, command)
        elapsed = (time.perf_counter()-began)*1000
        if not 0 <= safe[0] <= command[0]+1e-12:
            raise ValueError('independent safety increased requested speed')
        # 이상 가감속/조향. 40 ms 모터 토크·전류·타이어 마찰을 재현한 모델이 아니다.
        next_speed = float(np.clip(safe[0], max(0., actual_speed-2.*.02), actual_speed+1.*.02))
        next_steering = float(safe[1])
        sign = None if chosen is None or chosen['offset_m'] == 0. else math.copysign(1., chosen['offset_m'])
        if sign is not None:
            side_changes += last_nonzero_sign is not None and sign != last_nonzero_sign
            last_nonzero_sign = sign
        record = dict(t=now, phase='scheduled_stop' if scheduled_stop else 'entry',
            reason=decision['reason'], reference_reason=decision.get('reference', {}).get('reason'),
            guide_mode=decision.get('guide_mode'),
            outer_boundary_padding=decision.get('outer_boundary_padding'),
            path_reanchor=decision.get('path_reanchor'),
            entry_phase=decision.get('entry_phase'),rejoin_band=decision.get('rejoin_band'),
            rejoin_confirmations=decision.get('rejoin_confirmations'),center_confirmations=decision.get('center_confirmations'),
            intent_direction=decision['intent_direction'],
            selected=chosen, command=list(command), safe=list(safe), safety_reason=status['reason'],
            actual_speed_mps=actual_speed, measured_speed_mps=measured_speed, processing_ms=elapsed,
            produced_motor_counts=count if sensors.motion_enabled else None,
            candidate_reasons=dict(Counter(r['reason'] for r in decision['candidates'])),
            candidate_summary=[{key:row.get(key) for key in ('offset_m','transition_m','eligible','reason',
                'command_clearance_m','preview_return_progress_m','target_speed_mps','steering_rad','motion_region')}
                for row in decision['candidates']] if chosen is None else [],
            evaluation=evaluation_metrics(pose, previous, now, target_y, restart))
        records.append(record)
        if record['evaluation']['contact_sample']:
            break
        physical[-1].update(speed=next_speed, steering=next_steering)
        if not sensors.motion_enabled:
            history.add_mean_wheel(now, next_speed/.0325)
            history.add_gyro(now, next_speed*math.tan(next_steering)/.135)
        travel+=next_speed*.02
        pose = bicycle_step(*pose, next_speed, next_steering, .02, .135)
        actual_speed, previous = next_speed, next_steering
    stops = [r for r in records if r['phase'] == 'scheduled_stop']
    return dict(target_y=target_y, target_restarts=restart, use_guide=use_guide,
        stationary_outer_boundary=stationary_outer_boundary, scope='sensor_based_ideal_bicycle_entry_only',
        wide_entry=wide_entry,wall_reanchor=wall_reanchor,tracked_wall_fit=tracked_wall_fit,earlier_transition=earlier_transition,
        rejoin_enabled=rejoin_enabled,
        sensor_profile=sensors.description(),seed=seed,
        control_authorized=False, real_ros_scheduling_verified=False, full_bypass_verified=False,
        final_forward_progress_m=records[-1]['evaluation']['center_pose'][0],
        lateral_displacement_m=records[-1]['evaluation']['center_pose'][1],
        contact_samples=sum(r['evaluation']['contact_sample'] for r in records),
        maximum_wheels_touching_grass=max(r['evaluation']['wheels_touching_grass'] for r in records),
        minimum_body_gap_m=min(r['evaluation']['minimum_body_gap_m'] for r in records),
        side_selection_changes=side_changes, reasons=dict(Counter(r['reason'] for r in records if r['phase']=='entry')),
        stop_command_all_zero=bool(stops) and all(r['command'][0]==r['safe'][0]==0 for r in stops),
        final_speed_mps=records[-1]['actual_speed_mps'], records=records,
        motion_config=asdict(motion), limits=['synthetic sensors with declared optional errors; no measured distributions/slip',
            'synthetic sequential rays and sampled body/contact patches',
            'no ROS transport/Gazebo dynamics/real actuation', 'no operational bypass certification'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=3.)
    parser.add_argument('--guide', action='store_true', help='측정 이송한 진입 곡선의 현재 외곽 벽 재확인 비교')
    parser.add_argument('--stationary-outer-boundary',action='store_true',help='여러 현재 관측에서 일치한 긴 외곽 면의 고정 벽 가정')
    parser.add_argument('--wide-entry',action='store_true',help='27 cm 편이 추가 비교; 두 바퀴 노면은 평가 결과로 판정')
    parser.add_argument('--rejoin',action='store_true',help='현재 안쪽 띠 관측 뒤 중앙 복귀 비교')
    parser.add_argument('--sensor-profile',choices=('ideal','range','motion','combined'),default='ideal')
    parser.add_argument('--seed',type=int,default=17)
    parser.add_argument('--wall-reanchor',action='store_true',help='새 외벽 관측의 작은 회전/법선 오차만 목표 경로에 반영')
    parser.add_argument('--tracked-wall-fit',action='store_true',help='기억한 벽 근처의 현재 연속 반환을 직선 재적합')
    parser.add_argument('--earlier-transition',action='store_true',help='동일 진행/여유 순위에서 더 일찍 편이 완료하는 후보 비교')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 10.:
        parser.error('bounded duration in (0,10] required')
    if (args.stationary_outer_boundary or args.wide_entry or args.rejoin) and not args.guide:
        parser.error('boundary/wide entry assumptions require guide trial')
    if args.wall_reanchor and not (args.guide and args.stationary_outer_boundary):
        parser.error('path reanchor requires guide and explicit outer wall assumption')
    if args.tracked_wall_fit and not args.wall_reanchor:
        parser.error('tracked current wall fit requires wall reanchor trial')
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/validation'):
        parser.error('validation output required')
    output.mkdir(parents=True, exist_ok=False)
    paths = list((ROOT/'src').rglob('*.py'))+[Path(__file__), ROOT/'scripts/g0_geometry.py',
        ROOT/'scripts/g2_tracking_probe.py', SHADOW/'safety_shadow.py', ROOT/'tests/test_bypass_entry_sequence.py',
        ROOT/'tests/test_bypass_entry_trial.py', ROOT/'tests/test_bypass_entry_guide.py',
        ROOT/'scripts/bypass_probe_sensors.py',ROOT/'tests/test_bypass_probe_sensors.py',ROOT/'tests/test_bypass_wall_reanchor.py',ROOT/'tests/test_bypass_wall_fit.py']
    raw = {str(p.relative_to(ROOT)):p.read_bytes() for p in paths}
    with zipfile.ZipFile(output/'sources.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name, data in raw.items(): archive.writestr(name, data)
    results = []
    for target_y, restart in ((0., False), (-.15, False), (.15, False), (0., True)):
        result = rollout(target_y, restart, args.duration, args.guide, args.stationary_outer_boundary,args.wide_entry,
                         args.rejoin,args.sensor_profile,args.seed,args.wall_reanchor,args.tracked_wall_fit,args.earlier_transition)
        name = f'y{target_y}_restart{int(restart)}'
        (output/(name+'.json')).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        results.append({k:v for k,v in result.items() if k != 'records'})
    report = dict(scope='four_sensor_trials_ideal_bicycle_not_Gazebo', duration_s=args.duration, cases=results,
        source_sha256={name:hashlib.sha256(data).hexdigest() for name, data in raw.items()},
        source_unchanged=all((ROOT/name).read_bytes()==data for name, data in raw.items()))
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(results))
    if not report['source_unchanged']: raise RuntimeError('source changed')


if __name__ == '__main__': main()
