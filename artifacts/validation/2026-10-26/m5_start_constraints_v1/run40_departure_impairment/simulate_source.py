#!/usr/bin/env python3
"""M5 센서 연결 bicycle 비교. ROS 전달·MCU·타이어/제동 물리를 대신하지 않는다.

정답은 광선/정수 펄스/자이로 합성과 사후 평가에만 사용한다. 제어기에는 기존
MotionHistory로 보정한 LocalScene만 전달한다. 보호층 원문의 인프로세스 호출은
알고리즘 분리 확인이며 프로세스/통신 독립성을 입증하지 않는다.
"""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import heapq
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import zipfile

import numpy as np
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time
from sensor_msgs.msg import LaserScan
from shapely.geometry import MultiLineString, Polygon

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.lidar_safety import LidarSafety
from arena_autonomy.follow_trace import snapshot_call
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import mask_scan
from arena_autonomy.m3_execution_trial import ExecutionConfig, M3ExecutionFollow
from arena_autonomy.scene_motion import motion_scene
from arena_vehicle_interface.drive_feedback import MotorEncoderSpec
from audit_local_pursuit_road import road_polygon
from g0_geometry import Footprint, TruthBox, ray_box_distance
from g2_surface_geometry import footprint_at
from g2_tracking_probe import bicycle_step
from probe_corner_visibility_grid import cast_segments
from probe_grid_departure_observation import motion_history
from probe_grid_follow import ROOT, CAMERA, VEHICLE, RUNTIME, json_value, registry, wall_sections
from surface_contact_eval import VehicleGeometry


def stamp(t):
    ns = round(t*1e9)
    return Time(sec=ns//1_000_000_000, nanosec=ns % 1_000_000_000)


def event_key(when, order, kind, value):
    """정수 ns로 동일 시각을 묶고, 수신 완료를 제어 timer보다 먼저 실행한다.

    실 ROS callback 순서를 보장하는 규칙이 아니라 이 이상 시험의 명시적 순서다.
    제어 지연/순서 변동 시험은 별도로 주입해야 하며 float 반올림에 맡기지 않는다.
    """
    priority = 2 if kind == 'control' else 0 if kind.endswith('_delivery') else 1
    return round(when*1e9), priority, order, kind, value


class IdealPlant:
    """후축 정곡률 이상 추종. 실제 서보·속도 PI·slip·정지 거리를 모델링하지 않는다."""
    def __init__(self, slot, wheelbase):
        self.wheelbase = wheelbase
        yaw = slot['yaw_rad']
        self.rear = (slot['x']-wheelbase/2*math.cos(yaw), slot['y']-wheelbase/2*math.sin(yaw), yaw)
        self.t = self.travel = self.speed = self.steering = 0.

    def advance(self, now):
        if now < self.t-1e-12:
            raise ValueError('plant time reversed')
        dt = max(0., now-self.t)
        if dt:
            self.rear = bicycle_step(*self.rear, self.speed, self.steering, dt, self.wheelbase)
            self.travel += self.speed*dt
        self.t = now

    @property
    def center(self):
        x, y, yaw = self.rear
        return x+self.wheelbase/2*math.cos(yaw), y+self.wheelbase/2*math.sin(yaw), yaw

    @property
    def gyro(self):
        return self.speed*math.tan(self.steering)/self.wheelbase


def acquired_ranges(poses, times, peers, velocities, segments, sensor, minimum, maximum):
    """취득 시각마다 다른 자차 원점과 물체 위치. 정답 라벨을 반환하지 않는다."""
    poses, times = np.asarray(poses), np.asarray(times)
    if (poses.shape != (500, 3) or times.shape != (500,) or np.any(np.diff(times) <= 0)
            or not np.isfinite(poses).all() or not np.isfinite(times).all() or len(peers) != len(velocities)):
        raise ValueError('one complete sequential revolution required')
    sx, sy, height = sensor['xyz_m']
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    origins = poses[:, :2]+np.column_stack((c*sx-s*sy, s*sx+c*sy))
    angles = poses[:, 2]-math.pi+np.arange(500)*math.tau/500
    raw = cast_segments(segments, origins, angles)
    for i, (origin, angle, when) in enumerate(zip(origins, angles, times)):
        for peer, (vx, vy) in zip(peers, velocities):
            box = replace(peer, x_m=peer.x_m+vx*max(0., when-.5), y_m=peer.y_m+vy*max(0., when-.5))
            hit = ray_box_distance((*origin, height), (math.cos(angle), math.sin(angle), 0.), box, 0., maximum)
            if hit is not None:
                raw[i] = min(raw[i], hit)
    return np.where(raw < minimum, np.nan, np.where(raw <= maximum, raw, np.inf))


def range_bias(ranges, bias_m, minimum, maximum):
    """이미 판정한 근거리/무반환 상태를 복구하지 않고 유효 반환만 편향시킨다."""
    if isinstance(bias_m, bool) or not math.isfinite(bias_m):
        raise ValueError('finite range bias required')
    values = np.asarray(ranges, dtype=float).copy()
    if bias_m == 0.:
        return values
    valid = np.isfinite(values)
    values[valid] += bias_m
    values[valid & (values < minimum)] = np.nan
    values[valid & (values > maximum)] = np.inf
    return values


class IndependentGuard:
    """실제 LidarSafety.control에 독립 운동 이력과 원시 scan을 공급하는 테스트 어댑터."""
    def __init__(self, history, cfg):
        self.now = 0.
        self.outputs, self.statuses = [], []
        self.node = SimpleNamespace(command=None, scan=None, scan_wall=0., command_wall=0.,
            wall_timeout=3., limit=cfg.max_steering_rad, wheelbase=cfg.wheelbase_m,
            length=cfg.length_m, width=cfg.width_m, last_clock=None,
            motion=SimpleNamespace(project=self.project, history=history),
            publisher=SimpleNamespace(publish=self.outputs.append),
            status=SimpleNamespace(publish=self.statuses.append),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(self.now*1e9),
                to_msg=lambda: stamp(self.now))))

    def project(self, scan):
        first = scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
        points, indices, _ = self.node.motion.history.project(scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, first, scan.time_increment, self.now, 'both', True)
        return points, len(indices)

    def command(self, now, scan, speed, steering):
        self.now = now
        command = AckermannDriveStamped()
        command.header.stamp = stamp(now)
        command.drive.speed, command.drive.steering_angle = float(speed), float(steering)
        self.node.command, self.node.scan = command, mask_scan(scan, 30.)
        self.node.scan_wall = self.node.command_wall = time.monotonic()
        LidarSafety.control(self.node)
        return self.outputs[-1], json.loads(self.statuses[-1].data)


def simulate(slot, other_slots, segments, road, vehicle, sensor, *, rounded, fault=False, straight_priority=False,
             range_bias_m=0., additional_scan_delay_s=0.):
    if (isinstance(additional_scan_delay_s, bool) or not math.isfinite(additional_scan_delay_s)
            or additional_scan_delay_s < 0 or isinstance(range_bias_m, bool) or not math.isfinite(range_bias_m)):
        raise ValueError('finite bias and nonnegative delivery delay required')
    drivetrain, lidar = vehicle['drivetrain'], vehicle['sensors']['lidar_2d']
    feedback = vehicle['sensors']['drive_feedback']['drive_motor_encoder']
    wheelbase = drivetrain['wheelbase_m']
    # 이 시험의 등록값과 현행 설정이 다르면 자동으로 대체하지 않는다.
    assert feedback['counts_per_motor_revolution'] == 28 and drivetrain['wheel_radius_m'] == .0325
    assert drivetrain['motor']['gear_ratio'] == 15. and feedback['sample_rate_hz'] == 100.
    assert sensor['rate_hz'] == 10. and lidar['samples_per_scan'] == 500
    spec = MotorEncoderSpec(28, 15., .0325)
    plant = IdealPlant(slot, wheelbase)
    histories = [motion_history(wheelbase, sensor) for _ in range(2)]
    cfg = FollowConfig(wheelbase_m=wheelbase, occupancy_timing_mode='per_ray_candidate',
        boundary_evidence_mode='corner_distance_candidate', corridor_fallback_enabled=True)
    execution = ExecutionConfig(command_fallback=True, release_to_baseline=True,
        small_center_straight=True, straight_surface_commitment=True, rounded_occupancy=rounded,
        straight_with_threat=straight_priority)
    controller = M3ExecutionFollow(cfg, execution, allow_simulation_trial=True)
    guard = IndependentGuard(histories[1], cfg)
    footprint = Footprint(vehicle['footprint']['length_m'], vehicle['footprint']['width_m'])
    peers = [TruthBox(s['x'], s['y'], .12, footprint, s['yaw_rad'], label=f"grid_{s['index']}") for s in other_slots]
    c, s = math.cos(slot['yaw_rad']), math.sin(slot['yaw_rad'])
    velocities = [(.5*math.cos(p['yaw_rad']), .5*math.sin(p['yaw_rad']))
        if (p['x']-slot['x'])*c+(p['y']-slot['y'])*s > 0 else (0., 0.) for p in other_slots]
    geometry = VehicleGeometry(wheelbase_m=wheelbase, track_width_m=drivetrain['track_width_m'])
    walls = MultiLineString(segments.tolist())
    events, event_id = [], 0

    def enqueue(when, kind, value=None):
        nonlocal event_id
        heapq.heappush(events, event_key(when, event_id, kind, value))
        event_id += 1

    for i in range(601): enqueue(i*.01, 'motor')
    for i in range(math.floor(6.*208)+1): enqueue(i/208, 'gyro')
    for i in range(27_500): enqueue(.5+i*.0002, 'ray', i)
    for i in range(270): enqueue(.6018+i*.02, 'control')
    old_count = 0
    poses, times, scans, calls, measures = [], [], [], [], []
    latest = None
    last_steering = 0.
    motion_failures = 0
    started = None
    while events:
        ns, _, _, kind, value = heapq.heappop(events)
        now = ns*1e-9
        if now > 6.:
            break
        plant.advance(now)
        if kind == 'motor':
            count = spec.counts_from_motor_angle(plant.travel/.0325*15.)
            motor_rate = spec.motor_rad_s_from_counts(count-old_count, .01) if now else 0.
            old_count = count
            enqueue(now+.002, 'motor_delivery', (now, spec.mean_wheel_rad_s(motor_rate)))
            measures.append(dict(t_s=now, counts=count, motor_rad_s=motor_rate))
        elif kind == 'gyro':
            enqueue(now+.002, 'gyro_delivery', (now, plant.gyro))
        elif kind in ('motor_delivery', 'gyro_delivery'):
            for history in histories:
                if kind == 'motor_delivery': history.add_mean_wheel(*value)
                else: history.add_gyro(*value)
        elif kind == 'ray':
            poses.append(plant.center); times.append(now)
            if len(poses) == 500:
                frame = value//500
                ranges = acquired_ranges(poses, times, peers, velocities, segments, sensor,
                                         lidar['range_min_m'], lidar['range_max_m'])
                ranges = range_bias(ranges, range_bias_m, lidar['range_min_m'], lidar['range_max_m'])
                injected = bool(fault and 18 <= frame < 22)
                if injected: ranges[250] = np.nan
                scan = LaserScan(angle_min=-math.pi, angle_increment=math.tau/500,
                    time_increment=.0002, scan_time=.1, range_min=lidar['range_min_m'],
                    range_max=lidar['range_max_m'], ranges=ranges.tolist())
                scan.header.stamp, scan.header.frame_id = stamp(times[0]), 'laser_frame'
                delivery = now+.002+additional_scan_delay_s
                enqueue(delivery, 'scan_delivery', (scan, frame, injected))
                scans.append(dict(frame=frame, first_s=times[0], ranges=json_value(ranges), fault=injected,
                                  delivered_s=delivery))
                poses, times = [], []
        elif kind == 'scan_delivery':
            latest = value
        elif kind == 'control':
            if latest is None:
                continue
            scan, frame, injected = latest
            _, speeds = histories[0].wheels.available(now)
            measured = float(speeds[-1]) if len(speeds) else 0.
            explicit_stop = now >= 5.6
            captured = snapshot_call(scan,histories[0],now,0,measured,'left',last_steering,.02,True,
                                     sensor['masked_rear_half_angle_deg'])
            began = time.perf_counter_ns()
            try:
                scene = motion_scene(scan, histories[0], now, timing_verified=True,
                                     rear_half_angle_deg=sensor['masked_rear_half_angle_deg'])
                request = controller.command(scene, histories[0], measured, 'left', last_steering, .02)
            except MotionUnavailable as error:
                motion_failures += 1
                request = (0., last_steering, dict(reason=str(error)))
            if explicit_stop:
                controller.reset()
                request = (0., last_steering, dict(reason='explicit_test_stop'))
            cost_ms = (time.perf_counter_ns()-began)/1e6
            safe, status = guard.command(now, scan, request[0], request[1])
            plant.speed, plant.steering = safe.drive.speed, safe.drive.steering_angle
            last_steering = request[1]
            if plant.speed > 0 and started is None: started = now
            x, y, yaw = plant.center
            outline, wheels = footprint_at(x, y, yaw, plant.steering, geometry)
            moved = [Polygon(p.footprint.polygon(p.x_m+v[0]*max(0., now-.5),
                     p.y_m+v[1]*max(0., now-.5), p.yaw_rad)) for p, v in zip(peers, velocities)]
            gaps = [outline.distance(p) for p in moved]
            overlap = any(q.intersection(p).area > 1e-10 for q in outline.components for p in moved)
            wall_gap = outline.distance(walls)
            outside = sum(w['patch'].difference(road).area > 1e-10 for w in wheels.values())
            calls.append(dict(t_s=now, frame=frame, fault=injected, explicit_stop=explicit_stop,
                latest_scan_first_s=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9,
                captured_input=captured, controller_cost_ms=cost_ms,
                measured_mps=measured, request_mps=request[0], request_steering=request[1],
                safe_mps=plant.speed, safe_steering=plant.steering, reason=request[2]['reason'],
                safety_reason=status['reason'], metadata=request[2],
                truth_pose=[x, y, yaw], truth_travel_m=plant.travel,
                peer_overlap=overlap, minimum_peer_gap_m=min(gaps), wall_gap_m=wall_gap,
                wheels_outside_road=outside))
            if overlap or wall_gap <= 1e-9 or outside > 2:
                plant.speed = 0.
                break  # 평가 실패를 기록하고 종료한다. 성공할 때까지 다시 달리지 않는다.
    summary = dict(rounded=rounded, fault=fault, straight_priority=straight_priority,
        range_bias_m=range_bias_m,additional_scan_delay_s=additional_scan_delay_s,
        first_motion_s=started, calls=len(calls),
        controller_cost_p95_ms=float(np.percentile([c['controller_cost_ms'] for c in calls],95)),
        controller_cost_max_ms=max(c['controller_cost_ms'] for c in calls),
        event_contract='integer_ns_deliveries_before_control',
        progress_m=plant.travel, controller_positive=sum(c['request_mps'] > 0 for c in calls),
        safe_positive=sum(c['safe_mps'] > 0 for c in calls),
        fault_positive_requests=sum(c['fault'] and c['request_mps'] > 0 for c in calls),
        fault_positive_safe=sum(c['fault'] and c['safe_mps'] > 0 for c in calls),
        motion_input_failures=motion_failures, minimum_peer_gap_m=min(c['minimum_peer_gap_m'] for c in calls),
        peer_overlap_calls=sum(c['peer_overlap'] for c in calls),
        minimum_wall_gap_m=min(c['wall_gap_m'] for c in calls),
        maximum_wheels_outside_road=max(c['wheels_outside_road'] for c in calls),
        explicit_stop_checked=any(c['explicit_stop'] for c in calls),
        stopped_after_request=all(c['safe_mps'] == 0 for c in calls if c['explicit_stop']),
        reasons=dict(Counter(c['reason'] for c in calls)), safety_reasons=dict(Counter(c['safety_reason'] for c in calls)),
        scope='ideal bicycle and sampled geometry, no ROS transport/MCU/physical braking proof')
    return dict(summary=summary, calls=calls, scans=scans, motor_measurements=measures)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--compare-straight-priority', action='store_true',
        help='registered comparison: rounded ON, straight priority OFF/ON, then ON with central dropout')
    args = parser.parse_args(); output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/validation'):
        parser.error('new validation output required')
    output.mkdir(parents=True, exist_ok=False)
    vehicle = yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    sensor = yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    slots = registry(json.loads((RUNTIME/'scene.json').read_text(encoding='utf-8')), (RUNTIME/'world.sdf').read_bytes(), vehicle)[:5]
    ego = next(s for s in slots if s['index'] == 5)
    segments = wall_sections((RUNTIME/'world.sdf').read_bytes(), sensor['xyz_m'][2])
    road = road_polygon(RUNTIME/'world.sdf')
    paths = {Path(__file__), CAMERA, VEHICLE, RUNTIME/'world.sdf', RUNTIME/'scene.json',
             ROOT/'tests/test_grid_departure_dynamics.py'}
    paths.update((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    for module in tuple(sys.modules.values()):
        name = getattr(module, '__file__', None)
        if name:
            path = Path(name).resolve()
            if path.is_relative_to(ROOT) and path.suffix == '.py': paths.add(path)
    frozen = {p.relative_to(ROOT).as_posix(): p.read_bytes() for p in sorted(paths)}
    with zipfile.ZipFile(output/'sources.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name, raw in frozen.items(): archive.writestr(name, raw)
    results = []
    modes = ((True,False,False),(True,False,True),(True,True,True)) if args.compare_straight_priority else (
        (False,False,False),(True,False,False),(True,True,False))
    for rounded, fault, straight_priority in modes:
        results.append(simulate(ego, [s for s in slots if s != ego], segments, road, vehicle, sensor,
                                rounded=rounded, fault=fault, straight_priority=straight_priority))
        print(json.dumps(results[-1]['summary']), flush=True)
    unchanged = all((ROOT/n).read_bytes() == b for n, b in frozen.items())
    report = dict(cases=results, sources_unchanged=unchanged, source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},
                  scope='ideal sensor-in-loop; no Gazebo/ROS transport/MCU', driving_authorized=False)
    (output/'report.json').write_text(json.dumps(json_value(report), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    summary = dict(cases=[r['summary'] for r in results], sources_unchanged=unchanged,
        nominal_progress_pass=results[1]['summary']['progress_m'] >= .5)
    summary['integrity_pass'] = unchanged and all(not c['motion_input_failures'] and not c['fault_positive_safe'] and
        not c['fault_positive_requests'] and not c['peer_overlap_calls'] and c['minimum_wall_gap_m'] > 0 and
        c['maximum_wheels_outside_road'] <= 2 and c['explicit_stop_checked'] and c['stopped_after_request'] for c in summary['cases'])
    summary['scan_schedule_valid'] = all(row['frame'] == max(s['frame'] for s in case['scans']
        if round((s['first_s']+.0998+.002)*1e9) <= round(row['t_s']*1e9)) for case in results for row in case['calls'])
    summary['integrity_pass'] &= summary['scan_schedule_valid']
    (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    return 0 if summary['integrity_pass'] and summary['nominal_progress_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
