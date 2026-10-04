#!/usr/bin/env python3
"""G0-a 평가 전용: 합성 기하/관측과 실제 Gazebo 광선 probe. 구동 출력 없음."""
from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO/'src/arena_autonomy'))
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.local_path import pursuit_command, swept_limit, braking_speed, mask_scan
from arena_autonomy.lidar_observation import audit_front_observation
from g0_geometry import Footprint, TruthBox, cross_section_gaps, cast_rays, ray_box_distance, ray_direction

CONFIG = REPO/'config/tests/multi_vehicle_g0.json'
BASELINE = REPO/'config/tests/multi_vehicle_g0_r0_sha256.json'
# gz-sim 8 PosePublisher에는 topic 매개변수가 없고 model scope에서 이름을 만든다.
# multi_vehicle_g0_topic_probe_v1에서 실제 발행 토픽/child frame을 확인했다.
ACTOR_MODEL = 'g0_actor'
ACTOR_POSE_TOPIC = f'/model/{ACTOR_MODEL}/pose'
WORLD_NAME = 'g0_lab'
RAW_TOPIC = '/g0/raw'
SET_POSE_SERVICE = f'/world/{WORLD_NAME}/set_pose'


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def failure_source(error):
    """TimeoutError는 OSError 하위형이지만 원인을 환경으로 단정하지 않는다."""
    if isinstance(error, TimeoutError):
        return 'timeout_cause_undetermined'
    return 'environment' if isinstance(error, (ImportError, OSError)) else 'harness'


def require_horizontal_scene(sensor):
    """G0-a의 2D 관측/R0 연결은 수평 스캔만 지원한다.

    3D 광선 기하는 pitch를 지원하지만 LocalScene 변환은 아직 수평 거리와
    등간격 XY 방위를 가정한다. 비수평 거리를 그대로 평면 거리로 사용하지 않는다.
    """
    pitch = sensor['pitch_rad']
    if (isinstance(pitch, bool) or not isinstance(pitch, (int, float))
            or not math.isfinite(pitch) or pitch != 0.):
        raise ValueError('G0-a scene evaluation requires a horizontal scan (pitch_rad=0); tilted XY projection is not implemented')


def load_suite(path=CONFIG):
    suite = json.loads(Path(path).read_text(encoding='utf-8'))
    if suite.get('schema_version') != 1 or suite.get('seed') not in (0, 1, 2):
        raise ValueError('unsupported schema or fixed seed')
    require_horizontal_scene(suite['lidar'])
    cases = suite['cases']
    if not cases or len({c['name'] for c in cases}) != len(cases):
        raise ValueError('empty/duplicate cases')
    if not math.isclose(suite['road_width_m'] + 2*suite['grass_each_side_m'], suite['wall_gap_m'], rel_tol=0., abs_tol=1e-9):
        raise ValueError('road/grass/wall contract mismatch')
    tolerance = suite['ray_label_match_tolerance_m']
    if isinstance(tolerance, bool) or not math.isfinite(tolerance) or not 0 < tolerance:
        raise ValueError('invalid evaluation label tolerance')
    Footprint(*suite['ego_footprint_m'])
    for case in cases:
        if not math.isfinite(case['duration_s']) or not 0 < case['duration_s'] <= 20:
            raise ValueError('invalid duration')
        target_at(case, 0.)
        trajectory = case['trajectory']
        last = 0.
        for segment in trajectory:
            if (not math.isclose(segment['from_s'], last, rel_tol=0., abs_tol=1e-9) or not segment['to_s'] > last
                    or not math.isfinite(segment['speed_mps'])):
                raise ValueError('invalid/discontinuous trajectory')
            last = segment['to_s']
        if trajectory and not math.isclose(last, case['duration_s'], rel_tol=0., abs_tol=1e-9):
            raise ValueError('trajectory does not cover duration')
        if not isinstance(case['expected']['cross_section_possible'], bool):
            raise ValueError('missing preregistered geometry')
    return suite


def verify_sensor_config(suite):
    import yaml
    profile = yaml.safe_load((REPO/'src/arena_description/config/b0183_c1.yaml').read_text())
    vehicle = yaml.safe_load((REPO/'src/arena_description/config/vehicle.yaml').read_text())['vehicle']
    selected, general, sensor = profile['lidar'], vehicle['sensors']['lidar_2d'], suite['lidar']
    checks = {'profile': sensor['profile'] == 'b0183_c1', 'xyz': selected['xyz_m'] == sensor['xyz_m'],
              'rotation_hz': selected['rate_hz'] == sensor['rotation_hz'],
              'rear_mask': selected['masked_rear_half_angle_deg'] == sensor['rear_half_angle_deg'],
              'samples': general['samples_per_scan'] == sensor['samples'],
              'range_min': general['range_min_m'] == sensor['range_min_m'],
              'range_max': general['range_max_m'] == sensor['range_max_m']}
    if not all(checks.values()):
        raise ValueError(f'selected sensor configuration drift: {checks}')
    return {'checks': checks, 'selected_profile': 'b0183_c1',
            'general_profile_xyz_not_used_m': general['xyz_m'],
            'wheelbase_m': float(vehicle['drivetrain']['wheelbase_m'])}


def baseline_comparison(path=BASELINE):
    registered = json.loads(Path(path).read_text(encoding='utf-8'))
    expected = registered.get('sha256', {r['path']: r['sha256'] for r in registered.get('files', [])})
    if not expected:
        raise ValueError('empty baseline hash manifest')
    rows = {}
    for name, old in expected.items():
        source = REPO/name
        current = hashlib.sha256(source.read_bytes()).hexdigest() if source.is_file() else None
        rows[name] = {'expected': old, 'actual': current, 'matches': current == old}
    origin = registered.get('source_manifest')
    origin_matches = None
    if origin and (REPO/origin).exists():
        original = {r['path']: r['sha256'] for r in json.loads((REPO/origin).read_text())['files']}
        origin_matches = all(original.get(name) == value for name, value in expected.items())
    return {'manifest': str(path), 'source_manifest': origin,
            'source_manifest_subset_matches': origin_matches, 'files': rows,
            'unchanged': all(r['matches'] for r in rows.values()) and origin_matches is not False}


def target_at(case, elapsed_s):
    spec = case['target_box']
    if spec is None:
        return None
    travel = sum(max(0., min(elapsed_s, row['to_s']) - row['from_s'])*row['speed_mps']
                 for row in case['trajectory'])
    return TruthBox(spec['x_m']+travel, spec['y_m'], spec['height_m'],
                    Footprint(spec['length_m'], spec['width_m']), spec['yaw_rad'],
                    label='branch_wall' if spec.get('evaluation_kind') == 'branch_wall' else 'target')


def corridor_boxes(suite):
    half = suite['wall_gap_m']/2
    first, last = suite['corridor_wall_x_min_m'], suite['corridor_wall_x_max_m']
    boxes = [TruthBox((first+last)/2, sign*(half+.025), suite['wall_height_m'],
                     Footprint(last-first, .05), label='wall') for sign in (-1, 1)]
    if suite['end_caps']:
        boxes += [TruthBox(x, 0., suite['wall_height_m'], Footprint(.05, suite['wall_gap_m']+.10), label='wall')
                  for x in (first-.025, last+.025)]
    return boxes


def make_scene(ranges, stamps, suite, t_ref, epoch=0):
    sensor = suite['lidar']
    require_horizontal_scene(sensor)
    return scene_from_scan(list(map(float, ranges)), angle_min=-math.pi,
        angle_increment=math.tau/sensor['samples'], range_min=sensor['range_min_m'],
        range_max=sensor['range_max_m'], acquired_at_s=stamps, t_ref_s=t_ref,
        source_frame_id=sensor['frame_id'], source_epoch=epoch,
        sensor_xy_m=tuple(sensor['xyz_m'][:2]), rear_half_angle_deg=sensor['rear_half_angle_deg'])


def r0_diagnostics(ranges, scene, suite, wheelbase):
    from types import SimpleNamespace
    sensor = suite['lidar']
    scan = SimpleNamespace(angle_min=-math.pi, angle_increment=math.tau/sensor['samples'], ranges=list(ranges))
    masked = mask_scan(scan, sensor['rear_half_angle_deg'])
    audit = audit_front_observation(masked.ranges, scan.angle_min, scan.angle_increment,
                                   sensor['range_min_m'], sensor['range_max_m'])
    points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
    speed, steering, detail = pursuit_command(points, 'left', 0., wheelbase=wheelbase,
                                              max_speed=2.5, scan_age=scene.t_ref_s-scene.source_first_s)
    clearance = swept_limit(points, steering, wheelbase, length=suite['ego_footprint_m'][0],
                            width=suite['ego_footprint_m'][1])
    cap = braking_speed(clearance, scene.t_ref_s-scene.source_first_s)
    return {'scope': 'production pure-function diagnostic; no control/node/stop test',
            'pursuit_speed_mps': speed, 'pursuit_steering_rad': steering, 'pursuit': detail,
            'swept_limit_m': clearance, 'braking_speed_mps': cap,
            'audit_front_observation': {'reason': audit.reason, 'forward_samples': audit.forward_samples,
                                        'unknown_samples': audit.unknown_samples},
            'effective_diagnostic_speed_mps': min(speed, cap) if audit.reason == 'ok' else 0.,
            'r0_behavior_is_not_g0_tool_verdict': True}


def observation_metrics(scene, labels, target_label):
    visible = [ray for ray, label in zip(scene.rays, labels)
               if target_label is not None and label == target_label and ray.status == 'return']
    masked = sum(label == target_label and ray.status == 'masked'
                 for ray, label in zip(scene.rays, labels)) if target_label else 0
    distances = [math.dist(ray.ray_origin_xy_m, ray.hit_xy_m) for ray in visible]
    return {'visible_target_rays': len(visible), 'masked_target_rays': masked,
            'unknown_rays': sum(ray.status == 'unknown' for ray in scene.rays),
            'masked_rays': sum(ray.status == 'masked' for ray in scene.rays),
            'return_rays': sum(ray.status == 'return' for ray in scene.rays),
            'target_min_range_m': min(distances) if distances else None,
            'source_first_s': scene.source_first_s, 'source_last_s': scene.source_last_s,
            't_ref_s': scene.t_ref_s, 'source_epoch': scene.source_epoch,
            'truth_labels_evaluation_only': True}


def visibility_verdict(metrics, expected):
    # 이 무잡음·닫힌 회랑은 표적이 없어도 배경 반환이 있어야 한다. 표적 0개와
    # 입력 무효를 구분한다. 장착 masked는 제외하며 실차 일반 정책으로 쓰지 않는다.
    checks = {'minimum_visible': metrics['visible_target_rays'] >= expected['minimum_visible_target_rays'],
              'fixture_observation_complete': metrics['unknown_rays'] == 0 and metrics['return_rays'] > 0}
    if 'maximum_visible_target_rays' in expected:
        checks['maximum_visible'] = metrics['visible_target_rays'] <= expected['maximum_visible_target_rays']
    if 'minimum_masked_target_rays' in expected:
        checks['minimum_masked'] = metrics['masked_target_rays'] >= expected['minimum_masked_target_rays']
    return {'checks': checks, 'passed': all(checks.values())}


def analytic_case(suite, case, wheelbase):
    sensor = suite['lidar']
    first, period = 1., 1/sensor['rotation_hz']
    stamps = [first+i*period/sensor['samples'] for i in range(sensor['samples'])]
    ranges, labels = [], []
    # 합성 ray별 표적을 취득 시각에서 계산. 실제 렌더링의 대체 근거가 아니다.
    for i, stamp in enumerate(stamps):
        target = target_at(case, stamp-first)
        boxes = corridor_boxes(suite) + ([target] if target else [])
        direction = ray_direction(-math.pi+i*math.tau/sensor['samples'], sensor['pitch_rad'])
        hits = [(d, box.label) for box in boxes
                if (d := ray_box_distance(sensor['xyz_m'], direction, box, 0., sensor['range_max_m'])) is not None]
        distance, label = min(hits, default=(math.inf, None))
        ranges.append(distance if distance >= sensor['range_min_m'] else math.nan)
        labels.append(label)
    scene = make_scene(ranges, stamps, suite, first+period)
    target = target_at(case, 0.)
    geometry = cross_section_gaps(target, road_width_m=suite['road_width_m'],
        ego=Footprint(*suite['ego_footprint_m']), ego_yaw_rad=suite['ego_yaw_rad'], padding_m=suite['padding_each_side_m'])
    metrics = observation_metrics(scene, labels, target.label if target else None)
    visibility = visibility_verdict(metrics, case['expected'])
    geometry_ok = geometry['cross_section_possible'] == case['expected']['cross_section_possible']
    return {'case': case['name'], 'configuration': case, 'synthetic_rays': True,
            'duration_s': case['duration_s'], 'duration_executed': False,
            'geometry': geometry, 'geometry_matches_preregistered': geometry_ok,
            'observation_prediction': metrics, 'visibility_prediction': visibility,
            'r0': r0_diagnostics(ranges, scene, suite, wheelbase),
            'dynamic_phase_observation_verified': False,
            'analytic_contract_passed': geometry_ok and visibility['passed'],
            'rendered_observation_passed': None, 'vehicle_stop_verified': False,
            'collision_safety_verified': False, 'swept_curve_verified': False}


def select_cases(suite, name):
    if name == 'all':
        return suite['cases']
    matches = [case for case in suite['cases'] if case['name'] == name]
    if not matches:
        raise ValueError(f'unknown case: {name}')
    return matches


def snapshot(output, config, baseline):
    paths = [Path(__file__).resolve(), REPO/'scripts/g0_geometry.py', Path(config), Path(baseline),
             REPO/'scripts/check_g0_ros_context.py',
             REPO/'src/arena_autonomy/arena_autonomy/local_scene.py',
             REPO/'src/arena_vehicle_interface/arena_vehicle_interface/rotating_lidar.py',
             REPO/'scripts/lidar_shutdown.py']
    paths += [REPO/'tests'/name for name in ('test_local_scene.py', 'test_g0_geometry.py',
                                             'test_multi_vehicle_g0.py', 'test_r0_safety_contract.py')]
    paths += [REPO/p for p in json.loads(BASELINE.read_text())['sha256']]
    unique = list(dict.fromkeys(paths))
    with zipfile.ZipFile(output/'source_snapshot.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in unique:
            archive.write(path, str(path.relative_to(REPO)))
    return {str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest() for path in unique}


def make_world(output, suite, case):
    """고정 rig와 실제 렌더링 box. 공식 월드/차량 모델은 읽거나 변경하지 않는다."""
    root = ET.Element('sdf', version='1.9')
    world = ET.SubElement(root, 'world', name=WORLD_NAME)
    ET.SubElement(world, 'gravity').text = '0 0 0'
    physics = ET.SubElement(world, 'physics', name='g0_physics', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = str(suite['gazebo']['physics_step_s'])
    ET.SubElement(physics, 'real_time_factor').text = '1'
    for filename, name in (('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors')):
        plugin = ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
        if name == 'Sensors':
            ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    scene = ET.SubElement(world, 'scene')
    ET.SubElement(scene, 'ambient').text = '.8 .8 .8 1'
    ET.SubElement(scene, 'background').text = '.1 .1 .1 1'

    def box_model(name, box):
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = f'{box.x_m} {box.y_m} {box.bottom_m+box.height_m/2} 0 0 {box.yaw_rad}'
        link = ET.SubElement(model, 'link', name='body')
        for kind in ('visual', 'collision'):
            shape = ET.SubElement(link, kind, name=f'{name}_{kind}')
            ET.SubElement(ET.SubElement(ET.SubElement(shape, 'geometry'), 'box'), 'size').text = (
                f'{box.footprint.length_m} {box.footprint.width_m} {box.height_m}')
            if kind == 'visual':
                material = ET.SubElement(shape, 'material')
                ET.SubElement(material, 'ambient').text = '.5 .5 .5 1'
                ET.SubElement(material, 'diffuse').text = '.5 .5 .5 1'
    for index, box in enumerate(corridor_boxes(suite)):
        box_model(f'g0_wall_{index}', box)
    if (target := target_at(case, 0.)) is not None:
        box_model(ACTOR_MODEL, target)
        actor = world.find(f"model[@name='{ACTOR_MODEL}']")
        publisher = ET.SubElement(actor, 'plugin', filename='gz-sim-pose-publisher-system',
                                  name='gz::sim::systems::PosePublisher')
        # static model은 dynamic_pose/info에 없으므로 actor 평가용 자세를 별도 주기 발행한다.
        # 발행 토픽은 ACTOR_POSE_TOPIC으로 고정되며 SDF topic 요소는 지원되지 않는다.
        for name, value in (('publish_model_pose', 'true'), ('publish_link_pose', 'false'),
                            ('use_pose_vector_msg', 'true'), ('static_publisher', 'false'),
                            ('update_frequency', '100')):
            ET.SubElement(publisher, name).text = value
    rig = ET.SubElement(world, 'model', name='g0_sensor_rig')
    ET.SubElement(rig, 'static').text = 'true'
    link = ET.SubElement(rig, 'link', name='laser_frame')
    sensor_cfg = suite['lidar']
    sensor = ET.SubElement(link, 'sensor', name='g0_c1', type='gpu_lidar')
    # SDF +pitch는 앞쪽 downward이므로 평가의 앞쪽 upward 정의와 반대 부호.
    ET.SubElement(sensor, 'pose').text = ' '.join(map(str, [*sensor_cfg['xyz_m'], 0., -sensor_cfg['pitch_rad'], 0.]))
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'visualize').text = 'false'
    ET.SubElement(sensor, 'update_rate').text = str(sensor_cfg['raw_hz'])
    ET.SubElement(sensor, 'topic').text = RAW_TOPIC
    ET.SubElement(sensor, 'gz_frame_id').text = sensor_cfg['frame_id']
    lidar = ET.SubElement(sensor, 'lidar')
    horizontal = ET.SubElement(ET.SubElement(lidar, 'scan'), 'horizontal')
    for name, value in (('samples', sensor_cfg['samples']), ('resolution', 1),
                        ('min_angle', -math.pi), ('max_angle', math.pi-math.tau/sensor_cfg['samples'])):
        ET.SubElement(horizontal, name).text = str(value)
    limits = ET.SubElement(lidar, 'range')
    for name, value in (('min', sensor_cfg['range_min_m']), ('max', sensor_cfg['range_max_m']), ('resolution', .015)):
        ET.SubElement(limits, name).text = str(value)
    path = output/'world.sdf'
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
    return path


def renderer_environment(backend):
    env = dict(os.environ)
    if backend == 'software':
        env.update(GALLIUM_DRIVER='llvmpipe', LIBGL_ALWAYS_SOFTWARE='true')
    elif backend == 'wsl_nvidia':
        from ament_index_python.packages import get_package_prefix
        directory = Path(get_package_prefix('arena_gazebo'))/'lib'
        library = 'libarena-wsl-d3d12-lifetime.so'
        if not (directory/library).is_file():
            raise RuntimeError('missing WSL GPU lifetime helper in arena_gazebo/lib')
        env.update(GALLIUM_DRIVER='d3d12', LIBGL_ALWAYS_SOFTWARE='false',
                   MESA_D3D12_DEFAULT_ADAPTER_NAME=env.get('MESA_D3D12_DEFAULT_ADAPTER_NAME', 'NVIDIA'),
                   LD_LIBRARY_PATH=str(directory)+':'+env.get('LD_LIBRARY_PATH', ''),
                   LD_PRELOAD=library+(':'+env['LD_PRELOAD'] if env.get('LD_PRELOAD') else ''))
    return env


def bridge_arguments():
    """평가 전용 gz→ROS 단방향 센서/자세와 actor set_pose 서비스. 구동 토픽은 없다."""
    return [f'{RAW_TOPIC}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'{ACTOR_POSE_TOPIC}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'{SET_POSE_SERVICE}@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean',
            '--ros-args', '-p', f'qos_overrides.{RAW_TOPIC}.publisher.reliability:=reliable',
            '-p', f'qos_overrides.{RAW_TOPIC}.publisher.depth:=2000',
            '-p', f'qos_overrides.{ACTOR_POSE_TOPIC}.publisher.reliability:=reliable']


def startup_missing(raw_stamp, first_pose_stamp, service_ready, case):
    """시작 대기에서 아직 받지 못한 stream 이름. 실패 원인을 하나로 뭉치지 않는다.

    평가 정답 자세는 시작 원시 시각 이전부터 있어야 첫 회전의 모든 광선에 선행
    자세가 존재한다(central_v3: 첫 자세 0.010 s > 첫 광선 0.008 s로 10개 누락).
    """
    missing = []
    if raw_stamp is None:
        missing.append('raw_scan')
    if case['target_box'] is not None:
        if first_pose_stamp is None:
            missing.append('actor_pose')
        elif raw_stamp is not None and first_pose_stamp > raw_stamp:
            missing.append('actor_pose_before_raw_start')
    if case['trajectory'] and not service_ready:
        missing.append('actor_set_pose_service')
    return missing


def gz_topic_list(env):
    """startup 실패 시 gz 측 실제 토픽 목록만 기록한다. 판정 입력이 아니다."""
    try:
        result = subprocess.run(['gz', 'topic', '-l'], env=env, capture_output=True, text=True, timeout=10)
        return {'return_code': result.returncode, 'topics': result.stdout.split(), 'stderr': result.stderr[-2000:]}
    except (subprocess.TimeoutExpired, OSError) as error:
        return {'error': str(error)}


def fill_actor_request(request, target):
    """SetEntityPose request 구성만 수행. 응답과 실제 자세 확인은 분리한다."""
    request.entity.name = ACTOR_MODEL
    request.entity.type = request.entity.MODEL
    request.pose.position.x, request.pose.position.y = target.x_m, target.y_m
    request.pose.position.z = target.bottom_m+target.height_m/2
    request.pose.orientation.z, request.pose.orientation.w = math.sin(target.yaw_rad/2), math.cos(target.yaw_rad/2)
    return request


def process_cleanup(process):
    """본 하네스가 만든 단일 프로세스 그룹만 종료한다."""
    from lidar_shutdown import stop_launch, group_processes
    if process is None:
        return {'started': False, 'clean': False}
    result = stop_launch(process)
    remaining = group_processes(process.pid)
    # launcher가 먼저 끝난 경우도 자기 session/group 잔존만 정리하고 실패로 남긴다.
    if remaining:
        for sig in (signal.SIGTERM, signal.SIGKILL):
            current = group_processes(process.pid)
            if not current:
                break
            same = {pid: identity for pid, identity in current.items() if remaining.get(pid) == identity}
            if not same:
                break
            result['forced_cleanup'] = sig.name
            for pid in same:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic()+2.
            while group_processes(process.pid) and time.monotonic() < deadline:
                time.sleep(.05)
    remaining = group_processes(process.pid)
    return {'started': True, 'return_code': process.returncode,
            'remaining_owned_group': remaining,
            'clean': process.returncode == 0 and not result and not remaining, **result}


def source_ray_times(first, increment, count, raw_times):
    """완료 회전의 실제 previous raw 출처. assembler의 1e-10 경계를 유지한다."""
    result = []
    for index in range(count):
        nominal = first+index*increment
        source_index = bisect_right(raw_times, nominal+1e-10)-1
        if source_index < 0:
            raise ValueError('missing raw provenance')
        result.append(raw_times[source_index])
    return result


def renderer_log_state(output):
    paths = list((Path.home()/'.gz'/'rendering').glob('*.log'))
    paths += list((output/'gz_logs').rglob('*.log'))
    return {str(path): {'mtime_ns': path.stat().st_mtime_ns,
                        'sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'size': path.stat().st_size}
            for path in paths}


def renderer_evidence(output, before):
    paths = list((Path.home()/'.gz'/'rendering').glob('*.log'))+list((output/'gz_logs').rglob('*.log'))
    paths.append(output/'simulation.log')
    evidence = []
    for path in paths:
        if not path.is_file():
            continue
        old = before.get(str(path))
        content = path.read_bytes()
        if old and old['mtime_ns'] == path.stat().st_mtime_ns and old['sha256'] == hashlib.sha256(content).hexdigest():
            continue
        lines = [line.strip() for line in content.decode('utf-8', errors='replace').splitlines() if 'GL_RENDERER' in line]
        if lines:
            evidence.append({'path': str(path), 'new_or_changed_since_launch': True, 'lines': lines})
    return evidence


def normal_server_stop(server, env):
    from lidar_shutdown import group_processes
    result = {'acknowledged': False, 'server_exit_confirmed': False, 'owned_group_empty': False}
    if server is None or server.poll() is not None:
        return dict(result, reason='server_not_running')
    try:
        response = subprocess.run(['gz', 'service', '-s', '/server_control', '--reqtype', 'gz.msgs.ServerControl',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', 'stop: true'],
            env=env, capture_output=True, text=True, timeout=8)
        result.update(return_code=response.returncode, stdout=response.stdout, stderr=response.stderr)
        result['acknowledged'] = response.returncode == 0 and 'data: true' in response.stdout
        if result['acknowledged']:
            server.wait(timeout=20)
            result['server_exit_confirmed'] = server.returncode == 0
            remaining = group_processes(server.pid)
            result['remaining_owned_group'] = remaining
            result['owned_group_empty'] = not remaining
    except (subprocess.TimeoutExpired, OSError) as error:
        result['error'] = str(error)
    return result


def raw_artifact_metadata(output):
    files = {}
    for name in ('raw_scans.jsonl', 'actor_poses.jsonl', 'sequential_scans.jsonl'):
        path = output/name
        files[name] = {'relative_path': str(path.relative_to(REPO)), 'bytes': path.stat().st_size,
                       'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                       'storage': 'local raw evidence; excluded from public Git'}
    return files


def summarize_phases(scans, poses, case, start):
    if not case['trajectory']:
        return {'required': False, 'passed': None}
    phases = []
    expected = case['expected'].get('phase_range_change_m')
    for segment in case['trajectory']:
        # 각 전환 양끝 0.2초 제외: 0.1초 회전과 command/관측 오차의 사전 예산.
        low, high = segment['from_s']+.2, segment['to_s']-.2
        rows = [row for row in scans if low <= row['source_first_s']-start <= high and row['target_min_range_m'] is not None]
        truth = [row for row in poses if low <= row['stamp_s']-start <= high]
        delta = rows[-1]['target_min_range_m']-rows[0]['target_min_range_m'] if len(rows) >= 2 else None
        truth_delta = truth[-1]['x_m']-truth[0]['x_m'] if len(truth) >= 2 else None
        if segment['speed_mps'] == 0:
            threshold = expected['stopped_maximum'] if expected else .025
            ok = delta is not None and truth_delta is not None and abs(delta) <= threshold and abs(truth_delta) <= threshold
        else:
            threshold = expected['moving_minimum'] if expected else .10
            ok = delta is not None and truth_delta is not None and delta >= threshold and truth_delta >= threshold
        phases.append({'segment': segment, 'sensor_samples': len(rows), 'pose_samples': len(truth),
                       'sensor_range_delta_m': delta, 'observed_pose_x_delta_m': truth_delta,
                       'sensor_first_last_s': [rows[0]['source_first_s'], rows[-1]['source_first_s']] if rows else None,
                       'truth_first_last_s': [truth[0]['stamp_s'], truth[-1]['stamp_s']] if truth else None,
                       'passed': ok})
    return {'required': True, 'phases': phases, 'passed': bool(phases) and all(row['passed'] for row in phases)}


def gazebo_case(suite, case, output, backend, wheelbase):
    """ROS/gz는 이 실행 경로에서만 import. ego고정, source truth는 평가만."""
    row = analytic_case(suite, case, wheelbase)
    row.update(synthetic_rays=False, rendered_observation_passed=False, failures=[],
               fixed_ego=True, ego_virtual_origin_xy_m=[0., 0.], environment={}, shutdown={})
    sensor, limits, runtime = suite['lidar'], suite['failure_limits'], suite['gazebo']
    processes, actor_future, server, bridge, node, context, executor = [], None, None, None, None, None, None
    raw_times, pose_rows, commands, scans, completed = [], [], [], [], []
    status = {'stamp': None, 'start': None, 'error': None, 'epoch': 0, 'last_command_s': None}
    # stream별 시작 진단. raw_frame_count는 start 이후만 세므로 시작 전 수신을 따로 남긴다.
    startup = {'raw_messages_before_start': 0, 'raw_first_stamp_s': None, 'raw_first_wall_s': None,
               'actor_pose_topic': ACTOR_POSE_TOPIC, 'actor_pose_messages': 0, 'actor_pose_first_wall_s': None,
               'actor_pose_first_stamp_s': None,
               'pose_frames_seen': [], 'actor_service_ready_wall_s': None, 'missing_streams': None,
               'ready_wall_s': None}
    began = time.monotonic()
    env = dict(os.environ, ROS_DOMAIN_ID=str(80+os.getpid()%120), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_STATIC_PEERS='', GZ_PARTITION=f'arena_g0_{os.getpid()}_{case["name"]}')
    log = (output/'simulation.log').open('w', encoding='utf-8')
    raw_stream = (output/'raw_scans.jsonl').open('w', encoding='utf-8')
    pose_stream = (output/'actor_poses.jsonl').open('w', encoding='utf-8')
    scan_stream = (output/'sequential_scans.jsonl').open('w', encoding='utf-8')
    old_signal = {}
    renderer_before = renderer_log_state(output)
    stop_requested = False
    def interrupted(*_):
        nonlocal stop_requested
        stop_requested = True
    def stamp(message):
        return message.sec+message.nanosec*1e-9
    try:
        if os.name != 'posix':
            raise OSError('Gazebo probe requires the configured Linux/WSL ROS environment')
        world = make_world(output, suite, case)
        subprocess.run(['gz', 'sdf', '-k', str(world)], capture_output=True, check=True, text=True, timeout=10, env=env)
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage
        from ros_gz_interfaces.srv import SetEntityPose
        from arena_vehicle_interface.rotating_lidar import RevolutionAssembler
        # rclpy/RMW와 모든 자식이 같은 격리 domain/partition을 사용한다.
        previous_env = {key: os.environ.get(key) for key in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_STATIC_PEERS', 'GZ_PARTITION')}
        os.environ.update({key: env[key] for key in previous_env})
        context = Context()
        rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('g0_observation_probe', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        actor_client = node.create_client(SetEntityPose, SET_POSE_SERVICE)
        assembler = RevolutionAssembler(sensor['rotation_hz'], sensor['samples'], sensor['max_source_gap_s'])
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_signal[sig] = signal.getsignal(sig)
            signal.signal(sig, interrupted)

        def actor_pose(message):
            startup['actor_pose_messages'] += 1
            if startup['actor_pose_first_wall_s'] is None:
                startup['actor_pose_first_wall_s'] = time.monotonic()-began
            for transform in message.transforms:
                frames = [transform.header.frame_id, transform.child_frame_id]
                if frames not in startup['pose_frames_seen'] and len(startup['pose_frames_seen']) < 10:
                    startup['pose_frames_seen'].append(frames)
                if transform.child_frame_id != ACTOR_MODEL:
                    continue
                if transform.header.frame_id != WORLD_NAME:
                    # world 기준이 아닌 자세로 bbox 라벨을 만들지 않는다.
                    status['error'] = 'actor pose parent frame mismatch'
                    return
                when = stamp(transform.header.stamp)
                if pose_rows and when <= pose_rows[-1]['stamp_s']:
                    continue
                translation, orientation = transform.transform.translation, transform.transform.rotation
                observed = {'stamp_s': when, 'x_m': float(translation.x), 'y_m': float(translation.y),
                            'z_m': float(translation.z),
                            'yaw_rad': math.atan2(2*(orientation.w*orientation.z+orientation.x*orientation.y),
                                                 1-2*(orientation.y**2+orientation.z**2)),
                            'received_wall_s': time.monotonic()-began, 'evaluation_only': True}
                pose_rows.append(observed)
                pose_stream.write(json.dumps(observed, allow_nan=False)+'\n')

        def on_raw(message):
            when = stamp(message.header.stamp)
            if status['stamp'] is not None and when <= status['stamp']:
                if when < status['stamp']:
                    status['epoch'] += 1
                    status['error'] = 'raw source timestamp reversed'
                return
            status['stamp'] = when
            if (len(message.ranges) != sensor['samples'] or abs(message.angle_min+math.pi) > 1e-5
                    or abs(message.angle_increment-math.tau/sensor['samples']) > 1e-5
                    or abs(message.range_min-sensor['range_min_m']) > 1e-5
                    or abs(message.range_max-sensor['range_max_m']) > 1e-5):
                status['error'] = 'raw sensor interface mismatch'
                return
            if status['start'] is None:
                startup['raw_messages_before_start'] += 1
                if startup['raw_first_stamp_s'] is None:
                    startup.update(raw_first_stamp_s=when, raw_first_wall_s=time.monotonic()-began)
                return
            raw_times.append(when)
            raw_stream.write(json.dumps({'stamp_s': when, 'received_wall_s': time.monotonic()-began,
                'frame_id': message.header.frame_id, 'ranges': [float(r) if math.isfinite(r) else None for r in message.ranges]}, allow_nan=False)+'\n')
            for first, ranges, error in assembler.feed(when, message.ranges):
                completed.append({'first': first, 'ranges': list(map(float, ranges)),
                                  'capture_error': error, 'published': when, 'epoch': status['epoch']})

        def evaluate_completed():
            # 수집 중 bbox/R0 계산을 하지 않는다. actor 명령·원시 수신을 먼저 수행한다.
            for capture in completed:
                first, ranges, error, when = capture['first'], capture['ranges'], capture['capture_error'], capture['published']
                nominal = [first+i*assembler.increment for i in range(sensor['samples'])]
                actual = source_ray_times(first, assembler.increment, sensor['samples'], raw_times)
                scene = make_scene(ranges, actual, suite, when, capture['epoch'])
                target_template = target_at(case, 0.)
                labels, pose_ages, classification_misses = [], [], 0
                pose_times = [p['stamp_s'] for p in pose_rows]
                for index, (distance, source_time) in enumerate(zip(ranges, actual)):
                    target = None
                    if target_template is not None:
                        pi = bisect_right(pose_times, source_time+1e-10)-1
                        if pi >= 0:
                            observed = pose_rows[pi]
                            pose_ages.append(source_time-observed['stamp_s'])
                            target = replace(target_template, x_m=observed['x_m'], y_m=observed['y_m'],
                                             yaw_rad=observed['yaw_rad'], bottom_m=observed['z_m']-target_template.height_m/2)
                        else:
                            classification_misses += 1
                    direction = ray_direction(-math.pi+index*math.tau/sensor['samples'], sensor['pitch_rad'])
                    boxes = corridor_boxes(suite)+([target] if target else [])
                    hits = [(d, box.label) for box in boxes if (d := ray_box_distance(sensor['xyz_m'], direction, box, 0., sensor['range_max_m'])) is not None]
                    expected_distance, label = min(hits, default=(math.inf, None))
                    # bbox 일치 라벨은 평가에만 존재. 실제 range가 일치해야 target hit로 센다.
                    labels.append(label if math.isfinite(distance)
                                  and abs(distance-expected_distance) <= suite['ray_label_match_tolerance_m'] else None)
                metrics = observation_metrics(scene, labels, target_template.label if target_template else None)
                metrics.update(nominal_first_ray_s=first, nominal_last_ray_s=nominal[-1],
                               source_stamp_at_publish_s=when, max_capture_time_error_s=error,
                               source_pose_max_age_s=max(pose_ages, default=None),
                               source_pose_missing_rays=classification_misses,
                               visibility=visibility_verdict(metrics, case['expected']))
                metrics['r0'] = r0_diagnostics(ranges, scene, suite, wheelbase)
                scans.append(metrics)
                scan_stream.write(json.dumps(metrics, allow_nan=False)+'\n')

        qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(LaserScan, RAW_TOPIC, on_raw, qos)
        node.create_subscription(TFMessage, ACTOR_POSE_TOPIC, actor_pose, qos)
        render_env = renderer_environment(backend)
        render_env.update({key: env[key] for key in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_STATIC_PEERS', 'GZ_PARTITION')})
        # GZ_LOG_PATH는 gz 로그 위치만 요청한다. HOME은 바꾸지 않으므로 ogre2.log는
        # ~/.gz/rendering에 갱신되며(central_v2 확인), 시작 전 해시와 대조해 새 기록만 쓴다.
        render_env['GZ_LOG_PATH'] = str(output/'gz_logs')
        server = subprocess.Popen(['gz', 'sim', '-r', '-s', '-v', '3', str(world)],
            cwd=REPO, env=render_env, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
        processes.append(server)
        from ament_index_python.packages import get_package_prefix
        bridge_executable = Path(get_package_prefix('ros_gz_bridge'))/'lib/ros_gz_bridge/parameter_bridge'
        if not bridge_executable.is_file():
            raise OSError('ros_gz_bridge executable missing at ament prefix')
        bridge = subprocess.Popen([str(bridge_executable), *bridge_arguments()],
            cwd=REPO, env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
        processes.append(bridge)
        def spin():
            executor.spin_once(timeout_sec=.001)
            if stop_requested:
                raise RuntimeError('user interruption')
            if time.monotonic()-began > runtime['hard_wall_cap_s']:
                raise TimeoutError('hard wall cap')
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('owned simulator/bridge exited early')
            if status['error']:
                raise RuntimeError(status['error'])
        while True:
            ready = actor_client.service_is_ready()
            if ready and startup['actor_service_ready_wall_s'] is None:
                startup['actor_service_ready_wall_s'] = time.monotonic()-began
            startup['actor_pose_first_stamp_s'] = pose_rows[0]['stamp_s'] if pose_rows else None
            startup['missing_streams'] = startup_missing(status['stamp'], startup['actor_pose_first_stamp_s'], ready, case)
            if not startup['missing_streams']:
                break
            spin()
            if time.monotonic()-began > runtime['startup_wall_timeout_s']:
                startup['gz_topics_at_timeout'] = gz_topic_list(env)
                raise TimeoutError(f"startup timeout; missing streams: {', '.join(startup['missing_streams'])}")
        startup['ready_wall_s'] = time.monotonic()-began
        status['start'] = status['stamp']
        while status['stamp']-status['start'] < case['duration_s']:
            spin()
            now = status['stamp']
            if actor_future is not None and actor_future.done():
                response = actor_future.result()
                commands[-1].update(response_wall_s=time.monotonic()-began,
                                    acknowledged=response is not None and response.success)
                if not commands[-1]['acknowledged']:
                    raise RuntimeError('actor set_pose was not acknowledged')
                actor_future = None
            if actor_future is not None and time.monotonic()-commands[-1]['requested_wall_absolute_s'] > 8:
                raise TimeoutError('actor command timeout')
            if case['trajectory'] and actor_future is None and (status['last_command_s'] is None or now-status['last_command_s'] >= 1/runtime['actor_command_max_hz']):
                target = target_at(case, now-status['start'])
                actor_future = actor_client.call_async(fill_actor_request(SetEntityPose.Request(), target))
                commands.append({'requested_sim_s': now, 'requested_wall_s': time.monotonic()-began,
                    'requested_wall_absolute_s': time.monotonic(), 'x_m': target.x_m, 'y_m': target.y_m,
                    'evaluation_only': True, 'acknowledged': False, 'transport': 'ROS async SetEntityPose bridge'})
                status['last_command_s'] = now
        # 마지막 회전+실제 actor 자세를 회수하며 명령 응답을 종료 전에 기다린다.
        finish = status['stamp']
        while status['stamp'] < finish+.12 or actor_future is not None:
            spin()
            if actor_future is not None and actor_future.done():
                response = actor_future.result()
                commands[-1].update(response_wall_s=time.monotonic()-began,
                    acknowledged=response is not None and response.success)
                actor_future = None
            if actor_future is not None and time.monotonic()-commands[-1]['requested_wall_absolute_s'] > 8:
                raise TimeoutError('final actor command timeout')
            if time.monotonic()-began > runtime['hard_wall_cap_s']:
                raise TimeoutError('final drain timeout')
        # actor 프로세스 완료·sensor drain 후 분석하여 원시 수신을 방해하지 않는다.
        evaluate_completed()
        active = [r for r in scans if r['source_first_s'] >= status['start'] and r['source_last_s'] <= status['start']+case['duration_s']]
        raw_rate = (len(raw_times)-1)/(raw_times[-1]-raw_times[0]) if len(raw_times)>1 else 0.
        scan_rate = (len(scans)-1)/(scans[-1]['nominal_first_ray_s']-scans[0]['nominal_first_ray_s']) if len(scans)>1 else 0.
        source_gaps = np.diff(raw_times).tolist()
        active_poses = [p for p in pose_rows if status['start'] <= p['stamp_s'] <= status['start']+case['duration_s']+.12]
        pose_intervals = np.diff([p['stamp_s'] for p in active_poses]).tolist()
        changed = [active_poses[0]] if active_poses else []
        for pose in active_poses[1:]:
            if math.dist((pose['x_m'], pose['y_m']), (changed[-1]['x_m'], changed[-1]['y_m'])) > 1e-7:
                changed.append(pose)
        observed_update_intervals = np.diff([p['stamp_s'] for p in changed]).tolist()
        moving_observed_intervals = [b['stamp_s']-a['stamp_s'] for a, b in zip(changed, changed[1:])
            if any(segment['speed_mps'] != 0 and segment['from_s']+.02 <= a['stamp_s']-status['start']
                   and b['stamp_s']-status['start'] <= segment['to_s']-.02 for segment in case['trajectory'])]
        command_gaps = np.diff([c['requested_sim_s'] for c in commands]).tolist()
        actor_errors = []
        for pose in active_poses:
            if commands:
                ci = bisect_right([c['requested_sim_s'] for c in commands], pose['stamp_s'])-1
                if ci >= 0:
                    actor_errors.append(math.dist((pose['x_m'], pose['y_m']), (commands[ci]['x_m'], commands[ci]['y_m'])))
            else:
                target = target_at(case, 0.)
                if target:
                    actor_errors.append(math.dist((pose['x_m'], pose['y_m']), (target.x_m, target.y_m)))
        phases = summarize_phases(active, active_poses, case, status['start'])
        maximum_speed = max((abs(s['speed_mps']) for s in case['trajectory']), default=0.)
        max_command_gap = max(command_gaps, default=0.)
        max_observed_step = max(moving_observed_intervals, default=0.)
        checks = {'minimum_scan_count': len(active) >= limits['minimum_scans'],
                  'raw_rate': raw_rate >= sensor['raw_hz']*limits['minimum_raw_rate_fraction'],
                  'scan_rate': scan_rate >= sensor['rotation_hz']*limits['minimum_scan_rate_fraction'],
                  'source_gaps': bool(source_gaps) and max(source_gaps) <= limits['maximum_source_gap_s']+1e-8,
                  'no_discarded_source_gaps': assembler.discarded == 0,
                  'capture_time': bool(scans) and max(s['max_capture_time_error_s'] for s in scans) <= sensor['max_source_gap_s']+1e-8,
                  'preregistered_visibility': bool(active) and all(s['visibility']['passed'] for s in active),
                  'actual_pose_source_present': case['target_box'] is None or (bool(active_poses) and all(s['source_pose_missing_rays']==0 for s in active)),
                  'actual_pose_source_fresh': case['target_box'] is None or (bool(pose_intervals) and max(pose_intervals) <= limits['maximum_actor_command_step_s']+1e-8 and all(s['source_pose_max_age_s'] is not None and s['source_pose_max_age_s'] <= limits['maximum_actor_command_step_s'] for s in active)),
                  'actor_observation_error': case['target_box'] is None or bool(actor_errors) and max(actor_errors) <= limits['maximum_actor_observation_error_m'],
                  'actor_commands': not case['trajectory'] or bool(commands) and all(c['acknowledged'] for c in commands),
                  'actor_command_steps': not case['trajectory'] or bool(command_gaps) and max_command_gap <= limits['maximum_actor_command_step_s'],
                  'actor_observed_steps': not case['trajectory'] or bool(moving_observed_intervals) and max_observed_step <= limits['maximum_actor_command_step_s'],
                  'dynamic_sensor_and_pose_phases': not phases['required'] or phases['passed']}
        row.update(duration_executed=True, start_sim_s=status['start'], end_sim_s=status['stamp'],
            raw_frame_count=len(raw_times), sequential_scan_count=len(scans), active_scan_count=len(active),
            raw_first_last_s=[raw_times[0], raw_times[-1]] if raw_times else None,
            sequential_first_last_s=[scans[0]['source_first_s'], scans[-1]['source_last_s']] if scans else None,
            raw_rate_sim_hz=raw_rate, sequential_rate_sim_hz=scan_rate,
            raw_max_source_gap_s=max(source_gaps, default=None), discarded_source_gaps=assembler.discarded,
            max_capture_time_error_s=max((s['max_capture_time_error_s'] for s in scans), default=None),
            checks=checks, observation_summary={'visible_target_rays_min': min((s['visible_target_rays'] for s in active), default=None),
            'visible_target_rays_max': max((s['visible_target_rays'] for s in active), default=None),
            'unknown_rays_total': sum(s['unknown_rays'] for s in active), 'masked_rays_total': sum(s['masked_rays'] for s in active)},
            actor={'command_count': len(commands), 'observed_pose_count': len(active_poses),
                'max_commanded_step_s': max_command_gap, 'max_observed_pose_interval_s': max(pose_intervals, default=None),
                'max_observed_position_update_interval_s': max_observed_step,
                'max_all_position_change_interval_including_stop_s': max(observed_update_intervals, default=None),
                'max_observed_error_to_latest_command_m': max(actor_errors, default=None),
                'command_discretization_bound_m': maximum_speed*max_command_gap,
                'observed_update_discretization_bound_m': maximum_speed*max_observed_step,
                'bound_scope': 'observed moving-phase update intervals; separate from raw capture error; not continuous dynamic precision proof'},
            dynamic_phase_observation_verified=phases['passed'] if phases['required'] else False,
            dynamic_phases=phases)
        row['rendered_observation_passed'] = all(checks.values()) and row['geometry_matches_preregistered']
    except Exception as error:
        row['failures'].append({'source': failure_source(error), 'type': type(error).__name__, 'message': str(error),
                                'traceback': traceback.format_exc()})
    finally:
        if actor_future is not None:
            actor_future.cancel()
            row['shutdown']['actor_request_cancelled'] = True
        row['shutdown']['server_control'] = normal_server_stop(server, env)
        row['shutdown']['bridge'] = process_cleanup(bridge)
        row['shutdown']['server'] = process_cleanup(server)
        row['shutdown']['clean'] = (row['shutdown']['server_control']['server_exit_confirmed']
                                    and row['shutdown']['server_control']['owned_group_empty']
                                    and row['shutdown']['bridge']['clean'] and row['shutdown']['server']['clean'])
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown(timeout_sec=2.)
        if node is not None:
            node.destroy_node()
        if context is not None:
            context.try_shutdown()
        if 'previous_env' in locals():
            for key, value in previous_env.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value
        for sig, handler in old_signal.items():
            signal.signal(sig, handler)
        log.close()
        raw_stream.close()
        pose_stream.close()
        scan_stream.close()
        row['raw_artifacts'] = raw_artifact_metadata(output)
        write_json(output/'actor_commands.json', commands)
        row['environment'].update(render_backend_request=backend, ros_domain_id=env['ROS_DOMAIN_ID'], gz_partition=env['GZ_PARTITION'])
        row.setdefault('raw_frame_count', len(raw_times))
        row.setdefault('sequential_scan_count', len(completed))
        if 'assembler' in locals():
            row.setdefault('discarded_source_gaps', assembler.discarded)
        renderer_lines = renderer_evidence(output, renderer_before)
        row['environment']['actual_renderer_evidence'] = renderer_lines
        row['environment']['actual_renderer_verified'] = bool(renderer_lines)
        row['startup'] = startup
        row['rendered_observation_passed'] = (row['rendered_observation_passed'] and row['shutdown']['clean']
                                             and row['environment']['actual_renderer_verified'])
        write_json(output/'report.json', row)
    return row


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('analytic', 'gazebo'), default='analytic')
    parser.add_argument('--case', default='all')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=CONFIG)
    parser.add_argument('--baseline', type=Path, default=BASELINE)
    parser.add_argument('--render-backend', choices=('system', 'software', 'wsl_nvidia'), default='system')
    args = parser.parse_args(argv)
    suite = load_suite(args.config)
    cases = select_cases(suite, args.case)
    output = args.output.resolve()
    if not output.is_relative_to(REPO/'artifacts'):
        raise ValueError('output must be a new directory under repository artifacts')
    output.mkdir(parents=True, exist_ok=False)
    report = {'schema_version': 1, 'started_at_utc': datetime.now(timezone.utc).isoformat(),
              'mode': args.mode, 'seed': suite['seed'], 'scope': suite['scope'],
              'synthetic_rays': args.mode == 'analytic', 'cases': [], 'passed': False,
              'vehicle_stop_verified': False, 'collision_safety_verified': False,
              'full_course_verified': False, 'swept_curve_verified': False,
              'drive_output_topics': [], 'failures': [], 'environment': {}, 'shutdown': []}
    try:
        report['source_sha256'] = snapshot(output, args.config, args.baseline)
        report['r0_baseline'] = baseline_comparison(args.baseline)
        if not report['r0_baseline']['unchanged']:
            raise RuntimeError('R0 baseline changed before G0 probe')
        report['sensor_config'] = verify_sensor_config(suite)
        if args.mode == 'analytic':
            report['cases'] = [analytic_case(suite, case, report['sensor_config']['wheelbase_m']) for case in cases]
            report['passed'] = all(c['analytic_contract_passed'] for c in report['cases'])
        else:
            for case in cases:
                directory = output/case['name']
                directory.mkdir()
                row = gazebo_case(suite, case, directory, args.render_backend, report['sensor_config']['wheelbase_m'])
                report['cases'].append(row)
                report['shutdown'].append(row['shutdown'])
            report['passed'] = bool(report['cases']) and all(c['rendered_observation_passed'] for c in report['cases'])
    except Exception as error:
        report['failures'].append({'source': failure_source(error),
                                   'type': type(error).__name__, 'message': str(error), 'traceback': traceback.format_exc()})
    finally:
        report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        write_json(output/'report.json', report)
    print(json.dumps({'passed': report['passed'], 'report': str(output/'report.json')}, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
