#!/usr/bin/env python3
"""정지 C1 GPU 광선의 기울기/바닥 진단. 차량 통과·순차 운동·하드웨어 시험 아님."""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET
import zipfile

import yaml

from lidar_shutdown import stop_gazebo_service, stop_launch, audit_shutdown, group_processes
from validate_start_signal_render import stop_bridge_first, cleanup_owned_residue

ROOT = Path(__file__).resolve().parents[1]


def put(parent, name, value, **attributes):
    item = ET.SubElement(parent, name, attributes)
    item.text = str(value)
    return item


def build_fixture(sensor, height, pitch):
    if not .02 < height < .3 or not 0 < pitch < .2:
        raise ValueError('bounded positive sensor height and SDF nose-down pitch required')
    root = ET.Element('sdf', version='1.10')
    world = ET.SubElement(root, 'world', name='bump_lidar_pose')
    physics = ET.SubElement(world, 'physics', name='physics', type='ode')
    put(physics, 'max_step_size', .001)
    put(physics, 'real_time_factor', 1.)
    for plugin, name in [('physics', 'Physics'), ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors')]:
        item = ET.SubElement(world, 'plugin', filename=f'gz-sim-{plugin}-system', name=f'gz::sim::systems::{name}')
        if plugin == 'sensors':
            put(item, 'render_engine', 'ogre2')
    floor = ET.SubElement(world, 'model', name='floor')
    put(floor, 'static', 'true')
    link = ET.SubElement(floor, 'link', name='floor_link')
    for kind in ('visual', 'collision'):
        item = ET.SubElement(link, kind, name=kind)
        plane = ET.SubElement(ET.SubElement(item, 'geometry'), 'plane')
        put(plane, 'normal', '0 0 1')
        put(plane, 'size', '200 200')
    views = {}
    for i, (name, angle) in enumerate([('level', 0.), ('nose_down', pitch), ('nose_up', -pitch)]):
        topic = '/audit/bump_pose/'+name
        model = ET.SubElement(world, 'model', name=name)
        put(model, 'static', 'true')
        put(model, 'pose', f'0 {30*(i-1)} {height:.17g} 0 {angle:.17g} 0')
        holder = ET.SubElement(model, 'link', name='sensor_link')
        clone = copy.deepcopy(sensor)
        clone.find('topic').text = topic
        holder.append(clone)
        views[name] = dict(topic=topic, sensor_height_m=height, sdf_pitch_rad=angle,
                           note='isolated sensor, no vehicle self-occlusion or rear mask')
    return root, views


def expected_front_range(height, pitch, angle, maximum):
    vertical_down = math.sin(pitch)*math.cos(angle)
    if vertical_down <= 0:
        return None
    distance = height/vertical_down
    return distance if distance < maximum else None


def inspect_scan(row, view, tolerance=.03):
    """±10° 안 전방 광선만 비교; 다른 각도/가림의 검증으로 확대하지 않는다."""
    values = row['ranges']
    numeric = lambda v: isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
    fields = ('angle_min', 'angle_increment', 'range_min', 'range_max')
    if not all(numeric(row.get(k)) for k in fields) or not numeric(tolerance) or not 0 < tolerance <= .03:
        raise ValueError('invalid scan metadata or comparison tolerance')
    if abs(row['range_min']-.05) > 1e-6 or abs(row['range_max']-12.) > 1e-5:
        raise ValueError('scan range differs from selected C1 proxy')
    if len(values) != 500 or not math.isfinite(row['angle_min']) or not 0 < row['angle_increment'] < .1:
        raise ValueError('unexpected scan geometry')
    if abs(row['angle_min']+math.pi) > 1e-5 or abs(row['angle_increment']-2*math.pi/500) > 1e-6:
        raise ValueError('scan angular coverage differs from selected C1 proxy')
    errors, mismatches, compared = [], [], 0
    for i, measured in enumerate(values):
        angle = row['angle_min']+i*row['angle_increment']
        if abs(angle) > math.radians(10):
            continue
        compared += 1
        expected = expected_front_range(view['sensor_height_m'], view['sdf_pitch_rad'], angle, row['range_max'])
        if measured is not None and (not numeric(measured) or measured < row['range_min']):
            mismatches.append(dict(index=i, reason='invalid_value'))
        elif expected is None:
            if measured is not None and measured < row['range_max']-1e-5:
                mismatches.append(dict(index=i, reason='unexpected_floor_return', measured=measured))
        elif measured is None or measured >= row['range_max']:
            mismatches.append(dict(index=i, reason='missing_expected_floor_return'))
        else:
            error = abs(measured-expected)
            errors.append(error)
            if error > tolerance:
                mismatches.append(dict(index=i, reason='range_difference', error_m=error))
    return dict(passed=compared > 0 and not mismatches, front_rays_compared=compared,
                max_range_difference_m=max(errors) if errors else None, mismatches=mismatches,
                comparison_tolerance_m=tolerance,
                tolerance_note='fixture rendering comparison, not hardware accuracy or guard threshold')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--geometry-report', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=90.)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts') or output == ROOT/'artifacts' or not 30 <= args.timeout <= 180:
        parser.error('new artifacts subfolder and bounded 30..180 s timeout required')
    geometry = json.loads(args.geometry_report.read_text())
    case = next(c for c in geometry['cases'] if c['fine']['parameters']['length_m'] == .05)
    witness = case['fine']['maximum_nose_down_floor_witness']
    height, pitch = witness['sensor_z_m'], -witness['pitch_nose_up_rad']
    output.mkdir(parents=True, exist_ok=False)
    (output/'.gitignore').write_text('scan_frames.json\n', encoding='ascii')
    paths = ['scripts/validate_bump_lidar_pose.py', 'scripts/lidar_shutdown.py',
             'scripts/validate_start_signal_render.py', 'scripts/audit_start_signal_geometry.py',
             'src/arena_description/config/vehicle.yaml', 'src/arena_description/config/b0183_c1.yaml',
             'src/arena_description/models/arena_car/model.sdf.xacro']
    report = dict(passed=False, scope='isolated_static_gpu_lidar_front_rays_not_vehicle_or_rotating_motion',
                  geometry_report_sha256=hashlib.sha256(args.geometry_report.read_bytes()).hexdigest(),
                  source_hashes={p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths},
                  actual_vehicle_present=False, controller_present=False, real_sensor_verified=False)
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in paths:
            bundle.write(ROOT/path, path)
        bundle.write(args.geometry_report, 'geometry_input.json')
    import xacro
    lidar = yaml.safe_load((ROOT/'src/arena_description/config/vehicle.yaml').read_text())['vehicle']['sensors']['lidar_2d']
    mapping = dict(lidar_samples=str(lidar['samples_per_scan']), lidar_min=str(lidar['range_min_m']),
                   lidar_max=str(lidar['range_max_m']), lidar_resolution=str(lidar['nominal_range_resolution_m']),
                   lidar_noise=str(lidar['simulated_noise_stddev_m']), lidar_rate='10.0')
    generated = ET.fromstring(xacro.process_file(str(ROOT/paths[-1]), mappings=mapping).toxml())
    sensor = generated.find(".//sensor[@name='rplidar_c1']")
    if sensor is None:
        raise ValueError('source C1 sensor not generated')
    fixture, views = build_fixture(sensor, height, pitch)
    ET.indent(fixture)
    ET.ElementTree(fixture).write(output/'fixture.sdf', encoding='utf-8', xml_declaration=True)
    report.update(views=views, source_sensor_mapping=mapping)
    # 정지 센서 3개만 있는 진단. 자식 환경에서만 소프트웨어 렌더링을 지정한다.
    os.environ.update(GZ_PARTITION=f'arena_autonomy_test_{os.getpid()}', ROS_DOMAIN_ID=str(170+os.getpid()%20),
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='',
                      LIBGL_ALWAYS_SOFTWARE='true', GALLIUM_DRIVER='llvmpipe')
    bridges = [v['topic']+'@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' for v in views.values()]
    server = ['gz', 'sim', '-s', '-r', '-v', '4', str(output/'fixture.sdf')]
    launch = ('from launch import LaunchDescription\nfrom launch.actions import ExecuteProcess\n'
              'from launch_ros.actions import Node\n'
              'def generate_launch_description():\n    return LaunchDescription(['+
              f'ExecuteProcess(cmd={server!r}, output="screen"), '+
              f'Node(package="ros_gz_bridge", executable="parameter_bridge", arguments={bridges!r}, output="screen")])\n')
    (output/'fixture.launch.py').write_text(launch, encoding='utf-8')
    import rclpy
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import QoSProfile
    rclpy.init()
    node = rclpy.create_node('bump_pose_capture')
    frames = {name: [] for name in views}
    def receive(message, name):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if len(frames[name]) < 3 and (not frames[name] or stamp > frames[name][-1]['stamp']):
            values = [float(v) for v in message.ranges]
            # +Inf는 no-return 기록. NaN/-Inf는 오류로 보존해 성공 판정을 막는다.
            if any(math.isnan(v) or v == -math.inf for v in values):
                raise ValueError('NaN or negative infinity in captured scan')
            frames[name].append(dict(stamp=stamp, angle_min=float(message.angle_min),
                angle_increment=float(message.angle_increment), range_min=float(message.range_min),
                range_max=float(message.range_max), ranges=[None if v == math.inf else v for v in values]))
    subscriptions = [node.create_subscription(LaserScan, v['topic'],
                     lambda msg, name=name: receive(msg, name), QoSProfile(depth=3)) for name, v in views.items()]
    process = None
    log_path = output/'launch.log'
    log = log_path.open('wb')
    try:
        process = subprocess.Popen(['ros2', 'launch', str(output/'fixture.launch.py')], cwd=output,
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+args.timeout
        while min(map(len, frames.values())) < 3 and time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('fixture launch exited before capture')
            rclpy.spin_once(node, timeout_sec=.1)
        report['capture_complete'] = min(map(len, frames.values())) == 3
        report['captured_counts'] = {name: len(value) for name, value in frames.items()}
        report['scan_comparisons'] = {name: [inspect_scan(r, views[name]) for r in rows] for name, rows in frames.items()}
        report['geometry_matched'] = report['capture_complete'] and all(
            r['passed'] for rows in report['scan_comparisons'].values() for r in rows)
        (output/'scan_frames.json').write_text(json.dumps(frames, allow_nan=False), encoding='utf-8')
    except Exception as error:
        report['error'] = repr(error)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        before = group_processes(process.pid) if process else {}
        report['owned_before_shutdown'] = before
        report['bridge_shutdown'] = stop_bridge_first(process, log_path.read_text(errors='replace'))
        report['gazebo_stop_service'] = stop_gazebo_service(process, log_path.read_text(errors='replace'), os.environ['GZ_PARTITION'])
        cleanup = stop_launch(process)
        log.close()
        report['shutdown_audit'] = audit_shutdown(log_path.read_text(errors='replace'),
                                                 process.poll() if process else None, cleanup or None)
        report['residue_cleanup'] = cleanup_owned_residue(process.pid, before) if process else dict(actions=[], remaining={})
        report['source_unchanged'] = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p,h in report['source_hashes'].items())
        report['passed'] = bool(report.get('geometry_matched') and report['bridge_shutdown']['verified']
            and report['gazebo_stop_service']['verified'] and report['shutdown_audit']['clean']
            and not report['residue_cleanup']['actions'] and not report['residue_cleanup']['remaining']
            and report['source_unchanged'] and 'error' not in report)
        (output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: report.get(k) for k in ('passed', 'capture_complete', 'geometry_matched', 'error')}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
